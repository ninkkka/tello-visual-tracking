clear; clc; close all;

test_names = {'forward', 'backward', 'right', 'left', 'up', 'down'};

fprintf('============================================================\n');
fprintf('ОБРАБОТКА СТУПЕНЧАТЫХ ТЕСТОВ\n');
fprintf('============================================================\n');

for test_idx = 1:length(test_names)
    test_name = test_names{test_idx};
    fprintf('\n========== %s ==========\n', upper(test_name));
    
    pattern = sprintf('step_test_%s_*.csv', test_name);
    files = dir(pattern);
    
    if isempty(files)
        fprintf('  Файлы не найдены.\n');
        continue;
    end
    
    fprintf('  Найдено файлов: %d\n', length(files));
    
    % ============================================================
    % ЧТЕНИЕ ФАЙЛОВ
    % ============================================================
    all_steps = {};
    step_count = 0;
    
    for file_idx = 1:length(files)
        data = readtable(files(file_idx).name);
        
        if strcmp(test_name, 'forward') || strcmp(test_name, 'backward')
            cmd_col = data.cmd_fb;
        elseif strcmp(test_name, 'right') || strcmp(test_name, 'left')
            cmd_col = data.cmd_lr;
        else
            cmd_col = data.cmd_ud;
        end
        
        cmd_vals = cmd_col;
        if isnumeric(cmd_vals)
            cmd_vals(isnan(cmd_vals)) = 0;
        else
            continue;
        end
        
        step_idx = find(cmd_vals ~= 0);
        if isempty(step_idx) || length(step_idx) < 5
            continue;
        end
        
        start_idx = max(1, step_idx(1) - 20);
        end_idx = min(height(data), step_idx(end) + 40);
        
        t = data.time_sec(start_idx:end_idx);
        dist = data.distance_m(start_idx:end_idx);
        cmd = cmd_vals(start_idx:end_idx);
        
        valid = ~isnan(dist);
        if sum(valid) < 5
            continue;
        end
        
        t_valid = t(valid);
        dist_valid = dist(valid);
        cmd_valid = cmd(valid);
        
        Ts = 0.05;
        t_uniform = t_valid(1):Ts:t_valid(end);
        dist_interp = interp1(t_valid, dist_valid, t_uniform, 'pchip');
        cmd_interp = interp1(t_valid, cmd_valid, t_uniform, 'previous');
        
        cmd_start = find(cmd_interp ~= 0, 1, 'first');
        if isempty(cmd_start)
            continue;
        end
        t_centered = t_uniform - t_uniform(cmd_start);
        
        step_count = step_count + 1;
        all_steps{step_count}.t = t_centered;
        all_steps{step_count}.dist = dist_interp;
        all_steps{step_count}.cmd = cmd_interp;
    end
    
    fprintf('  Годных ступенек: %d\n', step_count);
    
    if step_count == 0
        fprintf('  -> Нет данных.\n');
        continue;
    end
    
    % ============================================================
    % УСРЕДНЕНИЕ И СГЛАЖИВАНИЕ
    % ============================================================
    min_len = inf;
    for i = 1:step_count
        min_len = min(min_len, length(all_steps{i}.dist));
    end
    
    t_avg = all_steps{1}.t(1:min_len);
    dist_all = zeros(step_count, min_len);
    
    for i = 1:step_count
        dist_all(i, :) = all_steps{i}.dist(1:min_len);
    end
    
    dist_avg = mean(dist_all, 1, 'omitnan');
    dist_smooth = sgolayfilt(dist_avg, 3, 11);
    
    pre_idx = t_avg < 0;
    if sum(pre_idx) > 3
        baseline = mean(dist_smooth(pre_idx), 'omitnan');
    else
        baseline = dist_smooth(1);
    end
    dist_centered = dist_smooth - baseline;
    
    % ============================================================
    % ИДЕНТИФИКАЦИЯ
    % ============================================================
    fit_idx = t_avg >= 0 & t_avg <= 2.0;
    t_fit = t_avg(fit_idx);
    dist_fit = dist_centered(fit_idx);
    
    if length(t_fit) < 10
        fprintf('  -> Мало точек.\n');
        continue;
    end
    
    % Сантиметры
    dist_fit_cm = dist_fit * 100;
    
    % Инверсия
    dist_end = mean(dist_fit_cm(end-5:end));
    dist_start = mean(dist_fit_cm(1:5));
    if dist_end < dist_start
        dist_fit_cm = -dist_fit_cm;
        fprintf('  (инвертировано)\n');
    end
    
    cmd_vals = all_steps{1}.cmd(all_steps{1}.cmd ~= 0);
    cmd_val = abs(mode(cmd_vals));
    
    Ts = 0.05;
    u_fit = cmd_val * ones(size(dist_fit_cm));
    data_id = iddata(dist_fit_cm(:), u_fit(:), Ts);
    
    best_fit = -Inf;
    sys = [];
    best_name = '';
    
    try
        m = procest(data_id, 'P1');
        f = m.Report.Fit.FitPercent;
        fprintf('  P1 fit: %.1f%%\n', f);
        if f > best_fit, best_fit = f; sys = m; best_name = 'P1'; end
    catch
    end
    
    try
        m = tfest(data_id, 2, 0);
        f = m.Report.Fit.FitPercent;
        fprintf('  tf(2,0) fit: %.1f%%\n', f);
        if f > best_fit, best_fit = f; sys = m; best_name = 'tf(2,0)'; end
    catch
    end
    
    if isempty(sys)
        fprintf('  -> Нет модели.\n');
        continue;
    end
    
    fprintf('  Лучшая: %s (%.1f%%)\n', best_name, best_fit);