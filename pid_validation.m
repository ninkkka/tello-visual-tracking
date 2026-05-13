 %% ============================================================
% ВАЛИДАЦИЯ ПИД-РЕГУЛЯТОРА — КАНАЛ LR (ФИНАЛЬНАЯ ВЕРСИЯ)
% Сравнение: Теоретический Step Response vs Реальное восстановление ошибки
% ============================================================
clear; close all; clc;

%% ==================== НАСТРОЙКИ ====================
PEAK_MIN_HEIGHT  = 0.04;   % Минимальная высота пика ошибки (м)
PEAK_MIN_PROM    = 0.03;   % Минимальная «выразительность» пика (м)
PEAK_MIN_DIST    = 25;     % Минимальное расстояние между пиками (кадры)
DECAY_MIN_LEN    = 10;     % Минимальная длина затухания (кадры)
DECAY_END_THRESH = 0.015;  % Порог «ошибка ≈ 0» для конца затухания (м)
Fs               = 20;     % Частота кадров (Гц)
dt               = 1/Fs;
MAX_VIEW_TIME    = 2.5;    % Максимальное время отображения на графике (сек)

%% ==================== 1. ЧТЕНИЕ ЛОГОВ ====================
log_file = 'flight_logs.txt';
if ~isfile(log_file)
    error('Файл %s не найден! Положите скрипт рядом с файлом логов.', log_file);
end

fid = fopen(log_file, 'r');
raw = textscan(fid, '%s', 'Delimiter', '\n');
fclose(fid);
lines = raw{1};

% Парсинг
n = 0;
err_x_data = [];

fprintf('Чтение логов...\n');
for i = 1:length(lines)
    line = lines{i};
    if ~contains(line, 'ID3 xy=('), continue; end
    
    try
        % err_x из xy=(..., ...)м
        xy_start = strfind(line, 'xy=(') + 4;
        xy_end   = strfind(line, ')м diag=') - 1;
        xy_str = line(xy_start:xy_end);
        xy_parts = strsplit(xy_str, ',');
        ex = str2double(xy_parts{1});
        
        n = n + 1;
        err_x_data(n) = ex;
    catch
    end
end

err_x = err_x_data(:);
N = length(err_x);
time = (0:N-1)' * dt;

fprintf('Прочитано кадров: %d (%.1f сек)\n', N, N*dt);

%% ==================== 2. ПОИСК ПИКОВ ОШИБКИ ====================
% Ищем пики |err_x| — моменты максимального отклонения
[~, peak_locs] = findpeaks(abs(err_x), ...
    'MinPeakHeight',  PEAK_MIN_HEIGHT, ...
    'MinPeakProminence', PEAK_MIN_PROM, ...
    'MinPeakDistance', PEAK_MIN_DIST);

fprintf('Найдено пиков ошибки: %d\n', length(peak_locs));

%% ==================== 3. ИЗВЛЕЧЕНИЕ ЗАТУХАНИЙ ====================
transients = {};   % Хранилище переходных процессов
t_max = 0;         % Максимальная длина затухания

for k = 1:length(peak_locs)
    pk = peak_locs(k);
    pk_val = err_x(pk);         % Сохраняем знак!
    
    % --- Ищем конец затухания (ошибка падает ниже порога) ---
    end_idx = pk;
    while end_idx < N && abs(err_x(end_idx)) > DECAY_END_THRESH
        end_idx = end_idx + 1;
        if end_idx - pk > 100 % Ограничение длины куска (5 секунд)
            break; 
        end
    end
    
    decay_len = end_idx - pk;
    
    % --- Фильтрация качества ---
    if decay_len < DECAY_MIN_LEN
        continue;
    end
    
    % --- Извлекаем затухание ---
    segment = err_x(pk:end_idx);
    
    % Нормируем: делим на величину пика → идёт от 1 к ~0
    % Формула: (e(t) - 0) / e(0)  =>  e(t) / e0
    % Но нам нужно, чтобы график шел СНИЗУ ВВЕРХ (как step response от 0 к 1)
    % Ошибка падает с 1 до 0. Step response растет с 0 до 1.
    % Значит: y_norm = 1 - (segment / pk_val)
    
    segment_norm = 1 - (segment / pk_val);
    
    % Сохраняем
    idx = length(transients) + 1;
    transients{idx} = segment_norm;
    t_max = max(t_max, length(segment_norm));
end

fprintf('Годных затуханий: %d\n', length(transients));

if length(transients) < 2
    warning('Мало переходных процессов (< 2). Уменьшите пороги PEAK_MIN_HEIGHT.');
end

%% ==================== 4. УСРЕДНЕНИЕ ====================
trans_matrix = NaN(length(transients), t_max);

for k = 1:length(transients)
    L = length(transients{k});
    trans_matrix(k, 1:L) = transients{k};
end

% Среднее и стандартное отклонение
avg_transient = nanmean(trans_matrix, 1);
std_transient = nanstd(trans_matrix, 0, 1);

% Временная ось для затухания
t_decay = (0:t_max-1) * dt;

%% ==================== 5. ТЕОРЕТИЧЕСКАЯ МОДЕЛЬ ====================
% Параметры из вашей идентификации (канал LR)
K_lr  = 1.0;
T_lr  = 0.30;
xi_lr = 0.65;

% Модель растения
sys_plant = tf(K_lr, [T_lr^2, 2*xi_lr*T_lr, 1]);

% ПИД-регулятор (коэффициенты из MATLAB Tuner)
Kp = 0.57;  Ki = 0.50;  Kd = 0.15;
C_pid = pid(Kp, Ki, Kd);

% Замкнутая система
CL = feedback(C_pid * sys_plant, 1);

% Step-отклик (идёт от 0 к 1)
[y_step, t_step] = step(CL, 3);

% !!! КЛЮЧЕВОЕ ИСПРАВЛЕНИЕ !!!
% err_theory = 1 - y_step;  <-- БЫЛО (идёт сверху вниз 1->0)
err_theory = y_step;        %<-- СТАЛО (идёт снизу вверх 0->1, как эксперимент)

%% ==================== 6. ГРАФИК СРАВНЕНИЯ ====================
figure('Name', 'Валидация ПИД — Канал LR', 'Position', [100 100 1000 700]);

% --- Подграфик 1: Все переходные процессы ---
subplot(2,2,[1,2]);
hold on;

% Рисуем каждый переход тонкой серой линией
for k = 1:length(transients)
    L = length(transients{k});
    plot(t_decay(1:L), transients{k}, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
end

% Доверительный интервал (среднее ± СКО)
fill([t_decay fliplr(t_decay)], ...
     [avg_transient+std_transient fliplr(avg_transient-std_transient)], ...
     [0.85 0.85 1], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

% Среднее экспериментальное
plot(t_decay, avg_transient, 'b-', 'LineWidth', 2);

% Теоретическое затухание (step response)
plot(t_step, err_theory, 'r--', 'LineWidth', 2);

xlabel('Время (с)');
ylabel('Нормированный отклик');
title(sprintf('Переходные процессы LR (n=%d)', length(transients)));
legend('Эксперимент (отдельные)', '± СКО', ...
       'Эксперимент (среднее)', 'Теория: step(CL)', ...
       'Location', 'northeast');
grid on;
xlim([0 MAX_VIEW_TIME]); % <--- ВАЖНО: Обрезаем хвост
ylim([-0.1 1.1]);

% --- Подграфик 2: Только сравнение среднее vs теория ---
subplot(2,2,3);
hold on;
plot(t_decay, avg_transient, 'b-', 'LineWidth', 2);
plot(t_step, err_theory, 'r--', 'LineWidth', 2);
fill([t_decay fliplr(t_decay)], ...
     [avg_transient+std_transient fliplr(avg_transient-std_transient)], ...
     [0.85 0.85 1], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
xlabel('Время (с)');
ylabel('Отклик');
title('Среднее vs Теория');
legend('Эксперимент', 'Теория', 'Location', 'northeast');
grid on;
xlim([0 MAX_VIEW_TIME]); % <--- ВАЖНО: Обрезаем хвост

%% ==================== 7. МЕТОД 2: СИМУЛЯЦИЯ ПИД ====================
fprintf('\n=== МЕТОД 2: Проверка команды ПИД ===\n');

% Дискретная симуляция ПИД на реальной ошибке
cmd_predicted = zeros(N, 1);
integral_err = 0;
prev_err = 0;

for i = 1:N
    e = err_x(i);
    
    % Мёртвая зона (как в Python коде)
    if abs(e) < 0.07
        e = 0;
        integral_err = 0; % Сброс интегратора в мертвой зоне
    else
        integral_err = integral_err + e * dt;
    end
    
    derivative = (e - prev_err) / dt;
    
    cmd_pred = Kp * e + Ki * integral_err + Kd * derivative;
    
    % Ограничения (клиппинг) как в дроне
    cmd_pred = max(-0.7, min(0.7, cmd_pred));
    
    cmd_predicted(i) = cmd_pred;
    prev_err = e;
end

% Сравниваем форму сигнала (нормируем для корреляции)
if max(abs(cmd_predicted)) > 0
    R_cmd = corrcoef(err_x, cmd_predicted);
    fprintf('Корреляция (ошибка vs предсказанная команда): %.4f\n', R_cmd(1,2));
end

%% ==================== 8. МЕТРИКИ (ИСПРАВЛЕННЫЙ) ====================
fprintf('\n=== МЕТРИКИ СРАВНЕНИЯ ===\n');

% 1. Определяем общую временную сетку (минимум из двух)
t_common_end = min(t_decay(end), t_step(end));

% 2. Создаем общую ось времени
t_common = (0:dt:t_common_end)';

% 3. Интерполируем ОБА сигнала на общую сетку
exp_interp = interp1(t_decay, avg_transient, t_common, 'pchip', NaN);
theo_interp = interp1(t_step, err_theory, t_common, 'pchip', NaN);

% 4. Убираем NaN
valid = ~isnan(exp_interp) & ~isnan(theo_interp);
exp_clean = exp_interp(valid);
theo_clean = theo_interp(valid);

if length(exp_clean) > 5
    % RMSE
    rmse_val = sqrt(mean((exp_clean - theo_clean).^2));
    fprintf('RMSE (среднее vs теория): %.4f\n', rmse_val);

    % Корреляция
    R = corrcoef(exp_clean, theo_clean);
    fprintf('Корреляция: %.4f\n', R(1,2));
    
    % Время регулирования (2%) - ищем где сигнал достигает 0.98 (2% от 1)
    % Для step response это 0.98
    idx_exp = find(exp_clean >= 0.98, 1);
    if ~isempty(idx_exp)
        fprintf('Время регулирования (эксп): %.2f с\n', t_common(idx_exp));
    else
        fprintf('Время регулирования (эксп): > %.2f с (не уложилось)\n', t_common_end);
    end
    
    idx_theo = find(theo_clean >= 0.98, 1);
    if ~isempty(idx_theo)
        fprintf('Время регулирования (теория): %.2f с\n', t_common(idx_theo));
    end
else
    fprintf('Недостаточно данных для расчета метрик.\n');
end

fprintf('\n=== ГОТОВО ===\n');
fprintf('График сохранён в текущей директории.\n');