% PID TUNER
    % ============================================================
    try
        if isdt(sys), sys = d2c(sys); end
        
        options = pidtuneOptions('PhaseMargin', 65);
        [C, info] = pidtune(sys, 'PID', options);
        
        % Сохраняем сырые значения
        raw_Kp = C.Kp;
        raw_Ki = C.Ki;
        raw_Kd = C.Kd;
        
        % НОРМИРОВКА ДЛЯ PYTHON
        % В Python ошибка в метрах, команда в долях (0.0-1.0)
        % В MATLAB ошибка в см, команда в % (0-100)
        % Коэффициент пересчёта: /100
        Kp = abs(raw_Kp) / 100;
        Ki = abs(raw_Ki) / 100;
        Kd = abs(raw_Kd) / 100;
        
        % Разумные пределы для Tello
        Kp = max(0.05, min(Kp, 2.0));
        Ki = max(0.0, min(Ki, 0.5));
        Kd = max(0.0, min(Kd, 0.3));
        
        fprintf('  Raw  PID: Kp=%.1f, Ki=%.1f, Kd=%.1f\n', raw_Kp, raw_Ki, raw_Kd);
        fprintf('  Python:   Kp=%.4f, Ki=%.4f, Kd=%.4f\n', Kp, Ki, Kd);
        
        results{test_idx} = struct('name', test_name, ...
            'model', best_name, 'fit', best_fit, ...
            'Kp', Kp, 'Ki', Ki, 'Kd', Kd, ...
            'raw_Kp', raw_Kp, 'raw_Ki', raw_Ki, 'raw_Kd', raw_Kd);
        
        % График
        figure('Name', test_name, 'Position', [100 100 900 400]);
        
        subplot(1,2,1);
        plot(t_avg, dist_centered*100, 'b-', 'LineWidth', 1.5); hold on;
        xline(0, 'r--', 'LineWidth', 1.5);
        xlabel('Время (с)'); ylabel('Δ Дистанция (см)');
        title(sprintf('%s (fit=%.1f%%)', test_name, best_fit));
        grid on;
        
        subplot(1,2,2);
        step(feedback(C * sys, 1));
        title(sprintf('Замкнутая система (Kp=%.3f, Ki=%.3f, Kd=%.3f)', Kp, Ki, Kd));
        grid on;
        
    catch e
        fprintf('  Ошибка PID: %s\n', e.message);
    end
end

% ============================================================
% ИТОГОВАЯ ТАБЛИЦА
% ============================================================
fprintf('\n\n');
fprintf('======================================================================\n');
fprintf('ИТОГОВЫЕ КОЭФФИЦИЕНТЫ ДЛЯ PYTHON\n');
fprintf('======================================================================\n');
fprintf('%-12s %-10s %6s | %10s %10s %10s\n', 'Тест', 'Модель', 'Fit%', 'Kp', 'Ki', 'Kd');
fprintf('----------------------------------------------------------------------\n');
for i = 1:length(results)
    if ~isempty(results{i})
        r = results{i};
        fprintf('%-12s %-10s %5.1f%% | %10.4f %10.4f %10.4f\n', ...
            r.name, r.model, r.fit, r.Kp, r.Ki, r.Kd);
    else
        fprintf('%-12s %-10s %6s | %10s %10s %10s\n', test_names{i}, '—', '—', '—', '—', '—');
    end
end
fprintf('======================================================================\n');

% ============================================================
% ГОТОВЫЕ СТРОКИ ДЛЯ PYTHON
% ============================================================
fprintf('\n\n');
fprintf('%% ===== СКОПИРУЙ ЭТИ СТРОКИ В PYTHON =====\n');
for i = 1:length(results)
    if ~isempty(results{i})
        r = results{i};
        if strcmp(r.name, 'forward') || strcmp(r.name, 'backward')
            fprintf('pid_track_fb = PID(%.2f, %.2f, %.2f, setpoint=0.0)  %% %s (fit=%.0f%%)\n', ...
                r.Kp, r.Ki, r.Kd, r.name, r.fit);
        elseif strcmp(r.name, 'right') || strcmp(r.name, 'left')
            fprintf('pid_track_lr = PID(%.2f, %.2f, %.2f, setpoint=0.0)  %% %s (fit=%.0f%%)\n', ...
                r.Kp, r.Ki, r.Kd, r.name, r.fit);
        else
            fprintf('pid_track_ud = PID(%.2f, %.2f, %.2f, setpoint=0.0)  %% %s (fit=%.0f%%)\n', ...
                r.Kp, r.Ki, r.Kd, r.name, r.fit);
        end
    end
end
fprintf('%% ==========================================\n');

save('pid_results.mat', 'results');
fprintf('\nРезультаты сохранены в pid_results.mat\n');