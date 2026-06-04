% =========================================================================
% MASTER SCRIPT: Porównanie obiektu rzeczywistego z modelami w Simulinku
% =========================================================================
% Wersja BEZ przycinania czasu - pokazuje cały oryginalny przebieg.

% Sprawdzenie czy nazwa pliku w workspace uległa zmianie. Jeśli tak, to
% stare dane z symulacji (ScopeData, ScopeData1, out) są już nieaktualne
% i powinny zostać skasowane.
nowa_nazwa = 'proba2.csv';
if exist('nazwa_pliku', 'var') && ~strcmp(nazwa_pliku, nowa_nazwa)
    clear ScopeData ScopeData1 out;
end
nazwa_pliku = nowa_nazwa;

% Jeśli w Workspace istnieje obiekt 'out' (czyli Simulink właśnie zapisał nową symulację),
% to musimy skasować lokalne zmienne ScopeData/ScopeData1, aby skrypt nie używał ich starych wersji.
if exist('out', 'var')
    clear ScopeData ScopeData1;
end

clearvars -except ScopeData ScopeData1 out nazwa_pliku;
close all; clc;

% === 1. WYBÓR PLIKU Z DANYMI RZECZYWISTYMI ===
% Zdefiniowano powyżej jako nowa_nazwa (np. proba1.csv lub proba2.csv) 

if ~exist(nazwa_pliku, 'file')
    error('Plik %s nie istnieje w tym folderze!', nazwa_pliku);
end

fprintf('Wczytywanie danych z pliku: %s...\n', nazwa_pliku);
data = readtable(nazwa_pliku);

% Wyciągnięcie danych surowych (pełnych)
t_raw = data.time;
y_raw = data.filtered;
setpoint_raw = data.setpoint;
u_raw = data.control;

% === 2. PARAMETRY CZASU PRÓBKOWANIA ===
dt_log = diff(t_raw);
Ts_log = mean(dt_log);
fprintf('Okres próbkowania zapisu (kamera): %.4f s (%.1f Hz)\n', Ts_log, 1/Ts_log);

% Czas pętli STM32 (32 ms po uwzględnieniu obliczeń)
Ts = 0.032; 
fprintf('Okres próbkowania pętli STM32 (regulacja): %.4f s (%.1f Hz)\n', Ts, 1/Ts);


% === 3. PARAMETRY FIZYCZNE I WZMOCNIENIA MODELU ===
T_serw = 0.095;     % Stała czasowa serwa
c_ball = 5.886;     % Przyspieszenie kulki [m/s^2]
k_mech = 0.2;       % Przekładnia mechaniczna (3cm / 15cm)

% Model Teoretyczny
K_rad = c_ball * 1000 * k_mech;  % [mm/(s^2*rad)]
K_t = K_rad * pi / 180;          % [mm/(s^2*deg)] -> 20.55
K_teor = K_t;       % Wzmocnienie dla bloku teoretycznego

% Model Doświadczalny
K_d = 11.9318;      % [mm/(s^2*deg)] -> z identyfikacji
K1 = K_d;           % Wzmocnienie dla bloku doświadczalnego


% === 4. PRZELICZENIE NASTAW REGULATORA (STM32 -> Simulink) ===
Kp_STM = 0.26;
Ki_STM = 0.0064;
Kd_STM = 4.4;

Kp = Kp_STM;
Ki = Ki_STM / Ts;   % Wzmocnienie całki w Simulinku (1/Ti)
Kd = Kd_STM * Ts;   % Wzmocnienie różniczki w Simulinku (Td)


% === 5. PRZYGOTOWANIE ZMIENNYCH DLA SIMULINKA ===
% Zamiast przycinać dane, wysyłamy do Simulinka pełny przebieg setpointu.
% Simulink odczyta wartości zadane dokładnie w tych samych sekundach, co rzeczywisty obiekt!
setpoint_timeseries = timeseries(setpoint_raw, t_raw);


% === 6. AUTOMATYCZNE USTAWIANIE STOP TIME W SIMULINKU ===
% Wczytujemy model do pamięci, ustawiamy StopTime zgodny z czasem trwania CSV
% i zapisujemy model, aby współczynniki całkowe były w 100% porównywalne.
try
    model_name = 'ballonbeam_matlab1';
    load_system(model_name);
    
    % Ustawienie StopTime
    set_param(model_name, 'StopTime', num2str(t_raw(end)));
    
    % Wymuszenie zapisu danych (DataLogging) dla Scope (pozycja) i Scope1 (sterowanie)
    set_param([model_name '/Scope'], 'DataLogging', 'on');
    set_param([model_name '/Scope'], 'DataLoggingVariableName', 'ScopeData');
    set_param([model_name '/Scope'], 'DataLoggingSaveFormat', 'Structure with time');
    
    set_param([model_name '/Scope1'], 'DataLogging', 'on');
    set_param([model_name '/Scope1'], 'DataLoggingVariableName', 'ScopeData1');
    set_param([model_name '/Scope1'], 'DataLoggingSaveFormat', 'Structure with time');
    
    save_system(model_name);
    fprintf('  -> Pomyślnie zaktualizowano StopTime = %.2f s oraz włączono logowanie Scope w pliku .slx.\n', t_raw(end));
catch
    fprintf('  -> Ostrzeżenie: Nie udało się automatycznie zapisać parametrów w pliku .slx (upewnij się, że model nie jest zablokowany lub otwarty jako tylko do odczytu).\n');
end


% === 7. PODSUMOWANIE ===
fprintf('\n=========================================================\n');
fprintf('  PARAMETRY EKSPORTOWANE DO WORKSPACE:\n');
fprintf('=========================================================\n');
fprintf('  Wzmocnienie teor. obiektu (K_teor):  %.4f mm/(s^2*deg)\n', K_teor);
fprintf('  Wzmocnienie dośw. obiektu (K1):      %.4f mm/(s^2*deg)\n', K1);
fprintf('  Wzmocnienie P regulatora (Kp):       %.4f\n', Kp);
fprintf('  Wzmocnienie I regulatora (Ki):       %.4f (w Simulinku jako 1/Ti)\n', Ki);
fprintf('  Wzmocnienie D regulatora (Kd):       %.4f (w Simulinku jako Td)\n', Kd);
fprintf('  Czas trwania symulacji (StopTime):   %.2f s\n', t_raw(end));
fprintf('=========================================================\n');
fprintf('\nGotowe! Plik Simulinka został przygotowany.\n');

% === 8. SPRAWDZENIE CZY SYMULACJA ZOSTAŁA URUCHOMIONA ===
% Ponieważ uruchamiasz symulację ręcznie w Simulinku, sprawdzamy czy dane 
% zostały wyeksportowane do Workspace (bezpośrednio lub w obiekcie 'out').
has_direct = evalin('base', 'exist(''ScopeData'', ''var'')') && evalin('base', 'exist(''ScopeData1'', ''var'')');
has_out = false;

if ~has_direct && evalin('base', 'exist(''out'', ''var'')')
    try
        out_var = evalin('base', 'out');
        if isa(out_var, 'Simulink.SimulationOutput')
            % Sprawdzamy na kilka sposobów w zależności od wersji MATLABa
            has_ScopeData = any(strcmp(out_var.who, 'ScopeData')) || ...
                            isprop(out_var, 'ScopeData') || ...
                            ~isempty(out_var.get('ScopeData'));
            has_ScopeData1 = any(strcmp(out_var.who, 'ScopeData1')) || ...
                             isprop(out_var, 'ScopeData1') || ...
                             ~isempty(out_var.get('ScopeData1'));
            if has_ScopeData && has_ScopeData1
                has_out = true;
            end
        elseif isstruct(out_var) || isa(out_var, 'struct')
            if isfield(out_var, 'ScopeData') && isfield(out_var, 'ScopeData1')
                has_out = true;
            end
        end
    catch
        % W razie niepowodzenia ignorujemy
    end
end

if ~has_direct && ~has_out
    fprintf('\n=========================================================\n');
    fprintf('  [WSKAZÓWKA]: Dane z symulacji nie są jeszcze załadowane.\n');
    fprintf('=========================================================\n');
    fprintf('  1. Otwórz model "%s.slx" w Simulinku.\n', model_name);
    fprintf('  2. Uruchom symulację ręcznie (kliknij zieloną strzałkę "Run").\n');
    fprintf('  3. Po zakończeniu symulacji, uruchom ten skrypt ponownie,\n');
    fprintf('     aby automatycznie wygenerować wykresy PDF i policzyć wskaźniki.\n');
    fprintf('=========================================================\n\n');
    return;
end

% === 9. POBIERANIE I EKSTRAKCJA DANYCH Z SYMULACJI ===
t_sim = [];
y_sim = [];
t_sim1 = [];
u_sim = [];

if has_direct
    ScopeData = evalin('base', 'ScopeData');
    ScopeData1 = evalin('base', 'ScopeData1');
else
    out_var = evalin('base', 'out');
    if isa(out_var, 'Simulink.SimulationOutput')
        try
            ScopeData = out_var.get('ScopeData');
        catch
            ScopeData = [];
        end
        if isempty(ScopeData)
            try
                ScopeData = out_var.ScopeData;
            catch
                ScopeData = [];
            end
        end
        
        try
            ScopeData1 = out_var.get('ScopeData1');
        catch
            ScopeData1 = [];
        end
        if isempty(ScopeData1)
            try
                ScopeData1 = out_var.ScopeData1;
            catch
                ScopeData1 = [];
            end
        end
    else
        ScopeData = out_var.ScopeData;
        ScopeData1 = out_var.ScopeData1;
    end
end

% Ekstrakcja danych
if ~isempty(ScopeData)
    [t_sim, y_sim] = extractScopeData(ScopeData);
end
if ~isempty(ScopeData1)
    [t_sim1, u_sim] = extractScopeData(ScopeData1);
end

% Przypisanie sygnałów (pozycja i sterowanie z symulacji)
% Dla Scope (pozycja): kolumna 1 to Setpoint (Yr), kolumna 2 to właściwa odpowiedź modelu.
% Dla Scope1 (sterowanie): kolumna 1 to sterowanie u, kolumna 2 to nasycone sterowanie usat.
y_model_sim = [];
u_model_sim = [];

if ~isempty(y_sim)
    if size(y_sim, 2) >= 2
        y_model_sim = y_sim(:, 2);
    else
        y_model_sim = y_sim(:, 1);
    end
end

if ~isempty(u_sim)
    if size(u_sim, 2) >= 2
        u_model_sim = u_sim(:, 2);
    else
        u_model_sim = u_sim(:, 1);
    end
end

% === 10. GENEROWANIE I ZAPISYWANIE WYKRESÓW DO PDF ===
[~, nazwa_bez_ext, ~] = fileparts(nazwa_pliku);
pdf_name1 = ['odpowiedz_' nazwa_bez_ext '.pdf'];
pdf_name2 = ['sterowanie_' nazwa_bez_ext '.pdf'];

% Wykres 1: Pozycja kulki (Odpowiedź czasowa)
fig1 = figure('Name', ['Odpowiedz - ' nazwa_pliku], 'Color', [1 1 1], 'Position', [100, 100, 800, 500]);
plot(t_raw, setpoint_raw, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Wartość zadana (Setpoint)'); hold on;
plot(t_raw, y_raw, 'Color', [0.15 0.15 0.15], 'LineWidth', 2, 'DisplayName', 'Obiekt rzeczywisty');
if ~isempty(y_model_sim)
    plot(t_sim, y_model_sim, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Model');
end
xlabel('Czas [s]');
ylabel('Pozycja kulki [mm]');
title(sprintf('Porównanie odpowiedzi czasowej pozycji (%s)', nazwa_pliku));
legend('Location', 'best');
grid on;
hold off;

% Wykres 2: Sygnał sterujący (Kąt belki/serwa)
fig2 = figure('Name', ['Sterowanie - ' nazwa_pliku], 'Color', [1 1 1], 'Position', [150, 150, 800, 500]);
plot(t_raw, u_raw, 'Color', [0.15 0.15 0.15], 'LineWidth', 2, 'DisplayName', 'Obiekt rzeczywisty'); hold on;
if ~isempty(u_model_sim)
    plot(t_sim1, u_model_sim, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Model');
end
xlabel('Czas [s]');
ylabel('Kąt serwa [deg]');
title(sprintf('Porównanie sygnału sterującego (%s)', nazwa_pliku));
legend('Location', 'best');
grid on;
hold off;

% Eksport do PDF na pełną szerokość (z uciętymi marginesami)
fprintf('\nZapisywanie wykresów do plików PDF...\n');

% Konfiguracja formatu strony figury do wymiarów okna przed zapisem (dla saveas/print/exportgraphics)
for fig_obj = [fig1, fig2]
    set(fig_obj, 'Units', 'inches');
    fig_pos = get(fig_obj, 'Position');
    set(fig_obj, 'PaperPositionMode', 'manual');
    set(fig_obj, 'PaperUnits', 'inches');
    set(fig_obj, 'PaperPosition', [0, 0, fig_pos(3), fig_pos(4)]);
    set(fig_obj, 'PaperSize', [fig_pos(3), fig_pos(4)]);
end

try
    exportgraphics(fig1, pdf_name1, 'ContentType', 'vector', 'BackgroundColor', 'none');
    fprintf('  -> Pomyślnie zapisano wykres pozycji: %s (exportgraphics)\n', pdf_name1);
catch
    print(fig1, pdf_name1, '-dpdf', '-r0');
    fprintf('  -> Zapisano wykres pozycji za pomocą print: %s\n', pdf_name1);
end

try
    exportgraphics(fig2, pdf_name2, 'ContentType', 'vector', 'BackgroundColor', 'none');
    fprintf('  -> Pomyślnie zapisano wykres sterowania: %s (exportgraphics)\n', pdf_name2);
catch
    print(fig2, pdf_name2, '-dpdf', '-r0');
    fprintf('  -> Zapisano wykres sterowania za pomocą print: %s\n', pdf_name2);
end

% === 11. OBLICZANIE WSKAŹNIKÓW JAKOŚCI DLA TABELI ===
% Wyznaczenie momentu skoku z danych surowych (stabilne wartości bez szumów)
idx_skoku_raw = find(diff(setpoint_raw) ~= 0, 1) + 1;
if isempty(idx_skoku_raw)
    idx_skoku_raw = 1;
end
t_step = t_raw(idx_skoku_raw);
y_start = setpoint_raw(idx_skoku_raw - 1);
y_final = setpoint_raw(idx_skoku_raw);
amplitude = y_final - y_start;
is_step_up = (amplitude > 0);

% Wskaźniki dla obiektu rzeczywistego (tylko od momentu skoku t >= t_step)
idx_real_post = find(t_raw >= t_step);
t_real_post = t_raw(idx_real_post) - t_step; % Czas relatywny (tau)
y_real_post = y_raw(idx_real_post);
u_real_post = u_raw(idx_real_post);
sp_real_post = setpoint_raw(idx_real_post);
dt_real_post = [0; diff(t_raw(idx_real_post))];

e_real_post = sp_real_post - y_real_post;
ITAE_real = sum(t_real_post .* abs(e_real_post) .* dt_real_post);
Cost_real = sum((u_real_post.^2) .* dt_real_post);
[overshoot_real, ts_real] = calculateMetrics(t_raw, y_raw, t_step, y_start, y_final, amplitude, is_step_up);

% Wskaźniki dla modelu symulacyjnego (tylko od momentu skoku t >= t_step)
if ~isempty(y_model_sim) && ~isempty(t_sim)
    sp_sim = interp1(t_raw, setpoint_raw, t_sim, 'linear', 'extrap');
    
    idx_sim_post = find(t_sim >= t_step);
    t_sim_post = t_sim(idx_sim_post) - t_step; % Czas relatywny (tau)
    y_sim_post = y_model_sim(idx_sim_post);
    sp_sim_post = sp_sim(idx_sim_post);
    dt_sim_post = [0; diff(t_sim(idx_sim_post))];
    
    e_sim_post = sp_sim_post - y_sim_post;
    ITAE_model = sum(t_sim_post .* abs(e_sim_post) .* dt_sim_post);
    
    [overshoot_model, ts_model] = calculateMetrics(t_sim, y_model_sim, t_step, y_start, y_final, amplitude, is_step_up);
    
    if ~isempty(u_model_sim)
        u_model_interp = interp1(t_sim1, u_model_sim, t_sim, 'linear', 'extrap');
        u_sim_post = u_model_interp(idx_sim_post);
        Cost_model = sum((u_sim_post.^2) .* dt_sim_post);
    else
        Cost_model = NaN;
    end
else
    ITAE_model = NaN; overshoot_model = NaN; ts_model = 'NaN'; Cost_model = NaN;
end

% Wyświetlenie wyników w konsoli
fprintf('\n=========================================================================\n');
fprintf('  TABELA WSKAŹNIKÓW JAKOŚCI DLA PLIKU: %s\n', nazwa_pliku);
fprintf('=========================================================================\n');
fprintf('  Obiekt / Model       | Przeregulowanie [%%] | Czas reg. ts [s] | ITAE [mm*s^2] | Koszt Ju [deg^2*s]\n');
fprintf('-------------------------------------------------------------------------\n');
fprintf('  Obiekt rzeczywisty   | %19.2f | %16s | %13.2f | %17.2f\n', overshoot_real, ts_real, ITAE_real, Cost_real);
fprintf('  Model                | %19.2f | %16s | %13.2f | %17.2f\n', overshoot_model, ts_model, ITAE_model, Cost_model);
fprintf('=========================================================================\n');
fprintf('  *Uwaga: Wskaźniki ITAE i Koszt liczone od momentu skoku. ts względem momentu skoku.\n');


% =========================================================================
% LOKALNE FUNKCJE POMOCNICZE (na końcu pliku)
% =========================================================================

function [t_sim, y_sim] = extractScopeData(scopeVar)
    % Wyciąga czas i wartości z zmiennej scope niezależnie od formatu zapisu w Simulinku
    if isnumeric(scopeVar)
        % Format: Array (pierwsza kolumna to czas, kolejne to sygnały)
        t_sim = scopeVar(:, 1);
        y_sim = scopeVar(:, 2:end);
    elseif isstruct(scopeVar) && isfield(scopeVar, 'signals')
        % Format: Structure with time
        t_sim = scopeVar.time;
        y_sim = [];
        for i = 1:length(scopeVar.signals)
            y_sim = [y_sim, scopeVar.signals(i).values];
        end
    elseif isa(scopeVar, 'Simulink.SimulationData.Dataset')
        % Format: Dataset
        t_sim = scopeVar.get(1).Values.Time;
        y_sim = [];
        for i = 1:scopeVar.numElements
            y_sim = [y_sim, scopeVar.get(i).Values.Data];
        end
    elseif isa(scopeVar, 'timeseries')
        % Format: Timeseries object directly
        t_sim = scopeVar.Time;
        y_sim = scopeVar.Data;
    elseif isstruct(scopeVar)
        % Format: Structure of timeseries
        fields = fieldnames(scopeVar);
        if ~isempty(fields) && isa(scopeVar.(fields{1}), 'timeseries')
            t_sim = scopeVar.(fields{1}).Time;
            y_sim = [];
            for i = 1:length(fields)
                y_sim = [y_sim, scopeVar.(fields{i}).Data];
            end
        else
            t_sim = [];
            y_sim = [];
        end
    else
        t_sim = [];
        y_sim = [];
    end
end

function [overshoot, ts_str] = calculateMetrics(t, y, t_step, y_start, y_final, amplitude, step_up)
    % Wyznacza przeregulowanie i czas regulacji dla tolerancji 5%
    idx_post = find(t >= t_step);
    if isempty(idx_post)
        idx_post = 1:length(t);
    end
    t_post = t(idx_post) - t_step;
    y_post = y(idx_post);
    
    % Filtracja wartości NaN (zabezpieczenie na wypadek problemów z czasem lub symulacją)
    valid_idx = ~isnan(y_post) & ~isnan(t_post);
    t_post_clean = t_post(valid_idx);
    y_post_clean = y_post(valid_idx);
    
    if isempty(y_post_clean)
        overshoot = NaN;
        ts_str = 'NaN';
        return;
    end
    
    if step_up
        max_val = max(y_post_clean);
        overshoot_abs = max_val - y_final;
        overshoot = (overshoot_abs / amplitude) * 100;
        if overshoot < 0, overshoot = 0; end
    else
        min_val = min(y_post_clean);
        overshoot_abs = y_final - min_val;
        overshoot = (overshoot_abs / abs(amplitude)) * 100;
        if overshoot < 0, overshoot = 0; end
    end
    
    % Czas regulacji (tolerancja 5% amplitudy wokół wartości końcowej)
    threshold = 0.05 * abs(amplitude);
    outside_indices = find(abs(y_post_clean - y_final) > threshold);
    
    if ~isempty(outside_indices)
        last_outside_idx = outside_indices(end);
        
        % Sygnał nie ustalił się, jeśli ostatnia znaleziona wartość poza pasmem
        % leży na samym końcu lub bardzo blisko końca symulacji,
        % bądź sama końcowa wartość leży poza zakresem tolerancji.
        % Sprawdzamy czy ostatni punkt poza tunelem jest w odległości mniejszej niż 2%
        % całkowitego czasu trwania od końca (ochrona przed szumami dyskretyzacji na końcu).
        t_max_post = t_post_clean(end);
        t_outside = t_post_clean(last_outside_idx);
        
        is_at_end = (last_outside_idx >= length(y_post_clean) - 1) || ...
                    (abs(y_post_clean(end) - y_final) > threshold) || ...
                    ((t_max_post - t_outside) < 0.02 * t_max_post);
                    
        if is_at_end
            ts_str = sprintf('> %.2f', t_max_post);
        else
            ts_str = sprintf('%.2f', t_outside);
        end
    else
        ts_str = '0.00';
    end
end
