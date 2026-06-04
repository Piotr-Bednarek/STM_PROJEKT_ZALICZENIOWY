% plot_ROZNICA1.m
filenameBase = ['ROZNICA51' ...
    ''];

% --- Load data (obsługuje .mat, .csv, .txt) ---
if isfile([filenameBase '.mat'])
    s = load([filenameBase '.mat']);
    if isfield(s,'data'), data = s.data;
    elseif isfield(s,'tbl'), data = s.tbl;
    else
        try
            data = struct2table(s);
        catch
            error('Nie można odczytać pliku .mat: sprawdź zawartość.');
        end
    end
elseif isfile([filenameBase '.csv'])
    data = readtable([filenameBase '.csv']);
elseif isfile([filenameBase '.txt'])
    data = readtable([filenameBase '.txt']);
else
    error('Plik ROZNICA1(.mat/.csv/.txt) nie istnieje w bieżącym katalogu.');
end

% Jeśli macierz liczb, zamień na tabelę z założoną kolejnością kolumn
if ~istable(data)
    if isnumeric(data) && size(data,2) >= 5
        data = array2table(data(:,1:5), 'VariableNames', {'time','setpoint','filtered','control','error'});
    else
        error('Nieobsługiwany format danych.');
    end
end

% --- Helper: pobierz kolumnę wg nazwy (case-insensitive) ---
function col = getColumn(tbl, name)
    idx = find(strcmpi(tbl.Properties.VariableNames, name), 1);
    if isempty(idx)
        error('Brakuje kolumny "%s" w pliku.', name);
    end
    col = tbl{:, idx};
end

% Pobierz wymagane kolumny
time     = getColumn(data, 'time');
setpoint = getColumn(data, 'setpoint');
filtered = getColumn(data, 'filtered');
control  = getColumn(data, 'control');
err      = getColumn(data, 'error');

% --- Rysowanie ---
fig = figure('Name','ROZNICA1','NumberTitle','off','Color',[1 1 1]);
tiledlayout(3,1, 'Padding','compact', 'TileSpacing','compact');

ax1 = nexttile;
plot(ax1, time, setpoint, '-b', 'LineWidth', 1.2); hold(ax1,'on');
plot(ax1, time, filtered, '-r', 'LineWidth', 1.2);
ylabel(ax1, 'Value'); legend(ax1, {'setpoint','filtered'}, 'Location','best');
grid(ax1,'on'); title(ax1, 'Setpoint i Filtered vs Time');

ax2 = nexttile;
plot(ax2, time, control, '-k', 'LineWidth', 1.2);
ylabel(ax2, 'Control'); grid(ax2,'on'); title(ax2, 'Control vs Time');

ax3 = nexttile;
plot(ax3, time, err, '-m', 'LineWidth', 1.2);
xlabel(ax3, 'Time'); ylabel(ax3, 'Error'); grid(ax3,'on'); title(ax3, 'Error vs Time');

linkaxes([ax1 ax2 ax3], 'x');
saveas(fig, [filenameBase '_plots.png']);
fprintf('Gotowe: wykresy zapisane w %s_plots.png\n', filenameBase);
