% Skrypt do obliczania wskaźników jakości regulacji (ITAE oraz Kosztu Sterowania)
% na podstawie danych z pliku CSV wyeksportowanego z aplikacji desktopowej.

clear all; close all; clc;

% 1. Wczytanie danych z pliku CSV
nazwa_pliku = 'odp-na-skok.csv';

if ~exist(nazwa_pliku, 'file')
    error('Brak pliku "%s" w bieżącym folderze. Upewnij się, że plik znajduje się w ścieżce MATLAB.', nazwa_pliku);
end

data = readtable(nazwa_pliku);

% 2. Wyciągnięcie kolumn
t = data.time;
setpoint = data.setpoint;
y = data.filtered;
u = data.control;

% 3. Obliczenie uchybu (błędu)
e = setpoint - y;

% 4. Określenie offsetu sterowania (neutralny kąt belki)
% Jeśli średnia wartość sterowania jest > 50, to dane są w starej skali (środek = 100).
% Jeśli oscylują wokół 0, to nowa wersja oprogramowania.
if mean(u) > 50
    u_neutral = 100;
else
    u_neutral = 0;
end
u_dev = u - u_neutral; % odchylenie od poziomu belki

% 5. Obliczenie dt (różnic czasu) pomiędzy próbkami
dt = [0; diff(t)]; 

% 6. Obliczenie wskaźników metodą prostokątów (całkowanie numeryczne)
ITAE = sum(t .* abs(e) .* dt);
Koszt_Sterowania = sum((u_dev.^2) .* dt);

% 7. Wyświetlenie wyników w oknie poleceń
fprintf('=======================================\n');
fprintf('  Wskaźniki jakości dla pliku: %s\n', nazwa_pliku);
fprintf('=======================================\n');
fprintf('ITAE (uchyb całkowity): %.2f [mm*s^2]\n', ITAE);
fprintf('Koszt sterowania:       %.2f [deg^2*s]\n', Koszt_Sterowania);
fprintf('Średni krok próbkowania: %.1f ms\n', mean(dt)*1000);
fprintf('Długość nagrania:       %.2f s\n', t(end) - t(1));
fprintf('=======================================\n');

% 8. Wykres pomocniczy
figure('Position', [100, 100, 1000, 600]);
subplot(2,1,1);
plot(t, y, 'b-', 'LineWidth', 2, 'DisplayName', 'Pozycja rzeczywista'); hold on;
plot(t, setpoint, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Wartość zadana');
ylabel('Pozycja [mm]');
title('Przebieg pozycji i uchybu w czasie');
legend('Location', 'best');
grid on;

subplot(2,1,2);
plot(t, u_dev, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Wychylenie sterujące');
xlabel('Czas [s]');
ylabel('Kąt (odchylenie od poziomu) [deg]');
title('Sygnał sterujący (odchylenie)');
grid on;
