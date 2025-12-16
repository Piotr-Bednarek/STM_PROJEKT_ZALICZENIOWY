# Aplikacja Webowa - STM32 Control

Nowoczesna aplikacja webowa napisana w React (Vite) do sterowania i wizualizacji danych z urządzenia STM32. Zastępuje poprzednią wersję desktopową (Python/Tkinter).

## Funkcje

-   **Połączenie Serial (Web Serial API)**: Bezpośrednie połączenie z portem COM z przeglądarki.
-   **Wizualizacja Real-time**: Wykresy odległości, błedu i sterowania.
-   **Panel Sterowania**: Zmiana parametrów PID i Setpoint na żywo.
-   **Terminal**: Podgląd logów TX/RX.
-   **Nowoczesny UI**: Ciemny motyw, responsywny layout.

## Uruchomienie

1. Zainstaluj zależności:
    ```bash
    npm install
    ```
2. Uruchom serwer developerski:
    ```bash
    npm run dev
    ```
3. Otwórz w przeglądarce (np. Chrome/Edge) adres wyświetlony w terminalu (zazwyczaj `http://localhost:5173`).

## Użycie

1. Kliknij **Connect Device** w prawym górnym rogu.
2. Wybierz port COM urządzenia STM32 z listy.
3. Dane zaczną pojawiać się na wykresach automatycznie.
4. Użyj suwaków by zmieniać `Setpoint` lub parametry `PID`.

**Uwaga**: Upewnij się, że inne aplikacje (np. stara aplikacja Python) nie blokują portu przed połączeniem.

-   [@vitejs/plugin-react-swc](https://github.com/vitejs/vite-plugin-react/blob/main/packages/plugin-react-swc) uses [SWC](https://swc.rs/) for Fast Refresh

## React Compiler

The React Compiler is not enabled on this template because of its impact on dev & build performances. To add it, see [this documentation](https://react.dev/learn/react-compiler/installation).

## Expanding the ESLint configuration

If you are developing a production application, we recommend using TypeScript with type-aware lint rules enabled. Check out the [TS template](https://github.com/vitejs/vite/tree/main/packages/create-vite/template-react-ts) for information on how to integrate TypeScript and [`typescript-eslint`](https://typescript-eslint.io) in your project.
