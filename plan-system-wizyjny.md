# Plan: Rozdział o systemie wizyjnym w pracy dyplomowej

## Context

Praca dyplomowa (`Formatka_praca_dyplomowa.tex`) opisuje projekt "Ball on Beam" z regulatorem PID i LQR. System wizyjny jest w pracy wspomniany tylko raz — w rozdziale 5.3 jako kierunek dalszych badań ("wizualna detekcja kulki"). **Jednak system jest w pełni zaimplementowany** w `1-desktop-app/widgets/opencv_panel.py` i aktywnie wysyła dane do firmware przez UART. Trzeba ten stan rzeczy odzwierciedlić w pracy.

## Gdzie wstawić

**Nowa sekcja 2.3** w rozdziale 2 (Budowa stanowiska i oprogramowanie), po obecnych sekcjach 2.2 (Mikrokontroler i peryferia).

Uzasadnienie:
- Rozdział 2 już opisuje wszystkie elementy sprzętowe (STM32, serwomechanek, VL53L0X, potencjometr)
- Kamera to kolejny sensor — logicznie pasuje tutaj
- Czytelnik musi znać wszystkie sensory przed przejściem do modelu matematycznego w rozdziale 3
- Sekcję 5.3 ("Kierunki dalszych badań") należy jednocześnie zaktualizować — usunąć wzmiankę o wizji jako "planie na przyszłość"

## Struktura nowej sekcji

```
2.3 Podsystem wizji komputerowej
  2.3.1 Konfiguracja kamery i akwizycja obrazu
  2.3.2 Detekcja kulki metodą progowania w przestrzeni HSV
  2.3.3 Wyznaczanie pozycji belki na podstawie znaczników ArUco
  2.3.4 Kalibracja geometryczna i przeliczanie piksel → mm
  2.3.5 Protokół komunikacyjny PC → STM32
```

## Co opisać w każdej podsekcji

### 2.3.1 Konfiguracja kamery
- Kamera USB, rozdzielczość 640×480, 30 FPS
- Wybór indeksu kamery z poziomu GUI
- `cv2.VideoCapture(camera_index)`

### 2.3.2 Detekcja kulki (HSV)
- Konwersja BGR→HSV, rozmycie Gaussa (kernel 21×21)
- Progowanie zakresu barwy (H: 8–64, S: 27–141, V: 121–255)
- Operacje morfologiczne (MORPH_CLOSE, MORPH_OPEN)
- Wykrywanie konturów (`cv2.findContours`) + `cv2.minEnclosingCircle`
- Dynamiczne ROI — śledzenie w oknie ±100 px wokół ostatniej pozycji

### 2.3.3 Znaczniki ArUco
- 4 znaczniki DICT_4X4_50 (ID 0–3) na ramie stanowiska
- ID 2 — lewy koniec belki, ID 0 — prawy koniec
- CLAHE + wyostrzanie dla małych/zacienionych znaczników
- AprilTag corner refinement, rozluźnione progi detekcji
- "Position holding" — trzymanie ostatniej pozycji przez ≤15 klatek przy chwilowej utracie widoczności

### 2.3.4 Kalibracja i przelicznik
- Rzut środka kulki na oś belki (iloczyn skalarny): t ∈ [0,0; 1,0]
- Kalibracja dwupunktowa: calib_min = 0,039, calib_max = 0,9896
- Konwersja: `position_mm = calibrated_t × 250 mm`
- Dwie metody pomiaru kąta belki: względem poziomu (domyślna) i względem linii referencyjnej ID1–ID3

### 2.3.5 Protokół UART
- Format ramki: `V:123.5;B:2.34;C:AB\n`
  - V = pozycja kulki [mm], B = kąt belki [°], C = CRC8 (hex)
- Częstotliwość: 30 Hz, baudrate 115200
- Po stronie firmware: `ParseVisionFrame()` w `main.c`, zmienne globalne `g_vision_ball_pos`, `g_vision_beam_angle`
- Fallback: jeśli brak ramki przez >200 ms → powrót do czujnika VL53L0X

## Pliki do edycji

| Plik | Zmiana |
|------|--------|
| `3-docs/1-projekt-przejsciowy/Formatka_praca_dyplomowa.tex` | Dodać sekcję 2.3 z podsekcjami 2.3.1–2.3.5 |
| Tamże, sekcja 5.3 | Usunąć / zaktualizować punkt o wizji jako "przyszłym kierunku" |

## Pliki źródłowe do opisania

- `1-desktop-app/widgets/opencv_panel.py` — cały pipeline wizyjny
- `1-desktop-app/opencv_params.json` — parametry konfiguracyjne
- `2-firmware/Core/Src/main.c` — `ParseVisionFrame()`, zmienne `g_vision_*`
- `2-firmware/Core/Inc/main.h` — deklaracje

## Weryfikacja po napisaniu

1. Skompilować PDF: `pdflatex Formatka_praca_dyplomowa.tex` — brak błędów
2. Sprawdzić numerację sekcji — stare 2.x nie "przesunęły się"
3. Upewnić się, że sekcja 5.3 nie mówi już o wizji jako nierealizowanej
4. Przejrzeć PDF pod kątem spójności terminologii
