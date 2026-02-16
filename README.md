# Arthor Telemetry (STM32 Nucleo F334R8 + Python GUI)

Projekt prezentuje prosty tor telemetrii lotu:

1. `STM32 Nucleo-F334R8` odczytuje dane z czujnikow (BMP180 + MPU6050).
2. Firmware wysyla ramki telemetryczne przez UART (Virtual COM przez ST-Link).
3. Aplikacja `arthor.py` odczytuje COM i wizualizuje dane (sztuczny horyzont, yaw, wartosci baro/IMU).

## Co robi firmware

Plik: `Core/Src/main.c`

- Odczyt BMP180:
  - temperatura (`T`)
  - cisnienie (`P`)
  - wysokosc (`A`)
- Odczyt MPU6050:
  - gyro + accel
  - obliczenia `Roll`, `Pitch`, `Yaw` (filtr komplementarny)
- Wysylka danych przez UART2 (`9600`, `8N1`) w formacie tekstowym.

Format ramki (zgodny z `sprintf` w `main.c`):

```text
[BARO] T:%d.%dC P:%dPa A:%d.%dm | [ATT] R:%d.%d P:%d.%d Y:%d.%d
```

Przyklad:

```text
[BARO] T:24.3C P:100845Pa A:40.7m | [ATT] R:-1.2 P:3.4 Y:182.8
```

## Aplikacja PC (`arthor.py`)

`arthor.py` to glowny plik aplikacji desktopowej. Program:

- otwiera port COM (ST-Link Virtual COM Port),
- odczytuje linie telemetryczne z UART,
- parsuje wartosci `T, P, A, Roll, Pitch, Yaw`,
- wyswietla je na interfejsie (horyzont + kompas yaw + wartosci liczbowe).

## Wymagania

- STM32 Nucleo-F334R8
- Czujniki:
  - BMP180 (I2C)
  - MPU6050 (I2C)
- Python 3.10+ (zalecane)
- Biblioteki Python:
  - `pyserial`
  - `PyQt5`

Instalacja bibliotek:

```bash
pip install pyserial PyQt5
```

## Szybki start

1. Wgraj firmware na Nucleo.
2. Podlacz plytke przez USB (ST-Link VCP).
3. Uruchom aplikacje:

```bash
python arthor.py
```

4. Upewnij sie, ze port COM i baudrate w aplikacji zgadzaja sie z firmware (`9600`).

## Struktura projektu

- `Core/Src/main.c` - glowna logika firmware i wysylka telemetrii
- `Core/Inc/` - naglowki STM32
- `Drivers/` - HAL/CMSIS
- `CMakeLists.txt` - konfiguracja builda
- `arthor.py` - aplikacja desktopowa (glowny plik GUI)
