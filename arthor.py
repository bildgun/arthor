import sys
import serial
import re
import math
from serial.tools import list_ports
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame
from PyQt5.QtGui import QPainter, QColor, QPen, QFont, QPolygon, QPainterPath
from PyQt5.QtCore import Qt, QTimer, QPoint


def find_stlink_port():
    for port in list_ports.comports():
        description = (port.description or "").lower()
        manufacturer = (port.manufacturer or "").lower()
        if "stlink virtual com port" in description or "stmicroelectronics" in manufacturer:
            return port.device
    return None

class ArtificialHorizon(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = 0.0
        self.pitch = 0.0
        self.setMinimumSize(520, 520)

    def setData(self, roll, pitch):
        self.roll, self.pitch = roll, pitch
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        h = self.height()
        center = self.rect().center()
        cx, cy = center.x(), center.y()

        size = min(w, h) - 14
        radius = size // 2
        left = cx - radius
        top = cy - radius

        # Zewnętrzna ramka przyrządu
        painter.setPen(QPen(QColor(45, 45, 45), 2))
        painter.setBrush(QColor(20, 20, 20))
        painter.drawEllipse(left - 6, top - 6, size + 12, size + 12)

        # Wewnętrzne pole robocze (okrągłe)
        clip_path = QPainterPath()
        clip_path.addEllipse(left, top, size, size)
        painter.save()
        painter.setClipPath(clip_path)

        # Ruchoma kula horyzontu
        painter.translate(center)
        painter.rotate(-self.roll)

        pitch_scale = 4.0  # px na 1 stopien
        horizon_y = self.pitch * pitch_scale
        far = radius * 3

        # Niebo i ziemia
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(60, 150, 230))
        painter.drawRect(-far, int(-far + horizon_y), far * 2, far)
        painter.setBrush(QColor(156, 96, 52))
        painter.drawRect(-far, int(horizon_y), far * 2, far)

        # Linia horyzontu
        painter.setPen(QPen(Qt.white, 2))
        painter.drawLine(-far, int(horizon_y), far, int(horizon_y))

        # Pitch ladder (co 5 stopni, opisy co 10)
        painter.setFont(QFont("Consolas", 8, QFont.Bold))
        for deg in range(-45, 50, 5):
            y = int(horizon_y - deg * pitch_scale)
            if abs(y) > radius + 18 or deg == 0:
                continue

            major = (deg % 10 == 0)
            half_len = 36 if major else 20
            painter.setPen(QPen(Qt.white, 2 if major else 1))
            painter.drawLine(-half_len, y, half_len, y)

            if major:
                label = str(abs(deg))
                painter.drawText(-half_len - 24, y + 4, label)
                painter.drawText(half_len + 8, y + 4, label)

        painter.restore()

        # Okrag ograniczajacy i znacznik kursu/krengu
        painter.setPen(QPen(QColor(230, 230, 230), 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(left, top, size, size)

        # Znaczniki przechylu (bank scale)
        tick_pen = QPen(QColor(240, 240, 240), 2)
        painter.setPen(tick_pen)
        for ang in (-60, -45, -30, -20, -10, 10, 20, 30, 45, 60):
            rad = math.radians(ang)
            outer_r = radius - 2
            inner_r = radius - (16 if abs(ang) in (30, 60) else 11)
            x1 = int(cx + math.sin(rad) * inner_r)
            y1 = int(cy - math.cos(rad) * inner_r)
            x2 = int(cx + math.sin(rad) * outer_r)
            y2 = int(cy - math.cos(rad) * outer_r)
            painter.drawLine(x1, y1, x2, y2)

        # Gorny znacznik 0 stopni
        painter.setBrush(QColor(240, 240, 240))
        marker = QPolygon([
            QPoint(cx, top + 6),
            QPoint(cx - 7, top + 18),
            QPoint(cx + 7, top + 18),
        ])
        painter.drawPolygon(marker)

        # Symbol samolotu (nieruchomy)
        wing_pen = QPen(QColor(255, 210, 90), 4)
        wing_pen.setCapStyle(Qt.RoundCap)
        painter.setPen(wing_pen)
        gap = 18
        wing = 72
        painter.drawLine(cx - wing, cy, cx - gap, cy)
        painter.drawLine(cx + gap, cy, cx + wing, cy)
        painter.drawLine(cx, cy - 10, cx, cy + 8)

        painter.setBrush(QColor(255, 80, 80))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(QPoint(cx, cy), 4, 4)


class YawCompass(QWidget):
    def __init__(self):
        super().__init__()
        self.yaw = 0.0
        self.setMinimumHeight(165)
        self.setMaximumHeight(165)

    def setYaw(self, yaw):
        self.yaw = yaw % 360.0
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(12, 12, 16))

        cx = self.width() // 2
        cy = self.height() // 2 + 8
        radius = min(self.width(), self.height()) // 2 - 20

        # Obramowanie i tlo kompasa
        painter.setPen(QPen(QColor(70, 70, 70), 2))
        painter.setBrush(QColor(25, 25, 30))
        painter.drawEllipse(cx - radius, cy - radius, radius * 2, radius * 2)

        # Ruchoma tarcza (obraca sie zgodnie z yaw)
        painter.save()
        painter.translate(cx, cy)
        painter.rotate(-self.yaw)

        for deg in range(0, 360, 10):
            rad = math.radians(deg)
            outer = radius - 4
            inner = radius - (15 if deg % 30 == 0 else 9)
            x1 = int(math.sin(rad) * inner)
            y1 = int(-math.cos(rad) * inner)
            x2 = int(math.sin(rad) * outer)
            y2 = int(-math.cos(rad) * outer)
            painter.setPen(QPen(QColor(235, 235, 235), 2 if deg % 30 == 0 else 1))
            painter.drawLine(x1, y1, x2, y2)

        painter.setPen(QPen(QColor(245, 245, 245), 2))
        painter.setFont(QFont("Consolas", 10, QFont.Bold))
        cardinal = [(0, "N"), (90, "E"), (180, "S"), (270, "W")]
        for deg, label in cardinal:
            rad = math.radians(deg)
            text_r = radius - 27
            tx = int(math.sin(rad) * text_r)
            ty = int(-math.cos(rad) * text_r)
            painter.drawText(tx - 8, ty + 5, 16, 12, Qt.AlignCenter, label)

        painter.restore()

        # Nieruchoma strzalka kierunku
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(255, 90, 90))
        pointer = QPolygon([
            QPoint(cx, cy - radius + 6),
            QPoint(cx - 8, cy - radius + 20),
            QPoint(cx + 8, cy - radius + 20),
        ])
        painter.drawPolygon(pointer)

        # Wartosc yaw
        painter.setPen(QPen(QColor(245, 245, 245), 1))
        painter.setFont(QFont("Consolas", 11, QFont.Bold))
        painter.drawText(0, self.height() - 24, self.width(), 18, Qt.AlignCenter, f"Yaw: {self.yaw:06.2f} deg")


class TelemetryWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GUI Telemetry - STLink Virtual COM Port")
        self.resize(700, 920)
        
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # 1. Sztuczny Horyzont (u gory, wiekszy)
        self.horizon = ArtificialHorizon()
        main_layout.addWidget(self.horizon, 1)

        # 2. Kompas Yaw (pod horyzontem)
        self.yaw_compass = YawCompass()
        main_layout.addWidget(self.yaw_compass)

        # Separator przed sekcja cyfrowa
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        main_layout.addWidget(line)

        # 3. Sekcja Danych Barometrycznych (T, P, H) - na dole
        # Używamy HBoxLayout, żeby były obok siebie
        baro_layout = QHBoxLayout()
        
        font_val = QFont("Arial", 14, QFont.Bold)
        font_lbl = QFont("Arial", 10)

        # Temperatura
        self.lbl_temp = QLabel("0.0 °C")
        self.lbl_temp.setFont(font_val)
        self.lbl_temp.setStyleSheet("color: red;")
        self.lbl_temp.setAlignment(Qt.AlignCenter)
        
        # Ciśnienie
        self.lbl_press = QLabel("0 hPa")
        self.lbl_press.setFont(font_val)
        self.lbl_press.setStyleSheet("color: green;")
        self.lbl_press.setAlignment(Qt.AlignCenter)

        # Wysokość
        self.lbl_alt = QLabel("0.0 m")
        self.lbl_alt.setFont(font_val)
        self.lbl_alt.setStyleSheet("color: blue;")
        self.lbl_alt.setAlignment(Qt.AlignCenter)

        # Dodawanie do układu z opisami
        for lbl, title in [(self.lbl_temp, "TEMP"), (self.lbl_press, "PRESS"), (self.lbl_alt, "ALT")]:
            v_box = QVBoxLayout()
            t_lbl = QLabel(title)
            t_lbl.setFont(font_lbl)
            t_lbl.setAlignment(Qt.AlignCenter)
            v_box.addWidget(t_lbl)
            v_box.addWidget(lbl)
            baro_layout.addLayout(v_box)

        main_layout.addLayout(baro_layout)

        # 4. Sekcja IMU (Kąty w formie tekstowej) - na dole
        self.label_imu = QLabel("Roll: 0 | Pitch: 0 | Yaw: 0")
        self.label_imu.setAlignment(Qt.AlignCenter)
        self.label_imu.setFont(QFont("Consolas", 11))
        main_layout.addWidget(self.label_imu)
        
        # Konfiguracja Serial
        try:
            port = find_stlink_port()
            if port is None:
                raise RuntimeError("Nie znaleziono STLink Virtual COM Port")
            self.ser = serial.Serial(port, 9600, timeout=0.1)
            self.label_imu.setText(f"Połączono z {port}")
        except Exception as e:
            self.ser = None
            self.label_imu.setText(f"Błąd portu: {e}")

        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial)
        self.timer.start(20)

    def read_serial(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                # Szukamy wszystkich liczb zmiennoprzecinkowych
                data = re.findall(r"[-+]?\d*\.\d+|\d+", line)
                
                # Oczekujemy min. 6 wartości: [T, P, H, Roll, Pitch, Yaw]
                if len(data) >= 6:
                    # Parsowanie danych
                    temp = float(data[0])
                    press = float(data[1])
                    alt = float(data[2])
                    roll = float(data[3])
                    pitch = -float(data[4])
                    yaw = float(data[5])

                    # Aktualizacja GUI
                    self.horizon.setData(roll, pitch)
                    
                    self.lbl_temp.setText(f"{temp:.1f} °C")
                    self.lbl_press.setText(f"{press:.0f} Pa")
                    self.lbl_alt.setText(f"{alt:.1f} m")
                    
                    self.label_imu.setText(f"R: {roll:.1f}°  |  P: {pitch:.1f}°  |  Y: {yaw:.1f}°")
                    self.yaw_compass.setYaw(yaw)
            except Exception as e:
                print(f"Błąd parsowania: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TelemetryWindow()
    window.show()
    sys.exit(app.exec_())