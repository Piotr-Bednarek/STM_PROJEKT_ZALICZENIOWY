from PySide6.QtWidgets import QFrame, QVBoxLayout, QLabel, QHBoxLayout, QWidget, QGridLayout
from PySide6.QtCore import Qt

class MetricCard(QFrame):
    def __init__(self, label, unit, color="#e2e8f0"):
        super().__init__()
        self.setObjectName("card")
        self.setProperty("class", "metric-card")
        self.setStyleSheet(f"QFrame#card {{ border-left: 4px solid {color}; }}")
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(2)
        
        self.lbl_title = QLabel(label)
        self.lbl_title.setProperty("class", "metric-label")
        
        self.lbl_value = QLabel("0.0")
        self.lbl_value.setProperty("class", "metric-value")
        self.lbl_value.setStyleSheet(f"color: {color}; font-size: 20px;")
        
        self.lbl_unit = QLabel(unit)
        self.lbl_unit.setProperty("class", "metric-unit")
        self.lbl_unit.setAlignment(Qt.AlignRight)
        
        # Value row
        val_layout = QHBoxLayout()
        val_layout.addWidget(self.lbl_value)
        val_layout.addStretch()
        val_layout.addWidget(self.lbl_unit)
        val_layout.setAlignment(Qt.AlignBottom)
        
        layout.addWidget(self.lbl_title)
        layout.addLayout(val_layout)
        
    def update_value(self, value):
        self.lbl_value.setText(str(value))

class MetricsPanel(QWidget):
    def __init__(self):
        super().__init__()
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        
        # Define Metrics
        # Row 1 equivalent
        self.card_dist = MetricCard("Dystans (Raw)", "mm", "#22c55e")
        self.card_filt = MetricCard("Dystans (Filtrowany)", "mm", "#3b82f6")
        self.card_err = MetricCard("Uchyb (Błąd)", "mm", "#ef4444")
        self.card_avg_err = MetricCard("Śr. Uchyb (25)", "%", "#f59e0b")
        
        # Row 2 equivalent (merged into single row for desktop width)
        self.card_freq = MetricCard("Częstotliwość", "Hz", "#94a3b8")
        self.card_angle = MetricCard("Kąt Serwa", "°", "#3498db")
        self.card_std = MetricCard("Odchylenie Std.", "mm", "#9b59b6")
        self.card_setpoint = MetricCard("Cel (Setpoint)", "mm", "#f39c12")
        
        # Add to layout
        layout.addWidget(self.card_dist)
        layout.addWidget(self.card_filt)
        layout.addWidget(self.card_err)
        layout.addWidget(self.card_avg_err)
        layout.addWidget(self.card_freq)
        layout.addWidget(self.card_angle)
        layout.addWidget(self.card_std)
        layout.addWidget(self.card_setpoint)
        
    def update_metrics(self, data, computed):
        """
        Update all cards with new data.
        data: dict from serial_manager (distance, filtered, error, setpoint, crc, ...)
        computed: dict from utils.metrics (avgErrorPercent, stdDev)
        """
        self.card_dist.update_value(f"{data.get('distance', 0):.0f}")
        self.card_filt.update_value(f"{data.get('filtered', 0):.0f}")
        self.card_err.update_value(f"{data.get('error', 0):.1f}")
        self.card_setpoint.update_value(f"{data.get('setpoint', 0):.0f}")
        
        self.card_freq.update_value(f"{data.get('freq', 0)}")
        self.card_angle.update_value(f"{data.get('control', 0):.0f}")
        
        self.card_std.update_value(f"{computed.get('stdDev', 0)}")
