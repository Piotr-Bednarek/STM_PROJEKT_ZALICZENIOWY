from PySide6.QtWidgets import (QWidget, QVBoxLayout, QLabel, QSlider, QHBoxLayout, 
                               QPushButton, QButtonGroup, QGridLayout, QFrame)
from PySide6.QtCore import Qt, Signal, QTimer
from widgets.beam_visualizer import BeamVisualizer

class SliderControl(QWidget):
    value_changed = Signal(float) # Emitted on drag
    value_committed = Signal(float) # Emitted on release
    
    def __init__(self, label, min_val, max_val, step, init_val, unit=""):
        super().__init__()
        self.step = step
        self.unit = unit
        self.factor = 1.0 / step if step < 1 else 1.0
        
        layout = QVBoxLayout(self)
        layout.setSpacing(4)
        
        # Header
        header = QHBoxLayout()
        self.lbl_name = QLabel(label)
        self.lbl_name.setStyleSheet("font-weight: bold; color: #94a3b8;")
        self.lbl_val = QLabel(f"{init_val} {unit}")
        self.lbl_val.setStyleSheet("color: #3b82f6; font-family: monospace; font-weight: bold;")
        
        header.addWidget(self.lbl_name)
        header.addStretch()
        header.addWidget(self.lbl_val)
        
        # Slider
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(int(min_val * self.factor))
        self.slider.setMaximum(int(max_val * self.factor))
        self.slider.setValue(int(init_val * self.factor))
        
        self.slider.valueChanged.connect(self._on_change)
        self.slider.sliderReleased.connect(self._on_release)
        
        layout.addLayout(header)
        layout.addWidget(self.slider)
        
        # Style container
        self.setObjectName("sliderContainer")
        self.setStyleSheet("""
            QWidget#sliderContainer {
                background-color: rgba(255, 255, 255, 0.03);
                border: 1px solid rgba(255, 255, 255, 0.08);
                border-radius: 6px;
            }
        """)

    def _on_change(self, val):
        real_val = val / self.factor
        if self.step < 1:
            fmt = "{:.4f}" if self.step < 0.001 else "{:.2f}"
        else:
            fmt = "{:.0f}"
        self.lbl_val.setText(f"{fmt.format(real_val)} {self.unit}")
        self.value_changed.emit(real_val)
        
    def _on_release(self):
        real_val = self.slider.value() / self.factor
        self.value_committed.emit(real_val)
        
    def set_value(self, val):
        self.slider.setValue(int(val * self.factor))

class ControlPanel(QWidget):
    pid_update = Signal(float, float, float) # Kept for batch if needed
    kp_update = Signal(float)
    ki_update = Signal(float)
    kd_update = Signal(float)
    setpoint_update = Signal(float)
    calibration_update = Signal(int, float, float) # index, raw, target
    
    def __init__(self):
        super().__init__()
        
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(10, 10, 10, 10)
        self.layout.setSpacing(15)
        
        # Use QFrame for background style
        self.setObjectName("panel")
        self.setProperty("class", "card")
        
        # Title
        title = QLabel("Panel Sterowania")
        title.setProperty("class", "panel-title")
        title.setStyleSheet("font-size: 16px; font-weight: bold; margin-bottom: 10px;")
        self.layout.addWidget(title)
        
        # --- Beam Visualizer ---
        self.viz = BeamVisualizer()
        self.viz.setpoint_changed.connect(self._on_viz_setpoint)
        self.layout.addWidget(self.viz)
        
        # --- Calibration Section ---
        cal_group = QFrame()
        cal_group.setStyleSheet("background-color: rgba(0,0,0,0.2); border-radius: 8px; padding: 10px;")
        cal_layout = QVBoxLayout(cal_group)
        
        cal_title = QLabel("KALIBRACJA 5-PUNKTOWA")
        cal_title.setStyleSheet("color: #f59e0b; font-weight: bold; font-size: 11px;")
        cal_layout.addWidget(cal_title)
        
        self.lbl_current_raw = QLabel("Aktualny RAW: ---")
        self.lbl_current_raw.setStyleSheet("color: #64748b; font-size: 10px; margin-bottom: 5px;")
        cal_layout.addWidget(self.lbl_current_raw)
        
        # 5 Buttons row
        btns_layout = QHBoxLayout()
        self.cal_points = [None] * 5
        self.cal_targets = [0, 75, 150, 225, 290]
        self.cal_btns = []
        labels = ["Start", "25%", "Środek", "75%", "Koniec"]
        colors = ["#ff4444", "#ff8800", "#ffcc00", "#88ff00", "#00ff00"]
        
        for i in range(5):
            btn = QPushButton(labels[i])
            btn.setCheckable(True)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: rgba(255,255,255,0.05);
                    border: 1px solid #444;
                    font-size: 11px;
                }}
                QPushButton:checked {{
                    background-color: {colors[i]};
                    color: black;
                    border: 1px solid {colors[i]};
                }}
            """)
            btn.clicked.connect(lambda checked, idx=i: self._on_cal_click(idx))
            self.cal_btns.append(btn)
            btns_layout.addWidget(btn)
            
        cal_layout.addLayout(btns_layout)
        
        # Save Button
        self.btn_save_cal = QPushButton("Zapisz kalibrację")
        self.btn_save_cal.setEnabled(False)
        self.btn_save_cal.clicked.connect(self._on_save_cal)
        self.btn_save_cal.setStyleSheet("background-color: #22c55e;") 
        cal_layout.addWidget(self.btn_save_cal)
        
        self.layout.addWidget(cal_group)
        
        # --- PID Controls ---
        pid_group = QFrame()
        pid_group.setObjectName("pidGroup")
        pid_group.setStyleSheet("#pidGroup { background-color: #1a1f2e; border: 1px solid #334155; border-radius: 8px; }")
        pid_layout = QVBoxLayout(pid_group)
        
        # PID Header + Mode Switch
        pid_header = QHBoxLayout()
        pid_label = QLabel("PARAMETRY REGULATORA (PID)")
        pid_label.setStyleSheet("color: #3b82f6; font-weight: bold; font-size: 11px;")
        
        # Mode buttons (Visual only functionality for now, logic is same)
        mode_layout = QHBoxLayout()
        self.btn_pid = QPushButton("PID")
        self.btn_pid.setCheckable(True)
        self.btn_pid.setChecked(True)
        self.btn_pid.setFixedSize(40, 20)
        self.btn_pid.setStyleSheet("font-size: 10px; padding:0;")
        
        self.btn_lqr = QPushButton("LQR")
        self.btn_lqr.setCheckable(True)
        self.btn_lqr.setFixedSize(40, 20)
        self.btn_lqr.setStyleSheet("font-size: 10px; padding:0;")
        
        bg = QButtonGroup(self)
        bg.addButton(self.btn_pid)
        bg.addButton(self.btn_lqr)
        
        pid_header.addWidget(pid_label)
        pid_header.addStretch()
        pid_header.addWidget(self.btn_pid)
        pid_header.addWidget(self.btn_lqr)
        
        pid_layout.addLayout(pid_header)
        
        # Sliders
        self.sli_kp = SliderControl("Kp (Proporcjonalny)", 0.0, 2.0, 0.01, 0.15)
        self.sli_ki = SliderControl("Ki (Całkujący)", 0.0, 0.01, 0.0001, 0.0025)
        self.sli_kd = SliderControl("Kd (Różniczkujący)", 0.0, 10.0, 0.1, 6.0)
        
        # Connect commit signals individually
        self.sli_kp.value_committed.connect(self._on_kp_change)
        self.sli_ki.value_committed.connect(self._on_ki_change)
        self.sli_kd.value_committed.connect(self._on_kd_change)
        
        pid_layout.addWidget(self.sli_kp)
        pid_layout.addWidget(self.sli_ki)
        pid_layout.addWidget(self.sli_kd)
        
        self.layout.addWidget(pid_group)
        self.layout.addStretch()
        
        # Internal state
        self.current_raw_distance = 0
        
    def _on_viz_setpoint(self, val):
        self.setpoint_update.emit(val)
        
    def _on_kp_change(self, val):
        self.kp_update.emit(val)

    def _on_ki_change(self, val):
        self.ki_update.emit(val)

    def _on_kd_change(self, val):
        self.kd_update.emit(val)
        
    def _on_cal_click(self, idx):
        # Toggle check state logic manually or trust QButtonGroup? No group here.
        # We need to capture current Raw distance
        raw = self.current_raw_distance
        self.cal_points[idx] = raw
        
        # Update button text
        self.cal_btns[idx].setText(f"{raw:.0f}")
        self.cal_btns[idx].setChecked(True)
        
        # Check if all filled
        if all(p is not None for p in self.cal_points):
            self.btn_save_cal.setEnabled(True)
            self.btn_save_cal.setText("Zapisz kalibrację (Gotowe)")
            
    def _on_save_cal(self):
        self.btn_save_cal.setEnabled(False)
        self.btn_save_cal.setText("Wysyłanie...")
        
        # Collect points to send
        self._pending_cal_points = []
        for i, raw in enumerate(self.cal_points):
            if raw is not None:
                self._pending_cal_points.append((i, raw, self.cal_targets[i]))
        
        self._send_next_cal(0)

    def _send_next_cal(self, idx):
        if idx < len(self._pending_cal_points):
            i, raw, target = self._pending_cal_points[idx]
            self.calibration_update.emit(i, raw, target)
            self.btn_save_cal.setText(f"Wysyłanie {idx+1}/{len(self._pending_cal_points)}...")
            # 500ms delay before next
            QTimer.singleShot(500, lambda: self._send_next_cal(idx + 1))
        else:
            self.btn_save_cal.setText("Wysłano!")
            self.btn_save_cal.setEnabled(True)
            QTimer.singleShot(2000, lambda: self.btn_save_cal.setText("Zapisz kalibrację (Gotowe)"))

    def update_data(self, data):
        self.viz.set_data(data.get("distance", 0), data.get("setpoint", 150))
        self.current_raw_distance = data.get("distance", 0) # Use raw distance for calibration?
        # Note: React app uses `rawDistance` prop for calibration, which comes from `latestData.distance`.
        # React app `distance` prop is filtered? No, `ControlPanel` gets `distance={latestData.filtered}` and `rawDistance={latestData.distance}`.
        # So viz uses filtered, calibration uses raw.
        # My `update_data` here receives the full dict, so I can split.
        
        self.viz.set_data(data.get("filtered", 0), data.get("setpoint", 150))
        self.current_raw_distance = data.get("distance", 0)
        self.lbl_current_raw.setText(f"Aktualny RAW: {self.current_raw_distance:.0f}")
