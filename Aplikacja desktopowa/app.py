import sys
from PySide6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox, QSplitter
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QIcon

from serial_manager import SerialManager
from widgets.metrics_panel import MetricsPanel
from widgets.control_panel import ControlPanel
from widgets.charts_panel import ChartsPanel
from widgets.terminal import Terminal
from utils.metrics import calculate_metrics

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("Panel Sterowania Ball on Beam")
        self.resize(1280, 800)
        
        # --- Managers ---
        self.serial = SerialManager()
        self.data_history = []
        
        # --- UI Setup ---
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        
        self.main_layout = QVBoxLayout(self.central_widget)
        self.main_layout.setContentsMargins(10, 10, 10, 10)
        self.main_layout.setSpacing(10)
        
        # 1. Top Row: Connection Tile + Metrics
        # We need to insert a connection tile before the metrics metrics
        # But MetricsPanel is a hardcoded grid. Let's wrap them in a HBox or modify MetricsPanel.
        # The user wants "przycisk z laczeniem niech bedzie jako kolejny kafelek ale na poczatku"
        
        self.top_row_layout = QHBoxLayout()
        self.top_row_layout.setSpacing(8)
        
        # 1.1 Connection Tile
        self.conn_tile = self._create_connection_tile()
        self.top_row_layout.addWidget(self.conn_tile)
        
        # 1.2 Metrics Panel (modified to be added to layout)
        self.metrics_panel = MetricsPanel()
        self.top_row_layout.addWidget(self.metrics_panel)
        # Assuming MetricsPanel expects to expand? It's a HBox of cards.
        
        self.main_layout.addLayout(self.top_row_layout)
        
        # 2. Main Content Splitter (Control | Charts)
        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(2)
        
        # Left Panel (Control + Terminal)
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 5, 0)
        left_layout.setSpacing(10)
        
        self.control_panel = ControlPanel()
        self.terminal = Terminal()
        
        left_layout.addWidget(self.control_panel)
        left_layout.addWidget(self.terminal, stretch=1)
        
        # Right Panel (Charts)
        self.charts_panel = ChartsPanel()
        
        splitter.addWidget(left_widget)
        splitter.addWidget(self.charts_panel)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 2) # Right side bigger
        
        self.main_layout.addWidget(splitter, stretch=1)
        
        # --- Wire Signals ---
        
        # Serial -> UI
        self.serial.ports_listed.connect(self._update_ports)
        self.serial.connected.connect(self._on_connected_changed)
        self.serial.rx_log.connect(self.terminal.append_rx)
        self.serial.tx_log.connect(self.terminal.append_tx)
        self.serial.new_data.connect(self._on_new_data)
        
        # Control UI -> Serial
        # self.control_panel.pid_update.connect(self.serial.send_pid) # Removed batch update
        self.control_panel.kp_update.connect(self.serial.send_pid_p)
        self.control_panel.ki_update.connect(self.serial.send_pid_i)
        self.control_panel.kd_update.connect(self.serial.send_pid_d)
        
        self.control_panel.setpoint_update.connect(self.serial.send_setpoint)
        self.control_panel.calibration_update.connect(self.serial.send_calibration)
        
        # Connection Tile Interactions
        self.btn_refresh.clicked.connect(self.serial.list_ports)
        self.btn_connect.clicked.connect(self._toggle_connection)
        
        # Timer for chart updates (throttled 10 FPS)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_ui_tick)
        self.update_timer.start(100)
        
        # Initial port list
        self.serial.list_ports()

    def _create_connection_tile(self):
        tile = QWidget()
        tile.setProperty("class", "card") 
        tile.setFixedWidth(320) # Wider to fit controls in one row
        
        layout = QVBoxLayout(tile)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(5)
        
        # Status Label (replaces Title and Badge)
        self.lbl_status = QLabel("STATUS: ROZŁĄCZONY")
        self.lbl_status.setStyleSheet("color: #ef4444; font-weight: bold; font-size: 11px; text-transform: uppercase;")
        layout.addWidget(self.lbl_status)
        
        # Controls Row (Combo + Refresh + Connect)
        row_controls = QHBoxLayout()
        row_controls.setSpacing(5)
        
        self.combo_ports = QComboBox()
        self.combo_ports.setStyleSheet("""
            QComboBox { 
                background-color: rgba(255,255,255,0.05); 
                border: 1px solid #444; 
                padding: 3px; 
                color: white;
            }
            QComboBox:disabled { color: #555; border-color: #222; }
        """)
        self.combo_ports.setSizePolicy(self.combo_ports.sizePolicy().horizontalPolicy(), self.combo_ports.sizePolicy().verticalPolicy())
        
        self.btn_refresh = QPushButton("↻")
        self.btn_refresh.setFixedSize(28, 28)
        self.btn_refresh.setStyleSheet("background-color: rgba(255,255,255,0.05); border: 1px solid #444; font-size: 14px;")
        
        self.btn_connect = QPushButton("Połącz")
        self.btn_connect.setObjectName("connectBtn")
        self.btn_connect.setFixedHeight(28) # Match refresh button height
        
        row_controls.addWidget(self.combo_ports, stretch=1)
        row_controls.addWidget(self.btn_refresh)
        row_controls.addWidget(self.btn_connect)
        
        layout.addLayout(row_controls)
        
        return tile

    def _update_ports(self, ports):
        current = self.combo_ports.currentText()
        self.combo_ports.clear()
        self.combo_ports.addItems(ports)
        
        # Auto-select "STM" port if not currently connected
        if not self.serial.thread: # only if not connected
            for i, p in enumerate(ports):
                if "STM" in p.upper() or "ST-LINK" in p.upper():
                    self.combo_ports.setCurrentIndex(i)
                    return

        if current in ports:
            self.combo_ports.setCurrentText(current)
            
    def _toggle_connection(self):
        if self.btn_connect.text() == "Połącz":
            port = self.combo_ports.currentText()
            if port:
                # Extract device name if format is "COMx - Desc"
                # Assuming serial_manager expects just COMx or handles full string?
                # serial_manager uses logic: `self.serial = serial.Serial(port_value, ...)`
                # list_ports returns just device name usually? 
                # Let's check serial_manager: `ports = [p.device for p in serial.tools.list_ports.comports()]`
                # So it returns "COM3", "COM4".
                # BUT wait, the user said "neuich lista comow niech ma tez tytylu urzadzen".
                # I need to modify serial_manager to return formatted strings or handle the split here.
                # Let's modify serial_manager list_ports first? Or just do it here if possible.
                # serial_manager just returns `p.device`.
                # I will quick-fix serial_manager to return "COMx - Desc" in next step or assume user is OK with COMx for now 
                # OR better: The user asked "niech w dropdownie automatycznie bedzie wybrany ten co ma w tytule stm".
                # This implies the dropdown DOES contain titles.
                # Currently serial_manager returns only `p.device`.
                # I should upgrade serial_manager to return full desc.
                
                # For now, pass what we have.
                self.serial.connect_serial(port.split(" - ")[0])
        else:
            self.serial.disconnect_serial()
            
    def _on_connected_changed(self, connected):
        if connected:
            self.btn_connect.setText("Rozłącz")
            self.btn_connect.setObjectName("disconnectBtn")
            self.lbl_status.setText("STATUS: POŁĄCZONO")
            self.lbl_status.setStyleSheet("color: #4ade80; font-weight: bold; font-size: 11px; text-transform: uppercase;")
            self.combo_ports.setEnabled(False)
            self.btn_refresh.setEnabled(False)
            self.btn_connect.setStyle(self.btn_connect.style()) 
        else:
            self.btn_connect.setText("Połącz")
            self.btn_connect.setObjectName("connectBtn")
            self.lbl_status.setText("STATUS: ROZŁĄCZONY")
            self.lbl_status.setStyleSheet("color: #ef4444; font-weight: bold; font-size: 11px; text-transform: uppercase;")
            self.combo_ports.setEnabled(True)
            self.btn_refresh.setEnabled(True)
            
    def _on_new_data(self, data):
        # Always append to history for charting
        # Note: serial_manager emits existing dict reference but updated values, 
        # so we should copy it for history to avoid mutations affecting old history
        snapshot = data.copy()
        snapshot['timestamp'] = 0 # Todo: add timestamp
        
        self.data_history.append(snapshot)
        if len(self.data_history) > 1000: # Keep more history in desktop
            self.data_history.pop(0)
            
        # Update Control Panel immediate values
        self.control_panel.update_data(data)
        
    def _update_ui_tick(self):
        # Called at 10Hz to refresh charts and metrics
        # This decouples high-freq serial data from UI rendering
        if not self.data_history:
            return
            
        latest = self.data_history[-1]
        
        # Metrics
        computed = calculate_metrics(self.data_history)
        self.metrics_panel.update_metrics(latest, computed)
        
        # Charts
        self.charts_panel.update_charts(self.data_history)
