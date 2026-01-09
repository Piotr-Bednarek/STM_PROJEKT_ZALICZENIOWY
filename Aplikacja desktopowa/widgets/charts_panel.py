from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PySide6.QtCore import Qt

# Try importing pyqtgraph; if it fails (e.g. Python 3.14 numpy issues), fallback to dummy
try:
    import pyqtgraph as pg
    PG_AVAILABLE = True
except ImportError:
    PG_AVAILABLE = False
except Exception as e:
    print(f"Warning: PyQtGraph/Numpy disabled due to error: {e}")
    PG_AVAILABLE = False

class ChartsPanel(QWidget):
    def __init__(self):
        super().__init__()
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        
        # Header
        header = QLabel("WYKRESY W CZASIE RZECZYWISTYM")
        header.setStyleSheet("color: #94a3b8; font-weight: bold; font-size: 12px; margin-left: 5px;")
        layout.addWidget(header)
        
        if PG_AVAILABLE:
            self._init_charts(layout)
        else:
            self._init_fallback(layout)
        
    def _init_fallback(self, layout):
        msg = QLabel("Wykresy niedostępne (Błąd PyQtGraph/Numpy)\nSprawdź konsolę pod kątem błędów.")
        msg.setAlignment(Qt.AlignCenter)
        msg.setStyleSheet("color: #ef4444; font-size: 14px; border: 1px dashed #ef4444; border-radius: 8px;")
        layout.addWidget(msg, stretch=1)
        
    def _init_charts(self, layout):
        # QtGraph configuration
        pg.setConfigOption('background', '#1e293b')
        pg.setConfigOption('foreground', '#94a3b8')
        pg.setConfigOptions(antialias=True)
        
        # --- Chart 1: Distance & Setpoint (Main) ---
        self.plot_main = pg.PlotWidget()
        self.plot_main.showGrid(x=False, y=True, alpha=0.3)
        self.plot_main.setYRange(0, 300, padding=0)
        self.plot_main.setMouseEnabled(x=False, y=False)
        
        # Curves
        self.curve_setpoint = self.plot_main.plot(pen=pg.mkPen(color='#f39c12', width=2, style=Qt.DashLine))
        self.curve_dist = self.plot_main.plot(pen=pg.mkPen(color='#22c55e', width=2, style=Qt.DotLine))
        self.curve_filter = self.plot_main.plot(pen=pg.mkPen(color='#3b82f6', width=2))
        
        layout.addWidget(self.plot_main, stretch=2)
        
        # --- Chart 2: Error ---
        self.plot_error = pg.PlotWidget()
        self.plot_error.showGrid(x=False, y=True, alpha=0.3)
        self.plot_error.setMouseEnabled(x=False, y=True)
        self.plot_error.setTitle("Uchyb Regulacji", color="#94a3b8", size="10pt")
        
        self.curve_error = self.plot_error.plot(pen=pg.mkPen(color='#ef4444', width=2), fillLevel=0, brush=(239, 68, 68, 50))
        
        layout.addWidget(self.plot_error, stretch=1)
        
        # --- Chart 3: Control ---
        self.plot_ctrl = pg.PlotWidget()
        self.plot_ctrl.showGrid(x=False, y=True, alpha=0.3)
        self.plot_ctrl.setMouseEnabled(x=False, y=True)
        self.plot_ctrl.setTitle("Sygnał Sterujący", color="#94a3b8", size="10pt")
        
        self.curve_ctrl = self.plot_ctrl.plot(pen=pg.mkPen(color='#3498db', width=2))
        
        layout.addWidget(self.plot_ctrl, stretch=1)
        
        # Data Buffers (fixed length for performance)
        self.max_points = 200
        self.ptr = 0
        self.data_dist = [0] * self.max_points
        self.data_filt = [0] * self.max_points
        self.data_set = [0] * self.max_points
        self.data_err = [0] * self.max_points
        self.data_ctrl = [0] * self.max_points
        
    def update_charts(self, history):
        if not PG_AVAILABLE or not history:
            return
            
        # Get data chunks
        # Slice last N points
        view_data = history[-self.max_points:]
        
        dists = [d['distance'] for d in view_data]
        filts = [d['filtered'] for d in view_data]
        sets = [d['setpoint'] for d in view_data]
        errs = [d['error'] for d in view_data]
        ctrls = [d['control'] for d in view_data]
        
        # Update curves
        self.curve_dist.setData(dists)
        self.curve_filter.setData(filts)
        self.curve_setpoint.setData(sets)
        
        self.curve_error.setData(errs)
        self.curve_ctrl.setData(ctrls)
