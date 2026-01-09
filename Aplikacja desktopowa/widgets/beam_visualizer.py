from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QBrush, QPen, QColor, QLinearGradient, QRadialGradient, QFont
from PySide6.QtCore import Qt, Signal, QRectF, QPointF

class BeamVisualizer(QWidget):
    setpoint_changed = Signal(float)
    
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(120)
        self.distance = 0
        self.setpoint = 150
        self.min_val = 0
        self.max_val = 290
        
        self.is_dragging = False
        
    def set_data(self, distance, setpoint):
        self.distance = distance
        self.setpoint = setpoint
        self.update() # Trigger repaint
        
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_dragging = True
            self._update_from_mouse(event.position().x())
            
    def mouseMoveEvent(self, event):
        if self.is_dragging:
            self._update_from_mouse(event.position().x())
            
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_dragging = False
            
    def _update_from_mouse(self, x):
        width = self.width()
        # Map x to 0-290
        val = (x / width) * (self.max_val - self.min_val) + self.min_val
        val = max(self.min_val, min(val, self.max_val))
        self.setpoint_changed.emit(val)
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        w = self.width()
        h = self.height()
        
        # --- Beam ---
        beam_y = h * 0.7
        beam_h = 6
        
        # Gradient for beam
        grad = QLinearGradient(0, beam_y, 0, beam_y + beam_h)
        grad.setColorAt(0, QColor("#666666"))
        grad.setColorAt(1, QColor("#444444"))
        
        painter.setBrush(QBrush(grad))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(0, beam_y, w, beam_h, 2, 2)
        
        # Pivot Triangle
        pivot_path = [
            (w * 0.5, beam_y + beam_h),
            (w * 0.48, h * 0.9),
            (w * 0.52, h * 0.9)
        ]
        # (Simplified drawing for pivot not strictly needed but looks nice)
        painter.setBrush(QColor("#555555"))
        painter.drawPolygon([QPointF(*p) for p in pivot_path])
        
        # --- Helper: map mm to px ---
        def mm_to_px(mm):
            pct = (mm - self.min_val) / (self.max_val - self.min_val)
            return pct * w
            
        # --- Ruler Marks ---
        marks = [0, 50, 100, 145, 200, 250, 290]
        painter.setPen(QPen(QColor("#666666"), 1))
        painter.setFont(QFont("Monospace", 8))
        
        for m in marks:
            x_pos = mm_to_px(m)
            painter.drawLine(int(x_pos), int(beam_y + 10), int(x_pos), int(beam_y + 15))
            if m % 100 == 0 or m in [145, 290]:
                painter.drawText(QRectF(x_pos - 15, beam_y + 18, 30, 15), Qt.AlignCenter, str(m))
                
        # --- Ball ---
        ball_radius = 15
        ball_x = mm_to_px(self.distance)
        ball_y = beam_y - ball_radius
        
        # Ball Gradient
        ball_grad = QRadialGradient(ball_x - 5, ball_y - 5, ball_radius * 2)
        ball_grad.setColorAt(0, QColor("#88ff88"))
        ball_grad.setColorAt(1, QColor("#00aa00"))
        
        painter.setBrush(QBrush(ball_grad))
        painter.setPen(QPen(QColor("#005500"), 1))
        painter.drawEllipse(QPointF(ball_x, ball_y), ball_radius, ball_radius)
        
        # --- Setpoint Marker (Ghost + Line) ---
        sp_x = mm_to_px(self.setpoint)
        
        # Ghost ball (dashed)
        painter.setBrush(QColor(243, 156, 18, 50)) # Orange transparent
        pen = QPen(QColor("#f39c12"), 2)
        pen.setStyle(Qt.DashLine)
        painter.setPen(pen)
        painter.drawEllipse(QPointF(sp_x, ball_y), ball_radius, ball_radius)
        
        # Vertical Line
        pen.setWidth(1)
        painter.setPen(pen)
        painter.drawLine(int(sp_x), int(h * 0.1), int(sp_x), int(beam_y + 10))
        
        # --- Text Info Overlay at Bottom ---
        info_rect = QRectF(10, h - 30, w - 20, 25)
        painter.setBrush(QColor(255,255,255, 10))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(info_rect, 4, 4)
        
        # Text
        # Simplified manual text drawing for "Aktualnie: ... Cel: ..."
        # For brevity, we trust the side labels or just drawing it centered
        # Qt's HTML text drawing is tricky inside paintEvent without QTextDocument, 
        # so let's stick to simple text
        
        painter.setPen(QColor("#cccccc"))
        painter.setFont(QFont("Segoe UI", 9))
        text = f"Aktualnie: {self.distance:.0f} mm   |   Cel: {self.setpoint:.0f} mm"
        painter.drawText(info_rect, Qt.AlignCenter, text)

