from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QTextEdit, QHBoxLayout, QSplitter
from PySide6.QtGui import QColor, QTextCursor
from PySide6.QtCore import Qt

class Terminal(QWidget):
    def __init__(self):
        super().__init__()
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        # Header
        header_frame = QWidget()
        header_frame.setObjectName("termHeader")
        header_frame.setStyleSheet("""
            QWidget#termHeader {
                background-color: #1a1f2e;
                border-bottom: 1px solid #334155;
            }
            QLabel {
                padding: 5px;
            }
        """)
        h_layout = QHBoxLayout(header_frame)
        h_layout.setContentsMargins(5, 5, 5, 5)
        
        title = QLabel("LOGI KOMUNIKACJI")
        title.setStyleSheet("color: #94a3b8; font-weight: bold; font-size: 11px;")
        h_layout.addWidget(title)
        
        layout.addWidget(header_frame)
        
        # Splitter for TX/RX
        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(1)
        splitter.setStyleSheet("QSplitter::handle { background-color: #334155; }")
        
        # --- TX Column ---
        tx_widget = QWidget()
        tx_layout = QVBoxLayout(tx_widget)
        tx_layout.setContentsMargins(0,0,0,0)
        tx_layout.setSpacing(0)
        
        tx_header = QLabel("📤 Wysłane (TX)")
        tx_header.setStyleSheet("background-color: #1e293b; color: #94a3b8; font-size: 10px; padding: 4px; border-right: 1px solid #334155;")
        tx_layout.addWidget(tx_header)
        
        self.txt_tx = QTextEdit()
        self.txt_tx.setReadOnly(True)
        self.txt_tx.setStyleSheet("""
            QTextEdit {
                background-color: #0f1419;
                color: #f59e0b;
                border: none;
                font-family: monospace;
                font-size: 11px;
                border-right: 1px solid #334155;
            }
        """)
        tx_layout.addWidget(self.txt_tx)
        
        # --- RX Column ---
        rx_widget = QWidget()
        rx_layout = QVBoxLayout(rx_widget)
        rx_layout.setContentsMargins(0,0,0,0)
        rx_layout.setSpacing(0)
        
        rx_header = QLabel("📥 Odebrane (RX)")
        rx_header.setStyleSheet("background-color: #1e293b; color: #94a3b8; font-size: 10px; padding: 4px;")
        rx_layout.addWidget(rx_header)
        
        self.txt_rx = QTextEdit()
        self.txt_rx.setReadOnly(True)
        self.txt_rx.setStyleSheet("""
            QTextEdit {
                background-color: #0f1419;
                color: #e2e8f0;
                border: none;
                font-family: monospace;
                font-size: 11px;
            }
        """)
        rx_layout.addWidget(self.txt_rx)
        
        splitter.addWidget(tx_widget)
        splitter.addWidget(rx_widget)
        
        layout.addWidget(splitter)
        
    def append_tx(self, msg):
        self.txt_tx.append(f"[{self._time()}] {msg}")
        self._scroll_to_bottom(self.txt_tx)
        
    def append_rx(self, msg, msg_type="info"):
        color = "#e2e8f0" # white
        if msg_type == "error": color = "#ef4444" # red
        elif msg_type == "success": color = "#22c55e" # green
        
        html = f'<span style="color: #64748b;">[{self._time()}]</span> <span style="color: {color};">{msg}</span>'
        self.txt_rx.append(html)
        self._scroll_to_bottom(self.txt_rx)
        
    def _time(self):
        from datetime import datetime
        return datetime.now().strftime("%H:%M:%S")
        
    def _scroll_to_bottom(self, text_edit):
        c = text_edit.textCursor()
        c.movePosition(QTextCursor.End)
        text_edit.setTextCursor(c)
