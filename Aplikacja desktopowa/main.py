import sys
import os
import traceback
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QApplication
from app import MainWindow




def main():
    try:
        app = QApplication(sys.argv)
        app.setFont(QFont("Segoe UI", 10))
        
        style_path = os.path.join(os.path.dirname(__file__), "styles", "dark_theme.qss")
        if os.path.exists(style_path):
            with open(style_path, "r") as f:
                app.setStyleSheet(f.read())
        else:
            print("Warning: Stylesheet not found.")
            
        window = MainWindow()
        window.show()
        
        sys.exit(app.exec())
    except Exception as e:
        print(f"\nCRITICAL ERROR: {e}")
        traceback.print_exc()
        input("Press Enter to exit...")

if __name__ == "__main__":
    main()
