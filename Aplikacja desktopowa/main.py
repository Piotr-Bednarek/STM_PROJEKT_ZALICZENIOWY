import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading
import time
import math

class OneEuroFilter:
    def __init__(self, min_cutoff=1.0, beta=0.007, d_cutoff=1.0):
        """
        min_cutoff: Minimalna częstotliwość odcięcia (dla wolnych ruchów) - im mniej, tym większe wygładzanie
        beta:  Współczynnik szybkości reakcji (dla szybkich ruchów) - im więcej, tym mniejsze opóźnienie
        d_cutoff: Częstotliwość odcięcia dla pochodnej (szumu szybkości)
        """
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = None
        self.t_prev = None

    def smoothing_factor(self, t_e, cutoff):
        r = 2 * math.pi * cutoff * t_e
        return r / (r + 1)

    def exponential_smoothing(self, a, x, x_prev):
        return a * x + (1 - a) * x_prev

    def filter(self, t, x):
        if self.x_prev is None:
            self.x_prev = x
            self.dx_prev = 0.0
            self.t_prev = t
            return x

        t_e = t - self.t_prev
        
        # Filtruj pochodną (szybkość zmiany)
        a_d = self.smoothing_factor(t_e, self.d_cutoff)
        dx = (x - self.x_prev) / t_e if t_e > 0 else 0
        dx_hat = self.exponential_smoothing(a_d, dx, self.dx_prev)

        # Oblicz cutoff na podstawie szybkości
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        
        # Filtruj sygnał główny
        a = self.smoothing_factor(t_e, cutoff)
        x_hat = self.exponential_smoothing(a, x, self.x_prev)

        self.x_prev = x_hat
        self.dx_prev = dx_hat
        return self.x

class UARTApp:
    def __init__(self, root):
        self.root = root
        self.root.title("UART Komunikator & Sensor Monitor")
        self.root.state('zoomed')
        self.serial = None
        self.running = False
        self.buffer = ""
        
        # --- Konfiguracja ---
        config_frame = ttk.LabelFrame(root, text="Konfiguracja Połączenia", padding=10)
        config_frame.pack(fill="x", padx=10, pady=5)
        
        # Porty szeregowe
        ttk.Label(config_frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.port = tk.StringVar()
        self.port_combo = ttk.Combobox(config_frame, textvariable=self.port, width=50)
        self.port_combo.grid(row=0, column=1, sticky="w", padx=5)
        
        ttk.Button(config_frame, text="Odśwież", command=self.refresh_ports).grid(row=0, column=2, padx=5)
        
        # Baudrate
        ttk.Label(config_frame, text="Baudrate:").grid(row=1, column=0, sticky="w", pady=5)
        self.baudrate = tk.StringVar(value="9600")
        self.baudrate_combo = ttk.Combobox(config_frame, textvariable=self.baudrate, 
                                        values=["9600", "115200", "19200", "38400", "57600"], width=15)
        self.baudrate_combo.grid(row=1, column=1, sticky="w", padx=5)
        
        # Pozostałe ustawienia UART (Data bits, Parity, Stop bits)
        ttk.Label(config_frame, text="Data bits:").grid(row=0, column=3, sticky="w", padx=5)
        self.databits = tk.StringVar(value="8")
        self.databits_combo = ttk.Combobox(config_frame, textvariable=self.databits, values=["5", "6", "7", "8"], width=5)
        self.databits_combo.grid(row=0, column=4, sticky="w")

        ttk.Label(config_frame, text="Parity:").grid(row=0, column=5, sticky="w", padx=5)
        self.parity = tk.StringVar(value="None")
        self.parity_combo = ttk.Combobox(config_frame, textvariable=self.parity, values=["None", "Even", "Odd", "Mark", "Space"], width=8)
        self.parity_combo.grid(row=0, column=6, sticky="w")

        ttk.Label(config_frame, text="Stop bits:").grid(row=0, column=7, sticky="w", padx=5)
        self.stopbits = tk.StringVar(value="1")
        self.stopbits_combo = ttk.Combobox(config_frame, textvariable=self.stopbits, values=["1", "1.5", "2"], width=5)
        self.stopbits_combo.grid(row=0, column=8, sticky="w")
        
        # Przycisk Połącz
        self.connect_button = ttk.Button(config_frame, text="Połącz", command=self.toggle_connection)
        self.connect_button.grid(row=1, column=8, padx=5, sticky="e")
        
        # Status
        self.status_label = ttk.Label(config_frame, text="Status: Rozłączony", foreground="red")
        self.status_label.grid(row=2, column=0, columnspan=9, sticky="w", pady=5)
        
        self.refresh_ports()

        self.refresh_ports()

        # --- Frame Wyświetlania Odległości (Nowy) ---
        display_frame = ttk.LabelFrame(root, text="Porównanie Filtrów", padding=10)
        display_frame.pack(fill="x", padx=10, pady=5)
        
        # Grid
        display_frame.columnconfigure(0, weight=1)
        display_frame.columnconfigure(1, weight=1)
        display_frame.columnconfigure(2, weight=1)

        # 1. RAW
        ttk.Label(display_frame, text="RAW (Surowe)", font=("Segoe UI", 9), foreground="#00aa00").grid(row=0, column=0)
        self.distance_value_var = tk.StringVar(value="---")
        self.distance_display = ttk.Label(display_frame, textvariable=self.distance_value_var, 
                  font=("Segoe UI", 24, "bold"), foreground="#00aa00")
        self.distance_display.grid(row=1, column=0)
        
        # 2. 1-EURO
        ttk.Label(display_frame, text="1-EURO (Smart)", font=("Segoe UI", 9, "bold"), foreground="#00aaaa").grid(row=0, column=1)
        self.filtered_value_var = tk.StringVar(value="---")
        self.filtered_display = ttk.Label(display_frame, textvariable=self.filtered_value_var, 
                  font=("Segoe UI", 32, "bold"), foreground="#00aaaa")
        self.filtered_display.grid(row=1, column=1)

        # 3. CASCADE
        ttk.Label(display_frame, text="CASCADE (1-Euro + EMA)", font=("Segoe UI", 9, "bold"), foreground="#aa00aa").grid(row=0, column=2)
        self.cascade_value_var = tk.StringVar(value="---")
        ttk.Label(display_frame, textvariable=self.cascade_value_var, 
                  font=("Segoe UI", 32, "bold"), foreground="#aa00aa").grid(row=1, column=2)


        # --- Wysyłanie (Transmit) ---
        transmit_frame = ttk.LabelFrame(root, text="Wysyłanie Komend", padding=10)
        transmit_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(transmit_frame, text="Komenda:").grid(row=0, column=0, sticky="w")
        self.transmit_entry = ttk.Entry(transmit_frame, width=80)
        self.transmit_entry.grid(row=0, column=1, padx=5)
        
        ttk.Button(transmit_frame, text="Wyślij", command=self.transmit_data).grid(row=0, column=2, padx=5)

        # --- Główny Frame (Receive + Wykres) ---
        main_frame = ttk.Frame(root)
        main_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # --- Terminal (Lewa strona) ---
        receive_frame = ttk.LabelFrame(main_frame, text="Terminal (Odebrane dane)", padding=10)
        receive_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))
        
        self.receive_text = scrolledtext.ScrolledText(receive_frame, height=10, width=30, state="disabled", font=("Consolas", 9))
        self.receive_text.pack(fill="both", expand=True)
        
        ttk.Button(receive_frame, text="Wyczyść", command=self.clear_receive).pack(side="left", pady=5)
        
        # --- Wykres (Prawa strona) ---
        chart_frame = ttk.LabelFrame(main_frame, text="Wykres Przebiegu", padding=10)
        chart_frame.pack(side="right", fill="both", expand=True, padx=(5, 0))
        
        # Ustawienia wykresu
        axis_frame = ttk.Frame(chart_frame)
        axis_frame.pack(fill="x", pady=(0, 10))
        
        ttk.Label(axis_frame, text="Tytuł:").pack(side="left", padx=5)
        self.title_label = tk.StringVar(value="Odległość w czasie")
        ttk.Entry(axis_frame, textvariable=self.title_label, width=20).pack(side="left", padx=5)
        
        self.chart_enabled = tk.BooleanVar(value=True)
        self.chart_button = ttk.Button(axis_frame, text="Stop Wykres", command=self.toggle_chart)
        self.chart_button.pack(side="left", padx=5)
        
        ttk.Button(axis_frame, text="Resetuj Wykres", command=self.clear_chart).pack(side="left", padx=5)

        # --- Kanwa Wykresu (Bez Matplotlib) ---
        # Używamy zwykłego Canvas, bo Matplotlib/Numpy sypie błędami na Twojej wersji Pythona
        self.canvas_width = 600
        self.canvas_height = 400
        self.chart_bg_color = "#1e1e1e"
        self.chart_line_color = "#00ff00"
        
        self.canvas = tk.Canvas(chart_frame, bg=self.chart_bg_color, height=self.canvas_height)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", self.on_canvas_resize)

        # Zmienne danych dla Canvas
        self.data_history = [] # Raw
        self.data_history_filtered = [] # 1-Euro
        self.data_history_cascade = [] # Cascade (1-Euro + EMA)
        self.max_history = 200 
        self.sample_counter = 0
        
        # Filtry Init
        self.start_time = time.time()
        self.one_euro = OneEuroFilter(min_cutoff=0.1, beta=0.1, d_cutoff=1.0)
        
        # Post-Processing EMA (na wyjściu 1-Euro)
        self.post_ema_val = None
        self.post_ema_alpha = 0.6 # 0.6 = Dość szybki, ale wygładza końcówkę
        
        # Clamp (Ograniczenie fizyczne belki)
        self.clamp_min = 0
        self.clamp_max = 280 # mm (Długość belki)
        
        # Wątek odczytu
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()
    
    def on_canvas_resize(self, event):
        self.canvas_width = event.width
        self.canvas_height = event.height
        self.draw_chart()

    def refresh_ports(self):
        ports = []
        stm_port = None
        for port in serial.tools.list_ports.comports():
            device = port.device
            description = port.description
            if "STMicroelectronics" in description or "STM32" in description:
                stm_port = f"{device} - {description}"
            if device:
                ports.append(f"{device} - {description}")
        
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)
        if stm_port:
            self.port_combo.set(stm_port)
    
    def toggle_connection(self):
        if self.serial:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        try:
            port_full = self.port.get()
            if not port_full: return
            
            port_value = port_full.split(" - ")[0]
            baudrate_value = int(self.baudrate.get())
            databits_value = int(self.databits.get())
            
            stopbits_map = {"1": serial.STOPBITS_ONE, "1.5": serial.STOPBITS_ONE_POINT_FIVE, "2": serial.STOPBITS_TWO}
            stopbits = stopbits_map[self.stopbits.get()]
            
            parity_map = {"None": serial.PARITY_NONE, "Even": serial.PARITY_EVEN, "Odd": serial.PARITY_ODD, "Mark": serial.PARITY_MARK, "Space": serial.PARITY_SPACE}
            parity = parity_map[self.parity.get()]
            
            self.serial = serial.Serial(port_value, baudrate_value, timeout=0.1, 
                                     bytesize=databits_value, parity=parity, stopbits=stopbits)
            
            self.status_label.config(text=f"Status: Połączony ({port_full})", foreground="green")
            self.connect_button.config(text="Rozłącz")
            self.port_combo.config(state="disabled")
            self.running = True
            self.log_message(f"[SYSTEM] Połączono z {port_full}\n")
            
        except Exception as e:
            messagebox.showerror(f"Błąd połączenia", str(e))
    
    def disconnect(self):
        if self.serial:
            self.serial.close()
            self.serial = None
        self.status_label.config(text="Status: Rozłączony", foreground="red")
        self.connect_button.config(text="Połącz")
        self.port_combo.config(state="normal")
        self.running = False
        self.log_message("[SYSTEM] Rozłączono\n")
    
    def transmit_data(self):
        if not self.serial: return
        data = self.transmit_entry.get()
        if data:
            try:
                self.serial.write((data + '\n').encode())
                self.log_message(f"[TX] {data}\n")
                self.transmit_entry.delete(0, tk.END)
            except Exception as e:
                self.log_message(f"[Error] {e}\n")
    
    def read_loop(self):
        while True:
            try:
                if self.serial and self.serial.is_open and self.running:
                    if self.serial.in_waiting > 0:
                        raw = self.serial.read(self.serial.in_waiting)
                        text = raw.decode(errors='ignore')
                        self.buffer += text
                        
                        while '\n' in self.buffer:
                            line, self.buffer = self.buffer.split('\n', 1)
                            line = line.strip()
                            if line:
                                self.root.after(0, self.process_line, line)
            except Exception:
                pass
            time.sleep(0.01)

    def process_line(self, line):
        self.log_message(f"{line}\n")
        self.update_chart(line)

    def log_message(self, message):
        self.receive_text.config(state="normal")
        self.receive_text.insert(tk.END, message)
        self.receive_text.see(tk.END)
        self.receive_text.config(state="disabled")
    
    def clear_receive(self):
        self.receive_text.config(state="normal")
        self.receive_text.delete(1.0, tk.END)
        self.receive_text.config(state="disabled")
    
    def toggle_chart(self):
        self.chart_enabled.set(not self.chart_enabled.get())
        label = "Stop Wykres" if self.chart_enabled.get() else "Start Wykres"
        self.chart_button.config(text=label)
    
    def update_chart(self, value_str):
        # Logika parsowania
        val_y = None
        
        # Próba sparsowania
        try:
            clean_str = value_str.lower().replace("mm", "").replace("distance:", "").strip()
            val_y = float(clean_str)
        except ValueError:
            # Sprawdź format csv
            if ',' in value_str:
                try:
                    parts = value_str.split(',')
                    val_y = float(parts[1]) # Zakładamy że druga wartość to Y
                except (ValueError, IndexError):
                    pass
        
        if val_y is not None:
            # --- CLAMPING (Ograniczenie zakresu) ---
            val_y = max(self.clamp_min, min(val_y, self.clamp_max))

            # 1. Update Wyświetlacza
            self.distance_value_var.set(f"{val_y:.0f}")
            
            # Kolory
            if val_y < 50:
                self.distance_display.config(foreground="#e74c3c") # Red (Blisko krańca)
            elif val_y > 230:
                self.distance_display.config(foreground="#e74c3c") # Red (Blisko drugiego krańca)
            else:
                self.distance_display.config(foreground="#2ecc71") # Green (Środek)

            # --- 1-EURO FILTER ---
            current_time = time.time() - self.start_time
            filtered_val = self.one_euro.filter(current_time, val_y)
            self.filtered_value_var.set(f"{filtered_val:.0f}")

            # --- CASCADE (POST-EMA) ---
            if self.post_ema_val is None:
                self.post_ema_val = filtered_val
            
            # EMA na wyniku z 1-Euro
            self.post_ema_val = (self.post_ema_alpha * filtered_val) + ((1 - self.post_ema_alpha) * self.post_ema_val)
            self.cascade_value_var.set(f"{self.post_ema_val:.0f}")

            # 2. Update Wykresu
            if self.chart_enabled.get():
                self.data_history.append(val_y)
                self.data_history_filtered.append(filtered_val)
                self.data_history_cascade.append(self.post_ema_val)
                
                # Utrzymaj rozmiar bufora
                if len(self.data_history) > self.max_history:
                    self.data_history.pop(0)
                    self.data_history_filtered.pop(0)
                    self.data_history_cascade.pop(0)
                
                self.draw_chart()

    def draw_chart(self):
        """Rysuje wykres ręcznie na płótnie Canvas"""
        self.canvas.delete("all")
        
        if not self.data_history:
            return

        w = self.canvas_width
        h = self.canvas_height
        
        # Znadź min/max żeby przeskalować
        min_y = min(self.data_history)
        max_y = max(self.data_history)
        
        if min_y == max_y:
            min_y -= 10
            max_y += 10
        
        # Marginesy
        margin_top = 20
        margin_bottom = 20
        plot_h = h - margin_top - margin_bottom
        range_y = max_y - min_y
        
        # Rysuj linię
        points = []
        n_points = len(self.data_history)
        
        # Odstęp między punktami
        step_x = w / (self.max_history - 1) if self.max_history > 1 else w
        
        for i, val in enumerate(self.data_history):
            # Skalowanie Y: odwracamy, bo w Canvas Y=0 jest na górze
            # Wartość (val - min_y) / range_y to procent wysokości (0..1)
            # Chcemy żeby max_y było na górze (małe Y w Canvas)
            
            rel_y = (val - min_y) / range_y
            y_pos = h - margin_bottom - (rel_y * plot_h)
            
            x_pos = i * step_x
            points.append(x_pos)
            points.append(y_pos)
        
        if len(points) >= 4:
            # Rysuj linię RAW (zielona, cienka, nieco przezroczysta - symulowana kropkowaniem lub kolorem)
            self.canvas.create_line(points, fill="#00ff00", width=1, dash=(2, 4))
            
        # --- Rysuj linię 1-Euro (Cyan, Gruba) ---
        if self.data_history_filtered:
            points_filt = []
            for i, val in enumerate(self.data_history_filtered):
                rel_y = (val - min_y) / range_y
                y_pos = h - margin_bottom - (rel_y * plot_h)
                x_pos = i * step_x
                points_filt.append(x_pos)
                points_filt.append(y_pos)
            
            if len(points_filt) >= 4:
                self.canvas.create_line(points_filt, fill="#00ffff", width=3, smooth=True)

        # --- Rysuj linię CASCADE (Magenta, Gruba) ---
        if self.data_history_cascade:
            points_cas = []
            for i, val in enumerate(self.data_history_cascade):
                rel_y = (val - min_y) / range_y
                y_pos = h - margin_bottom - (rel_y * plot_h)
                x_pos = i * step_x
                points_cas.append(x_pos)
                points_cas.append(y_pos)
            
            if len(points_cas) >= 4:
                self.canvas.create_line(points_cas, fill="#ffa500", width=3, smooth=True)

        # Legenda
        self.canvas.create_text(w-10, 10, text="Raw (STM32)", fill="#00ff00", anchor="ne", font=("Arial", 8))
        self.canvas.create_text(w-10, 25, text="1-Euro Filter", fill="#00ffff", anchor="ne", font=("Arial", 8))
        self.canvas.create_text(w-10, 40, text="Cascade (1E+EMA)", fill="#ff00ff", anchor="ne", font=("Arial", 9, "bold"))
            
        # Rysuj siatkę / tekst
        self.canvas.create_text(10, 10, text=f"Max: {max_y:.0f}", fill="white", anchor="nw")
        self.canvas.create_text(10, h-10, text=f"Min: {min_y:.0f}", fill="white", anchor="sw")

    def clear_chart(self):
        self.data_history = []
        self.data_history_filtered = []
        self.data_history_cascade = []
        self.post_ema_val = None
        self.start_time = time.time()
        self.draw_chart()
        self.distance_value_var.set("---")
        self.filtered_value_var.set("---")
        self.cascade_value_var.set("---")

if __name__ == "__main__":
    root = tk.Tk()
    app = UARTApp(root)
    root.mainloop()
