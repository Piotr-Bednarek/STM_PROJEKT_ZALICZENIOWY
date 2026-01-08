import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading
import time
import math


DEFAULT_KP = -0.11
DEFAULT_KI = -0.004
DEFAULT_KD = -2.0

# FAJNE WARTOSCI PID:
# DEFAULT_KP = -0.04
# DEFAULT_KI = -0.0027
# DEFAULT_KD = -3.0


class UARTApp:
    def __init__(self, root):
        self.root = root
        self.root.title("UART Komunikator & Sensor Monitor")
        self.root.state('zoomed')
        self.serial = None
        self.running = False
        self.buffer = ""
        
        # --- Konfiguracja ---
        # --- UKŁAD GŁÓWNY (SPLIT) ---
        main_split = tk.PanedWindow(root, orient=tk.HORIZONTAL, sashrelief=tk.RAISED)
        main_split.pack(fill="both", expand=True, padx=5, pady=5)
        
        # --- LEWY PANEL (Panel Sterowania, Logi, Konfiguracja) ---
        left_panel_outer = ttk.Frame(main_split, width=400) # Wrapper na canvas scrolla
        main_split.add(left_panel_outer, minsize=350)
        
        # Scrollbar dla lewego panelu (gdyby się nie mieściło)
        self.left_canvas = tk.Canvas(left_panel_outer)
        self.left_scrollbar = ttk.Scrollbar(left_panel_outer, orient="vertical", command=self.left_canvas.yview)
        self.left_scroll_frame = ttk.Frame(self.left_canvas)
        
        self.left_scroll_frame.bind(
            "<Configure>",
            lambda e: self.left_canvas.configure(
                scrollregion=self.left_canvas.bbox("all")
            )
        )
        self.left_canvas.create_window((0, 0), window=self.left_scroll_frame, anchor="nw")
        self.left_canvas.configure(yscrollcommand=self.left_scrollbar.set)
        
        self.left_canvas.pack(side="left", fill="both", expand=True)
        self.left_scrollbar.pack(side="right", fill="y")
        
        # 1. KONFIGURACJA (Przeniesiona tutaj)
        config_frame = ttk.LabelFrame(self.left_scroll_frame, text="Konfiguracja Połączenia", padding=10)
        config_frame.pack(fill="x", padx=10, pady=5)
        
        # Porty (dwa wiersze w wąskim panelu)
        ttk.Label(config_frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.port = tk.StringVar()
        self.port_combo = ttk.Combobox(config_frame, textvariable=self.port, width=25)
        self.port_combo.grid(row=0, column=1, sticky="w", padx=5)
        ttk.Button(config_frame, text="Odśwież", command=self.refresh_ports).grid(row=0, column=2, padx=5)

        # Baudrate
        ttk.Label(config_frame, text="Baud:").grid(row=1, column=0, sticky="w")
        self.baudrate = tk.StringVar(value="115200")
        self.baudrate_combo = ttk.Combobox(config_frame, textvariable=self.baudrate, 
                                        values=["9600", "115200", "19200", "38400", "57600"], width=10)
        self.baudrate_combo.grid(row=1, column=1, sticky="w", padx=5)
        
        # Connect btn
        self.connect_button = ttk.Button(config_frame, text="Połącz", command=self.toggle_connection)
        self.connect_button.grid(row=2, column=0, columnspan=3, pady=5, sticky="we")
        
        self.status_label = ttk.Label(config_frame, text="Status: Rozłączony", foreground="red")
        self.status_label.grid(row=3, column=0, columnspan=3, sticky="w")
        
        # Data bits etc (ukryte/domyślne żeby oszczędzić miejsce lub w submenu - tutaj uproszczone hardcoded w zmiennych, UI opcjonalne)
        self.databits = tk.StringVar(value="8")
        self.parity = tk.StringVar(value="None")
        self.stopbits = tk.StringVar(value="1")


        # 2. STATUS LICZBOWY
        display_frame = ttk.LabelFrame(self.left_scroll_frame, text="Aktualne Pomiary", padding=10)
        display_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(display_frame, text="RAW:", font=("Segoe UI", 9), foreground="#00aa00").grid(row=0, column=0, sticky="w")
        self.distance_value_var = tk.StringVar(value="---")
        self.distance_display = ttk.Label(display_frame, textvariable=self.distance_value_var, 
                  font=("Segoe UI", 16, "bold"), foreground="#00aa00")
        self.distance_display.grid(row=0, column=1, sticky="e")

        ttk.Label(display_frame, text="FILTERED:", font=("Segoe UI", 9, "bold"), foreground="#0000ff").grid(row=1, column=0, sticky="w")
        self.stm_filtered_var = tk.StringVar(value="---")
        ttk.Label(display_frame, textvariable=self.stm_filtered_var, 
                  font=("Segoe UI", 16, "bold"), foreground="#0000ff").grid(row=1, column=1, sticky="e")

        ttk.Label(display_frame, text="ERROR:", font=("Segoe UI", 9, "bold"), foreground="#ff0000").grid(row=2, column=0, sticky="w")
        self.error_value_var = tk.StringVar(value="---")
        ttk.Label(display_frame, textvariable=self.error_value_var,
                  font=("Segoe UI", 16, "bold"), foreground="#ff0000").grid(row=2, column=1, sticky="e")

        ttk.Label(display_frame, text="FREQ:", font=("Segoe UI", 9, "bold"), foreground="#666666").grid(row=3, column=0, sticky="w")
        self.hz_value_var = tk.StringVar(value="--- Hz")
        ttk.Label(display_frame, textvariable=self.hz_value_var,
                  font=("Segoe UI", 12, "bold"), foreground="#666666").grid(row=3, column=1, sticky="e")
        display_frame.columnconfigure(1, weight=1)


        # 3. CONTROL PANEL
        control_frame = ttk.LabelFrame(self.left_scroll_frame, text="Tuning PID", padding=10)
        control_frame.pack(fill="x", padx=10, pady=5)
        
        # Setpoint
        ttk.Label(control_frame, text="Setpoint (mm):").pack(anchor="w")
        self.setpoint_var = tk.DoubleVar(value=150.0)
        scale_frame = ttk.Frame(control_frame)
        scale_frame.pack(fill="x")
        self.setpoint_scale = tk.Scale(scale_frame, from_=0, to=290, orient="horizontal", variable=self.setpoint_var, command=self.on_setpoint_change)
        self.setpoint_scale.pack(side="left", fill="x", expand=True)
        self.setpoint_entry = ttk.Entry(scale_frame, textvariable=self.setpoint_var, width=5)
        self.setpoint_entry.pack(side="right")
        self.setpoint_entry.bind('<Return>', lambda e: self.on_setpoint_change(None))
        
        # Reliable Setpoint sending
        self.setpoint_scale.bind("<ButtonRelease-1>", lambda e: self.send_setpoint(force=True))
        
        # PID
        # Kp
        ttk.Label(control_frame, text="Kp:").pack(anchor="w", pady=(5,0))
        kp_frame = ttk.Frame(control_frame)
        kp_frame.pack(fill="x")
        self.kp_var = tk.DoubleVar(value=DEFAULT_KP)
        self.kp_scale = tk.Scale(kp_frame, from_=-5.0, to=0.0, resolution=0.01, orient="horizontal", variable=self.kp_var, command=self.on_pid_change)
        self.kp_scale.pack(side="left", fill="x", expand=True)
        ttk.Entry(kp_frame, textvariable=self.kp_var, width=6).pack(side="right")
        
        # Ki
        ttk.Label(control_frame, text="Ki:").pack(anchor="w")
        ki_frame = ttk.Frame(control_frame)
        ki_frame.pack(fill="x")
        self.ki_var = tk.DoubleVar(value=DEFAULT_KI)
        self.ki_scale = tk.Scale(ki_frame, from_=-0.01, to=0.0, resolution=0.0001, orient="horizontal", variable=self.ki_var, command=self.on_pid_change)
        self.ki_scale.pack(side="left", fill="x", expand=True)
        ttk.Entry(ki_frame, textvariable=self.ki_var, width=6).pack(side="right")

        # Kd
        ttk.Label(control_frame, text="Kd:").pack(anchor="w")
        kd_frame = ttk.Frame(control_frame)
        kd_frame.pack(fill="x")
        self.kd_var = tk.DoubleVar(value=DEFAULT_KD)
        self.kd_scale = tk.Scale(kd_frame, from_=-1000.0, to=0.0, resolution=1.0, orient="horizontal", variable=self.kd_var, command=self.on_pid_change)
        self.kd_scale.pack(side="left", fill="x", expand=True)
        ttk.Entry(kd_frame, textvariable=self.kd_var, width=6).pack(side="right")
        
        # BINDINGS FOR RELIABLE SENDING
        self.kp_scale.bind("<ButtonRelease-1>", lambda e: self.send_pid(force=True))
        self.ki_scale.bind("<ButtonRelease-1>", lambda e: self.send_pid(force=True))
        self.kd_scale.bind("<ButtonRelease-1>", lambda e: self.send_pid(force=True))
        
        ttk.Button(control_frame, text="Wyślij Parametry", command=lambda: self.send_all_params(force=True)).pack(fill="x", pady=10)


        # 4. PANEL KALIBRACJI
        cal_frame = ttk.LabelFrame(self.left_scroll_frame, text="Kalibracja 5-Punktowa", padding=10)
        cal_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(cal_frame, text="Połóż piłkę w pozycji, kliknij przycisk:", 
                  font=("Segoe UI", 9), foreground="#ff8800").pack(anchor="w", pady=(0,5))
        
        # Przyciski kalibracji
        cal_positions = [
            (0, "0 mm (Start)", "#ff4444"),
            (75, "75 mm (25%)", "#ff8800"),
            (150, "150 mm (Środek)", "#ffcc00"),
            (225, "225 mm (75%)", "#88ff00"),
            (290, "290 mm (Koniec)", "#00ff00")
        ]
        
        for idx, (pos, label, color) in enumerate(cal_positions):
            btn = tk.Button(cal_frame, text=f"📍 {label}", 
                           command=lambda p=pos, i=idx: self.calibrate_point(i, p),
                           bg=color, fg="black", font=("Segoe UI", 9, "bold"), 
                           relief="raised", bd=2)
            btn.pack(fill="x", pady=2)
        
        # Status kalibracji
        self.cal_status_var = tk.StringVar(value="Status: Gotowy do kalibracji")
        ttk.Label(cal_frame, textvariable=self.cal_status_var, 
                  font=("Segoe UI", 8), foreground="#666666").pack(anchor="w", pady=(5,0))
        
        # Przycisk zapisu kalibracji do STM32
        ttk.Button(cal_frame, text="💾 Zapisz Kalibrację do STM32", 
                   command=self.save_calibration).pack(fill="x", pady=(10,0))


        # 5. LOGI TERMINAL
        term_frame = ttk.LabelFrame(self.left_scroll_frame, text="Terminal", padding=5)
        term_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        self.receive_text = scrolledtext.ScrolledText(term_frame, height=15, width=30, state="disabled", font=("Consolas", 8))
        self.receive_text.pack(fill="both", expand=True)
        
        send_frame = ttk.Frame(term_frame)
        send_frame.pack(fill="x", pady=5)
        self.transmit_entry = ttk.Entry(send_frame)
        self.transmit_entry.pack(side="left", fill="x", expand=True)
        ttk.Button(send_frame, text=">", width=4, command=self.transmit_data).pack(side="right")
        ttk.Button(term_frame, text="Wyczyść", command=self.clear_receive).pack(anchor="w")


        # --- PRAWY PANEL (Wykresy) ---
        right_panel = ttk.Frame(main_split)
        main_split.add(right_panel, stretch="always")
        
        # Pasek narzędzi wykresu
        chart_tools = ttk.Frame(right_panel)
        chart_tools.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(chart_tools, text="Wykresy:").pack(side="left")
        self.chart_enabled = tk.BooleanVar(value=True)
        self.chart_button = ttk.Button(chart_tools, text="Stop", command=self.toggle_chart)
        self.chart_button.pack(side="left", padx=5)
        ttk.Button(chart_tools, text="Reset", command=self.clear_chart).pack(side="left", padx=5)
        
        # Kontener na wykresy (Split w pionie)
        self.charts_frame = tk.PanedWindow(right_panel, orient=tk.VERTICAL, sashrelief=tk.RAISED)
        self.charts_frame.pack(fill="both", expand=True)

        self.chart_bg_color = "#1e1e1e"
        
        # 1. Canvas Distance
        self.frame_dist = ttk.Frame(self.charts_frame)
        self.charts_frame.add(self.frame_dist, height=300) # Startowa wysokość
        self.canvas_dist = tk.Canvas(self.frame_dist, bg=self.chart_bg_color)
        self.canvas_dist.pack(fill="both", expand=True)
        self.canvas_dist.bind("<Configure>", lambda e: self.draw_chart_dist())

        # 2. Canvas Error
        self.frame_err = ttk.Frame(self.charts_frame)
        self.charts_frame.add(self.frame_err, height=150)
        self.canvas_err = tk.Canvas(self.frame_err, bg=self.chart_bg_color)
        self.canvas_err.pack(fill="both", expand=True)
        self.canvas_err.bind("<Configure>", lambda e: self.draw_chart_err())

        # 3. Canvas Control (Angle/Output)
        self.frame_ctrl = ttk.Frame(self.charts_frame)
        self.charts_frame.add(self.frame_ctrl, height=150)
        self.canvas_ctrl = tk.Canvas(self.frame_ctrl, bg=self.chart_bg_color)
        self.canvas_ctrl.pack(fill="both", expand=True)
        self.canvas_ctrl.bind("<Configure>", lambda e: self.draw_chart_ctrl())
        
        self.refresh_ports()

        # Zmienne danych dla Canvas
        self.data_history = [] # Raw
        self.data_history_stm = [] # STM32 Filtered (F:)
        self.data_history_setpoint = [] # Setpoint history
        self.data_history_error = [] # Error history (E:)
        self.data_history_ctrl = [] # Control/Angle history (A:)
        
        self.max_history = 200 
        self.sample_counter = 0
        
        # Licznik częstotliwości próbkowania
        self.sample_rate_counter = 0
        self.sample_rate_start_time = time.time()
        self.current_sample_rate = 0.0
        
        # Zmienne do obliczania metryk
        self.error_history_1s = []  # Historia błędów z ostatniej 1s
        self.last_metrics_update = time.time()
        self.ise_accumulator = 0.0
        self.settling_start_time = None
        self.max_overshoot = 0.0
        
        # Wątek odczytu
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()
    
    def on_canvas_resize(self, event):
        # Deprecated logic - now individual binds handle this
        pass

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
        # Debouncing dla sliderów (żeby nie zapchać UARTu)
        self.last_pid_send_time = 0
        self.last_setpoint_send_time = 0

    def calibrate_point(self, index, target_pos):
        """Zapisuje aktualny odczyt RAW jako punkt kalibracyjny"""
        try:
            raw_str = self.distance_value_var.get()
            self.log_message(f"[DEBUG] Próba kalibracji punktu {index}, RAW string: '{raw_str}'\n")
            
            # Sprawdź czy mamy wartość
            if raw_str == "---" or raw_str == "" or raw_str is None:
                self.cal_status_var.set(f"❌ Błąd: Brak odczytu RAW! Poczekaj na dane z czujnika.")
                self.log_message(f"[CAL ERROR] Brak danych RAW\n")
                return
            
            current_raw = float(raw_str)
            
            self.cal_points[index] = current_raw
            
            # Podświetl status
            completed = sum(1 for p in self.cal_points if p is not None)
            self.cal_status_var.set(f"✅ Punkt {index+1}/5: {target_pos}mm → RAW:{current_raw:.0f} | Zebrano: {completed}/5")
            self.log_message(f"[CAL] ✅ Punkt {index}: {target_pos}mm = RAW {current_raw:.0f}\n")
            
            if completed == 5:
                self.cal_status_var.set(f"🎉 Wszystkie 5 punktów zapisane! Kliknij 'Zapisz do STM32'")
        except ValueError as e:
            self.cal_status_var.set(f"❌ Błąd konwersji: {raw_str}")
            self.log_message(f"[CAL ERROR] ValueError: {e}, wartość: '{raw_str}'\n")
        except Exception as e:
            self.cal_status_var.set(f"❌ Błąd: {e}")
            self.log_message(f"[CAL ERROR] Nieoczekiwany błąd: {e}\n")

    def save_calibration(self):
        """Wysyła wszystkie punkty kalibracji do STM32"""
        if not self.serial or not self.running:
            self.cal_status_var.set("❌ Brak połączenia!")
            return
        
        # Sprawdź czy wszystkie punkty są zapisane
        if any(p is None for p in self.cal_points):
            missing = [i for i, p in enumerate(self.cal_points) if p is None]
            self.cal_status_var.set(f"❌ Brak punktów: {missing}")
            return
        
        try:
            # Wyślij komendę kalibracji dla każdego punktu
            for idx in range(5):
                raw_val = self.cal_points[idx]
                target_val = self.cal_targets[idx]
                cmd = f"CAL{idx}:{raw_val:.1f},{target_val:.1f}"
                crc = self.calculate_crc8(cmd)
                msg = f"{cmd};C:{crc:02X}\n"
                self.serial.write(msg.encode())
                self.log_message(f"[CAL TX] {msg.strip()}\n")
                time.sleep(0.1)  # Delay dla STM32
            
            self.cal_status_var.set("✅ Kalibracja wysłana do STM32!")
            self.log_message("[CAL] ✅ Kalibracja zapisana pomyślnie!\n")
        except Exception as e:
            self.cal_status_var.set(f"❌ Błąd: {e}")
            self.log_message(f"[CAL ERROR] {e}\n")

    def on_setpoint_change(self, event):
        if time.time() - self.last_setpoint_send_time > 0.1:
            self.send_setpoint()
            self.last_setpoint_send_time = time.time()
            
    def on_pid_change(self, event):
        if time.time() - self.last_pid_send_time > 0.2:
            self.send_pid()
            self.last_pid_send_time = time.time()

    def send_all_params(self, force=False):
        self.send_setpoint(force=force)
        time.sleep(0.05)
        self.send_pid(force=force)

    def send_setpoint(self, force=False):
        if not self.serial or not self.running: 
            pass # self.log_message("[DEBUG] Serial not ready\n")
            
        if not force and (time.time() - self.last_setpoint_send_time < 0.1):
            return

        try:
            val = float(self.setpoint_var.get())
            if force:
               # print(f"Sending SETPOINT: {val}")
               pass
            
            # Format CMD: "S:150.0"
            cmd = f"S:{val:.1f}"
            crc = self.calculate_crc8(cmd)
            msg = f"{cmd};C:{crc:02X}"
            self.serial.write((msg + '\n').encode())
            # self.log_message(f"[CMD] {msg}\n")
            self.last_setpoint_send_time = time.time()
        except: pass

    def send_pid(self, force=False):
        if not self.serial or not self.running: 
            return # self.log_message("[DEBUG] Serial not ready\n")
            
        # Debounce check skipping if force=True
        if not force and (time.time() - self.last_pid_send_time < 0.2):
            return

        try:
            # Format: Send P, I, D separately to match STM32 logic
            kp = self.kp_var.get()
            ki = self.ki_var.get()
            kd = self.kd_var.get()
            
            # Send P
            cmd_p = f"P:{kp:.4f}"
            self.serial.write((f"{cmd_p};C:{self.calculate_crc8(cmd_p):02X}\n").encode())
            time.sleep(0.2) # Delay > STM32 loop time (100ms budget)
            
            # Send I
            cmd_i = f"I:{ki:.5f}"
            self.serial.write((f"{cmd_i};C:{self.calculate_crc8(cmd_i):02X}\n").encode())
            time.sleep(0.2)
            
            # Send D
            cmd_d = f"D:{kd:.1f}"
            self.serial.write((f"{cmd_d};C:{self.calculate_crc8(cmd_d):02X}\n").encode())
            
            self.log_message(f"[PID update] P:{kp} I:{ki} D:{kd}\n")
            self.last_pid_send_time = time.time()
        except Exception as e:
            self.log_message(f"[Python Error] {e}\n")

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
        # Log active period
        current_time = time.time()
        elapsed = current_time - self.sample_rate_start_time
        
        if elapsed >= 1.0:  # Co sekundę
            self.current_sample_rate = self.sample_rate_counter / elapsed
            self.log_message(f"[SAMPLE RATE] {self.current_sample_rate:.2f} Hz\n")
            # Update GUI label
            self.hz_value_var.set(f"{self.current_sample_rate:.1f} Hz")
            self.sample_rate_counter = 0
            self.sample_rate_start_time = current_time
        
        # Logowanie surowych danych (opcjonalnie można wyłączyć dla wydajności przy wysokim Hz)
        # self.log_message(f"{line}\n") 
        # Zamiast logować wszystko, logujmy tylko błędy lub inne komunikaty, 
        # a dane "D:..." tylko jeśli chcemy. Na razie zostawię logowanie wszystkiego, 
        # ale warto pamiętać że to obciąża GUI.
        
        # Sprawdzamy czy to dane (nie logujemy ich wtedy do terminala żeby nie śmiecić, albo logujemy)
        # Decyzja: Logujemy wszystko jak było, ale licznik próbek tylko w update_chart
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
    
    def calculate_crc8(self, text):
        """Oblicza CRC8 (wielomian 0x07) dla zadanego stringa."""
        crc = 0x00
        for char in text:
            byte = ord(char)
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc

    def update_chart(self, value_str):
        # Format oczekiwany: "D:123;A:100;C:AB" lub "D:123;A:100;F:120;E:10.5;C:AB"
        val_y = None
        val_f = None # STM Filtered
        val_e = None # Error
        val_a = None # Angle/Control
        current_crc = None
        data_part = None
        
        try:
            # 1. Sprawdź czy mamy CRC w ramce
            if ";C:" in value_str:
                parts = value_str.split(";C:")
                if len(parts) == 2:
                    data_part = parts[0]
                    crc_str = parts[1].strip()
                    
                    # 2. Policz CRC lokalnie
                    calculated_crc = self.calculate_crc8(data_part)
                    received_crc = int(crc_str, 16)
                    
                    if calculated_crc != received_crc:
                        self.log_message(f"[CRC ERROR] Calc:{calculated_crc:02X} != Recv:{received_crc:02X}\n")
                        return # Odrzucamy ramkę
                    
                    # 3. Parsuj dane (D:xxx;A:yyy)
                    # Szukamy D:
                    segments = data_part.split(";")
                    for seg in segments:
                        if seg.startswith("D:"):
                            val_y = float(seg.replace("D:", ""))
                        elif seg.startswith("F:"):
                            val_f = float(seg.replace("F:", ""))
                        elif seg.startswith("E:"):
                            val_e = float(seg.replace("E:", ""))
                        elif seg.startswith("A:"):
                            val_a = float(seg.replace("A:", ""))
            
            else:
                # Fallback dla starego formatu (bez CRC)
                clean_str = value_str.lower().replace("mm", "").replace("distance:", "").strip()
                # Jeśli to CSV np D:123 A:456
                if "d:" in clean_str:
                     # Proste szukanie liczby po D:
                     import re
                     match = re.search(r"d:(\d+)", clean_str)
                     if match:
                         val_y = float(match.group(1))
                else:
                    val_y = float(clean_str)

        except Exception as e:
            # self.log_message(f"[Parse Error] {e}\n")
            pass
        
        if val_y is not None:
            # Zliczamy poprawną próbkę
            self.sample_rate_counter += 1
            
            # --- CLAMPING (Ograniczenie zakresu) ---
            val_y = max(self.clamp_min, min(val_y, self.clamp_max))

            # 2. Update Wyświetlacza
            self.distance_value_var.set(f"{val_y:.0f}")
            if val_f is not None:
                self.stm_filtered_var.set(f"{val_f:.0f}")
            if val_e is not None:
                self.error_value_var.set(f"{val_e:.1f}")
            
            # Kolory
            if val_y < 50:
                self.distance_display.config(foreground="#e74c3c") # Red (Blisko krańca)
            elif val_y > 230:
                self.distance_display.config(foreground="#e74c3c") # Red (Blisko drugiego krańca)
            else:
                self.distance_display.config(foreground="#2ecc71") # Green (Środek)

            # 2. Update Wykresu
            if self.chart_enabled.get():
                self.data_history.append(val_y)
                self.data_history_stm.append(val_f if val_f is not None else val_y) # STM Filtered
                self.data_history_setpoint.append(self.setpoint_var.get())
                
                # Jeśli dostaliśmy E z STM to użyjmy, jak nie to policzmy
                if val_e is not None:
                    self.data_history_error.append(val_e)
                else:
                    self.data_history_error.append(self.setpoint_var.get() - (val_f if val_f else val_y))
                
                # Control/Angle
                if val_a is not None:
                    self.data_history_ctrl.append(val_a)
                elif self.data_history_ctrl:
                    self.data_history_ctrl.append(self.data_history_ctrl[-1]) # Powtrzamy ostatni
                else:
                    self.data_history_ctrl.append(0)

                # Utrzymaj rozmiar bufora
                if len(self.data_history) > self.max_history:
                    self.data_history.pop(0)
                    self.data_history_stm.pop(0)
                    self.data_history_setpoint.pop(0)
                    self.data_history_error.pop(0)
                    if self.data_history_ctrl: self.data_history_ctrl.pop(0)
                
                self.draw_charts()

    def draw_charts(self):
        self.draw_chart_dist()
        self.draw_chart_err()
        self.draw_chart_ctrl()

    def draw_chart_dist(self):
        self.canvas_dist.delete("all")
        if not self.data_history: return
        
        w = self.canvas_dist.winfo_width()
        h = self.canvas_dist.winfo_height()
        
        # Marginesy
        margin = 20
        h_draw = h - 2 * margin
        
        min_y = 0 
        max_y = 300 
        # Dynamiczne skalowanie jeśli wyjdzie poza
        curr_max = max(max(self.data_history), max(self.data_history_setpoint))
        if curr_max > max_y: max_y = curr_max + 20
        
        range_y = max_y - min_y if max_y != min_y else 1
        step_x = w / (self.max_history - 1) if self.max_history > 1 else w
        
        def get_y(val):
            rel = (val - min_y) / range_y
            return (margin + h_draw) - (rel * h_draw)

        # Rysuj Setpoint (Yellow)
        pts_set = []
        for i, val in enumerate(self.data_history_setpoint):
            pts_set.extend([i * step_x, get_y(val)])
        if len(pts_set) >= 4: self.canvas_dist.create_line(pts_set, fill="#ffff00", width=2, dash=(4, 2))

        # Rysuj STM Filtered (Blue)
        pts_stm = []
        for i, val in enumerate(self.data_history_stm):
            pts_stm.extend([i * step_x, get_y(val)])
        if len(pts_stm) >= 4: self.canvas_dist.create_line(pts_stm, fill="#0000ff", width=2, smooth=True)
            
        # Rysuj Raw (Green)
        pts_raw = []
        for i, val in enumerate(self.data_history):
            pts_raw.extend([i * step_x, get_y(val)])
        if len(pts_raw) >= 4: self.canvas_dist.create_line(pts_raw, fill="#00ff00", width=1, dash=(2, 4))
        
        # Legenda
        self.canvas_dist.create_text(10, 5, text="Distance (mm)", fill="white", anchor="nw", font=("Arial", 10, "bold"))
        self.canvas_dist.create_text(w-50, 5, text=f"{self.data_history[-1]:.0f}", fill="#00ff00", anchor="ne", font=("Arial", 9))

    def draw_chart_err(self):
        self.canvas_err.delete("all")
        if not self.data_history_error: return
        w = self.canvas_err.winfo_width()
        h = self.canvas_err.winfo_height()
        margin = 15
        h_draw = h - 2 * margin
        
        max_err_abs = max([abs(e) for e in self.data_history_error])
        limit_err = max(30, max_err_abs + 5)
        
        step_x = w / (self.max_history - 1) if self.max_history > 1 else w
        
        def get_y(val):
            rel = (val + limit_err) / (2*limit_err)
            return (margin + h_draw) - (rel * h_draw)

        # Oś zero
        y_zero = get_y(0)
        self.canvas_err.create_line(0, y_zero, w, y_zero, fill="#555555", dash=(2, 2))

        pts_err = []
        for i, val in enumerate(self.data_history_error):
            pts_err.extend([i * step_x, get_y(val)])
        if len(pts_err) >= 4: self.canvas_err.create_line(pts_err, fill="#ff4444", width=2)
        
        self.canvas_err.create_text(10, 5, text="Error (mm)", fill="#ff4444", anchor="nw", font=("Arial", 10, "bold"))

    def draw_chart_ctrl(self):
        self.canvas_ctrl.delete("all")
        if not self.data_history_ctrl: return
        w = self.canvas_ctrl.winfo_width()
        h = self.canvas_ctrl.winfo_height()
        margin = 15
        h_draw = h - 2 * margin
        
        # Zakres dla serwa/kąta? Zakładam np 0-180 albo -90..90
        # Dynamiczny
        vals = self.data_history_ctrl
        min_v = min(vals)
        max_v = max(vals)
        if max_v - min_v < 10: 
            min_v -= 5
            max_v += 5
        
        step_x = w / (self.max_history - 1) if self.max_history > 1 else w
        
        def get_y(val):
            if max_v == min_v: return h/2
            rel = (val - min_v) / (max_v - min_v)
            return (margin + h_draw) - (rel * h_draw)

        pts = []
        for i, val in enumerate(vals):
            pts.extend([i * step_x, get_y(val)])
        if len(pts) >= 4: self.canvas_ctrl.create_line(pts, fill="#00ffff", width=2)
        
        self.canvas_ctrl.create_text(10, 5, text="Control Signal (Angle)", fill="#00ffff", anchor="nw", font=("Arial", 10, "bold"))
        self.canvas_ctrl.create_text(w-50, 5, text=f"{vals[-1]:.1f}", fill="#00ffff", anchor="ne", font=("Arial", 9))

    def clear_chart(self):
        self.data_history = []
        self.data_history_stm = []
        self.data_history_setpoint = []
        self.data_history_error = []
        self.data_history_ctrl = []
        self.draw_charts()
        self.distance_value_var.set("---")
        self.stm_filtered_var.set("---")
        self.error_value_var.set("---")

if __name__ == "__main__":
    root = tk.Tk()
    app = UARTApp(root)
    root.mainloop()
