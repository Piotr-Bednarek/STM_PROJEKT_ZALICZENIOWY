import sys
import time
import serial
import serial.tools.list_ports
from PySide6.QtCore import QObject, QThread, Signal, Slot, QMutex
from utils.crc8 import calculate_crc8

class SerialWorker(QObject):
    """
    Worker class that runs in a separate thread to handle continuous reading.
    """
    data_received = Signal(str) # Raw line received
    connection_lost = Signal(str)
    
    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port_name = port
        self.baudrate = baudrate
        self.serial_port = None
        self.is_running = True
        
    def run(self):
        try:
            self.serial_port = serial.Serial(self.port_name, self.baudrate, timeout=1)
            # DTR/RTS management: Some boards need True to power/enable, others need False to not reset.
            # Usually for STM32 Virtual COM Port, True is "Active".
            self.serial_port.dtr = True
            self.serial_port.rts = True
            
            # Flush existing garbage data to prevent initial CRC errors
            self.serial_port.reset_input_buffer()
            
            while self.is_running and self.serial_port.is_open:
                if self.serial_port.in_waiting > 0:
                    try:
                        line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            self.data_received.emit(line)
                    except Exception as e:
                        print(f"Read error: {e}")
                else:
                    QThread.msleep(5) # Prevent CPU hogging
                    
        except serial.SerialException as e:
            self.connection_lost.emit(str(e))
        except Exception as e:
            self.connection_lost.emit(str(e))
        finally:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()

    def stop(self):
        self.is_running = False

    def write(self, data: bytes):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(data)
            except Exception as e:
                print(f"Write error: {e}")

class SerialManager(QObject):
    """
    High-level manager for Serial communication. 
    Handles Thread management, Parsing, and Signal emission.
    """
    # Signals for UI
    connected = Signal(bool)
    rx_log = Signal(str, str) # msg, type (info/error/success)
    tx_log = Signal(str)
    new_data = Signal(dict) # Parsed data object
    ports_listed = Signal(list)
    
    def __init__(self):
        super().__init__()
        self.worker = None
        self.thread = None
        self.last_data = {
            "distance": 0,
            "filtered": 0,
            "error": 0,
            "control": 0,
            "setpoint": 150,
            "freq": 0
        }
        self.sample_count = 0
        self.last_time = time.time()
        
    def list_ports(self):
        ports = []
        for p in serial.tools.list_ports.comports():
            # Format: "COMx - Description"
            desc = p.description if p.description else "Generic Serial"
            ports.append(f"{p.device} - {desc}")
            
        port_list_sorted = sorted(ports)
        self.ports_listed.emit(port_list_sorted)
        return port_list_sorted

    def connect_serial(self, port_name):
        if self.thread and self.thread.isRunning():
            return
            
        # Robustness: Handle "COMx - Desc" input by taking only first part
        real_port = port_name.split(" - ")[0]
            
        self.thread = QThread()
        self.worker = SerialWorker(real_port)
        self.worker.moveToThread(self.thread)
        
        # Connect signals
        self.thread.started.connect(self.worker.run)
        self.worker.data_received.connect(self.handle_line)
        self.worker.connection_lost.connect(self.handle_error)
        
        self.thread.start()
        self.connected.emit(True)
        self.rx_log.emit(f"Connected to {port_name}", "success")

    def disconnect_serial(self):
        if self.worker:
            self.worker.stop()
        if self.thread:
            self.thread.quit()
            self.thread.wait()
        
        self.thread = None
        self.worker = None
        self.connected.emit(False)
        self.rx_log.emit("Disconnected", "info")

    def send_command(self, cmd):
        if self.worker and self.thread and self.thread.isRunning():
            crc = calculate_crc8(cmd)
            msg = f"{cmd};C:{crc:02X}\n"
            self.worker.write(msg.encode('utf-8'))
            self.tx_log.emit(msg.strip())
        else:
            self.rx_log.emit("Cannot send: Serial not connected", "error")

    def send_setpoint(self, val):
        self.send_command(f"S:{val:.1f}")
        self.last_data["setpoint"] = val # Optimistic update

    def send_pid(self, kp, ki, kd):
        # Sending with small delays to ensure STM32 processes them
        self.send_pid_p(kp)
        QThread.msleep(500)
        self.send_pid_i(ki)
        QThread.msleep(500)
        self.send_pid_d(kd)
        
    def send_pid_p(self, val): self.send_command(f"P:{-val:.4f}")
    def send_pid_i(self, val): self.send_command(f"I:{-val:.5f}")
    def send_pid_d(self, val): self.send_command(f"D:{-val:.1f}")
    
    def send_calibration(self, index, raw, target):
        self.send_command(f"CAL{index}:{raw:.1f},{target:.1f}")

    @Slot(str)
    def handle_line(self, line):
        # CRC Check
        valid = False
        payload = line
        crc_val_str = "00"
        
        if ";C:" in line:
            parts = line.split(";C:")
            if len(parts) == 2:
                data_part = parts[0]
                try:
                    crc_recv = int(parts[1], 16)
                    crc_calc = calculate_crc8(data_part)
                    if crc_calc == crc_recv:
                        valid = True
                        payload = data_part
                        crc_val_str = f"{crc_recv:02X}"
                        # DEBUG LOG: Show valid frame
                        self.rx_log.emit(f"RX [OK]: {line.strip()} (CRC:{crc_val_str})", "success")
                    else:
                        self.rx_log.emit(f"CRC Error: {line}", "error")
                except ValueError:
                    self.rx_log.emit(f"Fmt Error: {line}", "error")
        else:
             # DEBUG LOG: Show what we received even if no CRC
             self.rx_log.emit(f"RAW: {line}", "info")
             pass

        if valid:
            self.sample_count += 1
            # Update Frequency
            now = time.time()
            if now - self.last_time >= 1.0:
                self.last_data["freq"] = self.sample_count
                self.sample_count = 0
                self.last_time = now
            
            # Parse Segments
            segments = payload.split(";")
            current_data = self.last_data.copy()
            
            for seg in segments:
                if seg.startswith("D:"):
                    try:
                        val = float(seg[2:])
                        current_data["distance"] = max(0, min(val, 290))
                    except: pass
                elif seg.startswith("F:"):
                    try: val = float(seg[2:]); current_data["filtered"] = val
                    except: pass
                elif seg.startswith("E:"):
                    try: val = float(seg[2:]); current_data["error"] = val
                    except: pass
                elif seg.startswith("A:"):
                    try: val = float(seg[2:]); current_data["control"] = val
                    except: pass
            
            self.last_data = current_data
            self.new_data.emit(current_data)

    @Slot(str)
    def handle_error(self, msg):
        self.rx_log.emit(f"Error: {msg}", "error")
        self.disconnect_serial()
