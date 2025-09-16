import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import struct
import pysoem

# --- Constants based on CiA 402 Profile & Provided Manual ---

# Object Dictionary Addresses
OD_CONTROL_WORD = 0x6040
OD_STATUS_WORD = 0x6041
OD_MODES_OF_OPERATION = 0x6060
OD_MODES_OF_OPERATION_DISPLAY = 0x6061
OD_POSITION_ACTUAL_VALUE = 0x6064
OD_VELOCITY_ACTUAL_VALUE = 0x606C
OD_TARGET_TORQUE = 0x6071
OD_TARGET_POSITION = 0x607A
OD_TARGET_VELOCITY = 0x60FF

# Modes of Operation (from manual)
MODE_PROFILED_POSITION = 1
MODE_PROFILED_VELOCITY = 3
# Note: Cyclic modes require a more advanced real-time loop, 
# so we focus on the simpler profiled modes for this GUI application.

# Control Word Commands (Bits)
CW_SHUTDOWN = 0x06
CW_SWITCH_ON = 0x07
CW_ENABLE_OPERATION = 0x0F
CW_DISABLE_VOLTAGE = 0x00
CW_QUICK_STOP = 0x02
CW_DISABLE_OPERATION = 0x07
CW_FAULT_RESET = 0x80
CW_HALT = 0x100
# For Profiled Position mode, bit 4 (new_set_point) starts the movement
CW_START_MOVE_ABS = 0x1F
CW_START_MOVE_REL = 0x5F

# Status Word Masks (Bits) for human-readable status
SW_READY_TO_SWITCH_ON = 0x0001
SW_SWITCHED_ON = 0x0002
SW_OPERATION_ENABLED = 0x0004
SW_FAULT = 0x0008
SW_VOLTAGE_ENABLED = 0x0010
SW_QUICK_STOP = 0x0020
SW_SWITCH_ON_DISABLED = 0x0040
SW_WARNING = 0x0080
SW_TARGET_REACHED = 0x0400
SW_INTERNAL_LIMIT_ACTIVE = 0x0800


class EtherCATController:
    """
    Handles low-level EtherCAT communication and CiA 402 state machine.
    """
    def __init__(self, ifname):
        self._ifname = ifname
        self._master = pysoem.Master()
        self._master.open(self._ifname) # This will raise ConnectionError on failure
        
        self.slave_count = self._master.config_init()
        if self.slave_count == 0:
            self._master.close()
            raise ConnectionError("No EtherCAT slaves found.")
            
        self.drive_slave = self._master.slaves[0]
        # Assign the configuration function to be called by config_map
        self.drive_slave.config_func = self._drive_setup_func
        self.comm_thread = None
        self.is_running = False
        self.current_state = "INIT"

    def _drive_setup_func(self, slave_pos):
        """
        This function is called by pysoem's config_map at the correct time
        to configure the slave's PDOs.
        """
        # --- PDO Configuration (requires PRE-OP state, handled by pysoem) ---
        # The 'ca' parameter (complete access) should be False when writing to a single subindex.
        
        # Assign RxPDO
        self.drive_slave.sdo_write(0x1c12, 0, b'\x00', False) # Clear RxPDO count
        self.drive_slave.sdo_write(0x1c12, 1, struct.pack('H', 0x1600), False) # Set 1st RxPDO to 0x1600
        self.drive_slave.sdo_write(0x1c12, 0, b'\x01', False) # Set RxPDO count to 1
        
        # Assign TxPDO
        self.drive_slave.sdo_write(0x1c13, 0, b'\x00', False) # Clear TxPDO count
        self.drive_slave.sdo_write(0x1c13, 1, struct.pack('H', 0x1a00), False) # Set 1st TxPDO to 0x1a00
        self.drive_slave.sdo_write(0x1c13, 0, b'\x01', False) # Set TxPDO count to 1

        # Configure RxPDO mapping (Master -> Drive)
        self.drive_slave.sdo_write(0x1600, 0, b'\x00', False) # Clear mapping
        self.drive_slave.sdo_write(0x1600, 1, struct.pack('<I', 0x60400010), False) # Control Word
        self.drive_slave.sdo_write(0x1600, 2, struct.pack('<I', 0x607A0020), False) # Target Position
        self.drive_slave.sdo_write(0x1600, 3, struct.pack('<I', 0x60FF0020), False) # Target Velocity
        self.drive_slave.sdo_write(0x1600, 4, struct.pack('<I', 0x60600008), False) # Modes of Operation
        self.drive_slave.sdo_write(0x1600, 0, b'\x04', False) # Set number of mapped objects

        # Configure TxPDO mapping (Drive -> Master)
        self.drive_slave.sdo_write(0x1A00, 0, b'\x00', False) # Clear mapping
        self.drive_slave.sdo_write(0x1A00, 1, struct.pack('<I', 0x60410010), False) # Status Word
        self.drive_slave.sdo_write(0x1A00, 2, struct.pack('<I', 0x60640020), False) # Position Actual Value
        self.drive_slave.sdo_write(0x1A00, 3, struct.pack('<I', 0x606C0020), False) # Velocity Actual Value
        self.drive_slave.sdo_write(0x1A00, 4, struct.pack('<I', 0x60610008), False) # Modes of Operation Display
        self.drive_slave.sdo_write(0x1A00, 0, b'\x04', False) # Set number of mapped objects

    def start_communication(self):
        """Initializes the slave and starts the communication thread."""
        
        self._master.config_dc()
        self._master.config_map()
        
        self._master.read_state()
        if self._master.state_check(pysoem.SAFEOP_STATE, timeout=5000000) != pysoem.SAFEOP_STATE:
            self._master.read_state()
            al_status_code = self.drive_slave.al_status
            al_status_string = pysoem.al_status_code_to_string(al_status_code)
            raise RuntimeError(f"Not all slaves reached SAFE-OP state. "
                               f"Slave 1 is in state {self.drive_slave.state:#x} "
                               f"with AL Status Code {al_status_code:#06x} ({al_status_string})")

        self.is_running = True
        self.comm_thread = threading.Thread(target=self._communication_loop)
        self.comm_thread.daemon = True
        self.comm_thread.start()
        
        self._master.state = pysoem.OP_STATE
        self._master.write_state()
        self._master.state_check(pysoem.OP_STATE, timeout=5000000)
        
        if self._master.state != pysoem.OP_STATE:
            self.is_running = False
            self.comm_thread.join()
            self._master.read_state()
            al_status_code = self.drive_slave.al_status
            al_status_string = pysoem.al_status_code_to_string(al_status_code)
            raise RuntimeError(f"Not all slaves reached OP state. "
                               f"Slave 1 is in state {self.drive_slave.state:#x} "
                               f"with AL Status Code {al_status_code:#06x} ({al_status_string})")

        self.current_state = "OPERATIONAL"

    def stop_communication(self):
        if self.is_running:
            self.is_running = False
            if self.comm_thread:
                self.comm_thread.join()
        
        if self._master.context_initialized:
            self._master.state = pysoem.INIT_STATE
            self._master.write_state()
            self._master.close()

    def _communication_loop(self):
        while self.is_running:
            self._master.send_processdata()
            self._master.receive_processdata(timeout=2000)
            time.sleep(0.01)

    def _write_pdo(self, offset, data):
        """Writes data to a specific offset in the slave's output process data."""
        # Create a mutable copy of the current output process data
        output_data = bytearray(self.drive_slave.output)
        
        # Modify the slice with the new data
        output_data[offset:offset+len(data)] = data
        
        # Write the entire modified buffer back
        self.drive_slave.output = bytes(output_data)

    def _read_pdo(self, offset, length):
        return self.drive_slave.input[offset:offset+length]

    def reset_fault(self):
        control_word = struct.pack('<H', CW_FAULT_RESET)
        self._write_pdo(0, control_word)
        time.sleep(0.1)
        control_word = struct.pack('<H', 0)
        self._write_pdo(0, control_word)
        
    def shutdown(self):
        self._write_pdo(0, struct.pack('<H', CW_SHUTDOWN))

    def switch_on(self):
        self._write_pdo(0, struct.pack('<H', CW_SWITCH_ON))

    def enable_operation(self):
        self._write_pdo(0, struct.pack('<H', CW_ENABLE_OPERATION))

    def set_operation_mode(self, mode):
        self._write_pdo(10, struct.pack('b', mode))

    def set_target_velocity(self, velocity):
        self._write_pdo(6, struct.pack('<i', velocity))
    
    def set_target_position(self, position):
        self._write_pdo(2, struct.pack('<i', position))

    def trigger_position_move(self):
        self._write_pdo(0, struct.pack('<H', CW_ENABLE_OPERATION))
        time.sleep(0.01)
        self._write_pdo(0, struct.pack('<H', CW_START_MOVE_ABS))
        time.sleep(0.01)
        self._write_pdo(0, struct.pack('<H', CW_ENABLE_OPERATION))

    def get_status(self):
        try:
            sw_data = self._read_pdo(0, 2)
            pa_data = self._read_pdo(2, 4)
            va_data = self._read_pdo(6, 4)
            md_data = self._read_pdo(10, 1)

            status_word = struct.unpack('<H', sw_data)[0]
            actual_position = struct.unpack('<i', pa_data)[0]
            actual_velocity = struct.unpack('<i', va_data)[0]
            mode_display = struct.unpack('b', md_data)[0]
            
            return {
                "status_word": status_word,
                "actual_position": actual_position,
                "actual_velocity": actual_velocity,
                "mode_display": mode_display,
            }
        except (IndexError, struct.error):
            return {
                "status_word": 0, "actual_position": 0,
                "actual_velocity": 0, "mode_display": 0,
            }
            
    @staticmethod
    def parse_status_word(sw):
        if sw & SW_FAULT: return "Fault"
        if not (sw & SW_READY_TO_SWITCH_ON): return "Not Ready"
        if sw & SW_SWITCH_ON_DISABLED: return "Switch On Disabled"
        if (sw & 0x04F7) == 0x0031: return "Switched On"
        if (sw & 0x04F7) == 0x0037: return "Operation Enabled"
        if (sw & 0x04F7) == 0x0021: return "Ready to Switch On"
        return "Unknown State"

class ServoGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("EtherCAT Servo Controller")
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
        self.root.minsize(550, 450)

        self.controller = None
        self.update_job = None

        self.iface_name = tk.StringVar(value="\\Device\\NPF_{B0BE6AD4-7066-42CB-B5A1-B0E2503F8B9A}")
        self.connection_status = tk.StringVar(value="Disconnected")
        self.drive_state = tk.StringVar(value="N/A")
        self.status_word_val = tk.StringVar(value="0x0000")
        self.actual_pos_val = tk.StringVar(value="0")
        self.actual_vel_val = tk.StringVar(value="0")
        self.mode_display_val = tk.StringVar(value="N/A")
        self.operation_mode = tk.IntVar(value=MODE_PROFILED_POSITION)
        self.target_pos = tk.StringVar(value="100000")
        self.target_vel = tk.StringVar(value="50000")

        self._create_widgets()

    def _create_widgets(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        conn_frame = ttk.LabelFrame(main_frame, text="Connection", padding="10")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E))
        conn_frame.columnconfigure(1, weight=1)
        
        ttk.Label(conn_frame, text="Network Interface:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.iface_entry = ttk.Entry(conn_frame, textvariable=self.iface_name, width=50)
        self.iface_entry.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=5, pady=5)
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2, padx=5, pady=5)
        
        ttk.Label(conn_frame, text="Status:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        ttk.Label(conn_frame, textvariable=self.connection_status, foreground="blue").grid(row=1, column=1, columnspan=2, sticky=tk.W, padx=5, pady=5)
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)

        status_frame = ttk.LabelFrame(main_frame, text="Drive Status", padding="10")
        status_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5), pady=5)
        
        ttk.Label(status_frame, text="CiA 402 State:").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Label(status_frame, textvariable=self.drive_state, font=("TkDefaultFont", 10, "bold")).grid(row=0, column=1, sticky=tk.W, pady=2)
        ttk.Label(status_frame, text="Status Word:").grid(row=1, column=0, sticky=tk.W, pady=2)
        ttk.Label(status_frame, textvariable=self.status_word_val).grid(row=1, column=1, sticky=tk.W, pady=2)
        ttk.Label(status_frame, text="Actual Position:").grid(row=2, column=0, sticky=tk.W, pady=2)
        ttk.Label(status_frame, textvariable=self.actual_pos_val).grid(row=2, column=1, sticky=tk.W, pady=2)
        ttk.Label(status_frame, text="Actual Velocity:").grid(row=3, column=0, sticky=tk.W, pady=2)
        ttk.Label(status_frame, textvariable=self.actual_vel_val).grid(row=3, column=1, sticky=tk.W, pady=2)
        ttk.Label(status_frame, text="Actual Op Mode:").grid(row=4, column=0, sticky=tk.W, pady=2)
        ttk.Label(status_frame, textvariable=self.mode_display_val).grid(row=4, column=1, sticky=tk.W, pady=2)

        control_frame = ttk.LabelFrame(main_frame, text="Drive Control", padding="10")
        control_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0), pady=5)
        
        state_ctrl_frame = ttk.Frame(control_frame)
        state_ctrl_frame.pack(fill=tk.X, pady=5, expand=True)
        self.reset_btn = ttk.Button(state_ctrl_frame, text="Reset Fault", command=self.reset_drive_fault, state=tk.DISABLED)
        self.reset_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        self.enable_btn = ttk.Button(state_ctrl_frame, text="Enable Drive", command=self.enable_drive, state=tk.DISABLED)
        self.enable_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        mode_frame = ttk.Frame(control_frame)
        mode_frame.pack(fill=tk.X, pady=10)
        self.pos_radio = ttk.Radiobutton(mode_frame, text="Position Mode", variable=self.operation_mode, value=MODE_PROFILED_POSITION, command=self.set_mode, state=tk.DISABLED)
        self.pos_radio.pack(side=tk.LEFT, padx=5)
        self.vel_radio = ttk.Radiobutton(mode_frame, text="Velocity Mode", variable=self.operation_mode, value=MODE_PROFILED_VELOCITY, command=self.set_mode, state=tk.DISABLED)
        self.vel_radio.pack(side=tk.LEFT, padx=5)
        
        pos_ctrl_frame = ttk.LabelFrame(control_frame, text="Profile Position (PP)", padding=10)
        pos_ctrl_frame.pack(fill=tk.X, pady=5, expand=True)
        pos_ctrl_frame.columnconfigure(1, weight=1)
        ttk.Label(pos_ctrl_frame, text="Target Position:").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(pos_ctrl_frame, textvariable=self.target_pos).grid(row=0, column=1, sticky=(tk.W, tk.E), padx=5)
        self.move_btn = ttk.Button(pos_ctrl_frame, text="Move", command=self.move_to_position, state=tk.DISABLED)
        self.move_btn.grid(row=0, column=2, padx=5)

        vel_ctrl_frame = ttk.LabelFrame(control_frame, text="Profile Velocity (PV)", padding=10)
        vel_ctrl_frame.pack(fill=tk.X, pady=5, expand=True)
        vel_ctrl_frame.columnconfigure(1, weight=1)
        ttk.Label(vel_ctrl_frame, text="Target Velocity:").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(vel_ctrl_frame, textvariable=self.target_vel).grid(row=0, column=1, sticky=(tk.W, tk.E), padx=5)
        self.run_btn = ttk.Button(vel_ctrl_frame, text="Run", command=self.run_at_velocity, state=tk.DISABLED)
        self.run_btn.grid(row=0, column=2, padx=5)
        
    def _toggle_controls(self, state):
        self.reset_btn.config(state=state)
        self.enable_btn.config(state=state)
        self.move_btn.config(state=state)
        self.run_btn.config(state=state)
        self.pos_radio.config(state=state)
        self.vel_radio.config(state=state)
        self.iface_entry.config(state=tk.DISABLED if state == tk.NORMAL else tk.NORMAL)

    def toggle_connection(self):
        if self.controller and self.controller.is_running:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        try:
            self.controller = EtherCATController(self.iface_name.get())
            self.controller.start_communication()
            
            self.connection_status.set(f"Connected to {self.controller.slave_count} slave(s)")
            self.connect_btn.config(text="Disconnect")
            self._toggle_controls(tk.NORMAL)
            
            self.start_status_updates()

        except ConnectionError as e:
            messagebox.showerror("Connection Failed", f"Could not open the network interface.\n\nError: {e}\n\n- Ensure Npcap/WinPcap is installed (Windows).\n- Run this script with administrator/root privileges.\n- Verify the interface name is correct.")
            if self.controller: self.controller.stop_communication()
            self.controller = None
        except pysoem.SdoError as e:
            messagebox.showerror("Configuration Failed", f"Failed to configure the drive (SDO Error).\n\nError: {e}\n\n- Check drive's manual for the specific SDO and error code.")
            if self.controller: self.controller.stop_communication()
            self.controller = None
        except Exception as e:
            messagebox.showerror("An Error Occurred", str(e))
            if self.controller: self.controller.stop_communication()
            self.controller = None

    def disconnect(self):
        self.stop_status_updates()
        if self.controller:
            self.controller.stop_communication()
        
        self.connection_status.set("Disconnected")
        self.drive_state.set("N/A")
        self.connect_btn.config(text="Connect")
        self._toggle_controls(tk.DISABLED)
        
    def start_status_updates(self):
        self._update_status()
        self.update_job = self.root.after(100, self.start_status_updates)

    def stop_status_updates(self):
        if self.update_job:
            self.root.after_cancel(self.update_job)
            self.update_job = None

    def _update_status(self):
        if self.controller and self.controller.is_running:
            status = self.controller.get_status()
            sw = status["status_word"]
            
            self.status_word_val.set(f"0x{sw:04X}")
            self.actual_pos_val.set(str(status["actual_position"]))
            self.actual_vel_val.set(str(status["actual_velocity"]))
            self.mode_display_val.set(str(status["mode_display"]))
            self.drive_state.set(self.controller.parse_status_word(sw))

    def reset_drive_fault(self):
        if self.controller: self.controller.reset_fault()

    def enable_drive(self):
        if self.controller:
            threading.Thread(target=self._enable_drive_sequence).start()
            
    def _enable_drive_sequence(self):
        if self.controller:
            self.controller.shutdown()
            time.sleep(0.1)
            self.controller.switch_on()
            time.sleep(0.1)
            self.controller.enable_operation()

    def set_mode(self):
        if self.controller: self.controller.set_operation_mode(self.operation_mode.get())

    def move_to_position(self):
        if self.controller:
            try:
                pos = int(self.target_pos.get())
                self.controller.set_operation_mode(MODE_PROFILED_POSITION)
                time.sleep(0.01)
                self.controller.set_target_position(pos)
                self.controller.trigger_position_move()
            except ValueError: messagebox.showerror("Invalid Input", "Position must be an integer.")
            except Exception as e: messagebox.showerror("Error", f"Could not perform move: {e}")

    def run_at_velocity(self):
        if self.controller:
            try:
                vel = int(self.target_vel.get())
                self.controller.set_operation_mode(MODE_PROFILED_VELOCITY)
                time.sleep(0.01)
                self.controller.set_target_velocity(vel)
            except ValueError: messagebox.showerror("Invalid Input", "Velocity must be an integer.")
            except Exception as e: messagebox.showerror("Error", f"Could not run at velocity: {e}")

    def _on_closing(self):
        if self.controller and self.controller.is_running:
            self.disconnect()
        self.root.destroy()

if __name__ == '__main__':
    root = tk.Tk()
    app = ServoGUI(root)
    root.mainloop()

