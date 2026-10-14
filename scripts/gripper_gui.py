#!/usr/bin/env python3
"""Serial GUI for the q_gripper (firmware/nano33iot_qgrip/nano33iot_qgrip.ino).

Lets you pick the serial port the gripper is connected on, shows all
telemetry values it streams (position, torque, load-cell forces, IMU,
temperature, voltage, current, ...) as live-updating numbers, lets you
plot any subset of those channels over time, and provides a text field
to send raw commands to the gripper (see the "Command reference" button
for the command set implemented by the firmware's handle_command()).

Requires: pyserial, matplotlib.
    pip install pyserial matplotlib
"""
import sys
import time
import queue
import threading
import collections
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk



BAUD_RATES = [9600, 57600, 115200, 230400, 250000, 500000, 1000000]
DEFAULT_BAUD = 230400  # matches Serial.begin(230400) in the firmware's setup()
HISTORY_LEN = 3000
UI_POLL_MS = 100
PLOT_UPDATE_MS = 500
MAX_LOG_LINES = 1000

# Maps (line-prefix, mnemonic) from the firmware's printStamped*Array() lines
# to human-readable channel names. Anything stamped but not listed here still
# shows up, just named "<prefix>_<mnemonic>_<index>".
CHANNEL_MAP = {
    ("P", "pos"): ("position",),
    ("E", "pos"): ("position_error",),
    ("C", "goals"): ("position_goal",),
    ("M", "trq"): ("torque",),
    ("V", "vel"): ("velocity",),
    ("F", "forces"): ("loadcell0_raw", "loadcell1_raw"),
    ("N", "newtons"): ("force0_N", "force1_N"),
    ("T", "temp"): ("servo_temp",),
    ("U", "volts"): ("servo_voltage",),
    ("I", "amps"): ("servo_current",),
    ("A0", "counts"): ("analog0",),
    ("A1", "counts"): ("analog1",),
}

USAGE_TEXT = """Command reference (single character + optional arguments, sent as text + Enter)
mirrored from the firmware's usage()/handle_command():

h / ?     print this help
p         print servo position (SCS)
q         print servo position (SMS)
t         print servo temperature
i         print motor current (SCS)
j         print motor current (SMS)
l         print servo load/torque (SCS)
m         print servo load/torque (SMS)
r <id> <addr>            read register value
w <id> <addr> <value>    write register value
v         print supply voltage
@ <0/1>   enable/disable echo mode (test without servos)
# <0/1>   enable/disable servo(s)

A <int>   set servo acceleration (SMS) [0 .. 4095]
B <int>   set servo speed (SMS) [0 .. 4095]
C <int>   close fingers by given degrees
D         start/continue demo
E <int>   effort goal (counts) [not yet implemented in firmware]
F         freeze (hold at current position)
G <int>   position goal (counts)
H         move to home (all zero)
I         switch to trigger-torque control mode
J         switch to torque control mode
K         switch to position control mode
L         switch to idle mode
O <int>   open fingers by given degrees
Q <pos> [<vel> <acc>]    set servo goal position (SMS) [0 .. 4095]
s <id>    select active servo id (3..253)
T <int>   set torque limit (counts)
Y <int>   set verbose level
ESC       disable demo + freeze

Notes:
- Continuous telemetry (position/torque/forces/IMU/temp/voltage/current) is
  parsed automatically and shown in the "Live values" table and available
  for plotting.
- Replies to single-shot query commands (p, q, t, v, i, j, l, m) are only
  shown in the raw serial log below, not in the values table.
"""


def parse_telemetry_line(line):
    """Parse one line of gripper telemetry into {channel_name: value}.

    Returns None if the line is not a recognized stamped-data line (e.g.
    free-text log/debug output or a single-shot query reply).
    """
    tokens = line.split()
    if len(tokens) < 4:
        return None
    prefix = tokens[0]

    # "G <seq> <ms> 0 accel <ax> <ay> <az> gyro <gx> <gy> <gz>"
    if prefix == "G" and len(tokens) >= 12 and tokens[4] == "accel" and tokens[8] == "gyro":
        try:
            ax, ay, az = (float(v) for v in tokens[5:8])
            gx, gy, gz = (float(v) for v in tokens[9:12])
        except ValueError:
            return None
        return {"accel_x": ax, "accel_y": ay, "accel_z": az,
                "gyro_x": gx, "gyro_y": gy, "gyro_z": gz}

    # "D <iter> <ms> J <pos> V <vel> E <trq>"
    if prefix == "D" and len(tokens) == 9 and tokens[3] == "J" and tokens[5] == "V" and tokens[7] == "E":
        try:
            return {"position": float(tokens[4]), "velocity": float(tokens[6]), "torque": float(tokens[8])}
        except ValueError:
            return None

    # generic "<prefix> <seq> <ms> <mnemonic> <value...>"
    if len(tokens) < 5:
        return None
    try:
        int(tokens[1])
        int(tokens[2])
    except ValueError:
        return None

    mnemonic = tokens[3]
    try:
        values = [float(v) for v in tokens[4:]]
    except ValueError:
        return None

    names = CHANNEL_MAP.get((prefix, mnemonic))
    if names and len(names) == len(values):
        return dict(zip(names, values))
    return {f"{prefix}_{mnemonic}_{i}": v for i, v in enumerate(values)}


class SerialWorker:
    """Owns the serial.Serial instance; reads lines on a background thread
    and feeds them into a queue for the GUI thread to consume."""

    def __init__(self, out_queue):
        self.out_queue = out_queue
        self.ser = None
        self.thread = None
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()

    @property
    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def connect(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def disconnect(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        self.thread = None
        if self.ser is not None:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None

    def send(self, text):
        if not self.is_open:
            raise RuntimeError("not connected")
        data = (text + "\n").encode("ascii", errors="replace")
        with self.write_lock:
            self.ser.write(data)

    def _read_loop(self):
        while not self.stop_event.is_set():
            try:
                raw = self.ser.readline()
            except serial.SerialException as exc:
                self.out_queue.put(("error", str(exc)))
                return
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue
            parsed = parse_telemetry_line(line)
            if parsed:
                self.out_queue.put(("data", (time.time(), parsed)))
            else:
                self.out_queue.put(("raw", line))


class GripperGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("q_gripper serial monitor")

        self.queue = queue.Queue()
        self.worker = SerialWorker(self.queue)

        self.start_time = None
        self.latest = {}       # channel -> (value, timestamp)
        self.history = {}      # channel -> deque[(t_rel, value)]
        self.channels_in_table = set()

        self.cmd_history = []
        self.cmd_history_pos = None

        self._build_ui()
        self._refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.root.after(UI_POLL_MS, self._poll_queue)
        self.root.after(PLOT_UPDATE_MS, self._update_plot)

    # ---------------------------------------------------------------- UI ---
    def _build_ui(self):
        conn = ttk.Frame(self.root, padding=6)
        conn.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(conn, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=28, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(2, 6))

        ttk.Button(conn, text="Refresh", command=self._refresh_ports).pack(side=tk.LEFT)

        ttk.Label(conn, text="Baud:").pack(side=tk.LEFT, padx=(12, 0))
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        baud_combo = ttk.Combobox(conn, textvariable=self.baud_var, width=10,
                                   values=[str(b) for b in BAUD_RATES])
        baud_combo.pack(side=tk.LEFT, padx=(2, 6))

        self.connect_btn = ttk.Button(conn, text="Connect", command=self._toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, padx=(6, 0))

        self.status_var = tk.StringVar(value="disconnected")
        ttk.Label(conn, textvariable=self.status_var, foreground="#a00").pack(side=tk.LEFT, padx=12)

        ttk.Button(conn, text="Command reference", command=self._show_reference).pack(side=tk.RIGHT)

        main = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        main.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # -- left: live values table
        left = ttk.Frame(main, padding=4)
        main.add(left, weight=1)

        ttk.Label(left, text="Live values").pack(anchor=tk.W)
        self.tree = ttk.Treeview(left, columns=("value", "age"), show="tree headings", height=25)
        self.tree.heading("#0", text="channel")
        self.tree.heading("value", text="value")
        self.tree.heading("age", text="age")
        self.tree.column("#0", width=170)
        self.tree.column("value", width=110, anchor=tk.E)
        self.tree.column("age", width=70, anchor=tk.E)
        self.tree.pack(fill=tk.BOTH, expand=True)

        self.tree_menu = tk.Menu(self.tree, tearoff=0)
        self.tree_menu.add_command(label="Copy line", command=self._copy_line)
        self.tree_menu.add_command(label="Copy value", command=self._copy_value)
        self.tree_menu.add_separator()
        self.tree_menu.add_command(label="Copy all", command=self._copy_all_rows)
        self.tree.bind("<Control-c>", self._copy_line)
        self.tree.bind("<Button-3>", self._on_tree_right_click)

        # -- right: plot controls + canvas
        right = ttk.Frame(main, padding=4)
        main.add(right, weight=3)

        plot_ctrl = ttk.Frame(right)
        plot_ctrl.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(plot_ctrl, text="Plot channels (ctrl/shift-click to select):").pack(side=tk.LEFT)
        ttk.Button(plot_ctrl, text="Clear history", command=self._clear_history).pack(side=tk.RIGHT)

        body = ttk.Frame(right)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.channel_list = tk.Listbox(body, selectmode=tk.EXTENDED, exportselection=False, width=24)
        self.channel_list.pack(side=tk.LEFT, fill=tk.Y)

        fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = fig.add_subplot(111)
        self.ax.set_xlabel("time [s]")
        self.canvas = FigureCanvasTkAgg(fig, master=body)
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        toolbar_frame = ttk.Frame(right)
        toolbar_frame.pack(side=tk.TOP, fill=tk.X)
        NavigationToolbar2Tk(self.canvas, toolbar_frame)

        # -- bottom: command entry + raw log
        bottom = ttk.Frame(self.root, padding=6)
        bottom.pack(side=tk.BOTTOM, fill=tk.X)

        cmd_frame = ttk.Frame(bottom)
        cmd_frame.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(cmd_frame, text="Command:").pack(side=tk.LEFT)
        self.cmd_var = tk.StringVar()
        self.cmd_entry = ttk.Entry(cmd_frame, textvariable=self.cmd_var)
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        self.cmd_entry.bind("<Return>", lambda e: self._send_command())
        self.cmd_entry.bind("<Up>", self._cmd_history_prev)
        self.cmd_entry.bind("<Down>", self._cmd_history_next)
        ttk.Button(cmd_frame, text="Send", command=self._send_command).pack(side=tk.LEFT)

        ttk.Label(bottom, text="Raw serial log:").pack(side=tk.TOP, anchor=tk.W, pady=(6, 0))
        log_frame = ttk.Frame(bottom)
        log_frame.pack(side=tk.TOP, fill=tk.BOTH)
        self.log_text = tk.Text(log_frame, height=8, state=tk.DISABLED, wrap=tk.NONE)
        log_scroll = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_scroll.set)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        log_scroll.pack(side=tk.LEFT, fill=tk.Y)

    def _show_reference(self):
        top = tk.Toplevel(self.root)
        top.title("Command reference")
        text = tk.Text(top, width=78, height=32, wrap=tk.WORD)
        text.insert("1.0", USAGE_TEXT)
        text.configure(state=tk.DISABLED)
        text.pack(fill=tk.BOTH, expand=True)

    # ------------------------------------------------------------ serial ---
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.worker.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showerror("No port", "Select a serial port first.")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Invalid baud", "Baud rate must be an integer.")
            return
        try:
            self.worker.connect(port, baud)
        except serial.SerialException as exc:
            messagebox.showerror("Connection failed", str(exc))
            return

        self.start_time = time.time()
        self.status_var.set(f"connected: {port} @ {baud}")
        self.connect_btn.configure(text="Disconnect")
        self.port_combo.configure(state=tk.DISABLED)

    def _disconnect(self):
        self.worker.disconnect()
        self.status_var.set("disconnected")
        self.connect_btn.configure(text="Connect")
        self.port_combo.configure(state="readonly")

    def _on_close(self):
        self.worker.disconnect()
        self.root.destroy()

    # ----------------------------------------------------------- commands ---
    def _send_command(self):
        text = self.cmd_var.get()
        if not text:
            return
        if not self.worker.is_open:
            messagebox.showwarning("Not connected", "Connect to the gripper first.")
            return
        try:
            self.worker.send(text)
        except RuntimeError as exc:
            messagebox.showwarning("Not connected", str(exc))
            return
        self._append_log(f">> {text}")
        self.cmd_history.append(text)
        self.cmd_history_pos = None
        self.cmd_var.set("")

    def _cmd_history_prev(self, _event):
        if not self.cmd_history:
            return
        if self.cmd_history_pos is None:
            self.cmd_history_pos = len(self.cmd_history) - 1
        elif self.cmd_history_pos > 0:
            self.cmd_history_pos -= 1
        self.cmd_var.set(self.cmd_history[self.cmd_history_pos])
        self.cmd_entry.icursor(tk.END)
        return "break"

    def _cmd_history_next(self, _event):
        if self.cmd_history_pos is None:
            return
        if self.cmd_history_pos < len(self.cmd_history) - 1:
            self.cmd_history_pos += 1
            self.cmd_var.set(self.cmd_history[self.cmd_history_pos])
        else:
            self.cmd_history_pos = None
            self.cmd_var.set("")
        self.cmd_entry.icursor(tk.END)
        return "break"

    # ---------------------------------------------------------- copy/paste ---
    def _on_tree_right_click(self, event):
        row = self.tree.identify_row(event.y)
        if row and row not in self.tree.selection():
            self.tree.selection_set(row)
        self.tree_menu.tk_popup(event.x_root, event.y_root)

    def _copy_to_clipboard(self, text):
        self.root.clipboard_clear()
        self.root.clipboard_append(text)

    def _copy_line(self, _event=None):
        iids = self.tree.selection()
        if not iids:
            return "break"
        lines = ["channel\tvalue\tage"]
        for iid in iids:
            value, age = self.tree.item(iid, "values")
            lines.append(f"{iid}\t{value}\t{age}")
        self._copy_to_clipboard("\n".join(lines))
        return "break"

    def _copy_value(self):
        iids = self.tree.selection()
        if not iids:
            return
        values = [self.tree.set(iid, "value") for iid in iids]
        self._copy_to_clipboard("\n".join(values))

    def _copy_all_rows(self):
        iids = self.tree.get_children()
        if not iids:
            return
        lines = ["channel\tvalue\tage"]
        for iid in iids:
            value, age = self.tree.item(iid, "values")
            lines.append(f"{iid}\t{value}\t{age}")
        self._copy_to_clipboard("\n".join(lines))

    # --------------------------------------------------------------- data ---
    def _clear_history(self):
        self.history.clear()
        self.ax.cla()
        self.ax.set_xlabel("time [s]")
        self.canvas.draw_idle()

    def _append_log(self, line):
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, line + "\n")
        n_lines = int(self.log_text.index("end-1c").split(".")[0])
        if n_lines > MAX_LOG_LINES:
            self.log_text.delete("1.0", f"{n_lines - MAX_LOG_LINES}.0")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _ensure_channel_known(self, channel):
        if channel in self.channels_in_table:
            return
        self.channels_in_table.add(channel)
        self.history[channel] = collections.deque(maxlen=HISTORY_LEN)
        self.tree.insert("", tk.END, iid=channel, text=channel, values=("", ""))
        items = sorted(self.channels_in_table)
        self.channel_list.delete(0, tk.END)
        for name in items:
            self.channel_list.insert(tk.END, name)

    def _poll_queue(self):
        try:
            while True:
                kind, payload = self.queue.get_nowait()
                if kind == "data":
                    ts, values = payload
                    t_rel = ts - self.start_time if self.start_time else 0.0
                    for channel, value in values.items():
                        self._ensure_channel_known(channel)
                        self.latest[channel] = (value, ts)
                        self.history[channel].append((t_rel, value))
                elif kind == "raw":
                    self._append_log(payload)
                elif kind == "error":
                    self._append_log(f"### serial error: {payload}")
                    self._disconnect()
        except queue.Empty:
            pass

        now = time.time()
        for channel, (value, ts) in self.latest.items():
            age = now - ts
            self.tree.set(channel, "value", f"{value:g}")
            self.tree.set(channel, "age", f"{age:0.1f}s")

        self.root.after(UI_POLL_MS, self._poll_queue)

    def _update_plot(self):
        sel = [self.channel_list.get(i) for i in self.channel_list.curselection()]
        self.ax.cla()
        self.ax.set_xlabel("time [s]")
        for channel in sel:
            hist = self.history.get(channel)
            if not hist:
                continue
            xs = [p[0] for p in hist]
            ys = [p[1] for p in hist]
            self.ax.plot(xs, ys, label=channel)
        if sel:
            self.ax.legend(loc="upper left", fontsize="small")
        self.canvas.draw_idle()
        self.root.after(PLOT_UPDATE_MS, self._update_plot)


def main():
    root = tk.Tk()
    GripperGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
