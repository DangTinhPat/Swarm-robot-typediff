import os
import queue
import signal
import subprocess
import threading
import tkinter as tk
from tkinter import scrolledtext, ttk

# Each entry: (button label, ros2 launch argv). Kept as plain argv lists (not
# shell strings) so no shell quoting/injection concerns launching them.
LAUNCHES = {
    "1 robot": [
        ("Gazebo", ["ros2", "launch", "main_bot", "gz.launch.py"]),
        ("RViz", ["ros2", "launch", "main_bot", "rz.launch.py"]),
        ("SLAM", ["ros2", "launch", "main_bot", "slam.launch.py"]),
        ("Nav2", ["ros2", "launch", "main_bot", "nav2.launch.py"]),
    ],
    "2 robot (bay dan)": [
        ("Gazebo", ["ros2", "launch", "main_bot", "multi_robot.launch.py"]),
        ("Nav2", ["ros2", "launch", "main_bot", "multi_robot_nav2.launch.py"]),
        ("RViz robot1", ["ros2", "launch", "main_bot", "multi_robot_rviz.launch.py", "namespace:=robot1"]),
        ("RViz robot2", ["ros2", "launch", "main_bot", "multi_robot_rviz.launch.py", "namespace:=robot2"]),
    ],
}


class LaunchManager:
    """Starts/stops ros2 launch subprocesses and streams their output.

    Each subprocess runs in its own process group (preexec_fn=os.setsid) so
    stopping it can signal the whole group, not just the "ros2 launch"
    wrapper PID - ros2 launch fans out into many child processes (nodes,
    bridges), and killing only the parent leaves the rest running as orphans.
    """

    def __init__(self, log_queue):
        self.log_queue = log_queue
        self._processes = {}
        self._lock = threading.Lock()

    def is_running(self, name):
        with self._lock:
            proc = self._processes.get(name)
        return proc is not None and proc.poll() is None

    def start(self, name, cmd):
        if self.is_running(name):
            return
        self.log_queue.put((name, f"$ {' '.join(cmd)}\n"))
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            preexec_fn=os.setsid,
        )
        with self._lock:
            self._processes[name] = proc
        threading.Thread(target=self._pump_output, args=(name, proc), daemon=True).start()

    def _pump_output(self, name, proc):
        for line in proc.stdout:
            self.log_queue.put((name, line))
        self.log_queue.put((name, "--- process exited ---\n"))

    def stop(self, name):
        with self._lock:
            proc = self._processes.get(name)
        if proc is None or proc.poll() is not None:
            return
        pgid = os.getpgid(proc.pid)
        try:
            os.killpg(pgid, signal.SIGINT)
            proc.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
        # ros2 launch doesn't reliably cascade SIGINT to every child it spawned
        # (nodes, bridges) within the wait window - proc.wait() above only
        # confirms the "ros2 launch" process itself exited. Force-kill whatever
        # is left in the group so Stop never leaves an orphaned gazebo/nav2
        # process running in the background.
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def stop_all(self):
        with self._lock:
            names = list(self._processes.keys())
        for name in names:
            self.stop(name)


class App:
    def __init__(self, root):
        self.root = root
        self.root.title("main_bot control panel")
        self.log_queue = queue.Queue()
        self.manager = LaunchManager(self.log_queue)
        self.status_labels = {}

        left = ttk.Frame(root, padding=10)
        left.pack(side=tk.LEFT, fill=tk.Y)

        for group_name, items in LAUNCHES.items():
            ttk.Label(left, text=group_name, font=("", 10, "bold")).pack(anchor="w", pady=(8, 2))
            for label, cmd in items:
                # "Gazebo"/"Nav2" appear in both groups; key by group+label so
                # they don't collide in status_labels / LaunchManager's process map.
                self._make_row(left, f"{group_name}/{label}", label, cmd)

        ttk.Separator(left, orient="horizontal").pack(fill="x", pady=8)
        ttk.Button(left, text="Dung tat ca", command=self.manager.stop_all).pack(fill="x")

        self.log = scrolledtext.ScrolledText(root, width=110, height=38, state="disabled")
        self.log.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(100, self._poll_log)
        self.root.after(500, self._refresh_statuses)

    def _make_row(self, parent, name, label, cmd):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=2)
        status = ttk.Label(row, text="●", foreground="gray")
        status.pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(
            row, text=label, width=20, command=lambda: self._toggle(name, cmd)
        ).pack(side=tk.LEFT)
        self.status_labels[name] = status

    def _toggle(self, name, cmd):
        if self.manager.is_running(name):
            self.manager.stop(name)
        else:
            self.manager.start(name, cmd)

    def _refresh_statuses(self):
        for name, status in self.status_labels.items():
            status.config(foreground="green" if self.manager.is_running(name) else "gray")
        self.root.after(500, self._refresh_statuses)

    def _poll_log(self):
        try:
            while True:
                name, line = self.log_queue.get_nowait()
                self.log.configure(state="normal")
                self.log.insert("end", f"[{name}] {line}")
                self.log.see("end")
                self.log.configure(state="disabled")
        except queue.Empty:
            pass
        self.root.after(100, self._poll_log)

    def _on_close(self):
        self.manager.stop_all()
        self.root.destroy()


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
