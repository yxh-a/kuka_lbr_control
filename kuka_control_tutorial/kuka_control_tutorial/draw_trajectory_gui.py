"""Draw a trajectory with the mouse and have the arm trace it.

On startup the arm is moved smoothly to a home position, which becomes the
centre of a square drawing area in the robot's XY plane at the home height.
Draw a stroke in that square with one press-drag-release and the arm will move
to where the stroke began and then trace it, at a configurable fraction of the
speed it was drawn at.

This is the top-level demo; the robot-facing work is done by
``cartesian_move_server`` through :class:`MotionClient`, so nothing here talks
to the controller directly.
"""

import math
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from kuka_control_tutorial.motion_client import MotionClient
from kuka_control_tutorial.path_planning import prepare_drawn_path, reachable_radius_xy
from kuka_control_tutorial.stiffness_client import STIFFNESS_LABELS, StiffnessClient
from kuka_control_tutorial.trajectory import distance

# Canvas colours, kept together so the look is easy to adjust.
BG = "#1e1f24"
GRID = "#33353d"
AXIS = "#5a5e6b"
UNREACHABLE = "#4a2530"
STROKE = "#f5a623"
EE_MARKER = "#4ec9b0"
CENTRE = "#7f8596"


class DrawTrajectoryNode(Node):
    """Holds the ROS side: parameters, TF and the motion client."""

    def __init__(self):
        super().__init__("draw_trajectory_gui")

        self.declare_parameter("base_link", "lbr_link_0")
        self.declare_parameter("ee_link", "lbr_link_ee")
        # Startup pose. The drawing square is centred here, at this height.
        self.declare_parameter("home_position", [-0.396, 0.472, 0.405])
        self.declare_parameter("square_size", 0.30)  # [m] side of the drawing area
        self.declare_parameter("canvas_pixels", 600)  # [px] side of the canvas
        # Fraction of the drawn speed to replay at. 0.2 -> five times slower.
        self.declare_parameter("speed_ratio", 0.2)
        self.declare_parameter("approach_speed", 0.05)  # [m/s] for homing and approach
        self.declare_parameter("home_tolerance", 0.01)  # [m] skip homing if closer
        self.declare_parameter("max_reach", 0.85)  # [m] must match the server
        self.declare_parameter("max_speed", 0.25)  # [m/s] must match the server
        self.declare_parameter("smooth_window", 2)  # samples each side
        self.declare_parameter("min_spacing", 0.002)  # [m] between kept samples

        # Stiffness panel. The controller validates what it is sent, so these
        # bounds only decide what the panel will bother asking for.
        self.declare_parameter(
            "controller_node", "/lbr/cartesian_impedance_controller"
        )
        self.declare_parameter("max_stiffness_lin", 2000.0)  # [N/m]
        self.declare_parameter("max_stiffness_rot", 200.0)  # [Nm/rad]
        # How long after the last keystroke a value is pushed, in milliseconds.
        self.declare_parameter("stiffness_push_delay_ms", 400)

        self.base_link = self.get_parameter("base_link").value
        self.ee_link = self.get_parameter("ee_link").value
        self.home = tuple(self.get_parameter("home_position").value)
        self.square_size = self.get_parameter("square_size").value
        self.canvas_pixels = int(self.get_parameter("canvas_pixels").value)
        self.max_reach = self.get_parameter("max_reach").value
        self.max_speed = self.get_parameter("max_speed").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.motion = MotionClient(self)
        self.stiffness = StiffnessClient(
            self, self.get_parameter("controller_node").value
        )

    def ee_position(self):
        """Measured end effector position, or None if TF is not ready."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_link, self.ee_link, rclpy.time.Time()
            )
        except Exception:
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z)

    def wait_for_ee(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            position = self.ee_position()
            if position is not None:
                return position
            time.sleep(0.05)
        return None


class DrawTrajectoryApp:
    """The tkinter side. Runs on the main thread; ROS work goes to a worker."""

    def __init__(self, node: DrawTrajectoryNode):
        self.node = node
        self.events = queue.Queue()
        self.busy = False
        self.stroke_px = []      # canvas points of the stroke being drawn
        self.stroke_times = []   # wall-clock time of each sample
        self.drawing = False

        size = node.canvas_pixels
        self.size = size
        self.scale = size / node.square_size  # pixels per metre

        self.root = tk.Tk()
        self.root.title("Draw a trajectory for the LBR")
        self.root.configure(bg=BG)

        self.canvas = tk.Canvas(
            self.root, width=size, height=size, bg=BG, highlightthickness=0
        )
        self.canvas.grid(row=0, column=0, padx=(12, 6), pady=12)

        self._build_stiffness_panel()
        self._build_controls()
        self._draw_background()

        self.canvas.bind("<Button-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_motion)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)

        self.ee_marker = None
        self.root.after(100, self._pump)
        self.root.after(100, self._track_ee)

    # ------------------------------------------------------------------- layout

    def _build_stiffness_panel(self):
        """Six live stiffness fields down the right-hand side.

        Edits are pushed to the controller's parameter server automatically,
        one short delay after the last keystroke so that typing "200" does not
        travel through 2 and 20 on the way.
        """
        frame = tk.Frame(self.root, bg=BG)
        frame.grid(row=0, column=1, sticky="n", padx=(6, 12), pady=12)

        tk.Label(
            frame,
            text="End effector stiffness",
            bg=BG,
            fg="#d7dae0",
            font=("TkDefaultFont", 10, "bold"),
        ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 2))

        tk.Label(
            frame,
            text="live, pushed to the controller",
            bg=BG,
            fg=AXIS,
            font=("TkDefaultFont", 8),
        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(0, 8))

        self.stiffness_vars = []
        self.stiffness_entries = []
        for i, label in enumerate(STIFFNESS_LABELS):
            units = "N/m" if i < 3 else "Nm/rad"
            tk.Label(frame, text=label, bg=BG, fg="#d7dae0", width=7, anchor="w").grid(
                row=2 + i, column=0, sticky="w", pady=2
            )
            var = tk.StringVar(value="--")
            entry = ttk.Entry(frame, textvariable=var, width=9, justify="right")
            entry.grid(row=2 + i, column=1, sticky="w", pady=2)
            tk.Label(frame, text=units, bg=BG, fg=AXIS, width=7, anchor="w").grid(
                row=2 + i, column=2, sticky="w", padx=(4, 0)
            )
            # Any edit schedules a push; Enter or leaving the field pushes now.
            var.trace_add("write", lambda *_, idx=i: self._on_stiffness_edit(idx))
            entry.bind("<Return>", lambda _e: self._push_stiffness())
            entry.bind("<FocusOut>", lambda _e: self._push_stiffness())
            self.stiffness_vars.append(var)
            self.stiffness_entries.append(entry)

        self.stiffness_status = tk.Label(
            frame,
            text="Waiting for the controller...",
            bg=BG,
            fg=AXIS,
            wraplength=170,
            justify="left",
            anchor="w",
            font=("TkDefaultFont", 8),
        )
        self.stiffness_status.grid(
            row=2 + len(STIFFNESS_LABELS), column=0, columnspan=3,
            sticky="w", pady=(10, 0),
        )

        row = 3 + len(STIFFNESS_LABELS)
        ttk.Button(frame, text="Reload", command=self.on_reload_stiffness).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(8, 0)
        )

        # State for the debounce and for keeping polling out of the user's way.
        self._stiffness_push_job = None
        self._stiffness_dirty = False
        self._stiffness_applied = None

    def _build_controls(self):
        panel = tk.Frame(self.root, bg=BG)
        panel.grid(row=1, column=0, columnspan=2, sticky="ew", padx=12, pady=(0, 12))

        self.status = tk.Label(
            panel,
            text="Starting up...",
            bg=BG,
            fg="#d7dae0",
            anchor="w",
            justify="left",
            font=("TkDefaultFont", 10),
        )
        self.status.grid(row=0, column=0, columnspan=4, sticky="ew", pady=(0, 8))
        panel.columnconfigure(0, weight=1)

        tk.Label(panel, text="Speed ratio", bg=BG, fg="#d7dae0").grid(
            row=1, column=0, sticky="w"
        )
        self.ratio_var = tk.StringVar(
            value=str(self.node.get_parameter("speed_ratio").value)
        )
        ttk.Entry(panel, textvariable=self.ratio_var, width=8).grid(
            row=1, column=1, sticky="w", padx=(6, 16)
        )

        self.home_button = ttk.Button(panel, text="Go home", command=self.on_home)
        self.home_button.grid(row=1, column=2, padx=4)
        self.clear_button = ttk.Button(panel, text="Clear", command=self.clear_stroke)
        self.clear_button.grid(row=1, column=3, padx=4)
        self.stop_button = ttk.Button(panel, text="Stop", command=self.on_stop)
        self.stop_button.grid(row=1, column=4, padx=4)

    # ------------------------------------------------------- coordinate mapping

    def to_robot(self, px, py):
        """Canvas pixel -> robot (x, y, z). Screen right is +x, screen up is +y."""
        home = self.node.home
        x = home[0] + (px - self.size / 2.0) / self.scale
        y = home[1] - (py - self.size / 2.0) / self.scale
        return (x, y, home[2])

    def to_canvas(self, x, y):
        """Robot (x, y) -> canvas pixel."""
        home = self.node.home
        px = self.size / 2.0 + (x - home[0]) * self.scale
        py = self.size / 2.0 - (y - home[1]) * self.scale
        return (px, py)

    # --------------------------------------------------------------- background

    def _draw_background(self):
        """Grid, axes, centre mark and the edge of the reachable workspace."""
        size = self.size
        step_m = 0.05
        step_px = step_m * self.scale

        n = int(self.node.square_size / step_m / 2)
        for i in range(-n, n + 1):
            offset = i * step_px
            self.canvas.create_line(
                size / 2 + offset, 0, size / 2 + offset, size, fill=GRID
            )
            self.canvas.create_line(
                0, size / 2 + offset, size, size / 2 + offset, fill=GRID
            )

        # Shade whatever part of the square the arm cannot reach at this height.
        # The envelope check is a sphere about the base, so at constant z the
        # reachable set is a disc -- one corner of the square can fall outside.
        radius = reachable_radius_xy(self.node.max_reach, self.node.home[2])
        cell = 10
        unreachable_cells = 0
        for px in range(0, size, cell):
            for py in range(0, size, cell):
                x, y, _ = self.to_robot(px + cell / 2, py + cell / 2)
                if math.hypot(x, y) > radius:
                    unreachable_cells += 1
                    self.canvas.create_rectangle(
                        px, py, px + cell, py + cell, fill=UNREACHABLE, width=0
                    )
        self.unreachable_present = unreachable_cells > 0

        if self.unreachable_present:
            # Exact boundary, not just the shaded cells.
            cx, cy = self.to_canvas(0.0, 0.0)
            r = radius * self.scale
            self.canvas.create_oval(
                cx - r, cy - r, cx + r, cy + r, outline="#a8434f", dash=(4, 3)
            )

        self.canvas.create_line(0, size / 2, size, size / 2, fill=AXIS)
        self.canvas.create_line(size / 2, 0, size / 2, size, fill=AXIS)

        m = 7
        self.canvas.create_line(
            size / 2 - m, size / 2, size / 2 + m, size / 2, fill=CENTRE, width=2
        )
        self.canvas.create_line(
            size / 2, size / 2 - m, size / 2, size / 2 + m, fill=CENTRE, width=2
        )

        self.canvas.create_text(
            size - 8, size / 2 - 12, text="+X", fill=AXIS, anchor="e"
        )
        self.canvas.create_text(size / 2 + 12, 10, text="+Y", fill=AXIS, anchor="w")
        self.canvas.create_text(
            8,
            size - 8,
            text=f"{self.node.square_size * 100:.0f} x "
            f"{self.node.square_size * 100:.0f} cm  @  z = {self.node.home[2]:.3f} m",
            fill=AXIS,
            anchor="sw",
        )

    # ------------------------------------------------------------ drawing input

    def on_press(self, event):
        if self.busy:
            return
        self.clear_stroke()
        self.drawing = True
        self.stroke_px = [(event.x, event.y)]
        self.stroke_times = [time.monotonic()]

    def on_motion(self, event):
        if not self.drawing:
            return
        last = self.stroke_px[-1]
        self.canvas.create_line(
            last[0], last[1], event.x, event.y, fill=STROKE, width=3,
            capstyle=tk.ROUND, tags="stroke",
        )
        self.stroke_px.append((event.x, event.y))
        self.stroke_times.append(time.monotonic())

    def on_release(self, event):
        if not self.drawing:
            return
        self.drawing = False
        if len(self.stroke_px) < 2:
            self.set_status("That was a click, not a stroke. Draw a line.")
            return
        self.run_stroke()

    def clear_stroke(self):
        self.canvas.delete("stroke")
        self.stroke_px = []
        self.stroke_times = []

    # -------------------------------------------------------------- ROS actions

    def set_status(self, text):
        self.status.config(text=text)

    def _pump(self):
        """Drain messages posted by the worker thread onto the tkinter thread."""
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "status":
                    self.set_status(payload)
                elif kind == "busy":
                    self.busy = payload
                elif kind == "stiffness":
                    text, ok = payload
                    self.set_stiffness_status(text, ok)
                elif kind == "stiffness_values":
                    self._apply_stiffness_values(payload)
        except queue.Empty:
            pass
        self.root.after(100, self._pump)

    def _track_ee(self):
        """Show where the arm actually is, so the trace can be watched live."""
        position = self.node.ee_position()
        if position is not None:
            px, py = self.to_canvas(position[0], position[1])
            r = 5
            if self.ee_marker is None:
                self.ee_marker = self.canvas.create_oval(
                    px - r, py - r, px + r, py + r, outline=EE_MARKER, width=2
                )
            else:
                self.canvas.coords(self.ee_marker, px - r, py - r, px + r, py + r)
        self.root.after(100, self._track_ee)

    def post(self, text):
        self.events.put(("status", text))

    def in_worker(self, fn):
        """Run ``fn`` on a worker thread, holding the busy flag for its duration."""
        if self.busy:
            self.set_status("Busy: a motion is already running.")
            return

        def wrapper():
            self.events.put(("busy", True))
            try:
                fn()
            except Exception as exc:  # keep the GUI alive whatever goes wrong
                self.post(f"Error: {exc}")
            finally:
                self.events.put(("busy", False))

        self.busy = True
        threading.Thread(target=wrapper, daemon=True).start()

    def on_home(self):
        self.in_worker(self.go_home)

    def on_stop(self):
        if self.node.motion.cancel():
            self.set_status("Cancel sent.")
        else:
            self.set_status("Nothing running to cancel.")

    def go_home(self):
        """Move to the home position, unless already there."""
        home = self.node.home
        speed = self.node.get_parameter("approach_speed").value
        tolerance = self.node.get_parameter("home_tolerance").value

        position = self.node.wait_for_ee()
        if position is None:
            self.post(
                f"No TF {self.node.base_link} -> {self.node.ee_link}. "
                "Is the robot running?"
            )
            return False

        gap = distance(position, home)
        if gap <= tolerance:
            self.post(f"Already home ({gap * 1000:.0f} mm away). Draw a stroke.")
            return True

        self.post(f"Moving home, {gap:.3f} m away...")
        outcome = self.node.motion.move_to(home, speed=speed)
        if not outcome:
            self.post(f"Homing failed: {outcome.message}")
            return False
        self.post(f"Home. {outcome.message} Draw a stroke in the square.")
        return True

    def run_stroke(self):
        """Turn the drawn stroke into a path and hand it to the action server."""
        points_px = list(self.stroke_px)
        times = list(self.stroke_times)

        try:
            ratio = float(self.ratio_var.get())
            if ratio <= 0.0:
                raise ValueError
        except ValueError:
            self.set_status("Speed ratio must be a positive number.")
            return

        points = [self.to_robot(px, py) for px, py in points_px]

        radius = reachable_radius_xy(self.node.max_reach, self.node.home[2])
        if any(math.hypot(p[0], p[1]) > radius for p in points):
            self.set_status(
                "That stroke crosses the shaded out-of-reach corner. "
                "Redraw it inside the reachable area."
            )
            return

        # Mirror the server's limit so the user hears about a too-fast stroke
        # here, as an automatic slowdown, rather than as a rejected goal.
        max_speed = self.node.max_speed
        try:
            path, path_times, info = prepare_drawn_path(
                points,
                times,
                speed_ratio=ratio,
                max_speed=max_speed,
                min_spacing=self.node.get_parameter("min_spacing").value,
                smooth_window=int(self.node.get_parameter("smooth_window").value),
            )
        except ValueError as exc:
            self.set_status(f"Cannot use that stroke: {exc}")
            return

        self.in_worker(lambda: self.execute_path(path, path_times, info, ratio))

    def execute_path(self, path, path_times, info, ratio):
        speed = self.node.get_parameter("approach_speed").value

        summary = (
            f"{info['length'] * 100:.1f} cm in {info['waypoints']} waypoints, "
            f"drawn in {info['drawn_duration']:.1f} s, "
            f"replaying over {info['duration']:.1f} s at {ratio:g}x"
        )
        if info["extra_slowdown"] > 1.001:
            summary += (
                f" (slowed a further {info['extra_slowdown']:.1f}x to stay under "
                f"{self.node.max_speed:g} m/s)"
            )
        self.post(summary + ". Moving to the start...")

        outcome = self.node.motion.move_to(path[0], speed=speed)
        if not outcome:
            self.post(f"Could not reach the start of the stroke: {outcome.message}")
            return

        self.post("Tracing the stroke...")

        def on_feedback(feedback):
            self.events.put(
                (
                    "status",
                    f"Tracing... {feedback.percent_complete:.0f}%  "
                    f"{feedback.time_remaining:.1f} s left",
                )
            )

        outcome = self.node.motion.follow_path(path, path_times, feedback=on_feedback)
        if outcome:
            self.post(f"Done. {outcome.message} Draw another stroke.")
        else:
            self.post(f"Path stopped: {outcome.message}")

    # ------------------------------------------------------------- stiffness

    def _on_stiffness_edit(self, _index):
        """Restart the debounce timer; the push happens once typing settles."""
        if getattr(self, "_stiffness_loading", False):
            return
        self._stiffness_dirty = True
        if self._stiffness_push_job is not None:
            self.root.after_cancel(self._stiffness_push_job)
        delay = int(self.node.get_parameter("stiffness_push_delay_ms").value)
        self._stiffness_push_job = self.root.after(delay, self._push_stiffness)

    def _read_stiffness_fields(self):
        """Parse all six fields. Returns (values, error) with error None if ok."""
        values = []
        for i, var in enumerate(self.stiffness_vars):
            text = var.get().strip()
            if text in ("", "-", ".", "--"):
                return None, f"{STIFFNESS_LABELS[i]} is empty"
            try:
                value = float(text)
            except ValueError:
                return None, f"{STIFFNESS_LABELS[i]} is not a number"
            if value < 0.0:
                return None, f"{STIFFNESS_LABELS[i]} must be >= 0"
            limit = (
                self.node.get_parameter("max_stiffness_lin").value
                if i < 3
                else self.node.get_parameter("max_stiffness_rot").value
            )
            if value > limit:
                return None, f"{STIFFNESS_LABELS[i]} must be <= {limit:g}"
            values.append(value)
        return values, None

    def _push_stiffness(self):
        """Send the six fields to the controller, on a worker thread."""
        if self._stiffness_push_job is not None:
            self.root.after_cancel(self._stiffness_push_job)
            self._stiffness_push_job = None

        values, error = self._read_stiffness_fields()
        if error is not None:
            self.set_stiffness_status(error, ok=False)
            return
        if values == self._stiffness_applied:
            self._stiffness_dirty = False
            return

        def send():
            ok, reason = self.node.stiffness.set(values)
            if ok:
                self._stiffness_applied = values
                self._stiffness_dirty = False
                self.events.put(
                    (
                        "stiffness",
                        (
                            "Applied  "
                            + " ".join(f"{v:g}" for v in values[:3])
                            + "  /  "
                            + " ".join(f"{v:g}" for v in values[3:]),
                            True,
                        ),
                    )
                )
            else:
                self.events.put(("stiffness", (reason, False)))

        threading.Thread(target=send, daemon=True).start()

    def set_stiffness_status(self, text, ok=True):
        self.stiffness_status.config(text=text, fg=AXIS if ok else "#e0736f")

    def on_reload_stiffness(self):
        threading.Thread(target=self.load_stiffness, daemon=True).start()

    def load_stiffness(self):
        """Read the controller's current stiffness into the fields."""
        if not self.node.stiffness.wait_for_service(timeout_sec=5.0):
            self.events.put(
                (
                    "stiffness",
                    (
                        f"{self.node.stiffness.controller} has no parameter "
                        "service. Is the controller active?",
                        False,
                    ),
                )
            )
            return
        values = self.node.stiffness.get()
        if values is None:
            self.events.put(
                ("stiffness", ("Could not read the stiffness parameters.", False))
            )
            return
        self.events.put(("stiffness_values", values))

    def _apply_stiffness_values(self, values):
        """Fill the fields without triggering a push back to the controller."""
        self._stiffness_loading = True
        try:
            for var, value in zip(self.stiffness_vars, values):
                var.set(f"{value:g}")
        finally:
            self._stiffness_loading = False
        self._stiffness_applied = list(values)
        self._stiffness_dirty = False
        self.set_stiffness_status("In sync with the controller.")

    # ----------------------------------------------------------------- lifecycle

    def start(self):
        def startup():
            self.load_stiffness()
            self.post("Waiting for cartesian_move_server...")
            if not self.node.motion.wait_for_servers(timeout_sec=15.0):
                self.post(
                    "cartesian_move_server is not up. Start it with:  ros2 launch "
                    "kuka_control_tutorial move_to_pose.launch.py"
                )
                return
            self.go_home()

        self.in_worker(startup)
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = DrawTrajectoryNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = DrawTrajectoryApp(node)
    try:
        app.start()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
