#
# performance_plotter.py
# Coordinate system used in this class is ENU for plotting
# Conversion from NWU to ENU is in this class 
# 
import matplotlib
matplotlib.rcParams['font.family'] = 'DejaVu Sans'
matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans']
import logging
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import threading
import time  # Add this import at the top
from geometry_msgs.msg import Point
import multiprocessing

logger = logging.getLogger(__name__)

class PerformancePlotter:
    def __init__(self, plot_data, signposts_nwu_pos, endpoint_nwu_position):
        # xl, xu, yl, yu received in ENU for plotting
        self.x_lower = int(plot_data["xl"])
        self.x_upper = int(plot_data["xu"])
        self.y_lower = int(plot_data["yl"])
        self.y_upper = int(plot_data["yu"])
        self.testcase_description = plot_data["testcase_description"]
        self.scenario_image_path = plot_data["scenario_image_path"]
        self.signpost_image_path = plot_data["signpost_image_path"]
        self.endpoint_image_path = plot_data["endpoint_image_path"]
        self.signposts_nwu_position = signposts_nwu_pos
        self.endpoint_nwu_position = endpoint_nwu_position
        self.team_id = plot_data["team_id"]
        self.num_UAV_agents = plot_data["num_UAV_agents"]
        self.metric_label = ["Team Time (sec)", "Time Gap  (sec)", "Completed Agents", "Collision", "Crashed"]

        # Use interactive mode for real-time updates
        plt.ion()

        self.fig, self.ax = plt.subplots(figsize=(16, 9), dpi=100)
        self.fig.canvas.manager.set_window_title("Agent Trajectories (NWU)")
        self.fig.subplots_adjust(left=0.25, right=0.95, top=0.95, bottom=0.05)

        self.agent_lines = {}
        self.colors = ["lime", "g", "b", "c", "m", "y", "k", "pink", "purple", "brown"]
        self.lock = threading.Lock()

        # Set labels
        self.ax.set_xlabel('X (East)')
        self.ax.set_ylabel('Y (North)')
        self.ax.set_title(self.testcase_description)

        # Fix the plotting area to the rounded limits
        self.ax.set_xlim(self.x_lower, self.x_upper)
        self.ax.set_ylim(self.y_lower, self.y_upper)

        # Create ticks every 20 units on the rounded limits
        x_ticks = list(range(self.x_lower, self.x_upper + 1, 10))
        y_ticks = list(range(self.y_lower, self.y_upper + 1, 10))
        self.ax.set_xticks(x_ticks)
        self.ax.set_yticks(y_ticks)
        self.ax.grid(True)

        # Draw axes x=0 and y=0
        self.ax.axhline(y=0, color='black', linewidth=1)
        self.ax.axvline(x=0, color='black', linewidth=1)

        # -------------------------------------------------------------------
        # Setting up performance display
        # -------------------------------------------------------------------
        # 1) Load the env image as background for the entire plot
        try:
            self.scenario_image = mpimg.imread(str(self.scenario_image_path))
        except FileNotFoundError:
            self.scenario_image = None
            logger.warning(f"[WARNING] {self.scenario_image_path} not found. Please ensure the image path is correct.")

        # 2) Load endpoint_image for endpoint symbol usage
        try:
            self.endpoint_img = mpimg.imread(str(self.endpoint_image_path))
        except FileNotFoundError:
            self.endpoint_img = None
            logger.warning(f"[WARNING] {self.endpoint_image_path} not found. Please ensure the image path is correct.")

        # 3) Load signpost_image for signpost display
        try:
            self.signpost_img = mpimg.imread(str(self.signpost_image_path))
        except FileNotFoundError:
            self.signpost_img = None
            logger.warning(f"[WARNING] {self.signpost_image_path} not found. Please ensure the image path is correct.")

        plt.show()

        # 4) Place the background image behind everything
        if self.scenario_image is not None:
            self.place_background()

        # 5) Place the signposts symbol at 
        # logger.warning(f"[plotter] Number of signposts = {len(self.signposts_nwu_position)}")
        for i, pos in enumerate(self.signposts_nwu_position):
            # logger.warning(f"[plotter] Signpost[{i}]: x={pos.x}, y={pos.y}, z={pos.z}")
            self.place_symbol(pos.x, pos.y, self.signpost_img, symbol_size=5.0, label=i+1)

        # 6) Place the endpoint symbol at self.endpoint_nwu_position
        if self.endpoint_nwu_position:
            self.place_symbol(endpoint_nwu_position.x, endpoint_nwu_position.y, self.endpoint_img, symbol_size=3.0)
        else:
            print("No endpoint to plot.")

        # 7) Create a dedicated scoreboard panel on the left.
        # Add a new axes that occupies the left 20% of the figure.
        self.scoreboard_ax = self.fig.add_axes([0.01, 0.05, 0.22, 0.90])
        self.scoreboard_ax.set_axis_off()
        self.draw_scoreboard_layout()

        # 8) Add a placeholder time text on top of the scoreboard panel.
        # Position it centered at the top of the scoreboard axis.
        self.time_text = self.scoreboard_ax.text(
            0.5, 0.96,
            "Time: [00:00]", ha='center', va='top',
            fontsize=30, color='yellow', backgroundcolor='black'
        )
        # 9) Initialize list for dynamic metric texts
        # Note: Updating the metric score is controlled by Performance class
        self.dynamic_metric_texts = []

    def place_background(self):
        """
        Fits the width of scenario_image from x_lower to x_upper
        and maintains the aspect ratio. The bottom edge of the image is placed at
        y_lower. The top may extend beyond y_upper if the image is tall.
        """
        if self.scenario_image is None:
            return

        # Determine total X range
        total_x_width = self.x_upper - self.x_lower

        # Retrieve image dimensions (height, width, channels)
        img_height, img_width, _ = self.scenario_image.shape

        # Image aspect ratio (height / width)
        img_aspect = img_height / img_width

        # Scale the image height to match the total_x_width
        scaled_height = total_x_width * img_aspect

        # The bottom edge is at y_lower
        # so the top edge is at y_lower + scaled_height
        img_bottom = self.y_lower
        img_top    = self.y_lower + scaled_height

        # Place the image with zorder=-1 so it goes behind all lines
        self.ax.imshow(
            self.scenario_image,
            extent=[self.x_lower, self.x_upper, img_bottom, img_top],
            aspect='equal',
            zorder=-1
        )
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def place_symbol(self, nwu_x_center, nwu_y_center, symbol_img, symbol_size=1.0, label=None):
        """
        Place the signpost or endpoint symbol at (x_center, y_center).
        symbol_size indicates how large the symbol should appear
        in the plot's coordinate units (e.g., 1.0 => 1m x 1m).
        """
        if symbol_img is None:
            return
        
        # Convert NWU to ENU and plot the points in real-time.
        # NWU to ENU conversion: x(ENU)  = -y (NWU), y(ENU) = x (NWU)
        x_center = - nwu_y_center
        y_center = nwu_x_center

        # The extent defines the bounding box in data coordinates:
        # [left, right, bottom, top]
        half_size = symbol_size / 2.0
        extent = [
            x_center - half_size,  # left
            x_center + half_size,  # right
            y_center - half_size,  # bottom
            y_center + half_size   # top
        ]

        # Place the endpoint symbol in front of background and lines
        # by using a higher zorder 10
        self.ax.imshow(symbol_img, extent=extent, zorder=10)

        # If a label is provided, add it on top of the symbol
        if label is not None:
            self.ax.text(
                x_center, y_center, str(label),
                color='red', fontsize=10,
                ha='center', va='center',
                zorder=11,  # Ensure the text is above the symbol
                fontweight='bold'
            )

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def draw_scoreboard_layout(self):
        """
        Draw the static layout of the scoreboard on the left panel. This creates a box
        with a black background and uses white text for labels and red for dynamic data.
        """
        # Clear the axis, set background black, and remove ticks
        self.scoreboard_ax.cla()
        self.scoreboard_ax.set_facecolor('black')
        self.scoreboard_ax.set_xlim(0, 1)
        self.scoreboard_ax.set_ylim(0, 1)
        self.scoreboard_ax.set_xticks([])
        self.scoreboard_ax.set_yticks([])

        # --- Current Team Section (two columns) ---
        self.scoreboard_ax.text(0.25, 0.85, "TEAM", 
                     ha='left', va='center',
                     fontsize=30, color='green', fontweight='bold')
        self.scoreboard_ax.text(0.75, 0.85, str(self.team_id),
                     ha='right', va='center',
                     fontsize=30, color='green', fontweight='bold')
        self.scoreboard_ax.text(0.34, 0.80, f"{self.num_UAV_agents} agents",
                 ha='left', va='center',
                 fontsize=16, color='white')
        
        # --- Current Team Metrics Section (first column label) ---
        for i in range(5):
            self.scoreboard_ax.text(0.05, 0.73 - i*0.08, self.metric_label[i], 
                         ha='left', va='center',
                         fontsize=16, color='white')

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def plot_traj_point(self, agent_id, nwu_x, nwu_y):
        # Convert NWU to ENU and plot the points in real-time.
        # NWU to ENU conversion: x(ENU)  = -y (NWU), y(ENU) = x (NWU)
        x = - nwu_y
        y = nwu_x
        with self.lock:
            if agent_id not in self.agent_lines:
                color = self.colors[(agent_id - 1) % len(self.colors)]
                line, = self.ax.plot(
                    [x], [y],
                    marker='*', markersize=3,
                    linestyle='-',
                    color=color,
                    label=f"Agent {agent_id}"
                )
                self.agent_lines[agent_id] = (line, [x], [y])
                # Update legend after adding a new line
                self.ax.legend()
            else:
                line, xs, ys = self.agent_lines[agent_id]
                xs.append(x)
                ys.append(y)
                line.set_xdata(xs)
                line.set_ydata(ys)

            # Note: No need to redraw the figure here. update_time() will redraw it every second.
            # Redrawing the figure here would slow down the plotting a lot. 

    def plot_collision_crash(self, agent_id, nwu_x, nwu_y, is_collision, is_crashed):
        """
        Plot a '*' symbol for the agent and overlay with collision/crash indicators.

        Parameters:
        - agent_id (int): Identifier for the agent.
        - nwu_x (float): NWU X coordinate.
        - nwu_y (float): NWU Y coordinate.
        - is_collision (bool): Flag indicating a collision.
        - is_crashed (bool): Flag indicating a crash.
        """
        # Convert NWU to ENU coordinates
        x = -nwu_y
        y = nwu_x

        # Get the agent's color
        color = self.colors[(agent_id - 1) % len(self.colors)]

        # Plot Eight Pointed star in agent's color with fontsize 18
        self.ax.text(
            x, y, "\u2739",
            color=color, fontsize=18,
            fontname='DejaVu Sans',
            ha='center', va='center',
            zorder=10
        )

        # Determine overlay based on collision or crash
        if is_crashed:
            overlay_color = 'red'
            overlay_size = 12
        elif is_collision:
            overlay_color = 'orange'
            overlay_size = 10
        else:
            overlay_color = None

        # Overlay Eight Pointed star in red or orange if crash/collision occurred
        if overlay_color:
            self.ax.text(
                x, y, "\u2739",
                color=overlay_color, fontsize=overlay_size,
                fontname='DejaVu Sans',
                ha='center', va='center',
                zorder=11  # Ensure overlay is above the agent's '*'
            )

        # Refresh the plot to display updates
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def update_metric(self, results_metric):
        """
        Update the scoreboard metrics with new values.
        """
        metric = [
            int(results_metric['team_time']),
            int(results_metric['time_gap']),
            int(results_metric['completed']),
            int(results_metric['collision']),
            int(results_metric['crashed'])
        ]

        # Remove previous dynamic texts
        for t in self.dynamic_metric_texts:
            t.remove()
        self.dynamic_metric_texts.clear()
        
        # --- Metrics Section (second columns) ---
        for i in range(5):
            if metric[i] != 0:
                txt = self.scoreboard_ax.text(0.95, 0.73 - i*0.08, str(metric[i]),
                         ha='right', va='center',
                         fontsize=32, color='red', fontweight='bold')
                self.dynamic_metric_texts.append(txt)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def update_time(self, time_val=None):
        """
        Update the time text on the scoreboard.
        If time_val is None, use system time since epoch (seconds).
        If time_val is provided, it is elapsed seconds since SIM_START.
        """
        if time_val is None:
            now = time.localtime()
            seconds = now.tm_hour * 3600 + now.tm_min * 60 + now.tm_sec
        else:
            seconds = int(time_val)
        minutes = (seconds // 60)
        seconds = seconds % 60
        formatted_time = f"{minutes:02}:{seconds:02}"

        # Update the time text
        self.time_text.set_text(f"Time: [{formatted_time}]")
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

class PerformancePlotterProcess(multiprocessing.Process):
    def __init__(self, plot_data, signposts_nwu_pos, endpoint_nwu_position, plot_queue):
        super().__init__()
        self.plot_data = plot_data
        self.signposts_nwu_pos = signposts_nwu_pos
        self.endpoint_nwu_position = endpoint_nwu_position
        self.plot_queue = plot_queue
        self.daemon = True

    def run(self):
        plotter = PerformancePlotter(self.plot_data, self.signposts_nwu_pos, self.endpoint_nwu_position)
        start_time = time.time()
        last_time_update = 0
        while True:
            now = time.time()
            # Update time every second
            if now - last_time_update >= 1.0:
                elapsed = int(now - start_time)
                plotter.update_time(elapsed)
                last_time_update = now
            try:
                msg = self.plot_queue.get(timeout=0.05)
            except Exception:
                continue
            if not isinstance(msg, dict):
                continue
            msg_type = msg.get("type")
            if msg_type == "shutdown":
                break
            elif msg_type == "traj_point":
                # print(f"Received traj_point {msg['point_idx']} at {time.time()}", flush=True)
                plotter.plot_traj_point(msg["agent_id"], msg["x"], msg["y"])
            elif msg_type == "collision_crash":
                plotter.plot_collision_crash(
                    msg["agent_id"], msg["x"], msg["y"],
                    msg["is_collision"], msg["is_crashed"]
                )
            elif msg_type == "metric":
                plotter.update_metric(msg["metric"])
            # ...add more message types as needed...