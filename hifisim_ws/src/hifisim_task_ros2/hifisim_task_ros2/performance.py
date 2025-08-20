#
# performance.py
# Coordinate system used in this class is NWU
# 
import time
import rclpy
from rclpy.node import Node
from hifisim_msg_ros2.msg import MissionResult
from hifisim_msg_ros2.srv import SimulationSetupService
from hifisim_msg_ros2.msg import CollisionCrash
from geometry_msgs.msg import PoseStamped
from .read_testcase import *
from .sqlite_logger import SQLiteLogger
from .performance_plotter import PerformancePlotterProcess
from .data_path import *
from builtin_interfaces.msg import Time
from .entrance_plane import Plane
import multiprocessing

# Add module-level constant definitions
MIN_LOG_DISTANCE = 0.1
MIN_PLOT_DISTANCE = 1.5
ENDPOINT_TOLERANCE = 6.0
ENTRANCE_SHIFT = 2.5

class Performance(Node):
    class State:
        SIM_UNINITIALIZED = 'SIM_UNINITIALIZED'
        SIM_INITIALIZED = 'SIM_INITIALIZED'
        SIM_START = 'SIM_START'
        SIM_END = 'SIM_END'

    def __init__(self):
        super().__init__('performance_node')
        self.get_logger().info('performance_node started')
        self.min_log_distance = MIN_LOG_DISTANCE  # Default minimum distance for processing NWU pose data
        self.min_plot_distance = MIN_PLOT_DISTANCE  # Default minimum distance for plotting NWU pose data
        self.endpoint_tolerance = ENDPOINT_TOLERANCE  # Default endpoint tolerance for distance check
        self.sqlite_logger = None  # SQLiteLogger object
        self.team_id = None  # Team ID
        self.env_id = None  # Environment ID
        self.scenario_id = None  # Scenario ID
        self.last_positions = {}  # To store the last logged position of each agent
        self.last_plot_positions = {}  # To store the last plotted position of each agent
        self.state = self.State.SIM_UNINITIALIZED
        self.total_agents = 0 # Total number of agents including UAV and GCS
        self.num_UAV_agents = 0   # Number of UAV agents
        self.num_GCS_agents = 0 # Number of GCS agents
        self.agent_subscriptions = {}  # To track subscriptions for agents
        self.plot_data = {}  # Will initialize once logging is set up
        self.signposts_nwu_pos = []  # Will initialize once logging is set up
        self.endpoint_nwu_pos = None  # Will initialize once logging is set up
        self.signposts_plane = []  # Initialize the list to store Plane objects
        self.entrances_plane = []  # Will initialize once logging is set up
        self.entrance_shift = ENTRANCE_SHIFT  # Default shift distance from signpost to entrance center
        self.entrance_pass_state = "far" # Default entrance pass state
        self.nwu_pose_data = {}  # Initialized as an empty dictionary.
        self.sim_start_time = None  # Track SIM_START time
        self.plot_queue = multiprocessing.Queue()
        self.plotter_process = None  # Will initialize once logging is set up
        self.point_idx = {}  # <-- Add this line to initialize point_idx
        self.run_fsm()

    def initialize_results(self):
        results = {
            'agent_is_active': [],  # True/False [index is agent_id - 1], total index is num_UAV_agents
            'agent_is_crash': [],   # True/False [index is agent_id - 1], total index is num_UAV_agents
            'agent_num_collision': [],  # int [index is agent_id - 1], total index is num_UAV_agents
            'agent_entrance_pass': [],  # True/False [index is agent_id - 1][index is entrance_id - 1]
            'agent_is_completed': [],   # True/False [index is agent_id - 1], total index is num_UAV_agents
            'agent_start_time': [],
            'agent_end_time': [],
            'agent_duration': [],
            'team_completed': False,
            'team_start_time': Time(),
            'team_end_time': Time(),
            'team_duration': 0.0,
            'metric': {
                'team_time': 0,
                'time_gap': 0,
                'completed': 0,
                'collision': 0,
                'crashed': 0
            }
        }
        return results

    def run_fsm(self):
        if self.state == self.State.SIM_UNINITIALIZED:
            testcase_id = self.send_sim_setup_request()
            self.setup_performance_logging(testcase_id)
            self.state = self.State.SIM_INITIALIZED
            self.get_logger().info(f'State changed to: {self.state}')

        if self.state == self.State.SIM_INITIALIZED:
            # Request_Start_Sim. Wait for response from hifi_simulator_unity before proceeding
            self.request_start_sim()
            self.state = self.State.SIM_START
            self.sim_start_time = time.time()  # Record wall time at SIM_START
            self.get_logger().info(f'State changed to: {self.state}')

    def setup_performance_logging(self, testcase_id):
        self.get_logger().info(f'Setting up performance logging of testcase_id: {testcase_id}')
        testcase_struct = read_testcase_struct(testcase_id)
        self.get_logger().info(f"Read_Testcase> Testcase ID: {testcase_struct['Testcase']['TestcaseId']}")
        self.get_logger().info(f"Read_Testcase> Description: {testcase_struct['Testcase']['Description']}")
        self.get_logger().info(f"Read_Testcase> Team ID: {testcase_struct['Testcase']['TeamId']}")
        self.get_logger().info(f"Read_Testcase> Environment ID: {testcase_struct['Testcase']['EnvId']}")
        self.get_logger().info(f"Read_Testcase> Env Scenario ID: {testcase_struct['Testcase']['EnvScenarioId']}")
        self.team_id = testcase_struct['Testcase']['TeamId']
        self.env_id = testcase_struct['Testcase']['EnvId']
        self.scenario_id = testcase_struct['Testcase']['EnvScenarioId']

        # Initialize scenario and signpost data
        scenario_corners = read_scenario_corners_from_env(self.env_id, self.scenario_id)

        # get all the signposts position and rotation (The last signpost is the endpoint)
        signposts_endpoint_nwu_pos_rot = get_signposts_endpoint_nwu_pos_rot_env(self.env_id, self.scenario_id)
        for i, data in enumerate(signposts_endpoint_nwu_pos_rot):
            self.get_logger().info(
                f"Signpost {i + 1}: Position (X: {data['position'].x:.1f}, Y: {data['position'].y:.1f}, Z: {data['position'].z:.1f}), "
                f"Rotation (X: {data['rotation'].x:.1f}, Y: {data['rotation'].y:.1f}, Z: {data['rotation'].z:.1f})"
            )

        # Extract signposts position except the last one, which is the endpoint
        self.signposts_nwu_pos = get_signposts_nwu_pos(signposts_endpoint_nwu_pos_rot)

        # Extract endpoint position from the last item in the list
        self.endpoint_nwu_pos = get_endpoint_nwu_pos(signposts_endpoint_nwu_pos_rot)

        # Store the available agent IDs
        available_UAV_agent_ids = read_available_UAV_agent_ids(testcase_struct)
        self.num_UAV_agents = len(available_UAV_agent_ids)
        available_GCS_agent_ids = read_available_GCS_agent_ids(testcase_struct)
        self.num_GCS_agents = len(available_GCS_agent_ids)
        self.total_agents = self.num_UAV_agents + self.num_GCS_agents
        
        self.get_logger().info(f"<Read_Testcase> Total_Number of agents: {self.total_agents}")
        self.get_logger().info(f"<Read_Testcase> Total_Number of GCS agents: {self.num_GCS_agents}")
        self.get_logger().info(f"<Read_Testcase> Total_Number of UAV agents: {self.num_UAV_agents}")
        self.get_logger().info(f"<Read_Testcase> Available UAV Agent IDs: {available_UAV_agent_ids}")

        # Initialize results data structure for each agent
        self.results = self.initialize_results()  # Initialize results data structure
        self.results['agent_is_active'] = [True] * self.num_UAV_agents
        self.results['agent_is_crash'] = [False] * self.num_UAV_agents
        self.results['agent_num_collision'] = [0] * self.num_UAV_agents
        self.results['agent_entrance_pass'] = [[False] * len(self.signposts_nwu_pos) for _ in range(self.num_UAV_agents)]
        self.results['agent_is_completed'] = [False] * self.num_UAV_agents
        self.results['agent_start_time'] = [self.get_clock().now().to_msg()] * self.num_UAV_agents
        self.results['agent_end_time'] = [Time()] * self.num_UAV_agents
        self.results['agent_duration'] = [0.0] * self.num_UAV_agents
        self.results['team_start_time'] = self.get_clock().now().to_msg()
                                            
        # Create subscription for MissionResult topics
        self.create_subscription(
            MissionResult,
            'GCS/mission_results',
            self.mission_result_callback,
            10
        )

        # Create subscriptions for /agentXXX/global/nwu_pose topics
        ## Setup including GCS agent so that GCS agents pose can be logged and plotted
        for agent_id in range(1, self.num_UAV_agents + self.num_GCS_agents): 
            topic_name = f'/agent{agent_id:03d}/global/nwu_pose'
            self.agent_subscriptions[agent_id] = self.create_subscription(
                PoseStamped,
                topic_name,
                lambda msg, agent_id=agent_id: self.nwu_pose_callback(msg, agent_id),
                100
            )
            self.get_logger().info(f'Subscribed to {topic_name}')

        # Create subscriptions for /agentXXX/collision_crash topics
        for agent_id in range(1, self.num_UAV_agents): ## Setup excluding GCS agent
            topic_name = f'/agent{agent_id:03d}/collision_crash'
            self.create_subscription(
                CollisionCrash,
                topic_name,
                self.collision_crash_callback,
                10                
            )
            self.get_logger().info(f'Subscribed to {topic_name}')

        # Initialize the signposts_plane list with Plane objects
        self.signposts_plane = self.get_signposts_plane(signposts_endpoint_nwu_pos_rot)
        for idx, plane in enumerate(self.signposts_plane):
            point_formatted = [f"{value:.1f}" for value in plane.point.tolist()]
            normal_formatted = [f"{value:.1f}" for value in plane.normal.tolist()]
            signpost_id = idx + 1
            self.get_logger().info(f"Signpost {signpost_id} Plane(point={point_formatted}, normal={normal_formatted})")

        # Compute entrances_plane list from signposts_plane list
        self.entrances_plane = self.get_entrances_plane(self.signposts_plane, self.entrance_shift)
        for idx, plane in enumerate(self.entrances_plane):
            point_formatted = [f"{value:.1f}" for value in plane.point.tolist()]
            normal_formatted = [f"{value:.1f}" for value in plane.normal.tolist()]
            entrance_id = idx + 1
            self.get_logger().info(f"Entrance {entrance_id} Plane(point={point_formatted}, normal={normal_formatted})")

        # Initialize SQLiteLogger
        self.sqlite_logger = SQLiteLogger()
        self.sqlite_logger.setup_logging()
        self.get_logger().info(f"All databases initialized in directory: {self.sqlite_logger.log_dir}")

        # Initialize PerformancePlotter
        # Based on EUN , First point at upper left, go clockwise, xl, xu, yl, yu in plot coordinate system
        self.plot_data["testcase_description"] = testcase_struct['Testcase']['Description']
        self.plot_data["xl"] = scenario_corners[3].x  # lower left corner
        self.plot_data["xu"] = scenario_corners[1].x  # upper right corner
        self.plot_data["yl"] = scenario_corners[3].z  # lower left corner
        self.plot_data["yu"] = scenario_corners[1].z  # upper right corner
        self.get_logger().info(f"Scenario Corners for testcase_id: {testcase_id}")
        self.get_logger().info(f'X-Lower: = ({self.plot_data["xl"]}), X-Upper: = ({self.plot_data["xu"]})')
        self.get_logger().info(f'Y-Lower: = ({self.plot_data["yl"]}), Y-Upper: = ({self.plot_data["yu"]})')
        self.plot_data["scenario_image_path"] = get_scenario_png_path(self.env_id, self.scenario_id)
        self.plot_data["signpost_image_path"] = get_signpost_image_path()
        self.plot_data["endpoint_image_path"] = get_endpoint_image_path()
        self.plot_data["team_id"] = self.team_id
        self.plot_data["num_UAV_agents"] = self.num_UAV_agents
        
        self.plotter_process = PerformancePlotterProcess(
            self.plot_data, self.signposts_nwu_pos, self.endpoint_nwu_pos, self.plot_queue
        )
        self.plotter_process.start()

    def mission_result_callback(self, msg):
        if self.state != self.State.SIM_START:
            return
        if msg.agent_id < 0 or msg.agent_id > self.num_UAV_agents:
            self.get_logger().warning(f"Received mission result for invalid agent_id: {msg.agent_id}")
            return
        if msg.result_type == 'completed':
            if msg.agent_id != 0: # One agent complete
                if self.results['agent_is_active'][msg.agent_id - 1] and \
                   not self.results['agent_is_completed'][msg.agent_id - 1]:
                    if self.agent_pass_entrances(msg.agent_id):
                        last_position = self.last_positions.get(msg.agent_id, None)
                        end_distance = self.distance_to_endpoint(last_position)
                        if end_distance < self.endpoint_tolerance:
                            self.get_logger().info(f'Agent {msg.agent_id} completed the mission with distance to endpoint: {end_distance:.3f}')
                            self.results['agent_is_completed'][msg.agent_id - 1] = True
                            self.results['agent_end_time'][msg.agent_id - 1] = msg.stamp
                            self.results['agent_duration'][msg.agent_id - 1] \
                               = Performance.compute_duration(self.results['agent_start_time'][msg.agent_id - 1], msg.stamp)
                            self.results['metric']['completed'] += 1
                            # self.print_nwu_pose_data(msg.agent_id)
                        else:
                            self.get_logger().info(f'Agent {msg.agent_id} completed the mission but distance to endpoint: {end_distance:.3f} is not within tolerance')

            elif msg.agent_id == 0: # All agents completed 
                self.get_logger().info('Received mission_result: all agents COMPLETED and checking.....')
                if self.check_team_completed():
                    self.results['team_completed'] = True
                    self.results['team_end_time'] = msg.stamp             
                    self.results['team_duration'] \
                       = Performance.compute_duration(self.results['team_start_time'], msg.stamp)
                    # Update the metric for the team
                    self.results['metric']['team_time'] = int(round(self.results['team_duration']))
                    self.update_metric_time_gap()   
                    self.state = self.State.SIM_END
                    self.get_logger().info(f'State changed to: {self.state}')
                    self.print_all_nwu_pose_data()
                    self.check_metric()
                else:
                    self.get_logger().info('All agents mission not completed') 

        elif msg.result_type == 'ooi_found':
            self.get_logger().info(f'OOI ID: {msg.ooi_id} found at: x={msg.ooi_location.x}, y={msg.ooi_location.y}, z={msg.ooi_location.z}')
        
        if self.plotter_process is not None and self.plotter_process.is_alive():
            self.plot_queue.put({
                "type": "metric",
                "metric": self.results['metric']
            })
        self.print_results()

    def agent_pass_entrances(self, agent_id):
        """
        For a given agent, retrieve pose data from the SQLite database (via sqlite_logger),
        then sequentially evaluate each entrance in self.entrances_plane.
        
        For each entrance, starting from the agent’s start time, iterate through the retrieved
        poses until the pass criteria are met. Once the criteria for an entrance are met,
        update the time pointer and for the next entrance, resume processing from the last pose.
        
        Returns:
            bool: True if the agent passes all entrances; False otherwise.
        """
        self.get_logger().info(f'Agent {agent_id} processing entrances using database poses')
        
        # Retrieve agent poses from the database.
        # Each pose dict contains: 
        #   'pose_time': float, 
        #   'position': (x, y, z),
        #   'orientation': (x, y, z, w)
        agent_poses = self.sqlite_logger.retrieve_agent_poses(agent_id)
        # self.print_agent_poses(agent_poses)
        if not agent_poses:
            self.get_logger().info(f'No pose data retrieved for Agent {agent_id}')
            return False

        # Set initial time_pointer using the agent's start time.
        start_time_msg = self.results['agent_start_time'][agent_id - 1]
        time_pointer = start_time_msg.sec + start_time_msg.nanosec / 1e9

        # Use an index into agent_poses so we don't always process from the start.
        pose_idx = 0

        # Iterate over each entrance.
        for idx, plane in enumerate(self.entrances_plane):
            self.entrance_pass_state = "far"  # Reset the entrance pass state
            entrance_id = idx + 1
            success = False

            self.get_logger().info(
                f'Checking entrance {entrance_id} for Agent {agent_id} starting at time {time_pointer:.3f}'
            )

            # Advance through poses from the current pose index.
            while pose_idx < len(agent_poses):
                pose = agent_poses[pose_idx]
                pose_time = pose['pose_time']
                if pose_time < time_pointer:
                    pose_idx += 1
                    continue  # Skip poses that occurred before the current time_pointer.

                # Check if the current pose passes the entrance criteria.
                success = self.check_pass_criteria(pose_idx, pose, plane, entrance_id)
                if success:
                    # Entrance passed: update results and time_pointer.
                    self.results['agent_entrance_pass'][agent_id - 1][entrance_id - 1] = True
                    time_pointer = pose_time
                    self.get_logger().info(
                        f'Entrance {entrance_id} passed at time {time_pointer:.3f}'
                    )
                    # Do not increment pose_idx so that the same pose is reused for the next entrance.
                    break
                else:
                    # Move to the next pose if criteria were not met.
                    pose_idx += 1

            if not success:
                self.get_logger().info(f'Agent {agent_id} failed to pass entrance {entrance_id}')
                return False

        return True

    def check_pass_criteria(self, pose_idx, pose, plane, entrance_id):
        """
        Return True after passing the entrance otherwise return False.
        Parameters:
            pose_idx (int): The index of the current pose in the agent_poses list.
            pose (dict): A dictionary containing keys 'pose_time', 'position', 'orientation'.
            entrance_id (int): The current entrance id being evaluated.
            plane (Plane): The Plane object representing the entrance.
        Returns:
            bool: True aftering passing the entrance.
        """
        is_pass = False

        # Log the current pose & the entrance_plane
        # self.get_logger().info(
        #     f"    In check_pass_criteria(): Time {pose['pose_time']:.3f}: "
        #     f"Pos (X: {pose['position'].x:.1f}, Y: {pose['position'].y:.1f}, Z: {pose['position'].z:.1f}), "
        #     f"Ort (X: {pose['orientation'].x:.1f}, Y: {pose['orientation'].y:.1f}, "
        #     f"Z: {pose['orientation'].z:.1f}, W: {pose['orientation'].w:.1f})"
        # )
        # point_formatted = [f"{value:.1f}" for value in plane.point.tolist()]
        # normal_formatted = [f"{value:.1f}" for value in plane.normal.tolist()]
        # self.get_logger().info(f"    In check_pass_criteria(): Entrance {entrance_id} Plane(point={point_formatted}, normal={normal_formatted})")

        # Convert the Point to a tuple (x, y, z) for distance calculations.
        pos_tuple = (pose['position'].x, pose['position'].y, pose['position'].z)
        
        # The front side of the plane is in the direction the normal vector is pointing.
        # perp_distance > 0: point is on the front side (normal direction)
        # perp_distance < 0: point is on the back side (opposite normal)
        perp_distance = plane.perp_distance_to_pt(pos_tuple)
        center_distance = plane.center_distance_to_pt(pos_tuple)
        end_distance = self.distance_to_endpoint(pose['position'])

        match self.entrance_pass_state:
            case "far":
                if center_distance < self.entrance_shift*2 and perp_distance > 0.0:  # Check if the agent is on the correct side of the entrance
                    self.entrance_pass_state = "approach"
            case "approach":
                if perp_distance < -0.1:
                    self.entrance_pass_state = "pass"
                    is_pass = True 
            case "pass":
                if perp_distance < -1.0:
                    is_pass = True
            case _:
                # Default case if state is unrecognized
                self.get_logger().warning("State: Unknown — No action defined.")

        if self.entrance_pass_state != "far" and perp_distance < 1.0:
            self.get_logger().info(f"  Pose {pose_idx + 1}: Perp_dist: {perp_distance:.3f}, " \
                    f"Center_dist: {center_distance:.3f}, Entrance Pass State: {self.entrance_pass_state}, " \
                    f"Endpoint_dist: {end_distance:.3f}")

        return is_pass

    def check_team_completed(self):
        # Check completion criteria for all agents
        for agent_id in range(1, self.num_UAV_agents + 1):
            if not self.results['agent_is_completed'][agent_id - 1] and \
               self.results['agent_duration'][agent_id - 1] < 0.01:
                return False
            last_position = self.last_positions.get(agent_id, None)
            end_distance = self.distance_to_endpoint(last_position)
            if end_distance > self.endpoint_tolerance:
                return False
                
        self.get_logger().info(f'All Agents completed mission')    
        return True

    def nwu_pose_callback(self, msg, agent_id):
        if self.state != self.State.SIM_START:
            return

        # Extract current position
        current_position = msg.pose.position
        last_position = self.last_positions.get(agent_id, None)
        last_plot_position = self.last_plot_positions.get(agent_id, None)

        # Print info if agent_id == 1: tmp for testing
        # if agent_id == 1:
        #     self.get_logger().info(
        #         f"{msg.header.stamp.sec} {msg.header.frame_id} "
        #         f"{current_position.x:7.2f} {current_position.y:7.2f} {current_position.z:7.2f}"
        #     )

        # If no last position, log the first message and store position
        log_data = False
        if last_position is None:
            self.last_positions[agent_id] = current_position
            log_data = True
        else:
            # Compute the differences along each axis
            dx = abs(current_position.x - last_position.x)
            dy = abs(current_position.y - last_position.y)
            dz = abs(current_position.z - last_position.z)

            # Log only if movement exceeds the threshold along any axis
            if dx >= self.min_log_distance or dy >= self.min_log_distance or dz >= self.min_log_distance:
                self.last_positions[agent_id] = current_position
                log_data = True

        to_plot_data = False
        if last_plot_position is None:
            self.last_plot_positions[agent_id] = current_position
            to_plot_data = True
        else:
            # Compute the differences along each axis
            dx = abs(current_position.x - last_plot_position.x)
            dy = abs(current_position.y - last_plot_position.y)
            dz = abs(current_position.z - last_plot_position.z)

            # Plot only if movement exceeds the threshold along any axis
            if dx >= self.min_plot_distance or dy >= self.min_plot_distance or dz >= self.min_plot_distance:
                self.last_plot_positions[agent_id] = current_position
                to_plot_data = True

        if self.results['agent_is_crash'][agent_id - 1] == True:
            log_data = False
            to_plot_data = False

        # If we decided to log the pose
        if log_data:
            self.sqlite_logger.log_pose(
                agent_id,
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                msg.header.frame_id,
                current_position,
                msg.pose.orientation
            )
            # Store the data in memory
            if agent_id not in self.nwu_pose_data:
                self.nwu_pose_data[agent_id] = []  # a new list is created and assigned to the key agent_id
            self.nwu_pose_data[agent_id].append({  # NWU Pose data are appended to the list corresponding to each `agent_id`
                'stamp_sec': msg.header.stamp.sec,
                'stamp_nanosec': msg.header.stamp.nanosec,
                'frame_id': msg.header.frame_id,
                'position': current_position,
                'orientation': msg.pose.orientation
            })

        # Send trajectory point in NWU to plotter process
        if self.plotter_process is not None and self.plotter_process.is_alive() and to_plot_data:
            # Initialize and increment point_idx per agent
            if agent_id not in self.point_idx:
                self.point_idx[agent_id] = 0
            self.point_idx[agent_id] += 1
            point_idx = self.point_idx[agent_id]
            # print(f"Send traj_point {point_idx} at {time.time()}", flush=True)
            self.plot_queue.put({
                "type": "traj_point",
                "point_idx": point_idx,
                "agent_id": agent_id,
                "x": current_position.x,
                "y": current_position.y
            })

    def collision_crash_callback(self, msg):
        if self.state != self.State.SIM_START:
            return
        collision_status = 'COLLISION' if msg.is_collision else ''
        crash_status = 'CRASHED' if msg.is_crashed else ''
        agent_id = msg.agent_id
        collision_crash_position = msg.position   # Store the collision_crash_position in NWU from msg
        self.get_logger().info(f'Agent {agent_id}: {collision_status}, {crash_status}')
        self.get_logger().info(f'         at: x={collision_crash_position.x}, y={collision_crash_position.y}, z={collision_crash_position.z}')

        if msg.is_collision:
            self.results['agent_num_collision'][agent_id - 1] += 1
            self.results['metric']['collision'] += 1
        elif msg.is_crashed:     
            if self.results['agent_is_active'][msg.agent_id - 1] and \
               not self.results['agent_is_completed'][msg.agent_id - 1]:   # If agent is active and is not completed 
                # End time is set to infinity when agent is crashed
                self.results['agent_end_time'][agent_id - 1] = Time(sec=2**31-1, nanosec=0)
                self.results['agent_is_active'][agent_id - 1] = False
                self.results['agent_is_crash'][agent_id - 1] = True
                self.results['metric']['crashed'] += 1

        # Log the collision/crash data
        self.sqlite_logger.log_collision_crash(
            agent_id,
            msg.stamp.sec,
            msg.stamp.nanosec,
            msg.is_collision,
            msg.is_crashed,
            collision_crash_position
        )

        if self.plotter_process is not None and self.plotter_process.is_alive():
            self.plot_queue.put({
                "type": "collision_crash",
                "agent_id": agent_id,
                "x": collision_crash_position.x,
                "y": collision_crash_position.y,
                "is_collision": msg.is_collision,
                "is_crashed": msg.is_crashed
            })
            self.plot_queue.put({
                "type": "metric",
                "metric": self.results['metric']
            })

    def send_sim_setup_request(self):
        client = self.create_client(SimulationSetupService, 'simulation_setup_service')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        request = SimulationSetupService.Request()
        request.service_msg = 'sim_setup'
        request.requester = 'Performance'
        
        while True:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None and future.result().success:
                self.get_logger().info(f'sim_setup_request Response: success={future.result().success}, testcase_id={future.result().testcase_id}')
                return future.result().testcase_id
            else:
                self.get_logger().error('sim_setup_request failed or unsuccessful, retrying in 4 seconds...')
                time.sleep(4)
        
    def request_start_sim(self):
        client = self.create_client(SimulationSetupService, 'simulation_setup_service')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        request = SimulationSetupService.Request()
        request.service_msg = 'request_start_sim'
        request.requester = 'Performance'

        while True:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None and future.result().success:
                self.get_logger().info(f'request_start_sim Response: success={future.result().success}')
                break
            else:
                self.get_logger().error('request_start_sim failed or unsuccessful, retrying in 4 seconds...')
                time.sleep(4)

    def destroy_node(self):
        # Clean up SQLiteLogger and close databases
        if hasattr(self, 'sqlite_logger'):
            self.sqlite_logger.close()
        if hasattr(self, 'plotter_process') and self.plotter_process is not None:
            self.plot_queue.put({"type": "shutdown"})
            self.plotter_process.join(timeout=2)
        super().destroy_node()

    def get_signposts_plane(self, signposts_endpoint_nwu_pos_rot):
        """
        Initialize the signposts_plane list with Plane objects.
        Each Plane object is created using the position and computed normal vector.
        The normal vector is computed from the roll, pitch, and yaw angles.
        The last item in the list is excluded as it represents the endpoint.
        
        Parameters:
            signposts_endpoint_nwu_pos_rot (list): List of dictionaries (or objects) each containing:
                'position': geometry_msgs/Point,
                'rotation': geometry_msgs/Point (representing roll, pitch, yaw in its x, y, z fields).
        
        Returns:
            list: List of Plane objects constructed using the position and computed normal.
        """
        signposts_plane = []
        for data in signposts_endpoint_nwu_pos_rot[:-1]:
            pos = data['position']
            rot = data['rotation']
            roll = rot.x
            pitch = rot.y
            yaw = rot.z
            normal = Plane.rpy_to_normal(roll, pitch, yaw)
            point = [pos.x, pos.y, pos.z]  # Convert position Point into a list
            plane = Plane(point=point, normal=normal)
            signposts_plane.append(plane)
        return signposts_plane

    # def get_signposts_nwu_pos(self, signposts_endpoint_nwu_pos_rot):
    #     """
    #     Extract signposts position from the list of dictionaries (or objects).  
    #     The last item in the list is excluded as it represents the endpoint.
    #     The rest are the signposts.
    #     Parameters:
    #         signposts_endpoint_nwu_pos_rot (list): List of dictionaries (or objects) each containing:
    #             'position': geometry_msgs/Point,
    #             'rotation': geometry_msgs/Point (representing roll, pitch, yaw in its x, y, z fields).

    #     Returns:
    #         list: List of geometry_msgs/Point objects representing the signposts positions.
    #     """
    #     # Extract signposts except the last one   
    #     signposts_nwu_pos = []
    #     for data in signposts_endpoint_nwu_pos_rot[:-1]:
    #         pos = data['position']
    #         signposts_nwu_pos.append(pos)
    #     return signposts_nwu_pos
    
    # def get_endpoint_nwu_pos(self, signposts_endpoint_nwu_pos_rot):
    #     """
    #     Extract endpoint position from the last item in the list.
    #     The last item in the list is the endpoint.
    #     The rest are the signposts.
    #     Parameters:
    #         signposts_endpoint_nwu_pos_rot (list): List of dictionaries (or objects) each containing:
    #             'position': geometry_msgs/Point,
    #             'rotation': geometry_msgs/Point (representing roll, pitch, yaw in its x, y, z fields).
    #     Returns:
    #         geometry_msgs/Point: The endpoint position.
    #     """
    #     # Extract endpoint position from the last item in the list
    #     if signposts_endpoint_nwu_pos_rot:
    #         endpoint_nwu_pos = signposts_endpoint_nwu_pos_rot[-1]['position']
    #     else:
    #         endpoint_nwu_pos = None       
    #     return endpoint_nwu_pos 

    def get_entrances_plane(self, signposts_plane, entrance_shift):
        """
        Create the entrances_plane list from the signposts_plane list.
        
        Parameters:
            signposts_plane (list): List of Plane objects representing the signposts.
            entrance_shift (float): Distance to shift the point on the plane in the left_vector direction.
        
        Returns:
            list: List of Plane objects representing the entrances.
        """
        entrances_plane = []  # Initialize the list to store Plane objects for entrances

        # Define the rotation matrix for a -90-degree yaw rotation
        roll = 0.0    # roll angle in degrees
        pitch = 0.0   # pitch angle in degrees
        yaw = -90.0   # yaw angle in degrees
        R = Plane.rpy_to_rotation_matrix(roll, pitch, yaw)
        # The rotation matrix R is used to rotate the normal vector of the plane
        # to get the left_vector direction for entrance plane computation.
        # Right-Hand Rule:
        # In the NWU coordinate system, looking from the top down (Z-axis pointing up),
        # a negative yaw (-90 degrees) corresponds to a clockwise rotation.
        # This rotation aligns the left_vector direction for entrance plane computation.

        for plane in signposts_plane:
            normal = plane.normal
            left_vector = R @ normal  # Rotate the normal vector to get the left_vector
            shifted_point = plane.point + entrance_shift * left_vector  # Shift the point by entrance_shift in the left_vector direction
            entrances_plane.append(Plane(point=shifted_point, normal=normal))  # Create a new Plane object with the shifted point and same normal

        return entrances_plane

    def distance_to_endpoint(self, pos):
        """
        Compute the Euclidean distance between the supplied pos and the endpoint.
        
        Parameters:
            pos (geometry_msgs.msg.Point): The last known position of an agent.
            
        Returns:
            float: Distance in the NWU coordinate system, or None if pos or endpoint is not available.
        """
        if pos is None or self.endpoint_nwu_pos is None:
            self.get_logger().info("Cannot compute distance: last position or endpoint not available.")
            return None

                # Compute Euclidean distance.
        dx = pos.x - self.endpoint_nwu_pos.x
        dy = pos.y - self.endpoint_nwu_pos.y
        dz = pos.z - self.endpoint_nwu_pos.z 
        distance = (dx**2 + dy**2 + dz**2) ** 0.5

        # self.get_logger().info(f"Agent Position: (X: {pos.x:.1f}, Y: {pos.y:.1f}, Z: {pos.z:.1f})")
        # self.get_logger().info(f"Endpoint Position: (X: {self.endpoint_nwu_pos.x:.1f}, Y: {self.endpoint_nwu_pos.y:.1f}, Z: {self.endpoint_nwu_pos.z:.1f})")
        # self.get_logger().info(f"Distance to endpoint: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}") 
        # self.get_logger().info(f"Distance to endpoint: {distance:.3f}")
        return distance

    def update_metric_time_gap(self):
        # Update the time_gap metric for the team using the difference of max and min agent durations
        durations = self.results['agent_duration']
        if durations:
            self.results['metric']['time_gap'] = int(round(max(durations) - min(durations)))
        else:
            self.results['metric']['time_gap'] = 0

    def check_metric(self):
        completed = 0
        collision = 0
        crashed = 0

        for idx in range(self.num_UAV_agents):
            if self.results['agent_is_completed'][idx]:
                completed += 1
            if self.results['agent_num_collision'][idx] > 0:
                collision += 1
            if self.results['agent_is_crash'][idx]:
                crashed += 1
        self.get_logger().info(f"Metric: {self.results['metric']}")
        self.get_logger().info(f"Completed: {completed}, Collision: {collision}, Crashed: {crashed}")

    @staticmethod
    def compute_duration(start_time, end_time):
        """
        Computes the duration in seconds between two builtin_interfaces.msg.Time messages.
        
        Parameters:
            start_time (builtin_interfaces.msg.Time): The start time.
            end_time (builtin_interfaces.msg.Time): The end time.
        
        Returns:
            float: Duration in seconds.
        """
        sec_diff = end_time.sec - start_time.sec
        nsec_diff = end_time.nanosec - start_time.nanosec
        return sec_diff + nsec_diff / 1e9

    def print_nwu_pose_data(self, agent_id):
        """
        Print the NWU pose data for a specific agent.
        
        Parameters:
            agent_id (int): The agent_id whose pose data is to be printed. agent_id is the key of the nwu_pose_data dictionary.
        """
        if agent_id in self.nwu_pose_data:
            self.get_logger().info(f"NWU Pose Data for Agent {agent_id}:")
            for pose in self.nwu_pose_data[agent_id]:
                self.get_logger().info(f"  T: {pose['stamp_sec']}.{pose['stamp_nanosec']}, "
                                       f"Fid: {pose['frame_id']}, "
                                       f"Pos: ({pose['position'].x:.1f}, {pose['position'].y:.1f}, {pose['position'].z:.1f}), "
                                       f"Ort: ({pose['orientation'].x:.1f}, {pose['orientation'].y:.1f}, {pose['orientation'].z:.1f}, {pose['orientation'].w:.1f})")
        else:
            self.get_logger().info(f"No NWU Pose Data for Agent {agent_id}")

    def print_all_nwu_pose_data(self):
        self.get_logger().info("All NWU Pose Data:")
        for agent_id, poses in self.nwu_pose_data.items():
            self.get_logger().info(f"Agent {agent_id}:")
            # for pose in poses:
            #     self.get_logger().info(f"  T: {pose['stamp_sec']}.{pose['stamp_nanosec']}, "
            #                            f"Fid: {pose['frame_id']}, "
            #                            f"Pos: ({pose['position'].x}, {pose['position'].y}, {pose['position'].z}), "
            #                            f"Ort: ({pose['orientation'].x}, {pose['orientation'].y}, {pose['orientation'].z}, {pose['orientation'].w})")

    def print_results(self):
        self.get_logger().info("ALL PERFORMANCE RESULTS:")
        self.get_logger().info(f"All Agents Completed: {self.results['team_completed']}, "
                               f"Overall Duration: {self.results['team_duration']}")

        for idx in range(self.num_UAV_agents):
            agent_id = idx + 1
            self.get_logger().info(f"Agent {agent_id}: Active: {self.results['agent_is_active'][agent_id-1]}, "
                                   f"Crash: {self.results['agent_is_crash'][agent_id-1]}, "
                                   f"Collide: {self.results['agent_num_collision'][agent_id-1]}, "
                                   f"Ent_pass: {self.results['agent_entrance_pass'][agent_id-1]}, "
                                   f"Compl: {self.results['agent_is_completed'][agent_id-1]},  "
                                   f"EndTime: {self.results['agent_end_time'][agent_id-1]},  "
                                   f"Duration: {self.results['agent_duration'][agent_id-1]}")

    def print_agent_poses(self, agent_poses):
        """
        Print all the content of the given agent_poses list.

        Each element in agent_poses is expected to be a dictionary with the following keys:
        - 'pose_time': float (in seconds),
        - 'position': a Point object with attributes x, y, z,
        - 'orientation': a Quaternion object with attributes x, y, z, w.    
        """
        self.get_logger().info("Agent Poses:")
        for idx, pose in enumerate(agent_poses):
            self.get_logger().info(
                f"Pose {idx + 1}: Time: {pose['pose_time']:.3f} s, "
                f"Position (X: {pose['position'].x:.1f}, Y: {pose['position'].y:.1f}, Z: {pose['position'].z:.1f}), "
                f"Orientation (X: {pose['orientation'].x:.1f}, Y: {pose['orientation'].y:.1f}, "
                f"Z: {pose['orientation'].z:.1f}, W: {pose['orientation'].w:.1f})"
            )


def main(args=None):
    rclpy.init(args=args)
    performance_node = Performance()
    rclpy.spin(performance_node)
    performance_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
