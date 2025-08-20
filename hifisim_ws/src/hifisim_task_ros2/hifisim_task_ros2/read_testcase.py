#!/usr/bin/env python3
# To read json testcase based on testcase_id
# to data structure testcase_structure
# Use Unity C# naming convention CamelCase because data read from Unity created json
## This python library is designed to be not dependent on ROS
import json
import numpy as np
import os
from pathlib import Path
from hifisim_task_ros2.data_path import *
from geometry_msgs.msg import Point
import math

def read_json(file_path):
    with open(file_path, 'r') as file:
        return json.load(file)

def read_testcase_path(testcase_id):
    # Get the USERDATA_PATH environment variable
    userdata_path = os.getenv('USERDATA_PATH', "")
    
    if userdata_path:
        base_path = Path(userdata_path)
    else:
        base_path = Path.home() / 'Unity/build/hifi_simulator_unity_Data/StreamingAssets/UserData/'

    testcase_path = base_path / f'TestCases/TestCase{testcase_id}'
    return testcase_path

def read_testcase_struct(testcase_id):
    """
    Reads a new format testcase from JSON files (TestCase.json, SimulatorSetting.json, AgentGroupX.json)
    and organizes the data into a testcase_struct.
    """
    testcase_path = read_testcase_path(testcase_id)
    testcase_file_path = testcase_path / 'TestCase.json'
    agent_group_paths = [testcase_path / f'AgentGroup{i+1}.json' for i in range(4)]
    simulator_setting_path = testcase_path / 'SimulatorSetting.json'

    # Read TestCase.json with new format
    if not testcase_file_path.exists():
        raise FileNotFoundError(f"TestCase file not found: {testcase_file_path}")
    
    testcase = read_json(testcase_file_path)

    # Read SimulatorSetting.json
    if not simulator_setting_path.exists():
        raise FileNotFoundError(f"SimulatorSetting file not found: {simulator_setting_path}")
    simulator_setting = read_json(simulator_setting_path)

    # Initialize testcase_struct
    testcase_struct = {
        'Testcase': {
            'TestcaseId': testcase_id,
            'Description': testcase['Description'],
            'TeamId': testcase['TeamId'],
            'EnvId': testcase['EnvId'],
            'EnvScenarioId': testcase['EnvScenarioId']
        },
        'SimulatorSetting': {
            'RadioCommOn': simulator_setting['RadioCommOn'],
            'RadioCommRange': simulator_setting['RadioCommRange'],
            'PerformanceOn': simulator_setting['PerformanceOn'],
            'SimDetectionOn': simulator_setting['SimDetectionOn'],
            'SimDetectionRange': simulator_setting['SimDetectionRange']
        },
        'AgentGroups': []
    }

    # Read each AgentGroup file (modified to use key 'CameraConfigs')
    for path in agent_group_paths:
        if path.exists():
            group = read_json(path)
            # Extract each camera config from the list into separate keys.
            camera_configs = group.get('CameraConfigs', [])
            keyed_camera_configs = {}
            for index, config in enumerate(camera_configs, start=1):
                keyed_camera_configs[f'CameraConfig{index}'] = config

            # Extract each sensor data from the list into separate keys.
            sensor_datas = group.get('SensorDatas', [])
            keyed_sensor_datas = {}
            for index, sensor in enumerate(sensor_datas, start=1):
                keyed_sensor_datas[f'SensorData{index}'] = sensor

            agent_data = {
                'CoordinateSystem': group.get('CoordinateSystem', ""),
                'IsGCSAgent': group.get('IsGCSAgent', None),
                'AgentType': group.get('AgentType', ""),
                'FlashConfig': group.get('FlashConfig', {}),
                'CameraConfigs': keyed_camera_configs,
                'SensorDatas': keyed_sensor_datas,
                'Agents': []
            }
            for agent in group.get('Agents', []):
                # Convert waypoints into a numpy array; assume keys remain unchanged
                waypoints = np.array([[wp['X'], wp['Y'], wp['Z'], wp['RX'], wp['RY'], wp['RZ']] 
                                      for wp in agent.get('Waypoints', [])])
                agent_info = {
                    'AgentId': agent.get('AgentId'),
                    'IpAddress': agent.get('IpAddress'),
                    'Waypoints': waypoints
                }
                agent_data['Agents'].append(agent_info)
            testcase_struct['AgentGroups'].append(agent_data)
    
    return testcase_struct

def read_agents_list(testcase_struct):
    """
    Read the agents list from the given testcase structure and remove the agent groups information.
    Combine the agents in all the agent groups into a single list of agents_list.    
    """
    agents_list = []

    for group in testcase_struct['AgentGroups']:
        for agent in group['Agents']:
            waypoints = []
            for waypoint in agent['Waypoints']:
                waypoints.append([
                    waypoint[0],  # X
                    waypoint[1],  # Y
                    waypoint[2],  # Z
                    waypoint[3],  # RX
                    waypoint[4],  # RY
                    waypoint[5]   # RZ
                ])
            agents_list.append({
                'AgentId': agent['AgentId'],
                'IpAddress': agent['IpAddress'],
                'Waypoints': waypoints
            })

    return agents_list

def read_scenario_struct(env_id, scenario_id):
    """
    Reads the scenario structure from a JSON file based on the given environment ID and scenario ID.

    Args:
        env_id (str): The environment ID used to determine the path to the JSON file.
        scenario_id (str): The scenario ID used to determine the path to the JSON file.

    Returns:
        dict: The scenario structure read from the JSON file.

    Raises:
        FileNotFoundError: If the JSON file does not exist at the specified path.

    """
    scenario_struct = None
    scenario_json_path = get_scenario_json_path(env_id, scenario_id)

    # Read the scenario_json_path file
    if not scenario_json_path.exists():
        raise FileNotFoundError(f"scenario_json_path file not found: {scenario_json_path}")
    else:
        # print(f"Environment scenario JSON path is \n{scenario_json_path}")
        scenario_struct = read_json(scenario_json_path)

    return scenario_struct

def read_obstacles_struct(scenario_struct):
    obstacles_struct = scenario_struct.get('ObstacleDatas', None)
    return obstacles_struct

def read_signposts_struct(scenario_struct):
    signposts_struct = scenario_struct.get('Signposts', None)
    return signposts_struct

def read_scenario_info_struct(scenario_struct):
    scenario_info_struct = scenario_struct.get('ScenarioInfo', None)
    return scenario_info_struct

def read_scenario_corners(scenario_info_struct):
    scenario_corners = []
    # Corrdinate stored in scenario_info['ScenarioCorners'] is in Unity EUN coordinate system
    for corner in scenario_info_struct['ScenarioCorners']:
        point = Point()
        point.x = corner.get('x', 0.0)
        point.y = corner.get('y', 0.0)
        point.z = corner.get('z', 0.0)
        scenario_corners.append(point)
    return scenario_corners

def read_scenario_corners_from_env(env_id, scenario_id):
    scenario_corners = []
    scenario_struct = read_scenario_struct(env_id, scenario_id)
    scenario_info_struct = read_scenario_info_struct(scenario_struct)
    scenario_corners = read_scenario_corners(scenario_info_struct)
    return scenario_corners

def get_signposts_eun_position(signposts_struct):
    # print("In get_signposts_eun_position")
    signposts_eun_position = []
    # print(signposts_struct, flush=True)
    if (signposts_struct is not None):
        for signpost in signposts_struct:
            # print(signpost, flush=True)
            for transform_data in signpost['TransformDatas']:
                position = Point()
                position.x = transform_data['Pos']['x']
                position.y = transform_data['Pos']['y']
                position.z = transform_data['Pos']['z']
                # print(f"In get_signposts_eun_position-Position: x={position.x}, y={position.y}, z={position.z}", flush=True)
                signposts_eun_position.append(position)
    else:
        print("No signposts found in the scenario", flush=True)

    return signposts_eun_position

def get_signposts_nwu_position(signposts_struct):
    # print("In get_signposts_nwu_position")
    signposts_nwu_position = []
    if (signposts_struct is not None):
        for signpost in signposts_struct:
            # print(signpost, flush=True)
            for transform_data in signpost['TransformDatas']:
                position = Point()
                # Convert from EUN to NWU
                position.x = transform_data['Pos']['z']
                position.y = - transform_data['Pos']['x']
                position.z = transform_data['Pos']['y']
                # print(f"In get_signposts_nwu_position-Position: x={position.x}, y={position.y}, z={position.z}", flush=True)
                signposts_nwu_position.append(position)
    else:
        print("No signposts found in the scenario", flush=True)

    return signposts_nwu_position

def quaternion_to_euler_y_only(qx, qy, qz, qw):
    """
    Convert a quaternion to Euler angles with rotation about the X and Z axes set to zero.
    
    Parameters:
    qx, qy, qz, qw -- the components of the quaternion
    
    Returns:
    rot_x -- the rotation about the X-axis in degrees (set to zero)
    rot_y -- the rotation about the Y-axis in degrees 
    rot_z -- the rotation about the Z-axis in degrees (set to zero)
    """
    # Calculate the yaw (rotation about the Y-axis)
    yaw = math.atan2(2.0 * (qw * qy - qx * qz), 1.0 - 2.0 * (qy * qy + qz * qz))
    
    # Convert yaw from radians to degrees
    rot_y = math.degrees(yaw)
    
    # Set rot_x and rot_z to zero
    rot_x = 0.0
    rot_z = 0.0
    
    return rot_x, rot_y, rot_z

def quaternion_to_euler(qx, qy, qz, qw):
    """
    Convert a quaternion into euler angles (rot_x, rot_y, rot_z)
    rot_x is rotation around x in degrees
    rot_y is rotation around y in degrees
    rot_z is rotation around z in degrees
    """
    t0 = +2.0 * (qw * qx + qy * qz)
    t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    rot_x = math.atan2(t0, t1)

    t2 = +2.0 * (qw * qy - qz * qx)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    rot_y = math.asin(t2)

    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    rot_z = math.atan2(t3, t4)

    # Convert from radians to degrees
    rot_x = math.degrees(rot_x)
    rot_y = math.degrees(rot_y)
    rot_z = math.degrees(rot_z)

    return rot_x, rot_y, rot_z  # in degrees

def get_signposts_endpoint_nwu_pos_rot_yaw(signposts_struct):
    """
    Conversion of signposts positions and rotations from EUN to NWU.
    Rotation in NWU is in aerospace convention
    Set roll and pitch to zero, assume signpost plane is upright and not tilted

    Returns:
        list: A list of dictionaries with keys 'position' and 'rotation',
              each containing a geometry_msgs/Point.
    """

    signposts_endpoint_nwu_pos_rot = []
    if (signposts_struct is not None):
        for signpost in signposts_struct:
            for transform_data in signpost['TransformDatas']:
                # Process position conversion from EUN to NWU
                position = Point()
                position.x = transform_data['Pos']['z']
                position.y = -transform_data['Pos']['x']
                position.z = transform_data['Pos']['y']

                # Process rotation conversion using quaternion_to_euler
                qx = transform_data['Rot']['x']
                qy = transform_data['Rot']['y']
                qz = transform_data['Rot']['z']
                qw = transform_data['Rot']['w']
                # Calculate the yaw (rotation about the Y-axis)
                # Set roll and pitch to zero, assume signpost plane is upright and not tilted
                yaw = math.atan2(2.0 * (qw * qy - qx * qz), 1.0 - 2.0 * (qy * qy + qz * qz))
                rot_y = math.degrees(yaw)  # rotation about y in degrees (Zero degrees faces the user looking at the signpost)
                rot_x = 0.0
                rot_z = 0.0

                # Create a rotation Point with aerospace convention (ZYX) which is based on NWU
                rotation = Point()
                rotation.x = -rot_z  # roll
                rotation.y = rot_x   # pitch
                rotation.z = -rot_y  # yaw

                signposts_endpoint_nwu_pos_rot.append({'position': position, 'rotation': rotation})
    else:
        print("No signposts found in the scenario", flush=True)

    return signposts_endpoint_nwu_pos_rot

def get_signposts_endpoint_nwu_pos_rot_env(env_id, scenario_id):
    scenario_struct = read_scenario_struct(env_id, scenario_id)
    signposts_struct = read_signposts_struct(scenario_struct)
    signposts_endpoint_nwu_pos_rot = get_signposts_endpoint_nwu_pos_rot_yaw(signposts_struct)

    return signposts_endpoint_nwu_pos_rot

def get_signposts_nwu_pos(signposts_endpoint_nwu_pos_rot):
    """
    Extract signposts position from the list of dictionaries (or objects).  
    The last item in the list is excluded as it represents the endpoint.
    The rest are the signposts.
    Parameters:
        signposts_endpoint_nwu_pos_rot (list): List of dictionaries (or objects) each containing:
            'position': geometry_msgs/Point,
            'rotation': geometry_msgs/Point (representing roll, pitch, yaw in its x, y, z fields).

    Returns:
        list: List of geometry_msgs/Point objects representing the signposts positions.
    """
    # Extract signposts except the last one   
    signposts_nwu_pos = []
    for data in signposts_endpoint_nwu_pos_rot[:-1]:
        pos = data['position']
        signposts_nwu_pos.append(pos)
    return signposts_nwu_pos

def get_endpoint_nwu_pos(signposts_endpoint_nwu_pos_rot):
    """
    Extract endpoint position from the last item in the list.
    The last item in the list is the endpoint.
    The rest are the signposts.
    Parameters:
        signposts_endpoint_nwu_pos_rot (list): List of dictionaries (or objects) each containing:
            'position': geometry_msgs/Point,
            'rotation': geometry_msgs/Point (representing roll, pitch, yaw in its x, y, z fields).
    Returns:
        geometry_msgs/Point: The endpoint position.
    """
    # Extract endpoint position from the last item in the list
    if signposts_endpoint_nwu_pos_rot:
        endpoint_nwu_pos = signposts_endpoint_nwu_pos_rot[-1]['position']
    else:
        endpoint_nwu_pos = None       
    return endpoint_nwu_pos 
    
def read_available_UAV_agent_ids(testcase_struct):
    """
    Get the available UAV_agent IDs from the testcase structure.
    """
    available_UAV_agent_ids = []
    agent_groups = testcase_struct.get('AgentGroups', [])
    for group in agent_groups:
        if group.get('IsGCSAgent', False):
            # Skip GCS agents
            continue
        agents = group.get('Agents', [])
        for agent in agents:
            available_UAV_agent_ids.append(agent.get('AgentId'))
    return available_UAV_agent_ids

def read_available_GCS_agent_ids(testcase_struct):
    """
    Get the available GCS agent IDs from the testcase structure.
    """
    available_GCS_agent_ids = []
    agent_groups = testcase_struct.get('AgentGroups', [])
    for group in agent_groups:
        if not group.get('IsGCSAgent', False):
            # Skip non-GCS agents
            continue
        agents = group.get('Agents', [])
        for agent in agents:
            available_GCS_agent_ids.append(agent.get('AgentId'))
    return available_GCS_agent_ids

def read_available_agent_ids(testcase_struct):
    agents_list = read_agents_list(testcase_struct)

    # Store the available agent IDs
    available_agent_ids = [agent['AgentId'] for agent in agents_list]
    return available_agent_ids

def main(args=None):
    testcase_id = input("Enter the Test Case number: ")
    try:
        # Use the new function to read testcase struct
        testcase_struct = read_testcase_struct(testcase_id)
        print("Testcase ID:", testcase_struct['Testcase']['TestcaseId'])
        print("Description:", testcase_struct['Testcase']['Description'])
        print("TeamId:", testcase_struct['Testcase']['TeamId'])
        print("Environment ID:", testcase_struct['Testcase']['EnvId'])
        print("Env Scenario ID:", testcase_struct['Testcase']['EnvScenarioId'])

        print("\nSimulator Setting:")
        print("  RadioCommOn:", testcase_struct['SimulatorSetting']['RadioCommOn'])
        print("  RadioCommRange:", testcase_struct['SimulatorSetting']['RadioCommRange'])
        print("  PerformanceOn:", testcase_struct['SimulatorSetting']['PerformanceOn'])
        print("  SimDetectionOn:", testcase_struct['SimulatorSetting']['SimDetectionOn'])
        print("  SimDetectionRange:", testcase_struct['SimulatorSetting']['SimDetectionRange'])
        
        for i, group in enumerate(testcase_struct['AgentGroups']):
            print(f"\nAgent Group {i + 1} CoordinateSystem:", group['CoordinateSystem'])
            print(f"Agent Group {i + 1} Is GCS Agent:", group['IsGCSAgent'])
            print(f"Agent Group {i + 1} Type:", group['AgentType'])
            print(f"Agent Group {i + 1} FlashConfig:", group['FlashConfig'])
            print(f"Agent Group {i + 1} FlashConfig-IsFlashOn:", group['FlashConfig']['IsFlashOn'])
            print(f"Agent Group {i + 1} FlashConfig-FlashColor:", group['FlashConfig']['FlashColor'])
            print(f"Agent Group {i + 1} FlashConfig-IsBlinky:", group['FlashConfig']['IsBlinky'])
            
            print(f"Agent Group {i + 1} Camera Configs:")
            for cam_key, cam_config in group.get('CameraConfigs', {}).items():
                print(f"  {cam_key}:")
                print(f"    CameraId: {cam_config.get('CameraId')}")
                print(f"    Description: {cam_config.get('Description')}")
                print(f"    CameraType: {cam_config.get('CameraType')}")
                print(f"    Orientation: {cam_config.get('Orientation')}")
                print(f"    ImageResolution: {cam_config.get('ImageResolution')}")
                print(f"    SensorSize: {cam_config.get('SensorSize')}")
                print(f"    FocalLength: {cam_config.get('FocalLength')}")
                print(f"    PublishFrequency: {cam_config.get('PublishFrequency')}")
                print(f"    TopicNames: {cam_config.get('TopicNames')}")
                print(f"    IsStereo: {cam_config.get('IsStereo')}")
                print(f"    IsFisheye: {cam_config.get('IsFisheye')}")
                print(f"    IsDepth: {cam_config.get('IsDepth')}")
                print(f"    IsSimDetectionOn: {cam_config.get('IsSimDetectionOn')}")
                print(f"    Baseline: {cam_config.get('Baseline')}")
                print(f"    Alpha: {cam_config.get('Alpha')}")
                print(f"    Chi: {cam_config.get('Chi')}")
                print(f"    DepthResolution: {cam_config.get('DepthResolution')}")
                print(f"    DepthFarClipPlane: {cam_config.get('DepthFarClipPlane')}")
                print(f"    RGBFarClipPlane: {cam_config.get('RGBFarClipPlane')}")
            
            print(f"Agent Group {i + 1} SensorDatas:")
            for sensor_key, sensor_data in group.get('SensorDatas', {}).items():
                print(f"  {sensor_key}:")
                print(f"    SensorId: {sensor_data.get('SensorId')}")
                print(f"    Description: {sensor_data.get('Description')}")
                print(f"    SensorType: {sensor_data.get('SensorType')}")
                print(f"    Orientation: {sensor_data.get('Orientation')}")
                print(f"    ImageResolution: {sensor_data.get('ImageResolution')}")
                print(f"    SensorSize: {sensor_data.get('SensorSize')}")
                print(f"    FocalLength: {sensor_data.get('FocalLength')}")
                print(f"    PublishFrequency: {sensor_data.get('PublishFrequency')}")
                print(f"    TopicNames: {sensor_data.get('TopicNames')}")
                print(f"    IsStereo: {sensor_data.get('IsStereo')}")
                print(f"    IsFisheye: {sensor_data.get('IsFisheye')}")
                print(f"    IsDepth: {sensor_data.get('IsDepth')}")
                print(f"    IsSimDetectionOn: {sensor_data.get('IsSimDetectionOn')}")
                print(f"    Baseline: {sensor_data.get('Baseline')}")
                print(f"    Alpha: {sensor_data.get('Alpha')}")
                print(f"    Chi: {sensor_data.get('Chi')}")
                print(f"    DepthResolution: {sensor_data.get('DepthResolution')}")
                print(f"    DepthFarClipPlane: {sensor_data.get('DepthFarClipPlane')}")
                print(f"    RGBFarClipPlane: {sensor_data.get('RGBFarClipPlane')}")
                print(f"    HorizontalChannels: {sensor_data.get('HorizontalChannels')}")
                print(f"    VerticalChannels: {sensor_data.get('VerticalChannels')}")
                print(f"    VerticalFOVLow: {sensor_data.get('VerticalFOVLow')}")
                print(f"    VerticalFOVHigh: {sensor_data.get('VerticalFOVHigh')}")
                print(f"    MaxDistance: {sensor_data.get('MaxDistance')}")
                print(f"    HorizontalScanStep: {sensor_data.get('HorizontalScanStep')}")
                print(f"    LidarResolution: {sensor_data.get('LidarResolution')}")
                print(f"    NearBlindZone: {sensor_data.get('NearBlindZone')}")
                print(f"    MountingPosition: {sensor_data.get('MountingPosition')}")

            for j, agent in enumerate(group['Agents']):
                print(f"\n  Agent {j + 1} ID:", agent['AgentId'])
                print(f"  Agent {j + 1} IP Address:", agent['IpAddress'])
                print(f"  Agent {j + 1} Waypoints:")

                for k, wp in enumerate(agent['Waypoints']):
                    print(f"    Waypoint {k + 1}: [{wp[0]}, {wp[1]}, {wp[2]}, {wp[3]}, {wp[4]}, {wp[5]}]")

        agents_list = read_agents_list(testcase_struct)
        print("\nread_agents_list() - Agents List:", agents_list)

        # Store the available agent IDs get from agents list
        available_agent_ids = [agent['AgentId'] for agent in agents_list]
        print("Available Agent IDs: from agents list", available_agent_ids)

        # Get the number of available agent IDs
        num_available_agents = len(available_agent_ids)
        print(f"Number of available agent IDs from agents list: {num_available_agents}")

        # Given an agent_id, store its waypoints in wp_list
        agent_id_to_find = int(input("Enter the agent ID to find its waypoints: "))
        wp_list = next((agent['Waypoints'] for agent in agents_list if agent['AgentId'] == agent_id_to_find), None)

        if wp_list is not None:
            print(f"Waypoints for agent {agent_id_to_find}:")
            for wp in wp_list:
                print(f"print wp_list= ({wp[0]}, {wp[1]}, {wp[2]}, {wp[3]}, {wp[4]}, {wp[5]})")
        else:
            print(f"No waypoints found for agent {agent_id_to_find}")

        env_id = testcase_struct['Testcase']['EnvId']
        scenario_id = testcase_struct['Testcase']['EnvScenarioId']
        scenario_struct = read_scenario_struct(env_id, scenario_id)
        #print("\nScenario Struct:", scenario_struct)

        obstacles_struct = read_obstacles_struct(scenario_struct)
        if obstacles_struct is not None:
            print("\nObstacles Struct:")
            for obstacle in obstacles_struct:
                print("\nObstacle:")
                print(obstacle)
                for transform_data in obstacle['TransformDatas']:
                    print("\n\ttransform_data:")
                    print(f"Pos: {transform_data['Pos']}, Rot: {transform_data['Rot']}, Scl: {transform_data['Scl']}")
        else:
            print("No obstacles found in the scenario")

        signposts_struct = read_signposts_struct(scenario_struct)
        # if signposts_struct is not None:
        #     print("\nSignposts Struct:")
        #     for signpost in signposts_struct:
        #         print("\nSignpost:")
        #         print(signpost)
        #         for transform_data in signpost['TransformDatas']:
        #             print("\n\ttransform_data:")
        #             print(f"Pos: {transform_data['Pos']}, Rot: {transform_data['Rot']}, Scl: {transform_data['Scl']}")
        # else:
        #     print("No signposts found in the scenario")

        signposts_eun_position = get_signposts_eun_position(signposts_struct)
        print("\nSignposts EUN Position:")
        for position in signposts_eun_position:
            print(f"Position: x={position.x}, y={position.y}, z={position.z}")    

        # signposts_nwu_position = get_signposts_nwu_position(signposts_struct)
        # print("\nSignposts NWU Position:")
        # for position in signposts_nwu_position:
        #     print(f"Position: x={position.x}, y={position.y}, z={position.z}")  

        scenario_info_struct = read_scenario_info_struct(scenario_struct)
        if scenario_info_struct is not None:
            print("\nScenario Info Struct:")
            print(f"Scenario Offset: {scenario_info_struct['ScenarioOffset']}")
            print("Scenario Corners:")
            for corner in scenario_info_struct['ScenarioCorners']:
                print(f"Corner: {corner}")

            scenario_corners = read_scenario_corners(scenario_info_struct)
            print("\nScenario Corners (geometry_msgs/Point[]):")
            for i in range(0, len(scenario_corners)):
                print(f"Point {i}: x={scenario_corners[i].x}, y={scenario_corners[i].y}, z={scenario_corners[i].z}")
        else:
            print("No scenario info found in the scenario")

        available_UAV_agent_ids = read_available_UAV_agent_ids(testcase_struct)
        print("** Available UAV Agent IDs:", available_UAV_agent_ids)

        available_GCS_agent_ids = read_available_GCS_agent_ids(testcase_struct)
        print("** Available GCS Agent IDs:", available_GCS_agent_ids)

        available_agent_ids = read_available_agent_ids(testcase_struct)
        print("** Available Total UAV+GCS Agent IDs from list:", available_agent_ids)

    except FileNotFoundError as e:
        print(e)

if __name__ == "__main__":
    main()
