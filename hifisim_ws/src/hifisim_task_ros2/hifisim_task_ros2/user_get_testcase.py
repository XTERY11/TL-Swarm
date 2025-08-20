#!/usr/bin/env python3
# User accessible APIs 
# For users to read json testcase based on testcase_id
# to data structure testcase_structure
## This python library is designed to be not dependent on ROS

#from hifisim_task_ros2.data_path import *
from hifisim_task_ros2.read_testcase import *
from geometry_msgs.msg import Point
from .entrance_plane import Plane

def get_testcase_struct(testcase_id):
    """
    Reads a testcase from JSON files (TestCase.json, SimulatorSetting.json, AgentGroupX.json)
    and organizes the data into a testcase_struct.
    """
    testcase_struct = read_testcase_struct(testcase_id)
    return testcase_struct

def get_scenario_corners_from_testcase(testcase_id):
    testcase_struct = get_testcase_struct(testcase_id)
    if testcase_struct is None:
        print(f"Testcase struct not found for TestcaseId: {testcase_id}")
        return None
    env_id = testcase_struct['Testcase']['EnvId']
    scenario_id = testcase_struct['Testcase']['EnvScenarioId']
    
    scenario_struct = read_scenario_struct(env_id, scenario_id)
    if scenario_struct is None:
        print(f"Scenario struct not found for EnvId: {env_id}, ScenarioId: {scenario_id}")
        return None
    
    scenario_info_struct = read_scenario_info_struct(scenario_struct)
    if scenario_info_struct is None:
        print(f"Scenario info struct not found for EnvId: {env_id}, ScenarioId: {scenario_id}")
        return None
    
    scenario_corners = []
    scenario_corners = read_scenario_corners(scenario_info_struct)
    if scenario_corners is None:
        print(f"Scenario corners not found for EnvId: {env_id}, ScenarioId: {scenario_id}")
        return None
    return scenario_corners

def get_signposts_plane_testcase(testcase_id):
    """
    Extract signposts plane from testcase_id
    Each Plane object is created using the position and computed normal vector.
    The coordinate system is NWU (North-West-Up).
    The position is the center of the signpost.
    The normal vector is pointing out of the front of signpost. 
    It means the normal vector points toward the user looking at the signpost.
    The signposts_plane include all the signposts beside the entrance and at the endpoint.

    Returns:
        list: List of Plane objects constructed using the position and computed normal.
    """
    testcase_struct = get_testcase_struct(testcase_id)
    if testcase_struct is None:
        print(f"Testcase struct not found for TestcaseId: {testcase_id}")
        return None
    env_id = testcase_struct['Testcase']['EnvId']
    scenario_id = testcase_struct['Testcase']['EnvScenarioId']
    scenario_struct = read_scenario_struct(env_id, scenario_id)
    signposts_struct = read_signposts_struct(scenario_struct)
    signposts_endpoint_nwu_pos_rot = get_signposts_endpoint_nwu_pos_rot_yaw(signposts_struct)

    if signposts_endpoint_nwu_pos_rot is None:
        print(f"Signposts struct not found for EnvId: {env_id}, ScenarioId: {scenario_id}")
        return None
    
    signposts_plane = []
    for data in signposts_endpoint_nwu_pos_rot:
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

def get_available_UAV_agent_ids(testcase_id):
    """
    Extracts available UAV agent IDs from the testcase_id.
    Args:
        testcase_id (str): The ID of the testcase.
    Returns:
        list: List of available UAV agent IDs.
    """
    testcase_struct = get_testcase_struct(testcase_id)
    if testcase_struct is None:
        print(f"Testcase struct not found for TestcaseId: {testcase_id}")
        return None
    available_UAV_agent_ids = read_available_UAV_agent_ids(testcase_struct)
    return available_UAV_agent_ids

def get_available_GCS_agent_ids(testcase_id):
    """
    Extracts available GCS agent IDs from the testcase_id.
    Args:
        testcase_id (str): The ID of the testcase.
    Returns:
        list: List of available GCS agent IDs.
    """
    testcase_struct = get_testcase_struct(testcase_id)
    if testcase_struct is None:
        print(f"Testcase struct not found for TestcaseId: {testcase_id}")
        return None
    available_GCS_agent_ids = read_available_GCS_agent_ids(testcase_struct)
    return available_GCS_agent_ids


# 
# Example Usage of User APIs
#    
def main(args=None):
    testcase_id = input("Enter the Test Case number: ")
    try:
        #
        # Example 1: Usage of get_testcase_struct(testcase_id) to read parameters in testcase struct
        #
        testcase_struct = get_testcase_struct(testcase_id)
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
            for cam_key, cam_config in group['CameraConfigs'].items():
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
            
            for j, agent in enumerate(group['Agents']):
                print(f"  Agent {j + 1} ID:", agent['AgentId'])
                print(f"  Agent {j + 1} IP Address:", agent['IpAddress'])
                print(f"  Agent {j + 1} Waypoints:")
                for k, wp in enumerate(agent['Waypoints']):
                    print(f"    Waypoint {k + 1}: [{wp[0]}, {wp[1]}, {wp[2]}, {wp[3]}, {wp[4]}, {wp[5]}]")

        #
        # Example 2: Usage of get_scenario_corners_from_testcase(testcase_id) to read scenario corners
        #       Scenario corners are the horizontal boundaries of the scenario
        #
        scenario_corners = get_scenario_corners_from_testcase(testcase_id)
        print("\nScenario Corners from testcase_id (geometry_msgs/Point[]):")
        for i in range(0, len(scenario_corners)):
            print(f"Point {i}: x={scenario_corners[i].x}, y={scenario_corners[i].y}, z={scenario_corners[i].z}")
        
        #
        # Example 3: Usage of  get_signposts_plane_testcase(testcase_id) to read all signposts_plane
        #
        signposts_plane = get_signposts_plane_testcase(testcase_id)
        print("\nSignposts Plane:")
        for idx, plane in enumerate(signposts_plane):
            point_formatted = [f"{value:.1f}" for value in plane.point.tolist()]
            normal_formatted = [f"{value:.1f}" for value in plane.normal.tolist()]
            signpost_id = idx + 1
            print(f"Signpost {signpost_id} Plane(point={point_formatted}, normal={normal_formatted})")

        #
        # Example 4: Usage of get_available_UAV_agent_ids(testcase_id) to read available UAV agent IDs
        #
        available_UAV_agent_ids = get_available_UAV_agent_ids(testcase_id)
        print("\nAvailable UAV Agent IDs:", available_UAV_agent_ids)
        # Get the number of available UAV agents
        num_available_UAV_agents = len(available_UAV_agent_ids)
        print(f"Number of available UAV agents: {num_available_UAV_agents}")

        #
        # Example 5: Usage of get_available_GCS_agent_ids(testcase_id) to read available GCS agent IDs
        #
        available_GCS_agent_ids = get_available_GCS_agent_ids(testcase_id)
        print("\nAvailable GCS agent IDs:", available_GCS_agent_ids)
        # Get the number of available GCS
        num_available_GCS_agents = len(available_GCS_agent_ids)
        print(f"Number of available GCS agent: {num_available_GCS_agents}")

    except FileNotFoundError as e:
        print(e)

if __name__ == "__main__":
    main()
