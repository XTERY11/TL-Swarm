#!/usr/bin/env python3
# Use Unity C# naming convention CamelCase because data read from Unity created json
## This python library is designed to be not dependent on ROS
import json
import numpy as np
import os
from pathlib import Path

def read_json(file_path):
    with open(file_path, 'r') as file:
        return json.load(file)

def get_userdata_path():
    # Get the USERDATA_PATH environment variable
    env_userdata_path = os.getenv('USERDATA_PATH', "")
    
    if env_userdata_path:
        userdata_path = Path(env_userdata_path)
    else:
        userdata_path = Path.home() / 'Unity/build/hifi_simulator_unity_Data/StreamingAssets/UserData/'

    return userdata_path

def get_appdata_path():
    # Get the USERDATA_PATH environment variable
    env_userdata_path = os.getenv('USERDATA_PATH', "")
    
    if env_userdata_path:
        appdata_path = Path(env_userdata_path) / '../AppData'
    else:
        appdata_path = Path.home() / 'Unity/build/hifi_simulator_unity_Data/StreamingAssets/AppData/'

    return appdata_path

def get_testcase_path(testcase_id):
    base_path = get_userdata_path()
    testcase_path = base_path / f'TestCases/TestCase{testcase_id}'
    return testcase_path

def get_scenario_png_path(env_id, scenario_id):
    env_id, scenario_id 

    if scenario_id <= 100:
        scenario_png_path = get_appdata_path() / f'EnvScenarios/Env{env_id}Scenario{scenario_id}.png'
    else:
        scenario_png_path = get_userdata_path() / f'EnvScenarios/Env{env_id}Scenario{scenario_id}.png'
    
    return scenario_png_path

def get_scenario_json_path(env_id, scenario_id):
    env_id, scenario_id 

    if scenario_id <= 100:
        scenario_json_path = get_appdata_path() / f'EnvScenarios/Env{env_id}Scenario{scenario_id}.json'
    else:
        scenario_json_path = get_userdata_path() / f'EnvScenarios/Env{env_id}Scenario{scenario_id}.json'
    
    return scenario_json_path

def get_signpost_image_path():
    signpost_image_path = get_appdata_path() / 'PlotImages/signpost.png'
    return signpost_image_path 

def get_endpoint_image_path():
    endpoint_image_path = get_appdata_path() / 'PlotImages/endpoint_red.png'
    return endpoint_image_path

# Example usage
def main():
    testcase_id = input("Enter the Test Case number: ")
    testcase_path = get_testcase_path(testcase_id)
    print(f"Testcase path is {testcase_path}")
    env_id = input("Enter the Environment ID: ")
    scenario_id = int(input("Enter the Environment Scenario ID: "))
    scenario_png_path = get_scenario_png_path(env_id, scenario_id)
    print(f"Environment scenario PNG path is \n{scenario_png_path}")
    scenario_json_path = get_scenario_json_path(env_id, scenario_id)
    print(f"Environment scenario JSON path is \n{scenario_json_path}")

    signpost_image_path = get_signpost_image_path()
    print(f"Signpost image path is \n{signpost_image_path}")
    endpoint_image_path = get_endpoint_image_path()
    print(f"Endpoint image path is \n{endpoint_image_path}")

if __name__ == "__main__":
    main()
