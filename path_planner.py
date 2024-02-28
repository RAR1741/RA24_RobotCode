import copy
import importlib
import inspect
import json
import math
import os
import re
import sys

FIELD_LENGTH = 54 * 0.3048 # 54 feet in meters
# TESTING = True
TESTING = False

files_dir = os.path.join(os.path.dirname(__file__), 'src/main/deploy/pathplanner/paths')

# Load up all the files in ./src/main/deploy/pathplanner/paths
def make_red_autos():
    for file_name in os.listdir(files_dir):
        # Load the .path files as JSON
        if file_name.endswith('.path'):
            path = None
            original_path = None
            with open(os.path.join(files_dir, file_name), 'r') as file:
                path = json.load(file)

            original_path = copy.deepcopy(path)
            path = check_default_constraints(path)

            if not(file_name.upper().startswith("RED")):
                copy_path_to_red(file_name, copy.deepcopy(path))

            write_file(file_name, path, original_path=original_path)

def check_default_constraints(path):
    # If a command line argument is passed, set useDefaultConstraints in each path
    if len(sys.argv)>1 and sys.argv[1] == "true":
        path["useDefaultConstraints"] = True
    return path


def copy_path_to_red(blue_file_name, path):
    red_file_name = "RED|"+blue_file_name
    path["folder"] = "Red"

    # Flip the path
    for waypoint in path["waypoints"]:
        anchor = waypoint["anchor"]
        anchor["x"] = FIELD_LENGTH - anchor["x"]

        if waypoint["prevControl"]:
            waypoint["prevControl"]["x"] = FIELD_LENGTH - waypoint["prevControl"]["x"]

        if waypoint["nextControl"]:
            waypoint["nextControl"]["x"] = FIELD_LENGTH - waypoint["nextControl"]["x"]

    write_file(red_file_name, path)

    return path


def write_file(file_name, path, original_path=None):
    if TESTING:
        return

    with open(os.path.join(files_dir, file_name), 'w') as file:
        try:
            # Try to write the new path back to the file
            json.dump(path, file, indent=2)
        except Exception as e:
            # If it fails, write the original path back to the file
            if original_path:
                json.dump(original_path, file, indent=2)
            raise e


make_red_autos()
