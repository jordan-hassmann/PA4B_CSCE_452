
import yaml
from ament_index_python.packages import get_package_share_directory


def load_disc_robot(file_name):
    package_share_directory = get_package_share_directory('project4b') 
    robot_file_path = package_share_directory + '/robots/' + file_name

    with open(robot_file_path) as f:
        robot = yaml.safe_load(f)
    return robot


