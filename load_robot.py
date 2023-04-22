import yaml

def load_disc_robot(file_name):
    with open(file_name) as f:
        robot = yaml.safe_load(f)
    return robot

