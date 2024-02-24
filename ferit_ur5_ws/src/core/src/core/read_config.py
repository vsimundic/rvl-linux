import yaml
import rospkg
import os

def read_config(cfg_filename: str):
    rp = rospkg.RosPack()
    demo_pkg_path = rp.get_path('a_demo')
    cfg_filepath = os.path.join(demo_pkg_path, 'config', cfg_filename)
    
    if not os.path.isfile(cfg_filepath):
        raise Exception('The file does not exist. It needs to be placed in a_demo/config path.')
    if not cfg_filename.endswith('.yaml'):
        raise Exception('A config file must end with .yaml.')
    
    try:
        with open(cfg_filepath, 'r') as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
    except:
        raise Exception('Config must be a valid YAML file.')
    
    return config

def print_something():
    print('Done did')