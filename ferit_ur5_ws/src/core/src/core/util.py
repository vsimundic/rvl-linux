import yaml
import rospkg
import os
import pandas as pd
from pandas.errors import EmptyDataError

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

def read_csv_DataFrame(path:str, separator:str=',') -> pd.DataFrame:
    try:
        df = pd.read_csv(filepath_or_buffer=path, sep=separator, header=0)
    except EmptyDataError as e:
        return None
    return df