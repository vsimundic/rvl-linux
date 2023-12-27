import json
import os
import numpy as np

def read_ao_dict_from_json(path):
    ao = {}
    with open(path) as f:
        data = f.read()
        ao_js = json.loads(data)

        for key in ao_js:
            if isinstance(ao_js[key], list):
                ao[key] = np.array(ao_js[key])
    
    return ao