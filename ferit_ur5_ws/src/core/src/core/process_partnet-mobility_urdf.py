#!/usr/bin/python

import rospy
import open3d
import numpy as np
from bs4 import BeautifulSoup
import os
from stl import mesh

rospy.init_node('process_urdf_node')

def main():
    
    # Load the URDF file 
    urdf_filepath = rospy.get_param('~urdf_file')
    urdf_rootpath = os.path.dirname(os.path.abspath(urdf_filepath))
    
    print(urdf_filepath)
    with open(urdf_filepath, 'r') as f:
        urdf_data = f.read()
    
    bs_data = BeautifulSoup(urdf_data, 'xml')
    bs_links = bs_data.find_all('link')
    meshes = []
    
    stl_meshes_path = urdf_rootpath + '/stl_meshes'
    if not os.path.exists(stl_meshes_path):
        os.makedirs(stl_meshes_path)
    else:
        pass

    for idx, link in enumerate(bs_links):
        finalmesh = open3d.geometry.TriangleMesh()
        meshes = [os.path.join(urdf_rootpath, mesh_.find('mesh').get('filename')) for mesh_ in link.find_all('visual')]

        for mesh_ in meshes:
            finalmesh += open3d.io.read_triangle_mesh(mesh_)
            
        finalmesh = open3d.geometry.TriangleMesh.compute_triangle_normals(finalmesh)
        open3d.io.write_triangle_mesh(stl_meshes_path + '/link_%d.stl' % idx, finalmesh)
        
    for i in range(len(bs_links)):
        mesh_ = mesh.Mesh.from_file(stl_meshes_path + '/link_%d.stl' % i)
        print(mesh_.get_mass_properties())
    
    
    
    

if __name__ == '__main__':
    main()