import os
import numpy as np

def chebyshev_distance(p1, p2):
    return np.max(np.abs(np.array(p1) - np.array(p2)))

def process_trajectory(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    
    # Convert lines to numerical values
    trajectory = [list(map(float, line.strip().split(','))) for line in lines]
    
    # Skip the first three points
    trajectory = trajectory[3:]
    q = np.array(trajectory)
    # q[:, 0] -= np.pi
    # q[:, 5] -= np.pi
    q[q>np.pi]-=(2.0*np.pi)     
    q[q<-np.pi]+=(2.0*np.pi)
    q = np.unwrap(q, axis=0)
    trajectory = q.tolist()

    # Compute Chebyshev distances
    distances = [chebyshev_distance(trajectory[i], trajectory[i+1]) for i in range(len(trajectory)-1)]
    
    return max(distances) if distances else 0

def main(directory):
    max_distances = []
    
    for filename in os.listdir(directory):
        if filename.startswith("traj_") and filename.endswith(".txt"):
            file_path = os.path.join(directory, filename)
            max_distance = process_trajectory(file_path)
            max_distances.append(max_distance)
    
    # Compute the average of the largest Chebyshev distances
    average_max_chebyshev_distance = np.mean(max_distances)
    
    print(f"Average of Maximum Chebyshev Distances: {average_max_chebyshev_distance}")

# Specify the directory containing trajectory files
directory = "/home/RVLuser/rvl-linux/data/Exp-MC-Spheres_traj"  # Change this to the actual path
main(directory)
