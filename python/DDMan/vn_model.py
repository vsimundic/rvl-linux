import numpy as np

class vn():
    def __init__(self, A = None, d = None, HL = None):
        self.A = A
        self.d = d
        self.HL = HL
        if self.A is None:
            self.num_base_level_nodes = 0
        else:
            self.num_base_level_nodes = self.A.shape[0]
        if self.HL is None:
            self.HL = []
        self.num_higher_level_nodes = len(self.HL)
        self.num_nodes = self.num_base_level_nodes + self.num_higher_level_nodes

    def sdf(self, points):
        y = np.zeros((points.shape[0], self.num_nodes))
        y[:,:self.num_base_level_nodes] = points @ self.A.T - self.d
        for HL_node_idx in range(self.num_higher_level_nodes):
            node_idx = self.num_base_level_nodes + HL_node_idx
            o = self.HL[HL_node_idx]['o']
            y[:,node_idx] = o * np.max(o * y[:,self.HL[HL_node_idx]['nodes']], axis=1)
        return y[:,self.num_nodes-1]

    def line_obstacle(self, lines, r):
        num_lines = lines.shape[0]
        c = lines[:,:3]
        v = lines[:,3:] - c
        l = np.linalg.norm(v, axis=1)
        l = np.tile(l[:,np.newaxis], self.num_base_level_nodes)
        k = v @ self.A.T
        intersection = (k != 0)
        distance_to_obstacle = c @ self.A.T - self.d - r[:,np.newaxis]
        completely_out = np.logical_and(np.logical_not(intersection), distance_to_obstacle > 0.0)
        completely_in = np.logical_and(np.logical_not(intersection), distance_to_obstacle <= 0.0)
        s0 = np.zeros(k.shape)
        s0[intersection] = -distance_to_obstacle[intersection] / k[intersection]
        obstacle_interval_start = np.zeros(k.shape)
        obstacle_interval_end = np.zeros(k.shape)
        obstacle_interval_start[completely_in] = -1.0
        obstacle_interval_end[completely_in] = 2.0
        obstacle_interval_start[completely_out] = 2.0
        obstacle_interval_end[completely_out] = 2.0
        going_away = (k > 0.0)
        approaching = (k < 0.0)
        obstacle_interval_start[approaching] = np.minimum(s0[approaching], 2.0)
        obstacle_interval_end[approaching] = 2.0
        obstacle_interval_start[going_away] = -1.0
        obstacle_interval_end[going_away] = np.maximum(s0[going_away], -1.0)
        obstacle_interval_start = np.concatenate((obstacle_interval_start, np.zeros((num_lines, self.num_higher_level_nodes))), axis=1)
        obstacle_interval_end = np.concatenate((obstacle_interval_end, np.zeros((num_lines, self.num_higher_level_nodes))), axis=1)
        for HL_node_idx in range(self.num_higher_level_nodes):
            node_idx = self.num_base_level_nodes + HL_node_idx
            o = self.HL[HL_node_idx]['o']
            child_node_idx = self.HL[HL_node_idx]['nodes'][0]
            obstacle_interval_start[:,node_idx] = obstacle_interval_start[:,child_node_idx]
            obstacle_interval_end[:,node_idx] = obstacle_interval_end[:,child_node_idx]
            for child_idx in range(1, len(self.HL[HL_node_idx]['nodes'])):
                child_node_idx = self.HL[HL_node_idx]['nodes'][child_idx]
                parent_gap_child = (obstacle_interval_end[:,node_idx] < obstacle_interval_start[:,child_node_idx])
                child_gap_parent = (obstacle_interval_end[:,child_node_idx] < obstacle_interval_start[:,node_idx])
                merge_intervals = np.logical_and(np.logical_not(parent_gap_child), np.logical_not(child_gap_parent))
                gap = np.logical_not(merge_intervals)
                if o > 0.0:
                    obstacle_interval_start[gap,node_idx] = 2.0
                    obstacle_interval_end[gap,node_idx] = 2.0
                    obstacle_interval_start[merge_intervals,node_idx] = np.maximum(obstacle_interval_start[merge_intervals,node_idx], 
                        obstacle_interval_start[merge_intervals,child_node_idx])
                    obstacle_interval_end[merge_intervals,node_idx] = np.minimum(obstacle_interval_end[merge_intervals,node_idx], 
                        obstacle_interval_end[merge_intervals,child_node_idx])
                else:
                    obstacle_interval_start[child_gap_parent,node_idx] = obstacle_interval_start[child_gap_parent,child_node_idx]
                    obstacle_interval_end[child_gap_parent,node_idx] = obstacle_interval_end[child_gap_parent,child_node_idx]
                    obstacle_interval_start[merge_intervals,node_idx] = np.minimum(obstacle_interval_start[merge_intervals,node_idx], 
                        obstacle_interval_start[merge_intervals,child_node_idx])
                    obstacle_interval_end[merge_intervals,node_idx] = np.maximum(obstacle_interval_end[merge_intervals,node_idx], 
                        obstacle_interval_end[merge_intervals,child_node_idx])
        return np.stack((obstacle_interval_start[:,self.num_nodes-1], obstacle_interval_end[:,self.num_nodes-1]), axis=1)
    
    def create_base_18(self):
        A = np.zeros((18,3))
        for i in range(3):
            A[i,i] = 1.0
            A[3+i,i] = -1.0
        k = 6
        for i in range(6):
            for j in range(i):
                if np.abs(np.sum(A[i,:] * A[j,:])) < 0.5:
                    A[k,:] = A[i,:] + A[j,:]
                    A[k,:] /= np.linalg.norm(A[k,:])
                    k += 1
        return A
    
    def convex_hull(self, A, points):
        d = (points @ A.T).max(0)
        return d

    def concave_hull(self, A, points):
        d = (points @ A.T).min(0)
        return d        
    
    def unit_box_vertices(self):
        vertices = np.zeros((8,3))
        u = 1.0
        v = 1.0
        for i in range(4):
            vertices[i,0] = u
            vertices[i,1] = v
            vertices[i,2] = 1.0
            vertices[4+i,0] = u
            vertices[4+i,1] = v
            vertices[4+i,2] = -1.0
            tmp = u
            u = -v
            v = tmp
        return vertices

    def add_bl_nodes(self, A, d):
        if self.A is None:
            self.A = A
            self.d = d
        else:
            self.A = np.concatenate((self.A, A), axis=0)
            self.d = np.concatenate((self.d, d))
        self.num_base_level_nodes = self.A.shape[0]
        self.num_nodes = self.num_base_level_nodes + self.num_higher_level_nodes

    def add_hl_node(self, o, nodes):
        node = {'o': o, 'nodes': np.array(nodes)}
        self.HL.append(node)
        self.num_higher_level_nodes += 1
        self.num_nodes = self.num_base_level_nodes + self.num_higher_level_nodes

    def transform_bl_nodes(self, base_node_idx, T):
        self.A[base_node_idx,:] = self.A[base_node_idx,:] @ T[:3,:3].T
        self.d[base_node_idx] += self.A[base_node_idx,:] @ T[:3,3]


