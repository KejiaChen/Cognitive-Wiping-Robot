import numpy as np
import copy
import matplotlib.pyplot as plt
from dijkstar import Graph, find_path
from scipy.spatial import distance_matrix

realmax = 500


class Planner(object):
    """
    Integration of all planning algorithms
    """
    def __init__(self, nodes):
        self.nodes = nodes
        # distance matrix
        dst_matrix = distance_matrix(self.nodes, self.nodes, p=2, threshold=1000000)
        self.dst = dst_matrix
        self.dst[dst_matrix == 0] = realmax

        self.vision = 1./self.dst
        self.vision[dst_matrix == 0] = realmax

        self.start = self.nodes[0]

    def nearst_neighbor_planning(self):
        temp_nodes = self.nodes[:]
        current_idx = 0
        path = [self.start]
        travelled_dst = 0

        for i in range(len(temp_nodes) - 1):
            next_idx = np.argmin(self.dst[current_idx, :])
            min_dist = self.dst[current_idx, next_idx]

            path.append(self.nodes[next_idx])
            travelled_dst = travelled_dst + min_dist

            self.dst[:, current_idx] = realmax
            current_idx = next_idx

        # for p in path:
        #     print("path:", p)

        return path, travelled_dst

    def dynamic_programming_ref(self):
        n = len(self.nodes)
        all_sets = []
        g = {}
        p = []

        for x in range(1, n):
            g[x + 1, ()] = self.dst[x][0]

        def get_minimum(k, a):
            if (k, a) in g:
                # Already calculated Set g[%d, (%s)]=%d' % (k, str(a), g[k, a]))
                return g[k, a]

            values = []
            all_min = []
            for j in a:
                set_a = copy.deepcopy(list(a))
                set_a.remove(j)
                all_min.append([j, tuple(set_a)])
                result = get_minimum(j, tuple(set_a))
                values.append(self.dst[k - 1][j - 1] + result)

            # get minimun value from set as optimal solution for
            g[k, a] = min(values)
            p.append(((k, a), all_min[values.index(g[k, a])]))

            return g[k, a]

        origin_set = list(range(1, len(self.nodes)+1))
        origin_set.remove(1)
        get_minimum(1, tuple(origin_set))

        print('\n\nSolution to TSP: {1, ', end='')
        solution = p.pop()
        print(solution[1][0], end=', ')
        for x in range(n - 2):
            for new_solution in p:
                if tuple(solution[1]) == new_solution[0]:
                    solution = new_solution
                    print(solution[1][0], end=', ')
                    break
        print('1}')
        return

    def dynamic_programming(self):
        temp_nodes = self.nodes[:]
        current_idx = 0
        path = [self.start]
        travelled_dst = 0
        J={}  # dict to store the cost
        origin_set = list(range(len(self.nodes)))

        def get_min(i, set):
            cost_list = []
            for j in set:
                temp_set = copy.deepcopy(set)
                temp_set.remove(j)
                if not temp_set:
                    return J[j, tuple([])]
                J[j, tuple(temp_set)] = [(get_min(j, temp_set))[0] + self.dst[(get_min(j, temp_set))[1], j], (get_min(j, temp_set))[1]]  # accumulated cost + instant cost
                # str_set = ''.join(str(e) for e in temp_set)
                # print("node: " + str(j) + " set: " + str_set + " value: " + str((J[j, tuple(temp_set)])[0]))
                cost_list.append((J[j, tuple(temp_set)])[0])
            J_min = min(cost_list)
            min_index = cost_list.index(J_min)
            # path.append(self.nodes[set[min_index]])
            return [J_min, set[min_index]]

        origin_set.remove(0)
        # initialization
        for k in origin_set:
            J[k, tuple([])] = [0, k]

        J[0, tuple(origin_set)] = get_min(0, origin_set)
        cost_list = []
        for j in origin_set:
            temp_set = origin_set[:]
            temp_set.remove(j)
            cost_list.append((J[j, tuple(temp_set)])[0] + self.dst[j, 0])
        J_0_min = min(cost_list)
        min_index = cost_list.index(J_0_min) + 1
        J[0, tuple(origin_set)] = [J_0_min, min_index]

        # planning
        planning_set = origin_set[:]
        temp_index = 0
        for i in range(len(self.nodes)-1):
            # travelled_dst = travelled_dst + (J[i, tuple(planning_set)])[0]
            temp_index = (J[temp_index, tuple(planning_set)])[1]
            planning_set.remove(temp_index)
            path.append(self.nodes[temp_index])

        travelled_dst = J[0, tuple(origin_set)]

        return path, travelled_dst


    def dijkstra_planning(self):
        graph = Graph()

        temp_nodes = self.nodes[:]
        temp_nodes.pop(0)
        k = 0
        for i in range(len(self.nodes)):
            k = k + 1
            for j in range(len(temp_nodes)):
                node_a = self.nodes[i]
                node_b = temp_nodes[j]
                if np.linalg.norm(node_a - node_b) != 0:
                    print("add edge:", i, j + k)
                    graph.add_edge(i, j + k, np.linalg.norm(node_a - node_b))
            if temp_nodes:
                temp_nodes.pop(0)

        print("done")

        path = find_path(graph, 1, 3)
        print("path:", path)


# if __name__ == "__main__":
#     test_nodes = [np.array([21, 34]),
#                   np.array([45, 28]),
#                   np.array([76, 14]),
#                   np.array([12, 56]),
#                   np.array([48, 32])]
#
#     motion_planner = Planner(test_nodes)
#     motion_planner.dynamic_programming()