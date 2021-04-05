import numpy as np
from dijkstar import Graph, find_path
from scipy.spatial import distance_matrix

realmax = 500


def nearst_neighbor_planning(nodes):
    temp_nodes = nodes[:]
    dst = distance_matrix(nodes, nodes, p=2, threshold=1000000)
    dst[dst == 0] = realmax

    start = nodes[0]
    current_idx = 0
    path = [start]
    travelled_dst = 0

    for i in range(len(temp_nodes)-1):
        next_idx = np.argmin(dst[current_idx, :])
        min_dist = dst[current_idx, next_idx]

        path.append(nodes[next_idx])
        travelled_dst = travelled_dst + min_dist

        dst[:, current_idx] = realmax
        current_idx = next_idx

    # for p in path:
    #     print("path:", p)

    return path


def dijkstra_planning(nodes):

    graph = Graph()

    temp_nodes = nodes[:]
    temp_nodes.pop(0)
    k = 0
    for i in range(len(nodes)):
        k = k + 1
        for j in range(len(temp_nodes)):
            node_a = nodes[i]
            node_b = temp_nodes[j]
            if np.linalg.norm(node_a-node_b) != 0:
                print("add edge:", i, j+k)
                graph.add_edge(i, j+k, np.linalg.norm(node_a-node_b))
        if temp_nodes:
            temp_nodes.pop(0)

    print("done")

    path = find_path(graph, 1, 3)
    print("path:", path)


# if __name__ == "__main__":
#     test_nodes = [np.array([21, 34]),
#                   np.array([45, 28]),
#                   np.array([76, 14]),
#                   np.array([12, 56])]
#
#     nearst_neighbor_planning(test_nodes)
