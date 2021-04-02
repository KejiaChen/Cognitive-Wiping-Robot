import numpy as np
from dijkstar import Graph, find_path


# class DijkstraSPF(dij.AbstractDijkstraSPF):
#
#     @staticmethod
#     def get_adjacent_nodes(G, u):
#         return G.get_adjacent_nodes(u)
#
#     @staticmethod
#     def get_edge_weight(G, u, v):
#         return G.get_edge_weight(u, v)


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


if __name__ == "__main__":
    test_nodes = [np.array([21, 34]),
                  np.array([45, 28]),
                  np.array([76, 14]),
                  np.array([12, 56])]

    dijkstra_planning(test_nodes)