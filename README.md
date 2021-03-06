# Cognitive-Wiping-Robot
This project contains the visual cognition and motion planning part of a cognitive wiping robot. The task is assumed to be wiping off the words written on a white board.

[<img src="img/wiping_task_words.png" width="400px"/>](wiping_task_words.png)

## Setup
### Library for camera
Install Intel® RealSense™ SDK 2.0 following [official instruction](https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md)
### Environment
This project uses python 3.7. To install required packages:
```
pip install -r requirements.txt
```

## Cognition
### Detect the Working area
Choose region of interest manually as working area. In this case, the largest [contour](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html) ```max_cnt``` is usually the bounding rectangular of RoI.
Optionally: 
- Detect the largest contour ```max_cnt``` or
- Detect the bounding rectangular or
- Detect the contour of largest white area as working area..

### Detect the stains
Default: Reserve only contours inside ```max_cnt``` as stains to be cleaned. 

[<img src="img/contour_words.png" width="400px"/>](contour_words.png)

Option 1: Bounding rectangles can be used instead of contours.

[<img src="img/RealSense_words_detect.png" width="400px"/>](RealSense_words_detect.png)

However, the size of rectangular kernel needs to be set manually to get a suitable bounding rectangle.

### Covering SP
The wiping task can be formed as a [Covering Traveling Salesman Problem](https://www.scirp.org/journal/paperinformation.aspx?paperid=77781). To guarantee that the cleaner will pass each pixel of stains, each point on contours ```c``` is checked if it lies within an e-neighborhood of current nodes, where the value of e depends on the size of the tool (e.g. radius of the brush) used to accomplish the task. If not, ```c```  is added to the node_list.

[<img src="img/nodes_words.png" width="400px"/>](nodes_words.png)

As is shown in the image above, blue point stands for the starting point, red points are all the nodes for planning algorithms and green shadows marks the area the brush covers. 
 

## Motion Planning
The task of passing all the nodes is then formed as a [Traveling Salesman Problem](https://en.wikipedia.org/wiki/Travelling_salesman_problem#As_a_graph_problem), and will be solved by following algorithms.
### Nearst Neighbors
Starting from the initial position (current position of end effector, marked as blue), select the nearst node to be the next move.

[<img src="img/planned_path_nn_words.png" width="400px"/>](planned_path_nn_words.png)

The computation for nn is quite efficient, but the path is obviously not optimal:

[<img src="img/graph_nn_computation.png" width="400px"/>](compute_nn.png)

### Dynamic Programming
The path is planned by dynamic programming. However, the computation time is relatively long.

### Ant Colony Algorithm
The path is planned by ant colony algorithm. Area covered by the brush along the path is marked as blue shadow. Here the population is set as 50 and it runs for 200 iterations.

Path planned with neighborhood radius ```e=10```:

[<img src="img/planned_path_acs_words.png" width="400px"/>](planned_path_acs_words.png)

The computation for ac takes longer, but the resulted path is also more cost-efficient:

[<img src="img/graph_ac_false_computation.png" width="400px"/>](compute_ac_false.png)

Path planned with neighborhood radius ```e=5```:

[<img src="img/planned_path_acs_words_radius_5.png" width="400px"/>](planned_path_acs_words_radius_5.png)


## Demo
To detect stains and plan the path of the end-effector:
```
python viewer.py
```
[<img src="img/nn_planning_demo.gif" width="400px"/>](nn_demo.gif)

As is shown in the demo, the RoI is firstly selected as the working area. The planned path (marked as blue shadow) is always able to cover all the words.

Note that this demo uses Nearst Neighbor as planning algorithm. For more complex algorithms like Dynamic Programming and Ant Colony, real-time planning and response are not possible.

## Next Step & Discussion
### Undirected Graph
Results above were obatined under the assumption that nodes are all connected to each other. This guarantees that a path will be found, but also leads to computational burdens in planning. To speed up the planning process, a graph G=(V,E) is constructed following steps in [J.Hess2012](https://ieeexplore.ieee.org/abstract/document/6385960). The idea is that each node is only connected to top k nearest neighbors within a radius r. In this way, number of edges to be explored is supposed to be cut down. Below is the graph constructed with ```k=5``` and ```r=30```.

[<img src="img/graph_words.png" width="400px"/>](graph_words.png)

Below, two attempts to implement the planning based on this idea are introduced.

#### 1. Modify the distance matrix
An intuitive method is to modify the distance matrix. The entries(costs) that corresponde to unconnected edges are changed into some extremely large numbers. In this way, these edges will not (possibly) chosen by the planning algorithm.

The path planned by graph-based ant colony algorihtm with ```k=10``` and ```r=70```:

[<img src="img/graph_ac_k_10_r_70.png" width="400px"/>](path_ac_k_10_r_70.png)

[<img src="img/graph_ac_k_10_r_70_computation.png" width="400px"/>](compute_ac_k_10_r_70.png)

and with ```k=20``` and ```r=70```:

[<img src="img/graph_ac_k_20_r_70.png" width="400px"/>](path_ac_k_20_r_70.png)

[<img src="img/graph_ac_k_20_r_70_computation.png" width="400px"/>](compute_ac_k_20_r_70.png)

As is shown in the screenshots above, tuning k and r has the potential to find a slight more cost-efficient path. However, this method did not descrease the number of iterations in the algorithm, and thus cannot really speed up the computation. 

#### 2. Graph-based planning
At each node, graph-based path planning will ignore the unconnected nodes, i.e. only interate over the nodes adjecent to it in the graph. However, as each node is only allowed to be visited once, this method usually fails in finding a valid path.


### Joint GTSP



