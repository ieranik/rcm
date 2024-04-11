# Overview
This repository contains the source code for the paper titled [Failure-Resilient Coverage Maximization with Multiple Robots](https://arxiv.org/pdf/2007.02204.pdf). This paper was published in _IEEE Robotics and Automation - Letters_ back in 2021. This paper proposes approximation algorithms for the _Resilient Coverage Maximization_ (RCM) problem. The objective of the RCM problem is to maximize the target coverage of a multi-robot system where the robots are subject to adversarial attacks or failures. My algorithms provide improvement in running time and accuracy over the state-of-the-art solution to the RCM problem proposed by [Zhou et al.](https://ieeexplore.ieee.org/document/8534468) 

In this work, I develop two approximate algorithms, one based on a _greedy_ approach, in which I produce an ordering of the robots according to some sorting criteria, and greedily select the trajectories of each robot sequentially according to the sorted order, and another based on _local search_ technique, in which I start with an initial solution of the RCM problem, and gradually move towards a better neighbor of the current solution until a local optima is reached.

I conduct extensive experiments to demonstrate the effectiveness of the two proposed algorithms. I also test the performance of the proposed algorithm in a simulated environment using a case study. The case study is based on the multi-robot [persistent monitoring task](https://ieeexplore.ieee.org/abstract/document/8815211). I select trajectories of the robots such that target coverage is maximized in case of a worst-case failure of $\alpha$ robots. The simulation of the case-study environment is rendered using [OpenGL](https://open.gl/).

# Directory Tree and Code Explanation

<p align="center">
<img src="https://github.com/ieranik/rcm/blob/main/dataset.png">

* `datasets`: I generate two types of synthetic datasets: (i) rectangular (`datasets/gen_data_rect.cpp`) and (ii) elliptical (`datasets/gen_data_ellipse.cpp`), samples of which are shown in left and right respectively in the figure above. The rectangular and elliptical datasets have 4 and 7 candidate trajectories per robot respectively as shown in the figure. To generate the datasets, I select locations of the targets and robots within a 100 X 100 2D region with uniform probability. A trajectory covers all the targets located within its coverage region. The dataset generation procedure imitates a scenario in which a set of targets on the ground are being covered by a set of UAVs with downward-facing cameras.

* `algorithms`
  * `2PG.cpp`: This file contains my implementation of the existing work on the RCM problem by [Zhou et al.](https://ieeexplore.ieee.org/document/8534468), which involves two phases. In the first phase, the algorithm determines the worst-case subset of $\alpha$ robots that could fail. In the next phase, assuming that the robots selected in the first phase will actually fail, the rest of the robot trajectories are selected greedily such that the marginal gain in target coverage is maximized. We call this algorithm the 2 Phase Greedy (2PG) algorithm.
  * `obliv_greedy.cpp`: This file contains my implementation of the Oblivious Greedy Algorithm. In the Oblivious Greedy (ObG) algorithm, I simply select, for each robot, the trajectory that covers the maximum number of targets. 
  * `ordered_greedy.cpp`: This file contains the implementation of my proposed Ordered Greedy Algorithm for the RCM problem. In this algorithm, first, we sort the robots according to some sorting criteria. Then, for each robot according to the sorted order, we greedily select the trajectory that maximizes the marginal coverage of the targets. I consider two sorting criteria for the robots: (i) Size of Union of Target Coverage and (ii) Maximum Individual Target Coverage. For both sorting criteria, I consider increasing and decreasing orders of sorting, resulting in 4 variants of the algorithm.
  * `local_search.cpp`: This file contains the implementation of the Local Search Algorithm. Here, I start with an initial solution. In each iteration, I make small local changes to the current solution to form a set of neighbor solutions. Then I evaluate the objective function on the neighbors to determine if any improvement over the current solution is possible. If a better solution is found, the search moves in that direction. Otherwise, the algorithm terminates. I consider 4 variants of the local search algorithm based on 2 attack models and 2 initial solutions.

* `visualizer`: `visualizer/vis.cpp` contains the code to visualize how the proposed RCM algorithms work in the setting of the multi-robot persistent monitoring task. In this task, a team of aerial robots monitor an obstructed environment on the ground such that the unobstructed space in the environment is periodically visited by some robot. To this end, the environment is uniformly discretized into cells. The objective of the task is to minimize the total latency of all the unoccupied cells, where latency refers to the time between consecutive visits of a call. The visualization is rendered using OpenGL. A sample of the visualization has been made available on [YouTube](https://www.youtube.com/watch?v=XdQ5h5aOMAA&ab_channel=MdIshat-E-Rabban).


# How to Run

1. Generating Dataset: `dataset/gen_data_rect.cpp` or `dataset/gen_data_ellipse.cpp` can be used to generate synthetic data for the RCM problem. Along with other parameters, the number of robots, targets and attack size $\alpha$ can be varied using the `N`, `T` and `A` variables on Lines 12-23. The dataset is output in a text file named `data.txt`.
2. Executing Algorithms: The algorithms described above can be executed on the dataset using the codes from the folder `algorithms/`. The text file containing the dataset must be present in the same directory, and the name of the data file should be updated in the `myfile` filestream in the `main()` function of the code. 
3. Visualizing Results: `visualizer/vis.cpp` can be used to visualize the persistent monitoring case study. The environment to be visualized can be modified using the variables in Lines 23-31 of the code. Here `numobs`, `crange`, and `numtargets` denote the number of obstacles, communication range, and number of targets, respectively. This code uses the OpenGL API to visualize the simulation. OpenGL can be installed with the help of [this tutorial](https://www.opengl-tutorial.org/beginners-tutorials/tutorial-1-opening-a-window/).


# Links

* [Failure-Resilient Coverage Maximization with Multiple Robots](https://arxiv.org/pdf/2007.02204.pdf)
* [Resilient active target tracking with multiple robots](https://ieeexplore.ieee.org/document/8534468)
* [Visualization of Persistent Monitoring Case Study](https://www.youtube.com/watch?v=XdQ5h5aOMAA&ab_channel=MdIshat-E-Rabban)
* [OpenGL](https://open.gl/)


