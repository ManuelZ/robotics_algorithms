# SLAM Algorithms

Here I intend to implement mapping and localization algorithms related to SLAM. First I'll do it in Python, then I'll switch to C++. The objective is to have a solid understanding of them and have a working robot that I can improve with time. Should not be perfect or extremly fast, the only intention is to learn.

## Probabilistic grid mapping with inverse range model

My own implementation in Python of the probabilistic Occupancy Grid mapping Algorithm found in *Probabilistic Robotics* of Sebastian Thrun. 
The space is discretized in a grid where each cell is a random variable with a probability of being occupied. 

Assumptions:
- Grid cells are either free or occupied
- The world is static
- The random variables of the cells are independent of each other

### How to run
"""
ros2 launch slam_algorithms occupancy_grid.launch.py
"""

References
- Sebastian Thrun - *Probabilistic Robotics*
- Cyrill Stachniss - *Occupancy Grid Maps* https://www.youtube.com/watch?v=v-Rm9TUG9LA
- Sebastian Thrun - *Learning Occupancy Grids With Forward Sensor Models* https://www.cs.cmu.edu/~thrun/papers/thrun.occ-journal.pdf
