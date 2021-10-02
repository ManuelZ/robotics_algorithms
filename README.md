# SLAM Algorithms

Here I intend to implement mapping and localization algorithms related to SLAM. First I'll do it in Python, then I'll switch to C++. The objective is to have a solid understanding of them and have a working robot that I can improve with time. Should not be perfect or extremly fast, the only intention is to learn.

## Probabilistic grid mapping with inverse range model

My own implementation in Python of the probabilistic Occupancy Grid mapping Algorithm found in *Probabilistic Robotics* of Sebastian Thrun. 
The space is discretized in a grid where each cell is a random variable with a probability of being occupied. 

Implementation details:
- The map grid is stored as a 2D Numpy array where each cell stores a log(odds) value.
- The map is published to the `/map` topic at 1Hz; the log(odds) are converted to probabilities before the publishing.
- The Bresenham's line algorithm is used to get a list of cells traversed at each laser measurement
- The inverse range sensor model returns predefined probabilities of cell occupation for each of the cells traversed by the laser
- The cell that is farthest away from the laser base gets a high probability of being occupied, al the other cells get a low probability value.

Assumptions:
- Grid cells are either free or occupied
- The world is static
- The random variables of the cells are independent of each other

### How to run
```
ros2 launch slam_algorithms occupancy_grid.launch.py
```

### Demonstration

https://user-images.githubusercontent.com/115771/135700503-e6f52e58-f373-4586-a083-945de13c70a8.mp4


References
- Sebastian Thrun - *Probabilistic Robotics*
- Cyrill Stachniss - *Occupancy Grid Maps* https://www.youtube.com/watch?v=v-Rm9TUG9LA
- Sebastian Thrun - *Learning Occupancy Grids With Forward Sensor Models* https://www.cs.cmu.edu/~thrun/papers/thrun.occ-journal.pdf
