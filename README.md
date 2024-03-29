# Robotics Algorithms

Here I intend to implement mapping and localization algorithms related to robotics. First I'll do it in Python, then I'll switch to C++. The objective is to have a solid understanding of them and have a working robot that I can improve with time. Should not be perfect or extremly fast, the only intention is to learn.

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

In Windows follow [this guide](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Windows.html), which explains how to install ROS2 and `webots_ros2` in WSL2 while running Webots from the host, using the `WEBOTS_HOME=/mnt/c/Program\ Files/Webots` Linux environment variable as mentioned in the guide.

Then, whether in WSL2 or in native Linux, build the package and run the code:
```
cd /ros2_ws
colcon build
source /ros2_ws/install/setup.bash
ros2 launch robotics_algorithms occupancy_grid.launch.py
```

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Demonstration

https://user-images.githubusercontent.com/115771/135700503-e6f52e58-f373-4586-a083-945de13c70a8.mp4


References
- Sebastian Thrun - *Probabilistic Robotics*
- Cyrill Stachniss - *Occupancy Grid Maps* https://www.youtube.com/watch?v=v-Rm9TUG9LA
- Sebastian Thrun - *Learning Occupancy Grids With Forward Sensor Models* https://www.cs.cmu.edu/~thrun/papers/thrun.occ-journal.pdf
