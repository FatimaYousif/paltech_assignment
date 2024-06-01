# Paltech Assignment

## Part 1: Basic waypoint manager in ROS2

This forked repository contains completion of given task 1 to 3 in the **waypoint_manager.py** for the part 1.

## Part 2: Path Planning 

The following are the highligths of this task:
- In this part, the Dubins path planning is used
- Variables are used for number of points and area (so they can be changed accordingly)
- Testing was done on the following parameters:
    - num_points = 5 (sample points)
    - area = 100 m²
    - percentage = 0.5
    - std = 3
- The time and path length of the calculated path (was as by given assumption, a constant velocity of 1m/s)


### Challenges:

The implementation faced a challenge due to the lack of prior ROS2 installation, which was necessary for this task. As a workaround, two additional .ipynb files were provided: "paltech_part1.ipynb" and "paltech_part2.ipynb". These files were used to verify the functionality of the code.

Unfortunately, due to time constraints, further optimizations such as fine-tuning the path planning using Dubins and addressing the bonus tasks were not possible. Hence, they remain pending for future consideration.
