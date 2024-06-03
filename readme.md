# Paltech Assignment

This readme version details the required improved tasks.

## Task 1: Waypoints Manager - UTM conversions

- Utilized the pyproj with Proj, transform
- The two different coordinate systems Equirectangular projection and UTM projections, are affected by their range and distribution.
- The Equirectangular projection is more suited for local, small-scale navigation, while the UTM projection provides a broader, geographically accurate representation suitable for large-scale mapping and navigation.

## Task 2: Path Planning
Utilized both nearest neighbours and travelling salesman problem as highlighted below:

### Nearest Neighbour (NN) and Travelling Salesman Problem (TSP)
- For TSP made use of networkx python library to achieve this task.
- Both algorithms help sort the waypoints as in the previous presented code it was missing.
- Sorting helps gives weighting to the points important in respective criterias.
- In the comparsion of the two (NN and TSP) NN proved to give shorter path length as compared to TSP (same about time for 1 m/s constant velocity)

## Files for new tasks:
 - `utm_waypoints.ipynb`
 - `pathplanning_nn_tsp.ipynb` (Also in the already provided file named `path_planning.py`)
