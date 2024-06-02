# complete your code here
import numpy as np
import matplotlib.pyplot as plt
from dubins import Dubins

class WaypointGenerator:
    def __init__(self, num_points, area, percentage, std):
        self.num_points = num_points
        self.area = area
        self.percentage = percentage
        self.std = std
        self.waypoints = None
        self.robot_initial_position = None

    def generate_random_points(self):
        self.waypoints = np.random.uniform(0, self.area, (self.num_points, 2))

    def add_clusters(self):
        num_clusters = int(self.num_points * self.percentage)
        cluster_indices = np.random.choice(range(self.num_points), num_clusters, replace=False)

        for idx in cluster_indices:
            num_plants = np.random.randint(2, 10)
            cluster_center = self.waypoints[idx]
            x = np.random.normal(cluster_center[0], self.std, num_plants)
            y = np.random.normal(cluster_center[1], self.std, num_plants)
            cluster_points = np.column_stack((x, y))
            self.waypoints = np.concatenate((self.waypoints, cluster_points))

    def set_robot_initial_position(self):
        self.robot_initial_position = np.random.uniform(0, self.area, 2)


def plot_path(path, radius, start_pos, waypoints):
    plt.figure()
    plt.plot(path[:, 0], path[:, 1], label="Dubins path")
    plt.gca().set_aspect('equal', adjustable='box')
    plt.scatter(waypoints[:, 0], waypoints[:, 1], c='red', marker='o', label='Waypoints')

    plt.scatter(start_pos[0], start_pos[1], c='green', marker='s', label='Start Position')
    plt.title(f'Dubins Path Planning with Turn Radius {radius}')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.grid(True)
    plt.show()



def main():
    radius = 2.0
    point_separation = 0.1

    # Dubins path planner
    dubins_planner = Dubins(radius, point_separation)

    num_points = 5
    area_size = 100
    cluster_percentage = 0.5
    cluster_std = 3

    generator = WaypointGenerator(num_points, area_size, cluster_percentage, cluster_std)
    generator.generate_random_points()
    generator.add_clusters()
    generator.set_robot_initial_position()


    all_points = generator.waypoints
    clusters = all_points[num_points:]  # Separate cluster points
    points = all_points[:num_points]  # Original points
    start_pos = generator.robot_initial_position

  
    start = np.append(start_pos, np.random.uniform(0, 2 * np.pi))
    waypoints = [np.append(point, np.random.uniform(0, 2 * np.pi)) for point in points]

    # Compute the Dubins path between all waypoints and combine the paths
    combined_path = np.empty((0, 2))
    current_position = start

    for waypoint in waypoints:
        path, _ = dubins_planner.dubins_path(current_position, waypoint)
        combined_path = np.vstack((combined_path, path))
        current_position = waypoint

    # Plotting
    plot_path(combined_path, radius, start,  np.array(points))
    
    length, time = dubins_planner.calculate_path_length(combined_path)
    print("path length", length)
    print("time", time)
    

if __name__ == "__main__":
    main()