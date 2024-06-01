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
        self.robot_intial_position = None

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
        self.robot_intial_position = np.random.uniform(0, self.area, 2)

def plot_points(points, clusters, start_pos, path):
    plt.figure(figsize=(10, 10))
    plt.scatter(points[:, 0], points[:, 1], c='blue', marker='o', label='Waypoints')
    # plt.scatter(clusters[:, 0], clusters[:, 1], c='red', marker='x', label='Cluster Points')
    plt.scatter(start_pos[0], start_pos[1], c='green', marker='s', label='Start Position')
    path = np.array(path)
    plt.plot(path[:, 0], path[:, 1], c='orange', label='Dubins Path')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Dubins Path Planning')
    plt.show()


def main():
    num_points = 6
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
    start_pos = generator.robot_intial_position

    start = np.append(start_pos, np.random.uniform(0, 2 * np.pi))
    waypoints = [np.append(point, np.random.uniform(0, 2 * np.pi)) for point in points]

    dubins = Dubins(radius=2, point_separation=2)
    path = []
    current_pos = start

    for waypoint in waypoints:
        new_path, _ = dubins.dubins_path(current_pos, waypoint)
        path.extend(new_path)
        current_pos = waypoint

    plot_points(points, clusters, start_pos, path)
    path_length, path_time = dubins.calculate_path_length_time(path)
    print("Path Length:", path_length)
    print("Path Time:", path_time)

if __name__ == "__main__":
    main()
