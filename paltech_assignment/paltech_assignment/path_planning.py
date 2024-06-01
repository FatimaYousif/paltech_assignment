# complete your code here
import numpy as np
import matplotlib.pyplot as plt
from dubins import Dubins

class WaypointGenerator:
    def __init__(self, num_points, area_size, cluster_percentage, cluster_std):
        self.num_points = num_points
        self.area_size = area_size
        self.cluster_percentage = cluster_percentage
        self.cluster_std = cluster_std
        self.waypoints = None
        self.robot_start_pos = None

    def generate_random_points(self):
        self.waypoints = np.random.uniform(0, self.area_size, (self.num_points, 2))

    def add_clusters(self):
        num_clusters = int(self.num_points * self.cluster_percentage)
        cluster_indices = np.random.choice(range(self.num_points), num_clusters, replace=False)

        for idx in cluster_indices:
            num_plants = np.random.randint(2, 10)
            cluster_center = self.waypoints[idx]
            plants_x = np.random.normal(cluster_center[0], self.cluster_std, num_plants)
            plants_y = np.random.normal(cluster_center[1], self.cluster_std, num_plants)
            cluster_points = np.column_stack((plants_x, plants_y))
            self.waypoints = np.concatenate((self.waypoints, cluster_points))

    def assign_start_position(self):
        self.robot_start_pos = np.random.uniform(0, self.area_size, 2)

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
    num_points = 5
    area_size = 100
    cluster_percentage = 0.5
    cluster_std = 3

    generator = WaypointGenerator(num_points, area_size, cluster_percentage, cluster_std)
    generator.generate_random_points()
    generator.add_clusters()
    generator.assign_start_position()

    all_points = generator.waypoints
    clusters = all_points[num_points:]  # Separate cluster points
    points = all_points[:num_points]  # Original points
    start_pos = generator.robot_start_pos

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
    path_length = dubins.calculate_path_length(path)
    print("Path Length:", path_length)

if __name__ == "__main__":
    main()
