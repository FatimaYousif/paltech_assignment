import rclpy
from rclpy.node import Node
from custom_interfaces.srv import GetWaypoints, SetWaypoints
from custom_interfaces.msg import Waypoint
from std_srvs.srv import Trigger
import json
import math
import numpy as np
import matplotlib as plt

class WaypointManager(Node):
    def __init__(self):
        super().__init__("waypoint_manager")
        self.get_waypoints_srv = self.create_service(
            GetWaypoints, "get_robot_waypoints", self.get_robot_waypoints_callback
        )
        self.set_waypoint_srv = self.create_service(
            SetWaypoints, "set_waypoints", self.set_waypoints_callback
        )
        self.reset_waypoint_srv = self.create_service(
            Trigger, "reset_waypoints", self.reset_waypoints_callback
        )
        self.robot_inital_geo = (47.740114, 10.322442)

        self.waypoint_list_geo = []  # needs to be an array of Waypoint() messages
        self.waypoint_list_robot_frame = []
        self.set_waypoint_client = self.create_client(SetWaypoints, "set_waypoints")

    def set_waypoints_callback(self, request, response):
        response.success = False
        self.get_logger().info(f"Task 1: Loaded waypoints:  {self.waypoint_list_geo}")
        # COMPLETE YOUR CODE HERE
        try:
            # .geojson file handling
            with open(request.file_path, 'r') as f:
                data = json.load(f)
                features = data['features']
                self.waypoint_list_geo = []
                for feature in features:
                    coords = feature['geometry']['coordinates']

                    waypoint_msg = Waypoint()
                    waypoint_msg.latitude = coords[1]
                    waypoint_msg.longitude = coords[0]
                    waypoint_msg.elevation = coords[2]  
                    self.waypoint_list_geo.append(waypoint_msg)
                
                response.success = True
                self.get_logger().info(f"Task 1: Loaded waypoints: {self.waypoint_list_geo}")
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {str(e)}")
        
        return response

    def convert_waypoints_to_robot_frame(self):
        # COMPLETE YOUR CODE HERE
        R = 6371e3  # Radius of earth in meters
        initial_latitude_radians = math.radians(self.robot_inital_geo[0])  
        initial_longitude_radians = math.radians(self.robot_inital_geo[1])  
        
        for waypoint in self.waypoint_list_geo:

            # radians conversion for waypoints
            waypoint_latitude_radians = math.radians(waypoint.latitude)
            waypoint_longitude_radians = math.radians(waypoint.longitude)
            
            # formula using the Equirectangular approximation
            X = R * ( waypoint_longitude_radians- initial_longitude_radians) * math.cos(initial_longitude_radians)
            Y = R * (waypoint_latitude_radians - initial_latitude_radians)
           
            # appending in the init list
            self.waypoint_list_robot_frame.append((X, Y))

        self.get_logger().info(
            f"Task 2: Waypoints in robot frame: {self.waypoint_list_robot_frame}"
        )

    def plot_waypoints(self):
        self.get_logger().info(
            f" Task 3: Plot and save a graph of loaded waypoints in robot coordinate frame (png)"
        )
        # COMPLETE YOUR CODE HERE

        # x and y from waypoint_list_robot_frame 
        x = [point[0] for point in self.waypoint_list_robot_frame]
        y = [point[1] for point in self.waypoint_list_robot_frame]
        # no yaw information in the json so zeros
        yaw = [0] * len(self.waypoint_list_robot_frame)

        # plotting
        plt.figure(figsize=(8, 6))
        plt.quiver(x, y, np.cos(yaw), np.sin(yaw))
        plt.scatter(x, y, color='blue', label='Waypoints')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('loaded waypoints in robot coordinate frame')
        plt.grid(True)
        plt.legend()
        plt.savefig('waypoints_r_frame.png')
        plt.show()

    def get_robot_waypoints_callback(self, request, response):
        response.waypoints = self.waypoint_list_geo

        return response

    def reset_waypoints_callback(self, request, response):
        response.waypoints = []

        return response

    def call_set_waypoints_geo(self, request):
        self.future = self.set_waypoint_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    waypoint_manager = WaypointManager()

    set_waypoints_msg = SetWaypoints.Request()
    set_waypoints_msg.file_path = "../waypoints/waypoints.geojson"
    response = waypoint_manager.call_set_waypoints_geo(set_waypoints_msg)
    if response.success == True:
        waypoint_manager.convert_waypoints_to_robot_frame()
        waypoint_manager.plot_waypoints()
    else:
        print("No waypoints loaded")

    rclpy.spin(waypoint_manager)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
