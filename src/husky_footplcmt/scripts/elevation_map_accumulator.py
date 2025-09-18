#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from grid_map_msgs.msg import GridMap
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Float32MultiArray
import tf2_ros

class ElevationMapAccumulator(Node):
    def __init__(self):
        super().__init__('elevation_map_accumulator')
        
        # Global map parameters - reduced size for better RViz performance
        self.global_size = 10.0  # 20x20 meters
        self.resolution = 0.1  # 5cm per cell (matching elevation_mapping)
        self.grid_size = int(self.global_size / self.resolution)
        
        # Initialize global elevation and confidence maps
        self.global_elevation = np.full((self.grid_size, self.grid_size), np.nan)
        self.confidence = np.zeros((self.grid_size, self.grid_size))
        
        # TF listener to get robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to local elevation map
        self.local_map_sub = self.create_subscription(
            GridMap,
            '/elevation_map',  # Correct topic name
            self.local_map_callback,
            10
        )
        
        # Publisher for global map
        self.global_map_pub = self.create_publisher(
            GridMap,
            '/global_elevation_map',
            1
        )
        
        # Timer to publish global map at 2Hz
        self.timer = self.create_timer(0.5, self.publish_global_map)
        
        self.get_logger().info('Elevation Map Accumulator started')
        self.get_logger().info(f'Global map: {self.global_size}x{self.global_size}m at {self.resolution}m/cell')
        
    def local_map_callback(self, msg):
        """Process incoming local elevation map and add to global map"""
        self.get_logger().info('Received elevation map', throttle_duration_sec=2.0)
        try:
            # Get robot pose in odom frame (more stable than map)
            transform = self.tf_buffer.lookup_transform(
                'odom',  # Use odom frame to avoid transform issues
                'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            self.get_logger().info(f'Robot at ({robot_x:.2f}, {robot_y:.2f})', throttle_duration_sec=2.0)
            
            # Find elevation layer index
            elevation_idx = None
            for i, layer in enumerate(msg.layers):
                if 'elevation' in layer.lower():
                    elevation_idx = i
                    self.get_logger().debug(f'Found elevation layer at index {i}: {layer}')
                    break
            
            if elevation_idx is None:
                self.get_logger().warn(f'No elevation layer found. Available layers: {msg.layers}')
                return
            
            # Extract local map parameters
            try:
                local_data = np.array(msg.data[elevation_idx].data)
                local_size_x = msg.info.length_x
                local_size_y = msg.info.length_y
                local_resolution = msg.info.resolution
                
                self.get_logger().info(f'Local map: {local_size_x}x{local_size_y}m, resolution: {local_resolution}', 
                                      throttle_duration_sec=5.0)
            except Exception as e:
                self.get_logger().error(f'Error extracting data: {e}')
                return
            
            # Calculate grid dimensions
            local_cols = int(local_size_x / local_resolution)
            local_rows = int(local_size_y / local_resolution)
            
            # Reshape data to 2D grid
            local_grid = local_data.reshape((local_rows, local_cols))
            
            # Transform each cell from local to global coordinates
            for i in range(local_rows):
                for j in range(local_cols):
                    if not np.isnan(local_grid[i, j]):
                        # Local position relative to robot
                        local_x = (j - local_cols/2.0) * local_resolution
                        local_y = (i - local_rows/2.0) * local_resolution
                        
                        # Global position
                        global_x = robot_x + local_x
                        global_y = robot_y + local_y
                        
                        # Convert to global grid indices
                        global_i = int((global_y + self.global_size/2.0) / self.resolution)
                        global_j = int((global_x + self.global_size/2.0) / self.resolution)
                        
                        # Check bounds
                        if 0 <= global_i < self.grid_size and 0 <= global_j < self.grid_size:
                            # Debug: Log when we actually update a cell
                            if np.isnan(self.global_elevation[global_i, global_j]):
                                self.get_logger().debug(f'Adding elevation {local_grid[i, j]:.2f} at ({global_i}, {global_j})')
                            
                            # Update global map with weighted average
                            if np.isnan(self.global_elevation[global_i, global_j]):
                                self.global_elevation[global_i, global_j] = local_grid[i, j]
                                self.confidence[global_i, global_j] = 1.0
                            else:
                                # Weighted average with existing value
                                conf = self.confidence[global_i, global_j]
                                self.global_elevation[global_i, global_j] = \
                                    (self.global_elevation[global_i, global_j] * conf + local_grid[i, j]) / (conf + 1.0)
                                self.confidence[global_i, global_j] = min(conf + 1.0, 10.0)  # Cap confidence
                                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
            
    def publish_global_map(self):
        """Publish the accumulated global elevation map"""
        msg = GridMap()
        # GridMap has header at top level
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'  # Use odom frame to avoid RViz crashes
        msg.info.resolution = self.resolution
        msg.info.length_x = self.global_size
        msg.info.length_y = self.global_size
        msg.info.pose.position.x = 0.0  # Centered at origin
        msg.info.pose.position.y = 0.0
        msg.info.pose.position.z = 0.0
        msg.info.pose.orientation.w = 1.0
        
        # Add elevation layer
        msg.layers.append('elevation')
        elevation_data = Float32MultiArray()
        # Replace NaN with a default value for visualization
        data_to_publish = np.nan_to_num(self.global_elevation, nan=0.0)
        elevation_data.data = data_to_publish.flatten().tolist()
        msg.data.append(elevation_data)
        
        self.global_map_pub.publish(msg)
        
        # Log statistics
        valid_cells = np.sum(~np.isnan(self.global_elevation))
        total_cells = self.grid_size * self.grid_size
        coverage = (valid_cells / total_cells) * 100
        self.get_logger().info(f'Global map coverage: {coverage:.1f}% ({valid_cells}/{total_cells} cells)', 
                              throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = ElevationMapAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gracefully...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()