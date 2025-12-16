#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SerializePoseGraph, DeserializePoseGraph
import os

class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')
        
        self.declare_parameter('map_file', '/home/unitree/maps/auto_saved_map')
        self.declare_parameter('save_interval_seconds', 180.0)  # Save every 3 minutes
        
        self.map_file = self.get_parameter('map_file').value
        save_interval = self.get_parameter('save_interval_seconds').value
        
        # Ensure directory exists
        os.makedirs(os.path.dirname(self.map_file), exist_ok=True)
        
        self.serialize_client = self.create_client(
            SerializePoseGraph, 
            '/slam_toolbox/serialize_map'
        )
        self.deserialize_client = self.create_client(
            DeserializePoseGraph,
            '/slam_toolbox/deserialize_map'
        )
        
        # Try to load map after 3 seconds
        self.load_timer = self.create_timer(3.0, self.try_load_map)
        self.loaded = False
        
        # Periodic save timer
        self.save_timer = self.create_timer(save_interval, self.periodic_save)
        self.get_logger().info(f"Will auto-save map every {save_interval} seconds")
        
    def try_load_map(self):
        """Try to load existing map on startup"""
        if self.loaded:
            return
            
        if os.path.exists(f"{self.map_file}.posegraph"):
            self.get_logger().info(f"Found existing map: {self.map_file}")
            
            if self.deserialize_client.wait_for_service(timeout_sec=1.0):
                request = DeserializePoseGraph.Request()
                request.filename = self.map_file
                request.match_type = 1
                
                future = self.deserialize_client.call_async(request)
                self.get_logger().info("Loading saved map - SLAM will continue mapping")
                self.loaded = True
            else:
                self.get_logger().warn("SLAM Toolbox not ready, will retry...")
        else:
            self.get_logger().info("No existing map found, starting fresh")
            self.loaded = True
        
        if self.loaded:
            self.destroy_timer(self.load_timer)
    
    def periodic_save(self):
        """Periodically save the map"""
        if not self.serialize_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("SLAM Toolbox serialize service not available")
            return
        
        request = SerializePoseGraph.Request()
        request.filename = self.map_file
        
        future = self.serialize_client.call_async(request)
        future.add_done_callback(self.save_callback)
    
    def save_callback(self, future):
        """Callback when save completes"""
        try:
            result = future.result()
            if result:
                self.get_logger().info(f"✓ Map auto-saved to {self.map_file}")
            else:
                self.get_logger().warn("Map save returned no result")
        except Exception as e:
            self.get_logger().error(f"Map save failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MapManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Try one last save on shutdown
        node.get_logger().info("Shutting down, attempting final save...")
        node.periodic_save()
        import time
        time.sleep(1)  # Give it a moment
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
