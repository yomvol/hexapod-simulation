#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "gait_engine.hpp"

using namespace std::chrono_literals;

class LegWorkspaceVisualizer : public rclcpp::Node {
public:
  LegWorkspaceVisualizer() : Node("leg_workspace_visualizer"), 
                            gait_engine_(std::make_shared<GaitEngine>(1000)),
                            tf_buffer_(this->get_clock()),
                            tf_listener_(tf_buffer_) {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&LegWorkspaceVisualizer::ComputeWorkspace, this));
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<GaitEngine> gait_engine_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void ComputeWorkspace() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->get_clock()->now();
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    // I'll visualize only left front leg workspace for clarity
    std::string coxa_frame = "coxa1_lf";
    
    try {
      // get transform from coxa frame to base_link
      geometry_msgs::msg::TransformStamped transform_stamped = 
        tf_buffer_.lookupTransform("base_link", coxa_frame, tf2::TimePointZero);

      // sampling the workspace incrementally
      for (double q1 = -M_PI/4; q1 <= M_PI/4; q1 += 0.3) {
        for (double q2 = -M_PI/2; q2 <= M_PI/2; q2 += 0.3) {
          for (double q3 = -M_PI/2; q3 <= M_PI/2; q3 += 0.3) {
            Vec3 leg_tip = gait_engine_->computeLegFK({q1, q2, q3});
            
            // convert to geometry_msgs::msg::Point for transformation
            geometry_msgs::msg::PointStamped leg_tip_point;
            leg_tip_point.header.frame_id = coxa_frame;
            leg_tip_point.header.stamp = this->get_clock()->now();
            leg_tip_point.point.x = leg_tip.x; // adjust for z offset (in meters)
            leg_tip_point.point.y = leg_tip.y;
            leg_tip_point.point.z = leg_tip.z;
            
            // transform to base_link frame
            geometry_msgs::msg::PointStamped point_base;
            tf2::doTransform(leg_tip_point, point_base, transform_stamped);
            
            marker.points.push_back(point_base.point);
          }
        }
      }
      
      marker_pub_->publish(marker);
      RCLCPP_INFO(this->get_logger(), "Published workspace with %zu points", marker.points.size());
      
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform from %s to base_link: %s", 
                 coxa_frame.c_str(), ex.what());
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegWorkspaceVisualizer>());
  rclcpp::shutdown();
  return 0;
}