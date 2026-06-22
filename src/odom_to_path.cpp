#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class OdomToPath : public rclcpp::Node
{
public:
    OdomToPath() : Node("odom_to_path_node")
    {
        // Parameters
        this->declare_parameter<std::string>("odom_topic", "/sensing/gnss/robins/ros/gps_odom");
        this->declare_parameter<std::string>("path_topic", "/gps_path");
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<int>("max_path_size", 10000);
        this->declare_parameter<double>("min_distance", 0.1);  // Minimum distance (meters)

        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string path_topic = this->get_parameter("path_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        max_path_size_ = this->get_parameter("max_path_size").as_int();
        min_distance_ = this->get_parameter("min_distance").as_double();

        // Prepare path message
        path_.header.frame_id = frame_id_;

        // Publisher and subscriber
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&OdomToPath::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Odometry to Path converter started");
        RCLCPP_INFO(this->get_logger(), "Odom topic: %s", odom_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Path topic: %s", path_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Min distance: %.2f m", min_distance_);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Skip poses closer than min_distance to avoid redundant path points
        if (!path_.poses.empty())
        {
            auto& last_pose = path_.poses.back().pose.position;
            double dx = msg->pose.pose.position.x - last_pose.x;
            double dy = msg->pose.pose.position.y - last_pose.y;
            double dz = msg->pose.pose.position.z - last_pose.z;
            double distance = sqrt(dx*dx + dy*dy + dz*dz);

            if (distance < min_distance_)
            {
                return;  // Too close, skip
            }
        }

        // Create PoseStamped
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.header.frame_id = frame_id_;
        pose_stamped.pose = msg->pose.pose;

        // Append to path
        path_.poses.push_back(pose_stamped);

        // Enforce maximum path size
        if (path_.poses.size() > static_cast<size_t>(max_path_size_))
        {
            path_.poses.erase(path_.poses.begin());
        }

        // Update path header
        path_.header.stamp = msg->header.stamp;

        // Publish
        path_pub_->publish(path_);
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    nav_msgs::msg::Path path_;
    std::string frame_id_;
    int max_path_size_;
    double min_distance_;
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double last_z_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomToPath>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
