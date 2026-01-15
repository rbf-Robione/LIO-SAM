#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class NavSatToOdom : public rclcpp::Node
{
public:
    NavSatToOdom() : Node("navsat_to_odom_node")
    {
        // Parametreleri tanımla
        this->declare_parameter<std::string>("input_topic", "/gps/fix");
        this->declare_parameter<std::string>("output_topic", "/gps/odometry");
        this->declare_parameter<std::string>("frame_id", "odom");
        this->declare_parameter<std::string>("child_frame_id", "base_link");
        this->declare_parameter<bool>("use_first_fix_as_origin", true);

        // Parametreleri al
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        child_frame_id_ = this->get_parameter("child_frame_id").as_string();
        use_first_fix_as_origin_ = this->get_parameter("use_first_fix_as_origin").as_bool();

        // Publisher ve Subscriber oluştur
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);
        navsat_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            input_topic, 10,
            std::bind(&NavSatToOdom::navSatCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "NavSat to Odometry converter başlatıldı");
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic.c_str());
    }

private:
    void navSatCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // GPS fix kalitesini kontrol et
        if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                 "GPS fix yok, odometry yayınlanmıyor");
            return;
        }

        // İlk GPS fix'i origin olarak kullan
        if (!origin_set_ && use_first_fix_as_origin_)
        {
            origin_lat_ = msg->latitude;
            origin_lon_ = msg->longitude;
            origin_alt_ = msg->altitude;
            origin_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Origin set to: lat=%.8f, lon=%.8f, alt=%.2f",
                        origin_lat_, origin_lon_, origin_alt_);
        }

        // GPS koordinatlarını lokal koordinatlara çevir
        double x, y, z;
        latLonToXY(msg->latitude, msg->longitude, msg->altitude, x, y, z);

        // Odometry mesajı oluştur
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header = msg->header;
        odom_msg.header.frame_id = frame_id_;
        odom_msg.child_frame_id = child_frame_id_;

        // Pozisyon
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = z;

        // Oryantasyon (yön bilgisi yoksa identity quaternion)
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = 0.0;
        odom_msg.pose.pose.orientation.w = 1.0;

        // Covariance (GPS covariance'ından al)
        // Position covariance
        if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
        {
            // GPS covariance'ı ENU formatında (East, North, Up)
            // Odometry covariance'ı 6x6 matrix: [x, y, z, rotation_x, rotation_y, rotation_z]
            odom_msg.pose.covariance[0] = msg->position_covariance[0];  // x (East)
            odom_msg.pose.covariance[7] = msg->position_covariance[4];  // y (North)
            odom_msg.pose.covariance[14] = msg->position_covariance[8]; // z (Up)
        }
        else
        {
            // Varsayılan covariance değerleri
            odom_msg.pose.covariance[0] = 10.0;  // x
            odom_msg.pose.covariance[7] = 10.0;  // y
            odom_msg.pose.covariance[14] = 10.0; // z
        }

        // Oryantasyon için yüksek covariance (bilinmiyor)
        odom_msg.pose.covariance[21] = 1000.0; // rotation_x
        odom_msg.pose.covariance[28] = 1000.0; // rotation_y
        odom_msg.pose.covariance[35] = 1000.0; // rotation_z

        // Hız hesaplama (basit türev yaklaşımı)
        if (last_fix_time_.seconds() > 0.0)
        {
            double dt = (rclcpp::Time(msg->header.stamp) - last_fix_time_).seconds();
            if (dt > 0.0 && dt < 1.0) // Makul bir zaman aralığı
            {
                odom_msg.twist.twist.linear.x = (x - last_x_) / dt;
                odom_msg.twist.twist.linear.y = (y - last_y_) / dt;
                odom_msg.twist.twist.linear.z = (z - last_z_) / dt;

                // Hız covariance
                odom_msg.twist.covariance[0] = 1.0;  // vx
                odom_msg.twist.covariance[7] = 1.0;  // vy
                odom_msg.twist.covariance[14] = 1.0; // vz
            }
        }

        // Son pozisyonu ve zamanı sakla
        last_x_ = x;
        last_y_ = y;
        last_z_ = z;
        last_fix_time_ = rclcpp::Time(msg->header.stamp);

        // Mesajı yayınla
        odom_pub_->publish(odom_msg);
    }

    void latLonToXY(double lat, double lon, double alt, double& x, double& y, double& z)
    {
        // WGS84 elipsoid parametreleri
        const double a = 6378137.0;              // Ekvatoral yarıçap (metre)
        const double e_sq = 0.00669437999014;    // İlk eksantriklik karesi

        // Origin'den farkları hesapla (derece)
        double d_lat = lat - origin_lat_;
        double d_lon = lon - origin_lon_;
        double d_alt = alt - origin_alt_;

        // Ortalama enlem
        double lat_avg = (lat + origin_lat_) / 2.0;
        double lat_avg_rad = lat_avg * M_PI / 180.0;

        // Eğrilik yarıçapları
        double N = a / sqrt(1.0 - e_sq * sin(lat_avg_rad) * sin(lat_avg_rad));
        double M = a * (1.0 - e_sq) / pow(1.0 - e_sq * sin(lat_avg_rad) * sin(lat_avg_rad), 1.5);

        // Lokal koordinatlara çevir (ENU: East-North-Up)
        // x = East, y = North, z = Up
        x = d_lon * M_PI / 180.0 * N * cos(lat_avg_rad);  // East
        y = d_lat * M_PI / 180.0 * M;                      // North
        z = d_alt;                                          // Up
    }

    // ROS publishers ve subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;

    // Parametreler
    std::string frame_id_;
    std::string child_frame_id_;
    bool use_first_fix_as_origin_;

    // Origin koordinatları
    bool origin_set_ = false;
    double origin_lat_ = 0.0;
    double origin_lon_ = 0.0;
    double origin_alt_ = 0.0;

    // Hız hesaplama için son değerler
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double last_z_ = 0.0;
    rclcpp::Time last_fix_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavSatToOdom>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
