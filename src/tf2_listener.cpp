#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <string>
#include <memory>

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
    public:
        FrameListener() : Node("frame_listener")
        {
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            timer_ = this->create_wall_timer(100ms, std::bind(&FrameListener::timer_callback, this));
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        void timer_callback()
        {
            std::string target_frame = "base_link";
            std::string source_frame = "turret_link";

            geometry_msgs::msg::TransformStamped t;

            try
            {
                t = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            }
            catch(const tf2::TransformException & ex)
            {
                RCLCPP_INFO(this->get_logger(), "cannot get transformation: %s", ex.what());
                return;
            }
            
            tf2::Quaternion q(t.transform.rotation.x, 
                              t.transform.rotation.y,
                              t.transform.rotation.z,
                              t.transform.rotation.w);
            
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            RCLCPP_INFO(
                this->get_logger(),
                "[%s -> %s] Translation: X: %.2f, Y: %.2f, Z: %.2f   Rotation: R:%.2f, P: %.2f, Y: %.2f",
                source_frame.c_str(),
                target_frame.c_str(),
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
                roll, 
                pitch,
                yaw
            );
        }

        
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameListener>());
    rclcpp::shutdown();
    return 0;
}