#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>


#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class FramePublisher : public rclcpp::Node
{
    public:
        FramePublisher() : Node("frame_publisher")
        {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            timer_ = this->create_wall_timer(50ms, std::bind(&FramePublisher::broadcast_timer_callback, this));
        }

    private:
        
        std::shared_ptr<rclcpp::TimerBase> timer_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        void broadcast_timer_callback()
        {
            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "base_link";
            t.child_frame_id = "turret_link";

            t.transform.translation.x = 0.5;
            t.transform.translation.y = 0.5;
            t.transform.translation.z = 1.0;

            tf2::Quaternion q;
            q.setRPY(0 ,0 ,0);

            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(t);
        }

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}