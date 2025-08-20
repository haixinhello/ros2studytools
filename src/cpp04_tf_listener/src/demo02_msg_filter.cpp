#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
using namespace std::chrono_literals;

class TFPointListener: public rclcpp::Node
{
public:
    TFPointListener():Node("tf_point_listener_node_cpp")
    {
        buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
        timer_=std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface());
        buffer_->setCreateTimerInterface(timer_);
        listener_=std::make_shared<tf2_ros::TransformListener>(*buffer_);
        point_sub.subscribe(this,"point");
        filter_=std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>>(
            point_sub,
            *buffer_,
            "base_link",
            10,
            this->get_node_logging_interface(),
            this->get_node_clock_interface(),
            1s
        );
        filter_->registerCallback(&TFPointListener::transform_point,this);
    }
private:
    void transform_point(const geometry_msgs::msg::PointStamped & ps)
    {
        auto out=buffer_->transform(ps,"base_link");
        RCLCPP_INFO(this->get_logger(),"父级坐标系:%s,坐标(%.2f,%.2f,%.2f)",
        out.header.frame_id.c_str(),
        out.point.x,
        out.point.y,
        out.point.z
    );
    }
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    std::shared_ptr<tf2_ros::CreateTimerROS> timer_;
    message_filters::Subscriber<geometry_msgs::msg::PointStamped> point_sub;
    std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>> filter_;
};


int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TFPointListener>());
    rclcpp::shutdown();
    return 0;
}