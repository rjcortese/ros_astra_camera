/*rjc brand new*/

#ifndef ASTRA_CAMERA__PUBLISHER_COMPONENT_HPP_
#define ASTRA_CAMERA__PUBLISHER_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace astra_camera
{

class AstraPublisher : public rclcpp::Node
{
    public:
        explicit AstraPublisher(const rclcpp::NodeOptions & options);

    private:
        std::shared_ptr<AstraDevice> device_;
        std::shared_ptr<AstraDeviceManager> device_manager_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ir_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;

        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ir_camera_info_pub_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_camera_info_pub_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_pub_;
};

} // namespace astra_camera

#endif // ASTRA_CAMERA__PUBLISHER_COMPONENT_HPP_

/*end rjc*/
