#ifndef ASTRA_WRAPPER__ASTRA_DRIVER_HPP_
#define ASTRA_WRAPPER__ASTRA_DRIVER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "astra_camera/astra_device_manager.h"
#include "astra_camera/astra_device.h"
#include "astra_camera/astra_video_mode.h"

#include <rcl/time.h>
#include <rclcpp/rclcpp.hpp>

namespace astra_wrapper
{

    class AstraDriver : public rclcpp::Node
    {
        public:
            explicit AstraDriver(const rclcpp::NodeOptions & options);

        private:
            void newIRFrameCallback(sensor_msgs::msg::Image::SharedPtr image);
            void newColorFrameCallback(sensor_msgs::msg::Image::SharedPtr image);
            void newDepthFrameCallback(sensor_msgs::msg::Image::SharedPtr image);

            // Methods to get calibration parameters for the various cameras
            sensor_msgs::msg::CameraInfo::SharedPtr getDefaultCameraInfo(int width, int height, double f) const;
            sensor_msgs::msg::CameraInfo::SharedPtr getColorCameraInfo(int width, int height, builtin_interfaces::msg::Time time) const;
            sensor_msgs::msg::CameraInfo::SharedPtr getIRCameraInfo(int width, int height, builtin_interfaces::msg::Time time) const;
            sensor_msgs::msg::CameraInfo::SharedPtr getDepthCameraInfo(int width, int height, builtin_interfaces::msg::Time time) const;

            // resolves non-URI device IDs to URIs, e.g. '#1' is resolved to the URI of the first device
            std::string resolveDeviceURI(const std::string& device_id);
            void initDevice();

            void advertiseROSTopics();

            void colorConnectCb();
            void depthConnectCb();
            void irConnectCb();

            /// Start the IR stream unless IR streaming is disabled. Because of the
            ///   restriction on IR/RGB simultaneous streaming, IR streaming is initiated
            ///   from multiple methods.
            void irAttemptStream();

            void genVideoModeTableMap();
            int lookupVideoModeFromDynConfig(int mode_nr, AstraVideoMode& video_mode);

            sensor_msgs::msg::Image::SharedPtr rawToFloatingPointConversion(sensor_msgs::msg::Image::SharedPtr raw_image);

            void setIRVideoMode(const AstraVideoMode& ir_video_mode);
            void setColorVideoMode(const AstraVideoMode& color_video_mode);
            void setDepthVideoMode(const AstraVideoMode& depth_video_mode);

            std::shared_ptr<AstraDeviceManager> device_manager_;
            std::shared_ptr<AstraDevice> device_;

            std::string device_id_;

            bool config_init_;

            std::set<std::string>  alreadyOpen;
            std::mutex connect_mutex_;

            // published topics
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_raw_;
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_;
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ir_;
            rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_depth_camera_info_;


            AstraVideoMode ir_video_mode_;
            AstraVideoMode color_video_mode_;
            AstraVideoMode depth_video_mode_;

            std::string ir_frame_id_;
            std::string color_frame_id_;
            std::string depth_frame_id_ ;

            std::string color_info_url_, ir_info_url_;

            bool color_depth_synchronization_;
            bool depth_registration_;

            std::map<int, AstraVideoMode> video_modes_lookup_;

            // dynamic reconfigure config
            double depth_ir_offset_x_;
            double depth_ir_offset_y_;
            int z_offset_mm_;
            double z_scaling_;

            rcl_duration_t ir_time_offset_;
            rcl_duration_t color_time_offset_;
            rcl_duration_t depth_time_offset_;

            int data_skip_;

            int data_skip_ir_counter_;
            int data_skip_color_counter_;
            int data_skip_depth_counter_;

            bool auto_exposure_;
            bool auto_white_balance_;

            bool ir_subscribers_;
            bool color_subscribers_;
            bool depth_subscribers_;
            bool depth_raw_subscribers_;

            /// If false, then camera will never start an IR stream.
            bool can_publish_ir_;
            /// If false, then camera will never start a color stream.
            bool can_publish_color_;
            /// If false, then camera will never start a depth stream.
            bool can_publish_depth_;
    };

} // namespace astra_wrapper

#endif
