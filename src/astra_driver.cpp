/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * Copyright (c) 2016, Orbbec Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Tim Liu (liuhua@orbbec.com)
 */

#include "astra_camera/astra_driver.h"
#include "astra_camera/astra_exception.h"

#include <unistd.h>  
#include <stdlib.h>  
#include <stdio.h>  
#include <sys/shm.h>  

#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

/*rjc*/
/* #include <boost/date_time/posix_time/posix_time.hpp> */
/* #include <boost/thread/thread.hpp> */
#include <mutex>
#include <thread>
#include <functional>
#include <chrono>
#include <utility>
#include <cmath>

using namespace std::chrono_literals;
/*end rjc*/



#define  MULTI_ASTRA 1
namespace astra_wrapper
{

    AstraDriver::AstraDriver(rclcpp::Node::SharedPtr& n, rclcpp::Node::SharedPtr& pnh, size_t width, size_t height, double framerate, size_t dwidth, size_t dheight, double dframerate, PixelFormat dformat) :
        nh_(n),
        pnh_(pnh),
        device_manager_(AstraDeviceManager::getSingelton()),
        config_init_(false),
        color_frame_id_("openni_color_optical_frame"),
        depth_frame_id_("openni_depth_optical_frame"),
        depth_registration_(false),
        data_skip_ir_counter_(0),
        data_skip_color_counter_(0),
        data_skip_depth_counter_ (0),
        ir_subscribers_(false),
        color_subscribers_(false),
        depth_subscribers_(false),
        depth_raw_subscribers_(false),
        can_publish_ir_(true),
        can_publish_color_(true),
        can_publish_depth_(true)
    {

        genVideoModeTableMap();

        readConfigFromParameterServer();

#if MULTI_ASTRA
        int bootOrder, devnums;
        bootOrder = 1;
        devnums = 1;
        if( devnums>1 )
        {
            int shmid;
            char *shm = NULL;
            if(  bootOrder==1 )
            {
                if( (shmid = shmget((key_t)0401, 1, 0666|IPC_CREAT)) == -1 )   
                { 
                    ROS_ERROR("Create Share Memory Error:%s", strerror(errno));
                }
                shm = (char *)shmat(shmid, 0, 0);  
                *shm = 1;
                initDevice();
                ROS_WARN("*********** device_id %s already open device************************ ", device_id_.c_str());
                *shm = 2;
            }
            else 	
            {	
                if( (shmid = shmget((key_t)0401, 1, 0666|IPC_CREAT)) == -1 )   
                { 
                    ROS_ERROR("Create Share Memory Error:%s", strerror(errno));
                }
                shm = (char *)shmat(shmid, 0, 0);
                while( *shm!=bootOrder);
                initDevice();
                ROS_WARN("*********** device_id %s already open device************************ ", device_id_.c_str());
                *shm = (bootOrder+1);
            }
            if(  bootOrder==1 )
            {
                while( *shm!=(devnums+1)) ;
                if(shmdt(shm) == -1)  
                {  
                    ROS_ERROR("shmdt failed\n");  
                } 
                if(shmctl(shmid, IPC_RMID, 0) == -1)  
                {  
                    ROS_ERROR("shmctl(IPC_RMID) failed\n");  
                }
            }
            else
            {
                if(shmdt(shm) == -1)  
                {  
                    ROS_ERROR("shmdt failed\n");  
                } 
            }
        }
        else
        {
            initDevice();
        }
#else
        initDevice();

#endif

        z_scaling_ = 1.0;
        z_offset_mm_ = 0;

        AstraVideoMode color_video_mode{width, height, framerate, PIXEL_FORMAT_RGB888};
        setColorVideoMode(color_video_mode);

        AstraVideoMode depth_video_mode{dwidth, dheight, dframerate, dformat};
        setDepthVideoMode(depth_video_mode);

        advertiseROSTopics();
    }

    void AstraDriver::advertiseROSTopics()
    {
        // Allow remapping namespaces rgb, ir, depth, depth_registered

        // Prevent connection callbacks from executing until we've set all the publishers. Otherwise
        // connectCb() can fire while we're advertising (say) "depth/image_raw", but before we actually
        // assign to pub_depth_raw_. Then pub_depth_raw_.getNumSubscribers() returns 0, and we fail to start
        // the depth generator.
        /*rjc*/
        /* boost::lock_guard<boost::mutex> lock(connect_mutex_); */
        std::lock_guard<std::mutex> lock(connect_mutex_);
        /*end rjc*/

        rclcpp::QoS my_qos(rclcpp::KeepLast(5), rmw_qos_profile_sensor_data);

        // Asus Xtion PRO does not have an RGB camera
        if (device_->hasColorSensor())
        {
            pub_color_ = nh_->create_publisher<sensor_msgs::msg::Image>("image", my_qos);
            this->colorConnectCb();
        }

        if (device_->hasIRSensor())
        {
            pub_ir_ = nh_->create_publisher<sensor_msgs::msg::Image>("ir_image", my_qos);
            this->irConnectCb();
        }

        if (device_->hasDepthSensor())
        {
            pub_depth_raw_ = nh_->create_publisher<sensor_msgs::msg::Image>("depth", my_qos);
            pub_depth_camera_info_ = nh_->create_publisher<sensor_msgs::msg::CameraInfo>("depth_camera_info", my_qos);
            this->depthConnectCb();
        }

        ////////// CAMERA INFO MANAGER

        // Pixel offset between depth and IR images.
        // By default assume offset of (5,4) from 9x7 correlation window.
        // NOTE: These are now (temporarily?) dynamically reconfigurable, to allow tweaking.
        //param_nh.param("depth_ir_offset_x", depth_ir_offset_x_, 5.0);
        //param_nh.param("depth_ir_offset_y", depth_ir_offset_y_, 4.0);

        // The camera names are set to [rgb|depth]_[serial#], e.g. depth_B00367707227042B.
        // camera_info_manager substitutes this for ${NAME} in the URL.
        std::string serial_number = device_->getStringID();
        std::string color_name, ir_name;

        color_name = "rgb_"   + serial_number;
        ir_name  = "depth_" + serial_number;
    }


    void AstraDriver::setIRVideoMode(const AstraVideoMode& ir_video_mode)
    {
        if (device_->isIRVideoModeSupported(ir_video_mode))
        {
            if (ir_video_mode != device_->getIRVideoMode())
            {
                device_->setIRVideoMode(ir_video_mode);
            }

        }
        else
        {
            ROS_ERROR_STREAM("Unsupported IR video mode - " << ir_video_mode);
        }
    }
    void AstraDriver::setColorVideoMode(const AstraVideoMode& color_video_mode)
    {
        if (device_->isColorVideoModeSupported(color_video_mode))
        {
            if (color_video_mode != device_->getColorVideoMode())
            {
                device_->setColorVideoMode(color_video_mode);
            }
        }
        else
        {
            ROS_ERROR_STREAM("Unsupported color video mode - " << color_video_mode);
        }
    }
    void AstraDriver::setDepthVideoMode(const AstraVideoMode& depth_video_mode)
    {
        if (device_->isDepthVideoModeSupported(depth_video_mode))
        {
            if (depth_video_mode != device_->getDepthVideoMode())
            {
                device_->setDepthVideoMode(depth_video_mode);
            }
        }
        else
        {
            ROS_ERROR_STREAM("Unsupported depth video mode - " << depth_video_mode);
        }
    }

    void AstraDriver::colorConnectCb()
    {
        if (!device_->isColorStreamStarted())
        {
            if(can_publish_color_)
            {
                // Can't stream IR and RGB at the same time. Give RGB preference.
                if (device_->isIRStreamStarted())
                {
                    ROS_ERROR("Cannot stream RGB and IR at the same time. Streaming RGB only.");
                    ROS_INFO("Stopping IR stream.");
                    device_->stopIRStream();
                }

                /*rjc*/
                /* device_->setColorFrameCallback(boost::bind(&AstraDriver::newColorFrameCallback, this, _1)); */
                device_->setColorFrameCallback(std::bind(&AstraDriver::newColorFrameCallback, this, std::placeholders::_1));
                /*end rjc*/

                ROS_INFO("Starting color stream.");
                device_->startColorStream();
            }
            else
            {
                ROS_INFO("Attempted to start RGB stream, but RGB streaming was disabled.");
            }
        }
        //else if (!color_subscribers_ && device_->isColorStreamStarted())
        else if (device_->isColorStreamStarted())
        {
            ROS_INFO("Stopping color stream.");
            device_->stopColorStream();

            // TODO(Kukanani): With the IR subscriber check commented out, this section just
            // blindly starts streaming IR data when the RGB stream is shut down.
            // Start IR if it's been blocked on RGB subscribers
            //bool need_ir = pub_ir_.getNumSubscribers() > 0;
            //if (need_ir && !device_->isIRStreamStarted())
            if (!device_->isIRStreamStarted())
            {
                irAttemptStream();
            }
        }
    }

    void AstraDriver::depthConnectCb()
    {
        if (!device_->isDepthStreamStarted())
        {
            if(can_publish_depth_)
            {
                /*rjc*/
                /* device_->setDepthFrameCallback(boost::bind(&AstraDriver::newDepthFrameCallback, this, _1)); */
                device_->setDepthFrameCallback(std::bind(&AstraDriver::newDepthFrameCallback, this, std::placeholders::_1));
                /*end rjc*/

                ROS_INFO("Starting depth stream.");
                device_->startDepthStream();
            }
            else
            {
                ROS_INFO("Attempted to start depth stream, but depth streaming was disabled.");
            }
        }
        //else if (!need_depth && device_->isDepthStreamStarted())
        else if (device_->isDepthStreamStarted())
        {
            ROS_INFO("Stopping depth stream.");
            device_->stopDepthStream();
        }
    }

    void AstraDriver::irConnectCb()
    {
        if (!device_->isIRStreamStarted())
        {
            // Can't stream IR and RGB at the same time
            if (device_->isColorStreamStarted())
            {
                ROS_ERROR("Cannot stream RGB and IR at the same time. Streaming RGB only.");
            }
            else
            {
                irAttemptStream();
            }
        }
        //else if (!ir_subscribers_ && device_->isIRStreamStarted())
        else if (device_->isIRStreamStarted())
        {
            ROS_INFO("Stopping IR stream.");
            device_->stopIRStream();
        }
    }

    void AstraDriver::irAttemptStream()
    {
        if(can_publish_ir_)
        {
            /*rjc*/
            /* device_->setIRFrameCallback(boost::bind(&AstraDriver::newIRFrameCallback, this, _1)); */
            device_->setIRFrameCallback(std::bind(&AstraDriver::newIRFrameCallback, this, std::placeholders::_1));
            /*end rjc*/

            ROS_INFO("Starting IR stream.");
            device_->startIRStream();
        }
        else
        {
            ROS_INFO("Attemped to start IR stream, but IR streaming was disabled.");
        }
    }

    void AstraDriver::newIRFrameCallback(sensor_msgs::msg::Image::SharedPtr image)
    {
        data_skip_ir_counter_ = 0;
        /*rjc*/
        //sensor_msgs::msg::Image::UniquePtr my_img = std::make_unique<sensor_msgs::msg::Image>(image);
        image->header.frame_id = ir_frame_id_;
        //my_img->header.frame_id = ir_frame_id_;
        /* pub_ir_->publish(image); */
        pub_ir_->publish(*image);
        //pub_ir_->publish(std::move(my_img));
        /*end rjc*/
    }

    void AstraDriver::newColorFrameCallback(sensor_msgs::msg::Image::SharedPtr image)
    {
        data_skip_color_counter_ = 0;

        image->header.frame_id = color_frame_id_;
        /*rjc*/
        /* pub_color_->publish(image); */
        pub_color_->publish(*image);
        /*end rjc*/
    }

    void AstraDriver::newDepthFrameCallback(sensor_msgs::msg::Image::SharedPtr image)
    {
        data_skip_depth_counter_ = 0;

        if (z_offset_mm_ != 0)
        {
            uint16_t* data = reinterpret_cast<uint16_t*>(&image->data[0]);
            for (unsigned int i = 0; i < image->width * image->height; ++i)
            {
                if (data[i] != 0)
                {
                    data[i] += z_offset_mm_;
                }
            }
        }

        /*rjc*/
        /* if (fabs(z_scaling_ - 1.0) > 1e-6) */
        if (std::fabs(z_scaling_ - 1.0) > 1e-6)
        /*end rjc*/
        {
            uint16_t* data = reinterpret_cast<uint16_t*>(&image->data[0]);
            for (unsigned int i = 0; i < image->width * image->height; ++i)
            {
                if (data[i] != 0)
                {
                    data[i] = static_cast<uint16_t>(data[i] * z_scaling_);
                }
            }
        }

        sensor_msgs::msg::CameraInfo::SharedPtr cam_info;

        if (depth_registration_)
        {
            image->header.frame_id = color_frame_id_;
            cam_info = getColorCameraInfo(image->width,image->height, image->header.stamp);
        }
        else
        {
            image->header.frame_id = depth_frame_id_;
            cam_info = getDepthCameraInfo(image->width,image->height, image->header.stamp);
        }

        sensor_msgs::msg::Image::SharedPtr floating_point_image = rawToFloatingPointConversion(image);
        /*rjc*/
        /* pub_depth_raw_->publish(floating_point_image); */
        pub_depth_raw_->publish(*floating_point_image);
        /* pub_depth_camera_info_->publish(getDepthCameraInfo(image->width, image->height, image->header.stamp)); */
        pub_depth_camera_info_->publish(*getDepthCameraInfo(image->width, image->height, image->header.stamp));
        /*end rjc*/
    }

    // Methods to get calibration parameters for the various cameras
    sensor_msgs::msg::CameraInfo::SharedPtr AstraDriver::getDefaultCameraInfo(int width, int height, double f) const
    {
        sensor_msgs::msg::CameraInfo::SharedPtr info = std::make_shared<sensor_msgs::msg::CameraInfo>();

        info->width  = width;
        info->height = height;

        // No distortion
        info->d.resize(5, 0.0);
        info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

        // Simple camera matrix: square pixels (fx = fy), principal point at center
        info->k.fill(0.0);
        info->k[0] = info->k[4] = f;
        info->k[2] = (width / 2) - 0.5;
        // Aspect ratio for the camera center on Astra (and other devices?) is 4/3
        // This formula keeps the principal point the same in VGA and SXGA modes
        info->k[5] = (width * (3./8.)) - 0.5;
        info->k[8] = 1.0;

        // No separate rectified image plane, so R = I
        info->r.fill(0.0);
        info->r[0] = info->r[4] = info->r[8] = 1.0;

        // Then P=K(I|0) = (K|0)
        info->p.fill(0.0);
        info->p[0]  = info->p[5] = f; // fx, fy
        info->p[2]  = info->k[2];     // cx
        info->p[6]  = info->k[5];     // cy
        info->p[10] = 1.0;

        info->binning_x = 0;
        info->binning_y = 0;

        info->roi.x_offset = 0;
        info->roi.y_offset = 0;
        info->roi.height = 0;
        info->roi.width = 0;
        info->roi.do_rectify = 0;

        return info;
    }

    /// @todo Use binning/ROI properly in publishing camera infos
    sensor_msgs::msg::CameraInfo::SharedPtr AstraDriver::getColorCameraInfo(int width, int height, builtin_interfaces::msg::Time time) const
    {
        sensor_msgs::msg::CameraInfo::SharedPtr info;

        info = getDefaultCameraInfo(width, height, device_->getColorFocalLength(height));

        // Fill in header
        info->header.stamp    = time;
        info->header.frame_id = color_frame_id_;

        return info;
    }

    sensor_msgs::msg::CameraInfo::SharedPtr AstraDriver::getIRCameraInfo(int width, int height, builtin_interfaces::msg::Time time) const
    {
        sensor_msgs::msg::CameraInfo::SharedPtr info;

        {
            // If uncalibrated, fill in default values
            info = getDefaultCameraInfo(width, height, device_->getDepthFocalLength(height));
        }

        // Fill in header
        info->header.stamp    = time;
        info->header.frame_id = depth_frame_id_;

        return info;
    }

    sensor_msgs::msg::CameraInfo::SharedPtr AstraDriver::getDepthCameraInfo(int width, int height, builtin_interfaces::msg::Time time) const
    {
        // The depth image has essentially the same intrinsics as the IR image, BUT the
        // principal point is offset by half the size of the hardware correlation window
        // (probably 9x9 or 9x7 in 640x480 mode). See http://www.ros.org/wiki/kinect_calibration/technical

        double scaling = (double)width / 640;

        sensor_msgs::msg::CameraInfo::SharedPtr info = getIRCameraInfo(width, height, time);
        info->k[2] -= depth_ir_offset_x_*scaling; // cx
        info->k[5] -= depth_ir_offset_y_*scaling; // cy
        info->p[2] -= depth_ir_offset_x_*scaling; // cx
        info->p[6] -= depth_ir_offset_y_*scaling; // cy

        /// @todo Could put this in projector frame so as to encode the baseline in P[3]
        return info;
    }

    void AstraDriver::readConfigFromParameterServer()
    {
        // Load frame name parameters
        // TODO(Kukanani): reinstate ir_frame_id_ as listed in the commented section below
        pnh_->get_parameter("depth_frame_id", depth_frame_id_);
        ROS_INFO_STREAM("Using depth frame id " << depth_frame_id_);

        pnh_->get_parameter("color_frame_id", color_frame_id_);
        ROS_INFO_STREAM("Using color frame id " << color_frame_id_);

        // Load channel disabling parameters
        pnh_->get_parameter("use_ir", can_publish_ir_);
        if(!can_publish_ir_)
        {
            ROS_INFO("Astra IR camera disabled");
        }
        pnh_->get_parameter("use_color", can_publish_color_);
        if(!can_publish_color_)
        {
            ROS_INFO("Astra RGB camera disabled");
        }
        pnh_->get_parameter("use_depth", can_publish_depth_);
        if(!can_publish_depth_)
        {
            ROS_INFO("Astra depth camera disabled");
        }

        // Load the depth registration parameter, which may have been set before
        //   driver initialization.
        pnh_->get_parameter("depth_registration", depth_registration_);
        if(depth_registration_)
        {
            ROS_INFO("Astra depth registration enabled");
        }
    }

    std::string AstraDriver::resolveDeviceURI(const std::string& device_id) throw(AstraException)
    {
        // retrieve available device URIs, they look like this: "1d27/0601@1/5"
        // which is <vendor ID>/<product ID>@<bus number>/<device number>
        /*rjc*/
        /* boost::shared_ptr<std::vector<std::string> > available_device_URIs = */
        /*     device_manager_->getConnectedDeviceURIs(); */
        std::shared_ptr<std::vector<std::string>> available_device_URIs =
            device_manager_->getConnectedDeviceURIs();
        /*end rjc*/

        //for tes
#if 0
        for (size_t i = 0; i < available_device_URIs->size(); ++i)
        {
            std::string s = (*available_device_URIs)[i];
            ROS_WARN("------------id %d, available_device_uri is %s-----------", i, s.c_str());
        }
#endif
        //end
        // look for '#<number>' format
        if (device_id.size() > 1 && device_id[0] == '#')
        {
            std::istringstream device_number_str(device_id.substr(1));
            unsigned int device_number;
            device_number_str >> device_number;
            if (device_number == 0 || device_number > available_device_URIs->size())
            {
                THROW_OPENNI_EXCEPTION(
                        "Invalid device number %i, there are %zu devices connected.",
                        device_number, available_device_URIs->size());
            }
            else
            {
                return available_device_URIs->at(device_number - 1);  // #1 refers to first device
            }
        }
        // look for '<bus>@<number>' format
        //   <bus>    is usb bus id, typically start at 1
        //   <number> is the device number, for consistency with astra_camera, these start at 1
        //               although 0 specifies "any device on this bus"
        else if (device_id.size() > 1 && device_id.find('@') != std::string::npos && device_id.find('/') == std::string::npos)
        {
            // get index of @ character
            size_t index = device_id.find('@');
            if (index <= 0)
            {
                THROW_OPENNI_EXCEPTION(
                        "%s is not a valid device URI, you must give the bus number before the @.",
                        device_id.c_str());
            }
            if (index >= device_id.size() - 1)
            {
                THROW_OPENNI_EXCEPTION(
                        "%s is not a valid device URI, you must give a number after the @, specify 0 for first device",
                        device_id.c_str());
            }

            // pull out device number on bus
            std::istringstream device_number_str(device_id.substr(index+1));
            int device_number;
            device_number_str >> device_number;

            // reorder to @<bus>
            std::string bus = device_id.substr(0, index);
            bus.insert(0, "@");

            for (size_t i = 0; i < available_device_URIs->size(); ++i)
            {
                std::string s = (*available_device_URIs)[i];
                if (s.find(bus) != std::string::npos)
                {
                    // this matches our bus, check device number
                    --device_number;
                    if (device_number <= 0)
                        return s;
                }
            }

            THROW_OPENNI_EXCEPTION("Device not found %s", device_id.c_str());
        }
        else
        {
            // check if the device id given matches a serial number of a connected device
            for(std::vector<std::string>::const_iterator it = available_device_URIs->begin();
                    it != available_device_URIs->end(); ++it)
            {
#if 0
                try 
                {
                    std::string serial = device_manager_->getSerial(*it);
                    if (serial.size() > 0 && device_id == serial)
                        return *it;
                }
#else
                try 
                {
                    std::set<std::string>::iterator iter;
                    if((iter = alreadyOpen.find(*it)) == alreadyOpen.end())
                    {
                        // ROS_WARN("------------seraial num it is  %s, device_id is %s -----------", (*it).c_str(), device_id_.c_str());
                        std::string serial = device_manager_->getSerial(*it);
                        if (serial.size() > 0 && device_id == serial)
                        {
                            alreadyOpen.insert(*it);
                            return *it;
                        }
                    }
                }
#endif
                catch (const AstraException& exception)
                {
                    ROS_WARN("Could not query serial number of device \"%s\":", exception.what());
                }
            }

            // everything else is treated as part of the device_URI
            bool match_found = false;
            std::string matched_uri;
            for (size_t i = 0; i < available_device_URIs->size(); ++i)
            {
                std::string s = (*available_device_URIs)[i];
                if (s.find(device_id) != std::string::npos)
                {
                    if (!match_found)
                    {
                        matched_uri = s;
                        match_found = true;
                    }
                    else
                    {
                        // more than one match
                        THROW_OPENNI_EXCEPTION("Two devices match the given device id '%s': %s and %s.", device_id.c_str(), matched_uri.c_str(), s.c_str());
                    }
                }
            }
            return matched_uri;
        }

        return "INVALID";
    }

    void AstraDriver::initDevice()
    {
        while (rclcpp::ok() && !device_)
        {
            try
            {
                std::string device_URI = resolveDeviceURI(device_id_);
#if 0
                if( device_URI == "" ) 
                {
                    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                    continue;
                }
#endif
                device_ = device_manager_->getDevice(device_URI);
            }
            catch (const AstraException& exception)
            {
                if (!device_)
                {
                    ROS_INFO("No matching device found.... waiting for devices. Reason: %s", exception.what());
                    /*rjc*/
                    /* boost::this_thread::sleep(boost::posix_time::seconds(3)); */
                    std::this_thread::sleep_for(3s);
                    /*end rjc*/
                    continue;
                }
                else
                {
                    ROS_ERROR("Could not retrieve device. Reason: %s", exception.what());
                    exit(-1);
                }
            }
        }

        while (rclcpp::ok() && !device_->isValid())
        {
            ROS_DEBUG("Waiting for device initialization..");
            /*rjc*/
            /* boost::this_thread::sleep(boost::posix_time::milliseconds(100)); */
            std::this_thread::sleep_for(100ms);
            /*end rjc*/
        }

    }

    void AstraDriver::genVideoModeTableMap()
    {

        video_modes_lookup_.clear();

        AstraVideoMode video_mode;

        // SXGA_30Hz
        video_mode.x_resolution_ = 1280;
        video_mode.y_resolution_ = 1024;
        video_mode.frame_rate_ = 30;

        video_modes_lookup_[1] = video_mode;

        // SXGA_15Hz
        video_mode.x_resolution_ = 1280;
        video_mode.y_resolution_ = 1024;
        video_mode.frame_rate_ = 15;

        video_modes_lookup_[2] = video_mode;

        // XGA_30Hz
        video_mode.x_resolution_ = 1280;
        video_mode.y_resolution_ = 720;
        video_mode.frame_rate_ = 30;

        video_modes_lookup_[3] = video_mode;

        // XGA_15Hz
        video_mode.x_resolution_ = 1280;
        video_mode.y_resolution_ = 720;
        video_mode.frame_rate_ = 15;

        video_modes_lookup_[4] = video_mode;

        // VGA_30Hz
        video_mode.x_resolution_ = 640;
        video_mode.y_resolution_ = 480;
        video_mode.frame_rate_ = 30;

        video_modes_lookup_[5] = video_mode;

        // VGA_25Hz
        video_mode.x_resolution_ = 640;
        video_mode.y_resolution_ = 480;
        video_mode.frame_rate_ = 25;

        video_modes_lookup_[6] = video_mode;

        // QVGA_25Hz
        video_mode.x_resolution_ = 320;
        video_mode.y_resolution_ = 240;
        video_mode.frame_rate_ = 25;

        video_modes_lookup_[7] = video_mode;

        // QVGA_30Hz
        video_mode.x_resolution_ = 320;
        video_mode.y_resolution_ = 240;
        video_mode.frame_rate_ = 30;

        video_modes_lookup_[8] = video_mode;

        // QVGA_60Hz
        video_mode.x_resolution_ = 320;
        video_mode.y_resolution_ = 240;
        video_mode.frame_rate_ = 60;

        video_modes_lookup_[9] = video_mode;

        // QQVGA_25Hz
        video_mode.x_resolution_ = 160;
        video_mode.y_resolution_ = 120;
        video_mode.frame_rate_ = 25;

        video_modes_lookup_[10] = video_mode;

        // QQVGA_30Hz
        video_mode.x_resolution_ = 160;
        video_mode.y_resolution_ = 120;
        video_mode.frame_rate_ = 30;

        video_modes_lookup_[11] = video_mode;

        // QQVGA_60Hz
        video_mode.x_resolution_ = 160;
        video_mode.y_resolution_ = 120;
        video_mode.frame_rate_ = 60;

        video_modes_lookup_[12] = video_mode;

    }

    int AstraDriver::lookupVideoModeFromDynConfig(int mode_nr, AstraVideoMode& video_mode)
    {
        int ret = -1;

        std::map<int, AstraVideoMode>::const_iterator it;

        it = video_modes_lookup_.find(mode_nr);

        if (it != video_modes_lookup_.end())
        {
            video_mode = it->second;
            ret = 0;
        }

        return ret;
    }

    sensor_msgs::msg::Image::SharedPtr AstraDriver::rawToFloatingPointConversion(sensor_msgs::msg::Image::SharedPtr raw_image)
    {
        static const float bad_point = std::numeric_limits<float>::quiet_NaN ();

        sensor_msgs::msg::Image::SharedPtr new_image = std::make_shared<sensor_msgs::msg::Image>();

        new_image->header = raw_image->header;
        new_image->width = raw_image->width;
        new_image->height = raw_image->height;
        new_image->is_bigendian = 0;
        new_image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        new_image->step = sizeof(float)*raw_image->width;

        std::size_t data_size = new_image->width*new_image->height;
        new_image->data.resize(data_size*sizeof(float));

        const unsigned short* in_ptr = reinterpret_cast<const unsigned short*>(&raw_image->data[0]);
        float* out_ptr = reinterpret_cast<float*>(&new_image->data[0]);

        for (std::size_t i = 0; i<data_size; ++i, ++in_ptr, ++out_ptr)
        {
            if (*in_ptr==0 || *in_ptr==0x7FF)
            {
                *out_ptr = bad_point;
            }
            else
            {
                *out_ptr = static_cast<float>(*in_ptr)/1000.0f;
            }
        }

        return new_image;
    }
}
