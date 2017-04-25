#include "ros/ros.h"
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <image_transport/image_transport.h> // ROS library that allows sending compressed images
#include <camera_info_manager/camera_info_manager.h> // ROS library that publishes CameraInfo topics
#include <sensor_msgs/CameraInfo.h> // ROS message header for CameraInfo

#include <boost/thread.hpp> // Needed for the nodelet to launch the reading thread.

#include <dynamic_reconfigure/server.h> // Needed for the dynamic_reconfigure gui service to run

#include "usb_cam/usb_cam.h"


namespace usb_cam {
    class UsbCamNodelet: public nodelet::Nodelet {
    public:
        UsbCamNodelet() {}
        ~UsbCamNodelet() {
            // do something
        }

    private:
        // shared image message
        sensor_msgs::Image img_;
        image_transport::CameraPublisher image_pub_;

        // parameters
        std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_;
        //std::string start_service_name_, start_service_name_;
        bool streaming_status_;
        int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_,
                saturation_, sharpness_, focus_, white_balance_, gain_;
        bool autofocus_, autoexposure_, auto_white_balance_;
        boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

        UsbCam cam_;

        void onInit() {
            // Get nodeHandles
            ros::NodeHandle &nh = getMTNodeHandle();
            ros::NodeHandle &pnh = getMTPrivateNodeHandle();
            
            // Get parameters
            // grab the parameters
            pnh.param("video_device", video_device_name_, std::string("/dev/video0"));
            pnh.param("brightness", brightness_, -1); //0-255, -1 "leave alone"
            pnh.param("contrast", contrast_, -1); //0-255, -1 "leave alone"
            pnh.param("saturation", saturation_, -1); //0-255, -1 "leave alone"
            pnh.param("sharpness", sharpness_, -1); //0-255, -1 "leave alone"
            // possible values: mmap, read, userptr
            pnh.param("io_method", io_method_name_, std::string("mmap"));
            pnh.param("image_width", image_width_, 640);
            pnh.param("image_height", image_height_, 480);
            pnh.param("framerate", framerate_, 30);
            // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
            pnh.param("pixel_format", pixel_format_name_, std::string("mjpeg"));
            // enable/disable autofocus
            pnh.param("autofocus", autofocus_, false);
            pnh.param("focus", focus_, -1); //0-255, -1 "leave alone"
            // enable/disable autoexposure
            pnh.param("autoexposure", autoexposure_, true);
            pnh.param("exposure", exposure_, 100);
            pnh.param("gain", gain_, -1); //0-100?, -1 "leave alone"
            // enable/disable auto white balance temperature
            pnh.param("auto_white_balance", auto_white_balance_, true);
            pnh.param("white_balance", white_balance_, 4000);

            // load the camera info
            pnh.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
            pnh.param("camera_name", camera_name_, std::string("head_camera"));
            pnh.param("camera_info_url", camera_info_url_, std::string(""));
            cinfo_.reset(new camera_info_manager::CameraInfoManager(pnh, camera_name_, camera_info_url_));
        }
    };
}

// declaring the nodelet class
PLUGINLIB_DECLARE_CLASS(usb_cam, UsbCamNodelet, usb_cam::UsbCamNodelet, nodelet::Nodelet);