#include "ros/ros.h"
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <image_transport/image_transport.h> // ROS library that allows sending compressed images
#include <camera_info_manager/camera_info_manager.h> // ROS library that publishes CameraInfo topics
#include <sensor_msgs/CameraInfo.h> // ROS message header for CameraInfo

#include <boost/thread.hpp> // Needed for the nodelet to launch the reading thread.

#include <dynamic_reconfigure/server.h> // Needed for the dynamic_reconfigure gui service to run
#include <std_srvs/Empty.h>

#include "usb_cam/usb_cam.h"


namespace usb_cam {

    class UsbCamNodelet: public nodelet::Nodelet {
    public:
        UsbCamNodelet() {}
        ~UsbCamNodelet() {
            boost::mutex::scoped_lock scopedLock(connect_mutex_);

            if (pubThread_) {
                pubThread_->interrupt();
                pubThread_->join();

                cam_.stop_capturing();
            }
            cam_.shutdown();
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
        boost::shared_ptr<image_transport::ImageTransport> it_;
        image_transport::CameraPublisher it_pub_;
        boost::shared_ptr<boost::thread> pubThread_;

        boost::mutex connect_mutex_;

        UsbCam cam_;

        ros::ServiceServer service_start_, service_stop_;

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
            cinfo_.reset(new camera_info_manager::CameraInfoManager(nh, camera_name_, camera_info_url_));

            // create Services
            service_start_ = nh.advertiseService("start_capture", &UsbCamNodelet::service_start_cap, this);
            service_stop_ = nh.advertiseService("stop_capture", &UsbCamNodelet::service_stop_cap, this);

            // check for default camera info
            if (!cinfo_->isCalibrated())
            {
                cinfo_->setCameraName(video_device_name_);
                sensor_msgs::CameraInfo camera_info;
                camera_info.header.frame_id = img_.header.frame_id;
                camera_info.width = (unsigned int)image_width_;
                camera_info.height = (unsigned int)image_height_;
                cinfo_->setCameraInfo(camera_info);
            }

            ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(),
                     video_device_name_.c_str(), image_width_, image_height_, io_method_name_.c_str(),
                     pixel_format_name_.c_str(), framerate_);

            // set the IO method
            UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
            if(io_method == UsbCam::IO_METHOD_UNKNOWN)
            {
                ROS_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
                nh.shutdown();
                return;
            }

            // set the pixel format
            UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
            if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN)
            {
                ROS_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());
                nh.shutdown();
                return;
            }

            // start the camera
            cam_.start(video_device_name_.c_str(), io_method, pixel_format, image_width_,
                       image_height_, framerate_);

            // set camera parameters
            this->set_cam_parameters();

            // create publisher
            it_.reset(new image_transport::ImageTransport(nh));
            image_transport::SubscriberStatusCallback cb = boost::bind(&UsbCamNodelet::connect_callback, this);
            it_pub_ = it_->advertiseCamera("image_raw", 5, cb, cb);
        }

        void connect_callback() {
            NODELET_DEBUG("Connect callback!");
            boost::mutex::scoped_lock scoped_lock(connect_mutex_);
            if (it_pub_.getNumSubscribers() == 0) {
                if (pubThread_) {
                    // disconnect thread
                    NODELET_DEBUG("Disconnecting...");
                    pubThread_->interrupt();
                    scoped_lock.unlock();
                    pubThread_->join();
                    scoped_lock.lock();
                    pubThread_.reset();

                    // stop camera capture
                    NODELET_DEBUG("Stopping camera capture...");
                    cam_.stop_capturing();
                }
            }
            else if (!pubThread_) {
                // need to connect
                pubThread_.reset(new boost::thread(boost::bind(&usb_cam::UsbCamNodelet::take_and_send_image, this)));
            }
            else {
                NODELET_DEBUG("Do nothing in callback.");
            }
        }

        void take_and_send_image() {
            ros::Rate loop_rate(this->framerate_);

            while (!boost::this_thread::interruption_requested()) {
                if (cam_.is_capturing()) {
                    // grab the image
                    cam_.grab_image(&img_);

                    // grab the camera info
                    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
                    ci->header.frame_id = img_.header.frame_id;
                    ci->header.stamp = img_.header.stamp;

                    // publish the image
                    it_pub_.publish(img_, *ci);
                }

                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        void set_cam_parameters() {
            if (brightness_ >= 0)
            {
                cam_.set_v4l_parameter("brightness", brightness_);
            }

            if (contrast_ >= 0)
            {
                cam_.set_v4l_parameter("contrast", contrast_);
            }

            if (saturation_ >= 0)
            {
                cam_.set_v4l_parameter("saturation", saturation_);
            }

            if (sharpness_ >= 0)
            {
                cam_.set_v4l_parameter("sharpness", sharpness_);
            }

            if (gain_ >= 0)
            {
                cam_.set_v4l_parameter("gain", gain_);
            }

            // check auto white balance
            if (auto_white_balance_)
            {
                cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
            }
            else
            {
                cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
                cam_.set_v4l_parameter("white_balance_temperature", white_balance_);
            }

            // check auto exposure
            if (!autoexposure_)
            {
                // turn down exposure control (from max of 3)
                cam_.set_v4l_parameter("exposure_auto", 1);
                // change the exposure level
                cam_.set_v4l_parameter("exposure_absolute", exposure_);
            }

            // check auto focus
            if (autofocus_)
            {
                cam_.set_auto_focus(1);
                cam_.set_v4l_parameter("focus_auto", 1);
            }
            else
            {
                cam_.set_v4l_parameter("focus_auto", 0);
                if (focus_ >= 0)
                {
                    cam_.set_v4l_parameter("focus_absolute", focus_);
                }
            }
        }

        bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res ) {
            cam_.start_capturing();
            return true;
        }


        bool service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res ) {
            cam_.stop_capturing();
            return true;
        }
    };
}

// declaring the nodelet class
PLUGINLIB_DECLARE_CLASS(usb_cam, UsbCamNodelet, usb_cam::UsbCamNodelet, nodelet::Nodelet);