/*
 * split_bag.cpp
 *
 *  Created on: Oct 19, 2012
 *      Author: jelfring
 */


// ROS
#include <ros/ros.h>

// OpenCv
#include "cv.h"
#include "highgui.h"

// For transforming ROS/OpenCV images
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Messages
#include "sensor_msgs/Image.h"
#include "tf/tfMessage.h"
//#include "wire_msgs/WorldEvidence.h"
#include "sensor_msgs/CameraInfo.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

// Bag files
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <fstream>

// Boost
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include "boost/format.hpp"

using namespace std;

int main( int argc, char** argv ) {

    if (argc != 4) {
        std::cout << "Need 3 arguments: <INPUT.bag> <FRAME_RATE> <OUTPUT>" << std::endl;
        exit(-1);
    }

    // Default values
    std::string bag_in_name = argv[1];
    int video_frame_rate = atoi(argv[2]);
    std::string bag_out_name = std::string(argv[3]) + ".bag";
    std::string yaml_name = std::string(argv[3]) + ".yaml";

    // Check input
    if (bag_in_name == "") {
        ROS_ERROR("No input bag file defined!");
        return -1;
    }
    if (video_frame_rate <= 0) {
        ROS_ERROR("No video frame rate defined!");
        return -1;
    }

    // Video writers
    map<std::string, cv::VideoWriter*> topic_to_video_writer;
    map<std::string, std::string> topic_to_video_filename;

    // Load input bag file
    rosbag::Bag bag;
    bag.open(bag_in_name, rosbag::bagmode::Read);
    ROS_INFO("Opened %s", bag_in_name.c_str());

    // Create output bag file
    rosbag::Bag bag_out;
    bag_out.open(bag_out_name, rosbag::bagmode::Write);
    ROS_INFO("Created %s", bag_out_name.c_str());

    // Get topics
    rosbag::View view(bag);

    // Loop over messages
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {

        // Get data
        sensor_msgs::ImageConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
        tf::tfMessageConstPtr transform = m.instantiate<tf::tfMessage>();
        // wire_msgs::WorldEvidenceConstPtr evidence = m.instantiate<wire_msgs::WorldEvidence>();
        sensor_msgs::CameraInfoConstPtr cam_info = m.instantiate<sensor_msgs::CameraInfo>();

        // Images to video
        if (image_msg != NULL)
        {

            cv::Mat image;

            if (image_msg->encoding == "32FC1")
            { // depth image
                cv_bridge::CvImagePtr depth_image_ptr;
                try
                {
                    depth_image_ptr = cv_bridge::toCvCopy(image_msg);
                } catch (cv_bridge::Exception& e) {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                }

                // find max depth
                float max_depth = 0;
                for(int y = 0; y < depth_image_ptr->image.rows; y++)
                {
                    for(int x = 0; x < depth_image_ptr->image.cols; x++)
                    {
                        float distance = depth_image_ptr->image.at<float>(y, x);
                        if (distance == distance)
                        { // exclude NaN
                            max_depth = max(distance, max_depth);
                        }
                    }
                }

                image = cv::Mat(depth_image_ptr->image.rows, depth_image_ptr->image.cols, CV_8UC3);

                for(int y = 0; y < depth_image_ptr->image.rows; y++)
                {
                    for(int x = 0; x < depth_image_ptr->image.cols; x++)
                    {
                        float distance = depth_image_ptr->image.at<float>(y, x);
                        unsigned int dist_clr = (unsigned int)(distance / max_depth * 255);
                        image.at<cv::Vec3b>(y, x) = cv::Vec3b(dist_clr, dist_clr, dist_clr);
                    }
                }

                std_msgs::Float64 max_depth_msg;
                max_depth_msg.data = max_depth;
                bag_out.write(m.getTopic() +"/_trigger", image_msg->header.stamp, max_depth_msg);

            }
            else if (image_msg->encoding == "rgb8" || image_msg->encoding == "bgr8" )
            { // color image
                // Convert color OpenCV image
                cv_bridge::CvImageConstPtr color_image_ptr;
                try
                {
                    color_image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                }

                image = color_image_ptr->image;

                std_msgs::Bool b;
                b.data = true;
                bag_out.write(m.getTopic() +"/_trigger", image_msg->header.stamp, b);
            }
            else
            {
                ROS_ERROR("Unknown image encoding: %s", image_msg->encoding.c_str());
            }

            //cv::imshow(m.getTopic(), image);
            //cv::waitKey(1);

            cv::VideoWriter* video_writer;
            map<std::string, cv::VideoWriter*>::iterator it_video_writer = topic_to_video_writer.find(m.getTopic());
            if (it_video_writer != topic_to_video_writer.end()) {
                video_writer = it_video_writer->second;
            } else {
                stringstream s_filename;
                s_filename << std::string(argv[3]) << "_" << topic_to_video_writer.size() << ".avi";
                topic_to_video_filename[m.getTopic()] = s_filename.str();

                const cv::Size2i video_size(image.cols, image.rows);
                video_writer = new cv::VideoWriter(s_filename.str(), CV_FOURCC('D','I','V','X'), video_frame_rate, video_size);
                if (video_writer->isOpened()) {
                    ROS_INFO("Created video writer for topic %s (output file %s)", m.getTopic().c_str(), s_filename.str().c_str());
                } else {
                    ROS_ERROR("Unable to create video writer for topic %s (output file %s)", m.getTopic().c_str(), s_filename.str().c_str());
                    exit(-1);
                }
                topic_to_video_writer[m.getTopic()] = video_writer;
            }

            // Add image to video
            video_writer->write(image);
        }

        // Tfs to bag
        if (transform) {
            bag_out.write(m.getTopic(), m.getTime(), transform);
        }

        // World evidence to bag
        /*
        if (evidence) {
            bag_out.write(m.getTopic(), m.getTime(), evidence);
                }
*/

        if (cam_info) {
            bag_out.write(m.getTopic(), m.getTime(), cam_info);
        }
    }

    bag.close();
    bag_out.close();
    ROS_INFO("Finished bag file");

    std::ofstream fout(yaml_name.c_str());
    fout << "bag:   " << bag_out_name << std::endl;

    if (!topic_to_video_writer.empty()) {
        fout << "videos:" << std::endl;
    }

    for(map<std::string, cv::VideoWriter*>::iterator it_vw = topic_to_video_writer.begin(); it_vw != topic_to_video_writer.end(); ++it_vw) {
        fout << "    - {file: \"" << topic_to_video_filename[it_vw->first] << "\", topic: \"" << it_vw->first << "\"}" << std::endl;
    }

    return 0;

}

