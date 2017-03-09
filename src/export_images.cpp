/*
 * export_images.cpp
 *
 *  Created on: Dec 15, 2015
 *      Author: serhancosar
 */


// ROS
#include <ros/ros.h>

// OpenCv
#include "cv.h"
#include "highgui.h"

// For transforming ROS/OpenCV images
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

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
#include <sys/stat.h>
#include <time.h>

// Boost
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include "boost/format.hpp"

using namespace std;

void PrintMatrix2File(cv::Mat Mat, char filename[])
{
    //FILE* pFile;
    //pFile = fopen(filename, "w");
    //CvSize Size = cvGetSize(Mat);

    ROS_INFO("image path: %s",filename);

    std::ofstream fs;
    fs.open(filename);
    if (fs.is_open())
    {
        ROS_INFO("file opened: %s",filename);
    }
    else
    {
        ROS_INFO("error openning file: %s",filename);
    }
    for (int i = 0;i<Mat.rows;i++)
	{
        for(int j = 0;j<Mat.cols;j++)
		{
            float val = Mat.at<float>(i, j);
            fs << val << " ";
//            ROS_INFO("val:%f",val);
            //CvScalar El = cvGet2D(Mat, i, j);
            //fprintf(pFile, "%f;" , El.val[0]);
			//fprintf(pFile, "%f;" , cvmGet(Mat, i, j));
		}
        // fprintf(pFile, "\n");
        fs << std::endl;
	}
    //fclose(pFile);
    fs.close();
}

void PrintMatrix2File_ushort(cv::Mat Mat, char filename[])
{
    //FILE* pFile;
    //pFile = fopen(filename, "w");
    //CvSize Size = cvGetSize(Mat);

    ROS_INFO("image path: %s",filename);

    std::ofstream fs;
    fs.open(filename);
    if (fs.is_open())
    {
        ROS_INFO("file opened: %s",filename);
    }
    else
    {
        ROS_INFO("error openning file: %s",filename);
    }
    for (int i = 0;i<Mat.rows;i++)
    {
        for(int j = 0;j<Mat.cols;j++)
        {
            int val = Mat.at<ushort>(i, j);
            fs << val << " ";
//            ROS_INFO("val:%f",val);
            //CvScalar El = cvGet2D(Mat, i, j);
            //fprintf(pFile, "%f;" , El.val[0]);
            //fprintf(pFile, "%f;" , cvmGet(Mat, i, j));
        }
        // fprintf(pFile, "\n");
        fs << std::endl;
    }
    //fclose(pFile);
    fs.close();
}

void PrintMatrix2File_ushort_thermal(cv::Mat Mat, char filename[])
{
    //FILE* pFile;
    //pFile = fopen(filename, "w");
    //CvSize Size = cvGetSize(Mat);

    ROS_INFO("image path: %s",filename);

    std::ofstream fs;
    fs.open(filename);
    if (fs.is_open())
    {
        ROS_INFO("file opened: %s",filename);
    }
    else
    {
        ROS_INFO("error openning file: %s",filename);
    }
    for (int i = 0;i<Mat.rows;i++)
    {
        for(int j = 0;j<Mat.cols;j++)
        {
            int val = Mat.at<ushort>(i, j);
            fs << ((float)val - 1000.0f) / 10.0f << " ";
//            ROS_INFO("val:%f",val);
            //CvScalar El = cvGet2D(Mat, i, j);
            //fprintf(pFile, "%f;" , El.val[0]);
            //fprintf(pFile, "%f;" , cvmGet(Mat, i, j));
        }
        // fprintf(pFile, "\n");
        fs << std::endl;
    }
    //fclose(pFile);
    fs.close();
}

//void colorConvert(const sensor_msgs::ImageConstPtr& raw_image, sensor_msgs::Image& color_image) {
//  unsigned short* data = (unsigned short*)&raw_image->data[0];
//  image_builder_.setData(raw_image->width, raw_image->height, data);

//  if(thermal_buffer_ == NULL)
//    thermal_buffer_ = new unsigned char[raw_image->width * raw_image->height * 3];

//  image_builder_.convertTemperatureToPaletteImage(thermal_buffer_, true);

//  color_image.header.frame_id = "optris_human_reader";
//  color_image.height          = raw_image->height;
//  color_image.width           = raw_image->width;
//  color_image.encoding        = "rgb8";
//  color_image.step            = raw_image->width * 3;
//  color_image.header.seq      = raw_image->header.seq;
//  color_image.header.stamp    = ros::Time::now();

//  color_image.data.resize(color_image.height * color_image.step);
//  memcpy(&color_image.data[0], &thermal_buffer_[0], color_image.height * color_image.step * sizeof(*thermal_buffer_));
//}

int main( int argc, char** argv ) {

    if (argc != 3) {
        std::cout << "Need 2 arguments: <INPUT.bag> <WriteFlag>" << std::endl;
        std::cout << "WriteFlag: 0 - only depth (PNG) " << std::endl;
        std::cout << "WriteFlag: 1 - only depth (TXT) " << std::endl;
        std::cout << "WriteFlag: 2 - only color (PNG) " << std::endl;
        std::cout << "WriteFlag: 3 - depth and color (PNG) " << std::endl;
        std::cout << "WriteFlag: 4 - depth (TXT) and color (PNG) " << std::endl;
        std::cout << "WriteFlag: 5 - only thermal (TXT) " << std::endl;

        exit(-1);
    }

    // Default values
    std::string bag_in_name = argv[1];
    int writeFlag = atoi(argv[2]);
    //int video_frame_rate = atoi(argv[2]);
//    std::string bag_out_name = std::string(argv[2]) + ".bag";
//    std::string yaml_name = std::string(argv[2]) + ".yaml";

    // Check input
    if (bag_in_name == "") {
        ROS_ERROR("No input bag file defined!");
        return -1;
    }
    /*
    if (video_frame_rate <= 0) {
        ROS_ERROR("No video frame rate defined!");
        return -1;
    }
    */

    // Video writers
//    map<std::string, cv::VideoWriter*> topic_to_video_writer;
    map<std::string, std::string> topic_to_foldername;


    // Load input bag file
    rosbag::Bag bag;
    bag.open(bag_in_name, rosbag::bagmode::Read);
    ROS_INFO("Opened %s", bag_in_name.c_str());

    // Create output bag file
//    rosbag::Bag bag_out;
//    bag_out.open(bag_out_name, rosbag::bagmode::Write);
//    ROS_INFO("Created %s", bag_out_name.c_str());

    // Get topics
    rosbag::View view(bag);

    // Image Folders
    std::string folder_name;
    std::string topic_name;

    int fileNo=0;

    struct tm * ptm;

    bool depth2TXT=false;
    bool thermal2TXT=false;

    bool writeDepth=false;
    bool writeColor=false;
    bool writeThermal=false;

    ROS_INFO("writeFlag=%d",writeFlag);
    switch(writeFlag)
    {
        case 0:
            depth2TXT=false;
            writeDepth=true;
            thermal2TXT=false;
            break;
        case 1:
            depth2TXT=true;
            writeDepth=true;
            thermal2TXT=false;
            break;
        case 2:
            writeColor=true;
            thermal2TXT=false;
            break;
        case 3:
            depth2TXT=false;
            writeDepth=true;
            writeColor=true;
            thermal2TXT=false;
            break;
        case 4:
            depth2TXT=true;
            writeDepth=true;
            writeColor=true;
            thermal2TXT=false;
            break;
        case 5:
            thermal2TXT=true;
            depth2TXT=false;
            writeThermal=true;
            writeColor=false;
            writeDepth=false;
            break;
    }

    ROS_INFO("depth2TXT=%d",depth2TXT);
    ROS_INFO("writeDepth=%d",writeDepth);
    ROS_INFO("writeColor=%d",writeColor);
    ROS_INFO("thermal2TXT=%d",thermal2TXT);
    ROS_INFO("writeThermal=%d",writeThermal);

    // Loop over messages
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {

        // Get data
        sensor_msgs::ImageConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
        sensor_msgs::CompressedImageConstPtr compressedimage_msg = m.instantiate<sensor_msgs::CompressedImage>();
        // tf::tfMessageConstPtr transform = m.instantiate<tf::tfMessage>();
        // wire_msgs::WorldEvidenceConstPtr evidence = m.instantiate<wire_msgs::WorldEvidence>();
        // sensor_msgs::CameraInfoConstPtr cam_info = m.instantiate<sensor_msgs::CameraInfo>();

        cv_bridge::CvImagePtr depth_image_ptr;
        cv_bridge::CvImageConstPtr color_image_ptr;
        bool readDepth=false;
        bool kinect2=false;

        // Images to video
        if (image_msg != NULL)
        {

            cv::Mat image;

            if (image_msg->encoding == "32FC1" || image_msg->encoding == "16UC1")
            { // depth image

                try
                {
                    depth_image_ptr = cv_bridge::toCvCopy(image_msg);
                } catch (cv_bridge::Exception& e) {
                    ROS_INFO("cv_bridge exception: %s", e.what());
                }

                readDepth=true;
                if (image_msg->encoding == "16UC1")
                    kinect2=true;


                // find max depth
                float max_depth = 0;
                float distance;
                for(int y = 0; y < depth_image_ptr->image.rows; y++)
                {
                    for(int x = 0; x < depth_image_ptr->image.cols; x++)
                    {
                        if (kinect2)
                        {
                            distance = (float) depth_image_ptr->image.at<ushort>(y, x); // ushort (16UC1) new depth format of Kinect2
                        }
                        else
                        {
                            distance = depth_image_ptr->image.at<float>(y, x);
                        }
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
                        if (kinect2)
                        {
                            distance = (float) depth_image_ptr->image.at<ushort>(y, x); // ushort (16UC1) new depth format of Kinect2
                        }
                        else
                        {
                            distance = depth_image_ptr->image.at<float>(y, x);
                        }
                        unsigned int dist_clr = (unsigned int)(distance / max_depth * 255);
                        image.at<cv::Vec3b>(y, x) = cv::Vec3b(dist_clr, dist_clr, dist_clr);
                    }
                }


                // remove NaN
                if (depth2TXT)
                {
                    for(int y = 0; y < depth_image_ptr->image.rows; y++)
                    {
                        for(int x = 0; x < depth_image_ptr->image.cols; x++)
                        {
                            if (kinect2)
                            {
                                distance = (float) depth_image_ptr->image.at<ushort>(y, x); // ushort (16UC1) new depth format of Kinect2
                            }
                            else
                            {
                                distance = depth_image_ptr->image.at<float>(y, x);
                            }
                            if (distance != distance)
                            { // exclude NaN
//                                ROS_INFO("%d",distance);
                                if (kinect2)
                                    depth_image_ptr->image.at<ushort>(y, x) = -1; // ushort (16UC1) new depth format of Kinect2
                                else
                                    depth_image_ptr->image.at<float>(y, x) = -1;

                            }
                        }
                    }
                }
//                std_msgs::Float64 max_depth_msg;
//                max_depth_msg.data = max_depth;
//                bag_out.write(m.getTopic() +"/_trigger", image_msg->header.stamp, max_depth_msg);

            }
            else if (image_msg->encoding == "rgb8" || image_msg->encoding == "bgr8"  )
            {
                // color image
                // Convert color OpenCV image

                try
                {
                    color_image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_INFO("cv_bridge exception: %s", e.what());
                }

                readDepth=false;
                image = color_image_ptr->image;

//                std_msgs::Bool b;
//                b.data = true;
//                bag_out.write(m.getTopic() +"/_trigger", image_msg->header.stamp, b);
            }
            else if (image_msg->encoding=="mono16")
            {
//                sensor_msgs::Image color_image;
//                /*** raw (temperature) image -> RGB color image ***/
//                colorConvert(raw_image, color_image);

//                /*** ros image to opencv image ***/

//                try {
//                  cv_ptr = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
//                } catch(cv_bridge::Exception& e) {
//                  ROS_ERROR("cv_bridge exception: %s", e.what());
//                  return;
//                }



                try
                {
                    depth_image_ptr = cv_bridge::toCvCopy(image_msg);
                } catch (cv_bridge::Exception& e) {
                    ROS_INFO("cv_bridge exception: %s", e.what());
                }

                readDepth=true;
                if (image_msg->encoding == "mono16")
                    kinect2=true;


                // find max depth
                float max_depth = 0;
                float distance;
                for(int y = 0; y < depth_image_ptr->image.rows; y++)
                {
                    for(int x = 0; x < depth_image_ptr->image.cols; x++)
                    {
                        if (kinect2)
                        {
                            distance = ((float) depth_image_ptr->image.at<ushort>(y, x) - 1000.0f) / 10.0f; // ushort (16UC1) new depth format of Kinect2
                        }
                        else
                        {
                            distance = depth_image_ptr->image.at<float>(y, x);
                        }
                        if (distance == distance)
                        { // exclude NaN
                            max_depth = max(distance, max_depth);
                        }


                    }
                }



//                image = cv::Mat(depth_image_ptr->image.rows, depth_image_ptr->image.cols, CV_8UC3);

//                for(int y = 0; y < depth_image_ptr->image.rows; y++)
//                {
//                    for(int x = 0; x < depth_image_ptr->image.cols; x++)
//                    {
//                        if (kinect2)
//                        {
//                            distance = (float) depth_image_ptr->image.at<ushort>(y, x); // ushort (16UC1) new depth format of Kinect2
//                        }
//                        else
//                        {
//                            distance = depth_image_ptr->image.at<float>(y, x);
//                        }
//                        unsigned int dist_clr = (unsigned int)(distance / max_depth * 255);
//                        image.at<cv::Vec3b>(y, x) = cv::Vec3b(dist_clr, dist_clr, dist_clr);
//                    }
//                }


                // remove NaN
                if (depth2TXT)
                {
                    for(int y = 0; y < depth_image_ptr->image.rows; y++)
                    {
                        for(int x = 0; x < depth_image_ptr->image.cols; x++)
                        {
                            if (kinect2)
                            {
                                distance = ( (float) depth_image_ptr->image.at<ushort>(y, x) - 1000.0f) / 10.0f; // ushort (16UC1) new depth format of Kinect2
                            }
                            else
                            {
                                distance = depth_image_ptr->image.at<float>(y, x);
                            }
                            if (distance != distance)
                            { // exclude NaN
//                                ROS_INFO("%d",distance);
                                if (kinect2)
                                    depth_image_ptr->image.at<ushort>(y, x) = -1; // ushort (16UC1) new depth format of Kinect2
                                else
                                    depth_image_ptr->image.at<float>(y, x) = -1;

                            }
                        }
                    }
                }

            }
            else
            {
                ROS_INFO("Unknown image encoding: %s", image_msg->encoding.c_str());
            }

            //cv::imshow(m.getTopic(), image);
            //cv::waitKey(1);

            int pos = bag_in_name.find_first_of(".");
            folder_name = bag_in_name.substr(0,pos);
            topic_name = m.getTopic();
            int found = topic_name.find_first_of("/");
            while (found!=std::string::npos)
            {
              topic_name[found]='_';
              found=topic_name.find_first_of("/",found+1);
            }
            folder_name += topic_name;

            if (readDepth & writeDepth)
            {
                if (depth2TXT==true)
                {
                    folder_name += "-txt";
                }

            }

            if (writeThermal)
            {
                if (thermal2TXT==true)
                {
                    folder_name += "-txt";
                }

            }

            // ROS_INFO("time: %d",m.getTime().sec);
            time_t rawtime = m.getTime().sec;
            ptm = gmtime ( &rawtime );
//                ROS_INFO("hour: %d, min: %d",ptm->tm_hour,ptm->tm_min);
            stringstream dt;
            dt << ptm->tm_year+1900 << setw(2) << setfill('0') << ptm->tm_mon+1 << setw(2) << setfill('0') << ptm->tm_mday << "-" << setw(2) << setfill('0') << ptm->tm_hour << ":" << setw(2) << setfill('0') << ptm->tm_min << ":" << setw(2) << setfill('0') << ptm->tm_sec << "." << setw(3) << setfill('0') << (int)m.getTime().nsec/1000000;


            map<std::string, std::string>::iterator it_image_writer = topic_to_foldername.find(m.getTopic());
            if (it_image_writer != topic_to_foldername.end())
            {
                //                stringstream ss;
                //                ss << setw(7) << setfill('0') << fileNo;
                //                std::string fileNo_str = ss.str();


                std::string filename;
                //                ROS_INFO("image path: %s",filename.c_str());


                // Write image data
                //                std::string filename = folder_name + "/depth_" + fileNo_str + ".txt";
                //                ROS_INFO("image path: %s",filename.c_str());

                if (readDepth & writeDepth)
                {
                    if (depth2TXT)
                    {
                        filename = folder_name + "/depth_" + dt.str() + ".txt";
                        if (kinect2)
                            PrintMatrix2File_ushort(depth_image_ptr->image,(char*)filename.c_str()); // ushort (16UC1) new depth format of Kinect2
                        else
                            PrintMatrix2File(depth_image_ptr->image,(char*)filename.c_str());

                    }
                    else
                    {
                        filename = folder_name + "/depth_" + dt.str() + ".png";
                        cv::imwrite(filename.c_str(),image);
                    }
                }
                else if (writeThermal)
                {
                    if (thermal2TXT)
                    {
                        filename = folder_name + "/thermal_" + dt.str() + ".txt";
                        if (kinect2)
                            PrintMatrix2File_ushort_thermal(depth_image_ptr->image,(char*)filename.c_str()); // ushort (16UC1) new depth format of Kinect2
                        else
                            PrintMatrix2File(depth_image_ptr->image,(char*)filename.c_str());

                    }
                }
                else
                {
                    if (writeColor)
                    {
                        filename = folder_name + "/image_" + dt.str() + ".png";
                        cv::imwrite(filename.c_str(),image);
                    }
                }

                //                    PrintMatrix2File(color_image_ptr->image,(char*)filename.c_str());


                fileNo++;
            }
            else
            {
                topic_to_foldername[m.getTopic()] = folder_name;

                const int dir_err = mkdir(folder_name.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
                if (-1 == dir_err)
                {
                    ROS_INFO("Error creating directory %s! \n",folder_name.c_str());
                    //exit(-1);
                }

                fileNo=0;
//                stringstream ss;
//                ss << setw(7) << setfill('0') << fileNo;
//                std::string fileNo_str = ss.str();

                std::string filename;
//                cv::imwrite(filename.c_str(),image);

                if (readDepth & writeDepth)
                {
                    if (depth2TXT)
                    {
                        filename = folder_name + "/depth_" + dt.str() + ".txt";
//                        ROS_INFO("image path: %s",filename.c_str());
                        if (kinect2)
                            PrintMatrix2File_ushort(depth_image_ptr->image,(char*)filename.c_str()); // ushort (16UC1) new depth format of Kinect2
                        else
                            PrintMatrix2File(depth_image_ptr->image,(char*)filename.c_str());
                    }
                    else
                    {
                        filename = folder_name + "/depth_" + dt.str() + ".png";
                        cv::imwrite(filename.c_str(),image);
                    }
                }
                else if (writeThermal)
                {
                    if (thermal2TXT)
                    {
                        filename = folder_name + "/thermal_" + dt.str() + ".txt";
                        if (kinect2)
                            PrintMatrix2File_ushort_thermal(depth_image_ptr->image,(char*)filename.c_str()); // ushort (16UC1) new depth format of Kinect2
                        else
                            PrintMatrix2File(depth_image_ptr->image,(char*)filename.c_str());

                    }
                }
                else
                {
                    if (writeColor)
                    {
                        filename = folder_name + "/image_" + dt.str() + ".png";
                        cv::imwrite(filename.c_str(),image);
                    }
                }



                fileNo++;

            }


        }

//        if (compressedimage_msg != NULL)
//        {

//            cv::Mat image;
//            image_transport::ImageTransport it;


//            if (compressedimage_msg->format == "jpeg")
//            {
//                sensor_msgs::Image image_msg;

//                try
//                {
//                    color_image_ptr = cv_bridge::toCvShare(compressedimage_msg, sensor_msgs::image_encodings::BGR8);
//                }
//                catch (cv_bridge::Exception& e)
//                {
//                    ROS_INFO("cv_bridge exception: %s", e.what());
//                }

//                readDepth=false;
//                image = color_image_ptr->image;
//            }
//            else
//            {
//                ROS_INFO("Unknown compressed image format: %s", compressedimage_msg->format.c_str());
//            }


//            int pos = bag_in_name.find_first_of(".");
//            folder_name = bag_in_name.substr(0,pos);
//            topic_name = m.getTopic();
//            int found = topic_name.find_first_of("/");
//            while (found!=std::string::npos)
//            {
//              topic_name[found]='_';
//              found=topic_name.find_first_of("/",found+1);
//            }
//            folder_name += topic_name;

//            if (readDepth & writeDepth)
//            {
//                if (depth2TXT==true)
//                {
//                    folder_name += "-txt";
//                }

//            }

//            // ROS_INFO("time: %d",m.getTime().sec);
//            time_t rawtime = m.getTime().sec;
//            ptm = gmtime ( &rawtime );
////                ROS_INFO("hour: %d, min: %d",ptm->tm_hour,ptm->tm_min);
//            stringstream dt;
//            dt << ptm->tm_year+1900 << setw(2) << setfill('0') << ptm->tm_mon+1 << setw(2) << setfill('0') << ptm->tm_mday << "-" << setw(2) << setfill('0') << ptm->tm_hour << ":" << setw(2) << setfill('0') << ptm->tm_min << ":" << setw(2) << setfill('0') << ptm->tm_sec << "." << setw(3) << setfill('0') << (int)m.getTime().nsec/1000000;


//            map<std::string, std::string>::iterator it_image_writer = topic_to_foldername.find(m.getTopic());
//            if (it_image_writer != topic_to_foldername.end())
//            {
//                //                stringstream ss;
//                //                ss << setw(7) << setfill('0') << fileNo;
//                //                std::string fileNo_str = ss.str();


//                std::string filename;
//                //                ROS_INFO("image path: %s",filename.c_str());


//                // Write image data
//                //                std::string filename = folder_name + "/depth_" + fileNo_str + ".txt";
//                //                ROS_INFO("image path: %s",filename.c_str());

//                if (readDepth & writeDepth)
//                {
//                    if (depth2TXT)
//                    {
//                        filename = folder_name + "/depth_" + dt.str() + ".txt";
//                        if (kinect2)
//                            PrintMatrix2File_ushort(depth_image_ptr->image,(char*)filename.c_str()); // ushort (16UC1) new depth format of Kinect2
//                        else
//                            PrintMatrix2File(depth_image_ptr->image,(char*)filename.c_str());

//                    }
//                    else
//                    {
//                        filename = folder_name + "/depth_" + dt.str() + ".png";
//                        cv::imwrite(filename.c_str(),image);
//                    }
//                }
//                else
//                {
//                    if (writeColor)
//                    {
//                        filename = folder_name + "/image_" + dt.str() + ".jpg";
//                        cv::imwrite(filename.c_str(),image);
//                    }
//                }

//                //                    PrintMatrix2File(color_image_ptr->image,(char*)filename.c_str());


//                fileNo++;
//            }
//            else
//            {
//                topic_to_foldername[m.getTopic()] = folder_name;

//                const int dir_err = mkdir(folder_name.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
//                if (-1 == dir_err)
//                {
//                    ROS_INFO("Error creating directory %s! \n",folder_name.c_str());
//                    //exit(-1);
//                }

//                fileNo=0;
////                stringstream ss;
////                ss << setw(7) << setfill('0') << fileNo;
////                std::string fileNo_str = ss.str();

//                std::string filename;
////                cv::imwrite(filename.c_str(),image);

//                if (readDepth & writeDepth)
//                {
//                    if (depth2TXT)
//                    {
//                        filename = folder_name + "/depth_" + dt.str() + ".txt";
////                        ROS_INFO("image path: %s",filename.c_str());
//                        if (kinect2)
//                            PrintMatrix2File_ushort(depth_image_ptr->image,(char*)filename.c_str()); // ushort (16UC1) new depth format of Kinect2
//                        else
//                            PrintMatrix2File(depth_image_ptr->image,(char*)filename.c_str());
//                    }
//                    else
//                    {
//                        filename = folder_name + "/depth_" + dt.str() + ".png";
//                        cv::imwrite(filename.c_str(),image);
//                    }
//                }
//                else
//                {
//                    if (writeColor)
//                    {
//                        filename = folder_name + "/image_" + dt.str() + ".jpg";
//                        cv::imwrite(filename.c_str(),image);
//                    }
//                }



//                fileNo++;

//            }

//        }

        // Tfs to bag
//        if (transform)
//        {
//            bag_out.write(m.getTopic(), m.getTime(), transform);
//        }

        // World evidence to bag
        /*
        if (evidence) {
            bag_out.write(m.getTopic(), m.getTime(), evidence);
                }
*/

//        if (cam_info)
//        {
//            bag_out.write(m.getTopic(), m.getTime(), cam_info);
//        }
    }




    bag.close();
//    bag_out.close();
    ROS_INFO("Finished bag file");

//    std::ofstream fout(yaml_name.c_str());
//    fout << "bag:   " << bag_out_name << std::endl;

//    if (!topic_to_foldername.empty())
//    {
//        fout << "images:" << std::endl;
//    }

//    for(map<std::string, std::string>::iterator it_vw = topic_to_foldername.begin(); it_vw != topic_to_foldername.end(); ++it_vw)
//    {
//        fout << "    - {path: \"" << topic_to_foldername[it_vw->first] << "\", topic: \"" << it_vw->first << "\"}" << std::endl;
//    }

    return 0;

}


