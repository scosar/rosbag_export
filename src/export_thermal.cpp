// Optris
#include <libirimager/ImageBuilder.h>
// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
// GSL
#include <gsl/gsl_interp.h>
// C++ STD
#include <fstream>
#include <numeric>
// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/filesystem.hpp>
#include <sys/stat.h>

#include "optris_human_reader/PhysiologicalData.h"

// Optris
optris::ImageBuilder image_builder_;
image_transport::Publisher biometrics_pub_;
ros::Publisher pub_phy;
unsigned char* thermal_buffer_ = NULL;
double** temperature_map_;

int frameNo=0;

std::string folder_name("/media/scosar/My Passport/UOL_DATASET/20161122_thermal_dataset/");

std::string pub_topic_phy;

// Face detection
cv::Mat binary_image_;

// FPS
clock_t start_time_;
double fps_times_[11];
int fps_size_ = 0;

void colorConvert(const sensor_msgs::ImageConstPtr& raw_image, sensor_msgs::Image& color_image) {
  unsigned short* data = (unsigned short*)&raw_image->data[0];
  image_builder_.setData(raw_image->width, raw_image->height, data);

  if(thermal_buffer_ == NULL)
    thermal_buffer_ = new unsigned char[raw_image->width * raw_image->height * 3];

  image_builder_.convertTemperatureToPaletteImage(thermal_buffer_, true);

  color_image.header.frame_id = "optris_human_reader";
  color_image.height          = raw_image->height;
  color_image.width           = raw_image->width;
  color_image.encoding        = "rgb8";
  color_image.step            = raw_image->width * 3;
  color_image.header.seq      = raw_image->header.seq;
  color_image.header.stamp    = ros::Time::now();

  color_image.data.resize(color_image.height * color_image.step);
  memcpy(&color_image.data[0], &thermal_buffer_[0], color_image.height * color_image.step * sizeof(*thermal_buffer_));
}

void thermalImageCallback(const sensor_msgs::ImageConstPtr& raw_image) {
  if(biometrics_pub_.getNumSubscribers() == 0)
    return;

  sensor_msgs::Image color_image;
  /*** raw (temperature) image -> RGB color image ***/
  colorConvert(raw_image, color_image);

  /*** ros image to opencv image ***/
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
  } catch(cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  /*** temperature decoding ***/
  unsigned short* data = (unsigned short*)&raw_image->data[0];
  for(int i = 0; i < raw_image->height; i++) {
    for(int j = 0; j < raw_image->width; j++) {
      temperature_map_[i][j] = (double(data[i*raw_image->width+j]) - 1000.0f) / 10.0f;
    }
  }

  if (frameNo==0)
  {
      const int dir_err = mkdir(folder_name.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      if (-1 == dir_err)
      {
          ROS_INFO("Error creating directory %s! \n",folder_name.c_str());
          //exit(-1);
      }
  }

  std::stringstream fileName;
  fileName << folder_name << "thermal_" << frameNo << ".txt";

  std::fstream fs;
  fs.open(fileName, std::fstream::out | std::fstream::trunc);
  for(int i = 0; i < cv_ptr->image.rows; i++)
      for(int j = 0; j < cv_ptr->image.cols; j++)
          fs << i << " " << j << " " << temperature_map_[i][j] << "\n";
  fs.close();

    frameNo++;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "optris_human_reader");
  ros::NodeHandle private_nh("~");

  int coloring_palette;
  private_nh.param<int>("coloring_palette", coloring_palette, 6);
  image_builder_.setPalette((optris::EnumOptrisColoringPalette)coloring_palette);
  image_builder_.setPaletteScalingMethod(optris::eMinMax); // auto scaling

  // Default: Optris PI-450 output image size.
  int image_height, image_width;
  private_nh.param<int>("image_height", image_height, 288);
  private_nh.param<int>("image_width", image_width, 382);
  temperature_map_ = new double*[image_height];
  for(int i = 0; i < image_height; i++)
    temperature_map_[i] = new double[image_width];

  private_nh.param<bool>("data_visualization", data_visualization_, false);

  start_time_ = clock();

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber thermal_image_sub = it.subscribe("thermal_image", 100, thermalImageCallback); // raw image


  ros::spin();

  // Release storage space.
  if(thermal_buffer_)
    delete [] thermal_buffer_;

  for(int i = 0; i < image_height; i++)
    delete [] temperature_map_[i];
  delete [] temperature_map_;


  return 0;
}

