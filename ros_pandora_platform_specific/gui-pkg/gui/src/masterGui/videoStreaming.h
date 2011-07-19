/**
  * videoStreaming Module header files
  * Author: Dimitrios Vitsios
  * Date: 19 May 2011
  * Change History: - 
  */

#include <QtGui>
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "compressed_image_transport/compressed_subscriber.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cvwimage.h>


/**
* Class as a client node for theora video streaming, implemented in ROS.
*/
class videoStreaming : QWidget
{
	Q_OBJECT

	private:
		//The ROS Node handle 
		ros::NodeHandle _handle;
		image_transport::Subscriber sub;
		QImage videoStream;


		sensor_msgs::CvBridge img_bridge_;
		cv::Mat last_image_;
		cv::Mat mat_rgb;
		IplImage *last_image_Ipl;
	public:
		//videoStreaming Constructor
		videoStreaming();
		void videoReceiver(const sensor_msgs::ImageConstPtr& msg);
		QImage IplImage2QImage(const IplImage *iplImage);

};


