/**
  * videoStreaming Module header files
  * Author: Dimitrios Vitsios
  * Date: 19 May 2011
  * Change History: - 
  */

#include "videoStreaming.h"
#include "masterGui.h"

QImage videoDisplay;
QMutex videoMutex;

videoStreaming::videoStreaming() 
{
	std::string transport = "theora";
	image_transport::TransportHints transHints(transport, ros::TransportHints().unreliable());
	sub = image_transport::ImageTransport(_handle).subscribe("/vision/image2", 1, &videoStreaming::videoReceiver, this, transport);//transHints); 
}


void videoStreaming::videoReceiver(const sensor_msgs::ImageConstPtr& msg)
{
		//ROS_DEBUG("[GUI] Video callback function was called!");
		
	   // May want to view raw bayer data, which CvBridge doesn't know about
	   if (msg->encoding.find("bayer") != std::string::npos)
	   {
			last_image_ = cv::Mat(msg->height, msg->width, CV_8UC1,
						   const_cast<uint8_t*>(&msg->data[0]), msg->step);
			cvtColor(last_image_, mat_rgb, CV_BGR2RGB); 
			videoStream = QImage((uchar*)(mat_rgb.data), mat_rgb.cols,
		mat_rgb.rows, QImage::Format_RGB888); 
	   }
	   else
	   {
		 // Convert native BGR color to OpenCV 
		   last_image_Ipl = img_bridge_.imgMsgToCv(msg, "bgr8");
		   videoStream = IplImage2QImage(last_image_Ipl);
	   }
	 
	   videoMutex.lock();
	    videoDisplay = videoStream.convertToFormat(QImage::Format_ARGB32);	
	   videoMutex.unlock();
}



QImage videoStreaming::IplImage2QImage(const IplImage *iplImage)
{
    int height = iplImage->height;
    int width = iplImage->width;
 
    if  (iplImage->depth == IPL_DEPTH_8U && iplImage->nChannels == 3)
    {
      const uchar *qImageBuffer = (const uchar*)iplImage->imageData;
      QImage img(qImageBuffer, width, height, QImage::Format_RGB888);
      return img.rgbSwapped();
    } else if  (iplImage->depth == IPL_DEPTH_8U && iplImage->nChannels == 1){
	const uchar *qImageBuffer = (const uchar*)iplImage->imageData;
	QImage img(qImageBuffer, width, height, QImage::Format_Indexed8);
 
	QVector<QRgb> colorTable;
	for (int i = 0; i < 256; i++){
	    colorTable.push_back(qRgb(i, i, i));
	}
	img.setColorTable(colorTable);
	return img;
    }else{
      qWarning() << "Image cannot be converted.";
      return QImage();
    }
}
