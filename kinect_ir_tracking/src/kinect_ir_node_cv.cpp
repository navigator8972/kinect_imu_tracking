#include "libfreenect.hpp"
#include <iostream>
#include <vector>
#include <stack>
#include <cmath>
#include <pthread.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

//ros stuff
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

using namespace cv;
using namespace std;

class myMutex {
    public:
        myMutex() {
            pthread_mutex_init( &m_mutex, NULL );
        }
        void lock() {
            pthread_mutex_lock( &m_mutex );
        }
        void unlock() {
            pthread_mutex_unlock( &m_mutex );
        }
    private:
        pthread_mutex_t m_mutex;
};


/* thanks to Yoda---- from IRC */
class MyFreenectDevice : public Freenect::FreenectDevice {
public:
    MyFreenectDevice(freenect_context *_ctx, int _index)
        : Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes),
                                                m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_IR_8BIT).bytes), 
                                                m_gamma(2048), m_new_rgb_frame(false), m_new_depth_frame(false),
                                                depthMat(Size(640,480),CV_16UC1),
                                                //rgbMat(Size(640,488), CV_8UC1, Scalar(0)),
                                                //buffer for IR image is of length 640x488 and uint8, note this is different from 
                                                //RGB video, so in GL demo they just use an array with enough length
                                                rgbMat(Size(640,488), CV_8UC1, Scalar(0)),
                                                ownMat(Size(640,480),CV_8UC3,Scalar(0))
    {
        for( unsigned int i = 0 ; i < 2048 ; i++) {
            float v = i/2048.0;
            v = std::pow(v, 3)* 6;
            m_gamma[i] = v*6*256;
        }
    }

    // Do not call directly even in child
    void VideoCallback(void* _rgb, uint32_t timestamp) {
        //std::cout << "RGB callback" << std::endl;
        m_rgb_mutex.lock();
        uint8_t* rgb = static_cast<uint8_t*>(_rgb);
        rgbMat.data = rgb;
        // printf("Video Buffer Size: %d", getVideoBufferSize());
        m_new_rgb_frame = true;
        m_rgb_mutex.unlock();
    };
    
    // Do not call directly even in child
    void DepthCallback(void* _depth, uint32_t timestamp) {
        //std::cout << "Depth callback" << std::endl;
        m_depth_mutex.lock();
        uint16_t* depth = static_cast<uint16_t*>(_depth);
        depthMat.data = (uchar*) depth;
        m_new_depth_frame = true;
        m_depth_mutex.unlock();
    }

    bool getRGB(Mat& output) {
        m_rgb_mutex.lock();
        if(m_new_rgb_frame) {
            //cv::cvtColor(rgbMat, output, CV_RGB2BGR);
            rgbMat.copyTo(output);
            m_new_rgb_frame = false;
            m_rgb_mutex.unlock();
            return true;
        } else {
            m_rgb_mutex.unlock();
            return false;
        }
    }

    bool getDepth(Mat& output) {
        m_depth_mutex.lock();
        if(m_new_depth_frame) {
            depthMat.copyTo(output);
            m_new_depth_frame = false;
            m_depth_mutex.unlock();
            return true;
        } else {
            m_depth_mutex.unlock();
            return false;
        }
        return true;
    }

private:
    std::vector<uint8_t> m_buffer_depth;
    std::vector<uint8_t> m_buffer_video;
    std::vector<uint16_t> m_gamma;
    Mat depthMat;
    Mat rgbMat;
    Mat ownMat;
    myMutex m_rgb_mutex;
    myMutex m_depth_mutex;
    bool m_new_rgb_frame;
    bool m_new_depth_frame;
};

//freenect_video_format requested_format(FREENECT_VIDEO_RGB);
freenect_video_format requested_format(FREENECT_VIDEO_IR_8BIT);

int main(int argc, char **argv) {

    //ros stuff
    ros::init(argc, argv, "IRBlobDetectNode", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("/kinect_ir_node/ir_blob_pnt", 10);
    ros::Rate rate(50);

    geometry_msgs::PointStamped msg;

    string filename("snapshot");
    string suffix(".png");
    int i_snap(0);
    //int iter(0);
    bool bVisualize = true;
    bool bVerbose = true;
    bool bDetectBlobs = true;

    Mat depthMat(Size(640,480),CV_16UC1);
    // Mat depthf (Size(640,480),CV_8UC1);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat irMat(Size(640, 488), CV_8UC1, Scalar(0));
    Mat ownMat(Size(640,480),CV_8UC3,Scalar(0));

    // The next two lines must be changed as Freenect::Freenect
    // isn't a template but the method createDevice:
    // Freenect::Freenect<MyFreenectDevice> freenect;
    // MyFreenectDevice& device = freenect.createDevice(0);
    // by these two lines:
    
    Freenect::Freenect freenect;
    MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);
    device.setVideoFormat(requested_format);

    // namedWindow("rgb",CV_WINDOW_AUTOSIZE);
    // namedWindow("depth",CV_WINDOW_AUTOSIZE);
    device.startVideo();
    device.startDepth();

    //<hyin/Feb-25th-2015> use SimpleBlobDetector
    vector<KeyPoint> keyPoints;
    SimpleBlobDetector::Params params;
    params.minDistBetweenBlobs = 50.0f;
    params.filterByInertia = false;
    params.filterByConvexity = false;
    params.filterByColor = false;
    params.filterByCircularity = false;
    params.filterByArea = true;
    params.minArea = 0.2f;
    params.maxArea = 25.0f;

    SimpleBlobDetector blobDetector( params );
    blobDetector.create("SimpleBlob");
    
    while (ros::ok()) {
        device.getRGB(irMat);
        device.getDepth(depthMat);

        if(bDetectBlobs)
        {
            blobDetector.detect(irMat, keyPoints);

            if( keyPoints.size() > 0)
            {
                if(bVerbose)
                {
                    ROS_INFO("Found %d key points.\n", keyPoints.size());
                    ROS_INFO("Point Coord: %lf, %lf\n", keyPoints[0].pt.x, keyPoints[0].pt.y);
                }
                //construct message
                //make the center of image as origin point
                msg.header.stamp = ros::Time::now();
                msg.point.x = keyPoints[0].pt.x - 320;
                msg.point.y = keyPoints[0].pt.y - 244;

                pub.publish(msg);
            }
        }
        cv::imshow("infrared", irMat);

        // depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
        // cv::imshow("depth",depthf);
        char k = cvWaitKey(5);
        if( k == 27 ){
            // cvDestroyWindow("rgb");
            // cvDestroyWindow("depth");
            break;
        }
        if( k == 8 ) {
            std::ostringstream file;
            file << filename << i_snap << suffix;
            cv::imwrite(file.str(),rgbMat);
            i_snap++;
        }

        rate.sleep();

    }
    
    device.stopVideo();
    device.stopDepth();

    return 0;
}