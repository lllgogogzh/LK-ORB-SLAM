#ifndef LK_H
#define LK_H


#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Converter.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include <Eigen/Dense>

#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
using namespace std;
using namespace ORB_SLAM2;

class LK
{
    public:
        KeyFrame* mpRefKeyFrame;
        Frame* mpCurrentFrame;
        vector<cv::Point2f> mvRefKeyPoints2D;
        vector<cv::Point3f> mvRefKeyPoints3D;
        vector<cv::Point2f> mvCurKeyPoints2D;

        //PnP
        cv::Mat mTcw;
        //Trajec
        vector<cv::Mat> mvTvec; 
        vector<double> mvTimeStamp;

        //LK
        vector<cv::Point2f> mvRefPoints2DInLK;
        vector<cv::Point2f> mvCurPoints2DInLK;
        vector<unsigned char> status;
        vector<float> error;
        cv::Mat mCurrentColorImg;
        cv::Mat mRefColorImg;
        unsigned int mnMinTracked;
        unsigned long int mnId;

        vector<int> mvRansacInliers;

        //vector<cv::KeyPoint> mvKeyPointsInCurrentFrame;

        //
        LK();
        ~LK();
        LK(unsigned int MinTracked):mnMinTracked(MinTracked){mnId = 0;};

    public:
        void setRefKeyFrame(KeyFrame* pKFrame);
        void setCurrentFrame(Frame pFrame);
        void useLKPnPRansac(const cv::Mat &curColorFrame,cv::Mat K,cv::Mat mDistCoef,int &mnMatchesInliers);
        //void getRefKepPoints3D();//get the map points associate with ref keyframe
        //void getRefKepPoints2D();//get the pixel points associate with ref keyframe , which used in LK tracking.
        void getRefKepPoints3D2D();//get together

        bool lastFrameIsKeyFrame();//true yes/false no

        bool PoseOptimizerLK();

};




#endif