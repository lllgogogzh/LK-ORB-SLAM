#include "LK.h"



#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"



LK::LK()
{
    //
}

LK::~LK()
{
    //
}

void LK::setRefKeyFrame(KeyFrame* pKFrame)
{
    mpRefKeyFrame = pKFrame;
}

void LK::setCurrentFrame(Frame  pFrame)
{
    mnId = pFrame.mnId;
}

void LK::getRefKepPoints3D2D()
{
    int obsPlus = 0;
    if(mpRefKeyFrame->mnId>3)
        obsPlus = 3;//who can observate the MapPoints(number)

    if(mpRefKeyFrame==NULL)
    {
        cout<<"no ref keyframe"<<endl;
        return;
    }

    //clear 
    mvRefKeyPoints2D.clear();
    mvRefKeyPoints3D.clear();
    vector<MapPoint*> tMapPoints;
    //get map points associate with Reference KeyFrame
    tMapPoints = mpRefKeyFrame->GetMapPointMatches();

    for(unsigned int i=0;i<tMapPoints.size();i++)
    {
        if(tMapPoints[i]==NULL)
        {
            continue;
        }

        if(tMapPoints[i]->Observations()>obsPlus)
        {
            //valid MapPoint
            //copy MapPoint to LK(include ref 2d , ref 3d in  ref KeyFrame)
            mvRefKeyPoints2D.push_back(mpRefKeyFrame->mvKeysUn[i].pt);
            cv::Mat tWorldPos = tMapPoints[i]->GetWorldPos();
            cv::Point3f tPoints3D;
            tPoints3D.x=tWorldPos.at<float>(0);
            tPoints3D.y=tWorldPos.at<float>(1);
            tPoints3D.z=tWorldPos.at<float>(2);
            mvRefKeyPoints3D.push_back(tPoints3D);
        }
    }
}

/*void LK::getRefKepPoints2D()
{
    //mvKeysUn为去畸变的关键点
    //define in KeyFrame.h
    //const std::vector<cv::KeyPoint> mvKeysUn;
    //
}*/

void LK::useLKPnPRansac(const cv::Mat &curColorFrame,cv::Mat mK,cv::Mat mDistCoef,int &mnMatchesInliers)
{
    //LK
    //we use: ref pixel coordinate keyPoints(2D)
    //ref frame and current frame (gray)
    //We will track the keypoints in ref frame , get 2D keypoints in current frame
    //and status of tracking , and error of tracking 
    if(mRefColorImg.empty())
    {
        //empty
        mRefColorImg = curColorFrame.clone();
        mCurrentColorImg = curColorFrame.clone();
        return;
    }

    //update new Frame captured
    mCurrentColorImg = curColorFrame.clone();

    if(lastFrameIsKeyFrame()||mvRefPoints2DInLK.empty())
    {
        //update data from KeyFrame
        getRefKepPoints3D2D();//this function fullfilled the mvRefKeyPoints2D and mvRefKeyPoints3D
        mvRefPoints2DInLK = mvRefKeyPoints2D;
    }
    
    //tracking with LK
    cv::Mat refGray,curGray;
    cv::cvtColor(mCurrentColorImg,curGray,CV_BGR2GRAY);
    cv::cvtColor(mRefColorImg,refGray,CV_BGR2GRAY);
    cv::calcOpticalFlowPyrLK(refGray,curGray,mvRefPoints2DInLK,mvCurPoints2DInLK,status,error);

    //delete the points that tracked failed
    int i=0;
    for(auto iter = mvCurPoints2DInLK.begin();iter!=mvCurPoints2DInLK.end();++i)
    {
        if(status[i]==0)
        {
            iter = mvCurPoints2DInLK.erase(iter);
            continue;
        }
        ++iter;
    }
    //delete the 3D MapPoint which is tracked failed
    i=0;
    for(auto iter = mvRefKeyPoints3D.begin();iter!=mvRefKeyPoints3D.end();++i)
    {
        if(status[i]==0)
        {
            iter = mvRefKeyPoints3D.erase(iter);
            continue;
        }
        ++iter;
    }

    //transfer curPoints to refPoints(next time)
    mvRefPoints2DInLK.clear();
    for(int i=0;i<mvCurPoints2DInLK.size();++i)
    {
        mvRefPoints2DInLK.push_back(mvCurPoints2DInLK[i]);
    }
    
    //Use PnP to calc the Tcw
    //usage :  cv::solvePnPRansac(3dPoints,2dPoints,K,distCoeffs,rvec,tvec,); 
    //vector<int> ransacInlier;//inlier points number
    if(mvCurPoints2DInLK.size()<mnMinTracked)
    {
        //not enough points tracked
        cout<<"not enough points tracked"<<endl;
        return;
    }

    if (mvCurPoints2DInLK.size() == 0)
    {
        //LK track lost
        cout<<"LK -- all keypoints are lost."<<endl;
        return;
    }
    cout<<mvCurPoints2DInLK.size()<<endl;
    mvRansacInliers.clear();
    if(!(mvRefKeyPoints3D.empty()||mvCurPoints2DInLK.empty()||mDistCoef.empty()))
    {
        //use PnP to solve Tcw
        cv::Mat rvec,tvec;
        cv::Mat R,t;
        cv::solvePnPRansac(mvRefKeyPoints3D,mvCurPoints2DInLK,mK,mDistCoef,rvec,tvec,false,500,8.0,0.99,mvRansacInliers,0);//K and mDistCoef
        cv::Rodrigues(rvec, R);//transfer vector(R) to rotatx matrix R
        cv::Mat_<double> Rt = (cv::Mat_<double>(4, 4) << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),tvec.at<double>(0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),tvec.at<double>(1),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2),tvec.at<double>(2),
        0, 0, 0, 1);
        cv::Mat Rt_float;
        Rt.convertTo(Rt_float,CV_32FC1);
        mTcw = Rt_float;//位姿
    }

    //draw pictures and keyPoints
    cv::Mat drawColorImg = curColorFrame.clone();
    for(auto kp:mvCurPoints2DInLK)
    {
        cv::circle(drawColorImg, kp, 5, cv::Scalar(0, 255, 0), 1);
        cv::circle(drawColorImg, kp, 1, cv::Scalar(0, 255, 0), -1);
    }
    imshow("LK",drawColorImg);
    //update inliers number of points matches
    mnMatchesInliers = mvRansacInliers.size();
    //update image
    mRefColorImg=mCurrentColorImg.clone();
}

bool LK::lastFrameIsKeyFrame()
{
    return (mnId-mpRefKeyFrame->mnFrameId) == 1;
}

bool LK::PoseOptimizerLK()
{
    if(mvCurPoints2DInLK.empty()||mvRefKeyPoints3D.empty())
    {
        return false;
    }
    g2o::SparseOptimizer optimizer;
    //optimizer.initializeOptimization();
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    //set vertex
    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    //add vertex
    optimizer.addVertex(vSE3);

    //set edge
    const float deltaMono = sqrt(5.991);
    size_t keyPointNum=mvRansacInliers.size();
    for(size_t i=0;i<keyPointNum;i++)
    {
        g2o::EdgeSE3ProjectXYZOnlyPose *e = new g2o::EdgeSE3ProjectXYZOnlyPose();
        e->fx=mpRefKeyFrame->fx;
        e->fy=mpRefKeyFrame->fy;
        e->cx=mpRefKeyFrame->cx;
        e->cy=mpRefKeyFrame->cy;

        Eigen::Vector2d obs;
        obs<<mvCurPoints2DInLK[mvRansacInliers[i]].x , mvCurPoints2DInLK[mvRansacInliers[i]].y;
        e->setMeasurement(obs);
        e->setId(i+1);
        e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e->setInformation(Eigen::Matrix2d::Identity());

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(deltaMono);

        e->Xw[0] = mvRefKeyPoints3D[mvRansacInliers[i]].x;
        e->Xw[1] = mvRefKeyPoints3D[mvRansacInliers[i]].y;
        e->Xw[2] = mvRefKeyPoints3D[mvRansacInliers[i]].z;

        optimizer.addEdge(e);
    }
    optimizer.initializeOptimization();//初始化
    optimizer.optimize(40);

    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    mTcw = pose.clone();
    return true;
}



