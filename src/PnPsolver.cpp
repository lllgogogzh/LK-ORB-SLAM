/**
* This file is part of ORB-SLAM2.
* This file is a modified version of EPnP <http://cvlab.epfl.ch/EPnP/index.php>, see FreeBSD license below.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

/**
* Copyright (c) 2009, V. Lepetit, EPFL
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
*   either expressed or implied, of the FreeBSD Project
*/

//这里的pnp求解用的是EPnP的算法。
// 参考论文：EPnP:An Accurate O(n) Solution to the PnP problem
// https://en.wikipedia.org/wiki/Perspective-n-Point
// http://docs.ros.org/fuerte/api/re_vision/html/classepnp.html
// 如果不理解，可以看看中文的："摄像机位姿的高精度快速求解" "摄像头位姿的加权线性算法"

// PnP求解：已知世界坐标系下的3D点与图像坐标系对应的2D点，求解相机的外参(R t)，即从世界坐标系到相机坐标系的变换。
// 而EPnP的思想是：
// 将世界坐标系所有的3D点用四个虚拟的控制点来表示，将图像上对应的特征点转化为相机坐标系下的四个控制点
// 根据世界坐标系下的四个控制点与相机坐标系下对应的四个控制点（与世界坐标系下四个控制点有相同尺度）即可恢复出(R t)


//                                   |x|
//   |u|   |fx r  u0||r11 r12 r13 t1||y|
// s |v| = |0  fy v0||r21 r22 r23 t2||z|
//   |1|   |0  0  1 ||r32 r32 r33 t3||1|

// step1: 构造质心坐标系，用四个控制点来表达所有的3D点
// p_w = sigma(alphas_j * pctrl_w_j), j从0到4
// p_c = sigma(alphas_j * pctrl_c_j), j从0到4
// sigma(alphas_j) = 1,  j从0到4

// step2:根据针孔投影模型
// s * u = K * sigma(alphas_j * pctrl_c_j), j从0到4

// step3:将step2的式子展开, 消去s
// sigma(alphas_j * fx * Xctrl_c_j) + alphas_j * (u0-u)*Zctrl_c_j = 0
// sigma(alphas_j * fy * Xctrl_c_j) + alphas_j * (v0-u)*Zctrl_c_j = 0

// step4:将step3中的12未知参数（4个控制点*3维参考点坐标）提成列向量
// Mx = 0,计算得到初始的解x后可以用Gauss-Newton来提纯得到四个相机坐标系的控制点

// step5:根据得到的p_w和对应的p_c，最小化重投影误差即可求解出R t

#include <iostream>

#include "PnPsolver.h"

#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>
#include "Thirdparty/DBoW2/DUtils/Random.h"
#include <algorithm>

using namespace std;

namespace ORB_SLAM2
{

// pcs表示3D点在camera坐标系下的坐标
// pws表示3D点在世界坐标系下的坐标
// us表示图像坐标系下的2D点坐标
// alphas为真实3D点用4个虚拟控制点表达时的系数
PnPsolver::PnPsolver(const Frame &F, const vector<MapPoint*> &vpMapPointMatches):
    pws(0), us(0), alphas(0), pcs(0), maximum_number_of_correspondences(0), number_of_correspondences(0), mnInliersi(0),
    mnIterations(0), mnBestInliers(0), N(0)
{
    // mnIterations记录当前已经Ransac的次数
    // pws 每个3D点有(X Y Z)三个值
    // us 每个图像2D点有(u v)两个值
    // alphas 每个3D点由四个控制点拟合，有四个系数
    // pcs 每个3D点有(X Y Z)三个值
    // maximum_number_of_correspondences 用于确定当前迭代所需内存是否够用

    // 根据点数初始化容器的大小
    mvpMapPointMatches = vpMapPointMatches;
    mvP2D.reserve(F.mvpMapPoints.size());
    mvSigma2.reserve(F.mvpMapPoints.size());
    mvP3Dw.reserve(F.mvpMapPoints.size());
    mvKeyPointIndices.reserve(F.mvpMapPoints.size());
    mvAllIndices.reserve(F.mvpMapPoints.size());

    int idx=0;
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];//依次获取一个MapPoint

        if(pMP)
        {
            if(!pMP->isBad())
            {
                const cv::KeyPoint &kp = F.mvKeysUn[i];//得到2维特征点, 将KeyPoint类型变为Point2f

                mvP2D.push_back(kp.pt);//存放到mvP2D容器
                mvSigma2.push_back(F.mvLevelSigma2[kp.octave]);//记录特征点是在哪一层提取出来的

                cv::Mat Pos = pMP->GetWorldPos();//世界坐标系下的3D点
                mvP3Dw.push_back(cv::Point3f(Pos.at<float>(0),Pos.at<float>(1), Pos.at<float>(2)));

                mvKeyPointIndices.push_back(i);//记录被使用特征点在原始特征点容器中的索引, mvKeyPointIndices是跳跃的
                mvAllIndices.push_back(idx);//记录被使用特征点的索引, mvAllIndices是连续的

                idx++;
            }
        }
    }

    // Set camera calibration parameters
    fu = F.fx;
    fv = F.fy;
    uc = F.cx;
    vc = F.cy;

    SetRansacParameters();
}

PnPsolver::~PnPsolver()
{
  delete [] pws;
  delete [] us;
  delete [] alphas;
  delete [] pcs;
}

// 设置RANSAC迭代的参数
void PnPsolver::SetRansacParameters(double probability, int minInliers, int maxIterations, int minSet, float epsilon, float th2)
{
    mRansacProb = probability;
    mRansacMinInliers = minInliers;
    mRansacMaxIts = maxIterations;
    mRansacEpsilon = epsilon; // inlier的比例
    mRansacMinSet = minSet;   // mRansacMinSet为每次RANSAC需要的特征点数，默认为4组3D-2D对应点

    N = mvP2D.size(); // number of correspondences, 所有二维特征点个数

    mvbInliersi.resize(N);// inlier index, mvbInliersi记录每次迭代inlier的点

    // Adjust Parameters according to number of correspondences
    // 根据输入参数，综合确定一个ransac inlier数，避免函数参数中minInliers设置过小
    int nMinInliers = N*mRansacEpsilon;
    if(nMinInliers<mRansacMinInliers)
        nMinInliers=mRansacMinInliers;
    if(nMinInliers<minSet)
        nMinInliers=minSet;
    mRansacMinInliers = nMinInliers;

    if(mRansacEpsilon<(float)mRansacMinInliers/N)
        mRansacEpsilon=(float)mRansacMinInliers/N;

    // Set RANSAC iterations according to probability, epsilon, and max iterations
    int nIterations;

    if(mRansacMinInliers==N)  //根据期望的残差大小来计算RANSAC需要迭代的次数
        nIterations=1;
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(mRansacEpsilon,3)));

    mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts));

    mvMaxError.resize(mvSigma2.size()); // 每个2D特征点对应不同的误差阈值
    for(size_t i=0; i<mvSigma2.size(); i++) // 不同的尺度，设置不同的最大偏差
        mvMaxError[i] = mvSigma2[i]*th2;    // th2: 判断是否满足inlier的重投影误差阈值的平方
}

cv::Mat PnPsolver::find(vector<bool> &vbInliers, int &nInliers)
{
    bool bFlag;
    return iterate(mRansacMaxIts,bFlag,vbInliers,nInliers);
}

// ！！！！！iterator函数内部会有ransac迭代，外部会以while的方式多次调用iterate
// do {
//    iterator();
// } while (!bNomore);
cv::Mat PnPsolver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers)
{
    bNoMore = false;
    vbInliers.clear();
    nInliers=0;

    // mRansacMinSet为每次RANSAC需要的特征点数，默认为4组3D-2D对应点
    set_maximum_number_of_correspondences(mRansacMinSet);

    // N为所有2D点的个数, mRansacMinInliers为RANSAC迭代终止的inlier阈值，如果已经大于所有点的个数，则停止迭代
    if(N<mRansacMinInliers)
    {
        bNoMore = true;
        return cv::Mat();
    }

    // mvAllIndices为所有参与PnP的2D点的索引
    // 每次ransac，将mvAllIndices赋值一份给vAvailableIndices，并从中随机挑选mRansacMinSet组3D-2D对应点进行一次RANSAC
    vector<size_t> vAvailableIndices;

    // nIterations: 根据ransac概率值计算出来的迭代次数
    // nCurrentIterations：记录每调用一次iterate函数，内部会有多次迭代
    // mnIterations：记录iterate调用次数 * 内部迭代次数总迭代次数
    int nCurrentIterations = 0;
    while(mnIterations<mRansacMaxIts || nCurrentIterations<nIterations)
    {
        nCurrentIterations++;
        mnIterations++;
        reset_correspondences();

        // 这个赋值稍微有些低效
        vAvailableIndices = mvAllIndices;

        // Get min set of points
        for(short i = 0; i < mRansacMinSet; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];

            // 将对应的3D-2D压入到pws和us
            add_correspondence(mvP3Dw[idx].x,mvP3Dw[idx].y,mvP3Dw[idx].z,mvP2D[idx].x,mvP2D[idx].y);

            // ！！！将已经被选中参与ransac的点去除（用vector最后一个点覆盖），避免抽取同一个数据参与ransac
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        // Compute camera pose
        // EPnP算法求解R，t
        compute_pose(mRi, mti);

        // Check inliers
        // 统计和记录inlier个数以及符合inlier的点：mnInliersi, mvbInliersi
        CheckInliers();

        if(mnInliersi>=mRansacMinInliers)
        {
            // If it is the best solution so far, save it
            // 记录inlier个数最多的一组解
            if(mnInliersi>mnBestInliers)
            {
                mvbBestInliers = mvbInliersi;
                mnBestInliers = mnInliersi;

                cv::Mat Rcw(3,3,CV_64F,mRi);
                cv::Mat tcw(3,1,CV_64F,mti);
                Rcw.convertTo(Rcw,CV_32F);
                tcw.convertTo(tcw,CV_32F);
                mBestTcw = cv::Mat::eye(4,4,CV_32F);
                Rcw.copyTo(mBestTcw.rowRange(0,3).colRange(0,3));
                tcw.copyTo(mBestTcw.rowRange(0,3).col(3));
            }

            // 将所有符合inlier的3D-2D匹配点一起计算PnP求解R, t
            if(Refine())
            {
                nInliers = mnRefinedInliers;
                vbInliers = vector<bool>(mvpMapPointMatches.size(),false);
                for(int i=0; i<N; i++)
                {
                    if(mvbRefinedInliers[i])
                        vbInliers[mvKeyPointIndices[i]] = true;
                }
                return mRefinedTcw.clone();
            }

        }
    }

    if(mnIterations>=mRansacMaxIts)
    {
        bNoMore=true;
        if(mnBestInliers>=mRansacMinInliers)
        {
            nInliers=mnBestInliers;
            vbInliers = vector<bool>(mvpMapPointMatches.size(),false);
            for(int i=0; i<N; i++)
            {
                if(mvbBestInliers[i])
                    vbInliers[mvKeyPointIndices[i]] = true;
            }
            return mBestTcw.clone();
        }
    }

    return cv::Mat();
}

bool PnPsolver::Refine()
{
    vector<int> vIndices;
    vIndices.reserve(mvbBestInliers.size());

    for(size_t i=0; i<mvbBestInliers.size(); i++)
    {
        if(mvbBestInliers[i])
        {
            vIndices.push_back(i);
        }
    }

    set_maximum_number_of_correspondences(vIndices.size());

    reset_correspondences();

    for(size_t i=0; i<vIndices.size(); i++)
    {
        int idx = vIndices[i];
        add_correspondence(mvP3Dw[idx].x,mvP3Dw[idx].y,mvP3Dw[idx].z,mvP2D[idx].x,mvP2D[idx].y);
    }

    // Compute camera pose
    compute_pose(mRi, mti);

    // Check inliers
    CheckInliers();

    // 通过CheckInliers函数得到那些inlier点用来提纯
    mnRefinedInliers =mnInliersi;
    mvbRefinedInliers = mvbInliersi;

    if(mnInliersi>mRansacMinInliers)
    {
        cv::Mat Rcw(3,3,CV_64F,mRi);
        cv::Mat tcw(3,1,CV_64F,mti);
        Rcw.convertTo(Rcw,CV_32F);
        tcw.convertTo(tcw,CV_32F);
        mRefinedTcw = cv::Mat::eye(4,4,CV_32F);
        Rcw.copyTo(mRefinedTcw.rowRange(0,3).colRange(0,3));
        tcw.copyTo(mRefinedTcw.rowRange(0,3).col(3));
        return true;
    }

    return false;
}

// 通过之前求解的(R t)检查哪些3D-2D点对属于inliers
void PnPsolver::CheckInliers()
{
    mnInliersi=0;

    for(int i=0; i<N; i++)
    {
        cv::Point3f P3Dw = mvP3Dw[i];
        cv::Point2f P2D = mvP2D[i];

        // 将3D点由世界坐标系旋转到相机坐标系
        float Xc = mRi[0][0]*P3Dw.x+mRi[0][1]*P3Dw.y+mRi[0][2]*P3Dw.z+mti[0];
        float Yc = mRi[1][0]*P3Dw.x+mRi[1][1]*P3Dw.y+mRi[1][2]*P3Dw.z+mti[1];
        float invZc = 1/(mRi[2][0]*P3Dw.x+mRi[2][1]*P3Dw.y+mRi[2][2]*P3Dw.z+mti[2]);

        // 将相机坐标系下的3D进行针孔投影
        double ue = uc + fu * Xc * invZc;
        double ve = vc + fv * Yc * invZc;

        // 计算残差大小
        float distX = P2D.x-ue;
        float distY = P2D.y-ve;

        float error2 = distX*distX+distY*distY;

        if(error2<mvMaxError[i])
        {
            mvbInliersi[i]=true;
            mnInliersi++;
        }
        else
        {
            mvbInliersi[i]=false;
        }
    }
}

// number_of_correspondences为RANSAC每次PnP求解时时3D点和2D点匹配对数
// maximum_number_of_correspondences当前匹配点个数
// 这个变量用于决定pws us alphas pcs容器的大小
// 如果maximum_number_of_correspondences当前设置的过小，则重新设置，并重新初始化pws us alphas pcs的大小
// 如果新的设定值比maximum_number_of_correspondences小，即之前分配的内存大小够用，则不重新分配内存
void PnPsolver::set_maximum_number_of_correspondences(int n)
{
  if (maximum_number_of_correspondences < n) {
    if (pws != 0) delete [] pws;
    if (us != 0) delete [] us;
    if (alphas != 0) delete [] alphas;
    if (pcs != 0) delete [] pcs;

    maximum_number_of_correspondences = n;
    pws = new double[3 * maximum_number_of_correspondences];    // 每个3D点有(X Y Z)三个值
    us = new double[2 * maximum_number_of_correspondences];     // 每个图像2D点有(u v)两个值
    alphas = new double[4 * maximum_number_of_correspondences]; // 每个3D点由四个控制点拟合，有四个系数
    pcs = new double[3 * maximum_number_of_correspondences];    // 每个3D点有(X Y Z)三个值
  }
}

void PnPsolver::reset_correspondences(void)
{
  number_of_correspondences = 0;
}

void PnPsolver::add_correspondence(double X, double Y, double Z, double u, double v)
{
  pws[3 * number_of_correspondences    ] = X;
  pws[3 * number_of_correspondences + 1] = Y;
  pws[3 * number_of_correspondences + 2] = Z;

  us[2 * number_of_correspondences    ] = u;
  us[2 * number_of_correspondences + 1] = v;

  number_of_correspondences++;
}

// 获得4个控制点坐标，存在4*3的二维数组cws中
void PnPsolver::choose_control_points(void)
{
  // Take C0 as the reference points centroid:
  // 步骤1：第一个控制点：参与PnP计算的参考3D点的几何中心
  cws[0][0] = cws[0][1] = cws[0][2] = 0;
  for(int i = 0; i < number_of_correspondences; i++)
    for(int j = 0; j < 3; j++)
      cws[0][j] += pws[3 * i + j];

  for(int j = 0; j < 3; j++)
    cws[0][j] /= number_of_correspondences;


  // Take C1, C2, and C3 from PCA on the reference points:
  // 步骤2：计算其它三个控制点，C1, C2, C3通过PCA分解得到
  // 将所有的3D参考点写成矩阵，(number_of_correspondences *　３)的矩阵
  CvMat * PW0 = cvCreateMat(number_of_correspondences, 3, CV_64F);

  double pw0tpw0[3 * 3], dc[3], uct[3 * 3];
  CvMat PW0tPW0 = cvMat(3, 3, CV_64F, pw0tpw0);
  CvMat DC      = cvMat(3, 1, CV_64F, dc);
  CvMat UCt     = cvMat(3, 3, CV_64F, uct);

  // 步骤2.1：将存在pws中的参考3D点减去第一个控制点的坐标（相当于把第一个控制点作为原点）, 并存入PW0
  for(int i = 0; i < number_of_correspondences; i++)
    for(int j = 0; j < 3; j++)
      PW0->data.db[3 * i + j] = pws[3 * i + j] - cws[0][j];

  // 步骤2.2：利用SVD分解P'P可以获得P的主分量
  // 类似于齐次线性最小二乘求解的过程，
  // PW0的转置乘以PW0
  cvMulTransposed(PW0, &PW0tPW0, 1);
  cvSVD(&PW0tPW0, &DC, &UCt, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);

  cvReleaseMat(&PW0);

  // 步骤2.3：得到C1, C2, C3三个3D控制点，最后加上之前减掉的第一个控制点这个偏移量
  for(int i = 1; i < 4; i++) {
    // ！！！
    double k = sqrt(dc[i - 1] / number_of_correspondences);
    for(int j = 0; j < 3; j++)
      cws[i][j] = cws[0][j] + k * uct[3 * (i - 1) + j];
  }
}

// 求解四个控制点的系数alphas
// (a2 a3 a4)' = inverse(cws2-cws1 cws3-cws1 cws4-cws1)*(pws-cws1)，a1 = 1-a2-a3-a4
// 每一个3D控制点，都有一组alphas与之对应
// cws1 cws2 cws3 cws4为四个控制点的坐标
// pws为3D参考点的坐标
void PnPsolver::compute_barycentric_coordinates(void)
{
  double cc[3 * 3], cc_inv[3 * 3];
  CvMat CC     = cvMat(3, 3, CV_64F, cc);
  CvMat CC_inv = cvMat(3, 3, CV_64F, cc_inv);

  // 第一个控制点在质心的位置，后面三个控制点减去第一个控制点的坐标（以第一个控制点为原点）
  // 步骤1：减去质心后得到x y z轴
  // 
  // cws的排列 |cws1_x cws1_y cws1_z|  ---> |cws1|
  //          |cws2_x cws2_y cws2_z|       |cws2|
  //          |cws3_x cws3_y cws3_z|       |cws3|
  //          |cws4_x cws4_y cws4_z|       |cws4|
  //          
  // cc的排列  |cc2_x cc3_x cc4_x|  --->|cc2 cc3 cc4|
  //          |cc2_y cc3_y cc4_y|
  //          |cc2_z cc3_z cc4_z|
  for(int i = 0; i < 3; i++)
    for(int j = 1; j < 4; j++)
      cc[3 * i + j - 1] = cws[j][i] - cws[0][i];

  cvInvert(&CC, &CC_inv, CV_SVD);
  double * ci = cc_inv;
  for(int i = 0; i < number_of_correspondences; i++) {
    double * pi = pws + 3 * i;    // pi指向第i个3D点的首地址
    double * a = alphas + 4 * i;  // a指向第i个控制点系数alphas的首地址

    // pi[]-cws[0][]表示将pi和步骤1进行相同的平移
    for(int j = 0; j < 3; j++)
      a[1 + j] = ci[3 * j    ] * (pi[0] - cws[0][0]) +
                 ci[3 * j + 1] * (pi[1] - cws[0][1]) +
                 ci[3 * j + 2] * (pi[2] - cws[0][2]);
    a[0] = 1.0f - a[1] - a[2] - a[3];
  }
}

// 填充最小二乘的M矩阵
// 对每一个3D参考点：
// |[ai1*fu, 0,   ai1*(uc-ui)], [ai2*fu, 0,   ai2*(uc-ui)], [ai3*fu, 0,    ai3*(uc-ui), [ai4*fu, 0,   ai4*(uc-ui)]|
// |[0,   ai1*fv, ai1*(vc-vi),  [0,   ai2*fv, ai2*(vc-vi)], [0,    ai3*fv, ai3*(vc-vi), [0,   ai1*fv, ai4*(vc-vi)]|
// 其中i从0到4
void PnPsolver::fill_M(CvMat * M,
		  const int row, const double * as, const double u, const double v)
{
  double * M1 = M->data.db + row * 12;
  double * M2 = M1 + 12;

  for(int i = 0; i < 4; i++) {
    M1[3 * i    ] = as[i] * fu;
    M1[3 * i + 1] = 0.0;
    M1[3 * i + 2] = as[i] * (uc - u);

    M2[3 * i    ] = 0.0;
    M2[3 * i + 1] = as[i] * fv;
    M2[3 * i + 2] = as[i] * (vc - v);
  }
}

// 每一个控制点在相机坐标系下都表示为特征向量乘以beta的形式，EPnP论文的公式16
void PnPsolver::compute_ccs(const double * betas, const double * ut)
{
  for(int i = 0; i < 4; i++)
    ccs[i][0] = ccs[i][1] = ccs[i][2] = 0.0f;

  for(int i = 0; i < 4; i++) {
    const double * v = ut + 12 * (11 - i);
    for(int j = 0; j < 4; j++)
      for(int k = 0; k < 3; k++)
    ccs[j][k] += betas[i] * v[3 * j + k];
  }
}

// 用四个控制点作为单位向量表示下的世界坐标系下3D点的坐标
void PnPsolver::compute_pcs(void)
{
  for(int i = 0; i < number_of_correspondences; i++) {
    double * a = alphas + 4 * i;
    double * pc = pcs + 3 * i;

    for(int j = 0; j < 3; j++)
      pc[j] = a[0] * ccs[0][j] + a[1] * ccs[1][j] + a[2] * ccs[2][j] + a[3] * ccs[3][j];
  }
}

double PnPsolver::compute_pose(double R[3][3], double t[3])
{
  // 步骤1：获得EPnP算法中的四个控制点（构成质心坐标系）
  choose_control_points();
  // 步骤2：计算世界坐标系下每个3D点用4个控制点线性表达时的系数alphas，公式1
  compute_barycentric_coordinates();

  // 步骤3：构造M矩阵，公式(3)(4)-->(5)(6)(7)
  CvMat * M = cvCreateMat(2 * number_of_correspondences, 12, CV_64F);

  for(int i = 0; i < number_of_correspondences; i++)
    fill_M(M, 2 * i, alphas + 4 * i, us[2 * i], us[2 * i + 1]);

  double mtm[12 * 12], d[12], ut[12 * 12];
  CvMat MtM = cvMat(12, 12, CV_64F, mtm);
  CvMat D   = cvMat(12,  1, CV_64F, d);
  CvMat Ut  = cvMat(12, 12, CV_64F, ut);

  // 步骤3：求解Mx = 0
  // SVD分解M'M
  cvMulTransposed(M, &MtM, 1);
  // 通过（svd分解）求解齐次最小二乘解得到相机坐标系下四个控制点：ut
  // ut的每一行对应一组可能的解
  // 最小特征值对应的特征向量最接近待求的解，由于噪声和约束不足的问题，导致真正的解可能是多个特征向量的线性叠加
  cvSVD(&MtM, &D, &Ut, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);//得到向量ut
  cvReleaseMat(&M);

  // 上述通过求解齐次最小二乘获得解不具有尺度，这里通过构造另外一个最小二乘（L*Betas = Rho）来求解尺度Betas
  // L_6x10 * Betas10x1 = Rho_6x1
  double l_6x10[6 * 10], rho[6];
  CvMat L_6x10 = cvMat(6, 10, CV_64F, l_6x10);
  CvMat Rho    = cvMat(6,  1, CV_64F, rho);

  // Betas10        = [B00 B01 B11 B02 B12 B22 B03 B13 B23 B33]
  // |dv00, 2*dv01, dv11, 2*dv02, 2*dv12, dv22, 2*dv03, 2*dv13, 2*dv23, dv33|, 1*10
  // 4个控制点之间总共有6个距离，因此为6*10
  compute_L_6x10(ut, l_6x10);
  compute_rho(rho);

  double Betas[4][4], rep_errors[4];
  double Rs[4][3][3], ts[4][3];

  // 不管什么情况，都假设论文中N=4，并求解部分betas（如果全求解出来会有冲突）
  // 通过优化得到剩下的betas
  // 最后计算R t

  // Betas10        = [B00 B01 B11 B02 B12 B22 B03 B13 B23 B33]
  // betas_approx_1 = [B00 B01     B02         B03]
  // 建模为除B11、B12、B13、B14四个参数外其它参数均为0进行最小二乘求解，求出B0、B1、B2、B3粗略解
  find_betas_approx_1(&L_6x10, &Rho, Betas[1]);
  // 高斯牛顿法优化B0、B1、B2、B3
  gauss_newton(&L_6x10, &Rho, Betas[1]);
  rep_errors[1] = compute_R_and_t(ut, Betas[1], Rs[1], ts[1]);

  // betas_approx_2 = [B00 B01 B11                            ]
  // 建模为除B00、B01、B11三个参数外其它参数均为0进行最小二乘求解，求出B0、B1、B2、B3粗略解
  find_betas_approx_2(&L_6x10, &Rho, Betas[2]);
  gauss_newton(&L_6x10, &Rho, Betas[2]);
  rep_errors[2] = compute_R_and_t(ut, Betas[2], Rs[2], ts[2]);

  // betas_approx_3 = [B00 B01 B11 B02 B12                    ]
  // 建模为除B00、B01、B11、B02、B12五个参数外其它参数均为0进行最小二乘求解，求出B0、B1、B2、B3粗略解
  find_betas_approx_3(&L_6x10, &Rho, Betas[3]);
  gauss_newton(&L_6x10, &Rho, Betas[3]);
  rep_errors[3] = compute_R_and_t(ut, Betas[3], Rs[3], ts[3]);

  int N = 1;
  if (rep_errors[2] < rep_errors[1]) N = 2;
  if (rep_errors[3] < rep_errors[N]) N = 3;

  copy_R_and_t(Rs[N], ts[N], R, t);

  return rep_errors[N];
}

void PnPsolver::copy_R_and_t(const double R_src[3][3], const double t_src[3],
			double R_dst[3][3], double t_dst[3])
{
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++)
      R_dst[i][j] = R_src[i][j];
    t_dst[i] = t_src[i];
  }
}

double PnPsolver::dist2(const double * p1, const double * p2)
{
  return
    (p1[0] - p2[0]) * (p1[0] - p2[0]) +
    (p1[1] - p2[1]) * (p1[1] - p2[1]) +
    (p1[2] - p2[2]) * (p1[2] - p2[2]);
}

double PnPsolver::dot(const double * v1, const double * v2)
{
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

double PnPsolver::reprojection_error(const double R[3][3], const double t[3])
{
  double sum2 = 0.0;

  for(int i = 0; i < number_of_correspondences; i++) {
    double * pw = pws + 3 * i;
    double Xc = dot(R[0], pw) + t[0];
    double Yc = dot(R[1], pw) + t[1];
    double inv_Zc = 1.0 / (dot(R[2], pw) + t[2]);
    double ue = uc + fu * Xc * inv_Zc;
    double ve = vc + fv * Yc * inv_Zc;
    double u = us[2 * i], v = us[2 * i + 1];

    sum2 += sqrt( (u - ue) * (u - ue) + (v - ve) * (v - ve) );
  }

  return sum2 / number_of_correspondences;
}

// 根据世界坐标系下的四个控制点与机体坐标下对应的四个控制点（和世界坐标系下四个控制点相同尺度），求取R t
// 见《视觉SLAM十四讲从理论到实践》 7.9.1 3D-3D: ICP SVD方法
// [U, S, Vt] = svd(A*B')，A为pc(i)列向量构成的矩阵，B为pw(i)列向量构成的矩阵
// R = U*Vt
// t = pc0 - R*pw0，pc0和pw0分别为相机坐标系和世界坐标系下3D点的中心坐标
void PnPsolver::estimate_R_and_t(double R[3][3], double t[3])
{
  // 对相机坐标系和世界坐标系下3D点分别求平均得到中心点坐标：pc0和pw0
  double pc0[3], pw0[3];

  pc0[0] = pc0[1] = pc0[2] = 0.0;
  pw0[0] = pw0[1] = pw0[2] = 0.0;

  for(int i = 0; i < number_of_correspondences; i++) {
    const double * pc = pcs + 3 * i;
    const double * pw = pws + 3 * i;

    for(int j = 0; j < 3; j++) {
      pc0[j] += pc[j];
      pw0[j] += pw[j];
    }
  }
  for(int j = 0; j < 3; j++) {
    pc0[j] /= number_of_correspondences;
    pw0[j] /= number_of_correspondences;
  }

  //     |pc_x_0, pc_x_1, pc_x_2,....pc_x_n|
  // A = |pc_y_0, pc_y_1, pc_y_2,....pc_x_n|
  //     |pc_z_0, pc_z_1, pc_z_2,....pc_x_n|

  //     |pw_x_0, pw_x_1, pw_x_2,....pw_x_n|
  // B = |pw_y_0, pw_y_1, pw_y_2,....pw_x_n|
  //     |pw_z_0, pw_z_1, pw_z_2,....pw_x_n|
  double abt[3 * 3], abt_d[3], abt_u[3 * 3], abt_v[3 * 3];
  CvMat ABt   = cvMat(3, 3, CV_64F, abt);
  CvMat ABt_D = cvMat(3, 1, CV_64F, abt_d);
  CvMat ABt_U = cvMat(3, 3, CV_64F, abt_u);
  CvMat ABt_V = cvMat(3, 3, CV_64F, abt_v);

  // 计算 A * B.transpose
  cvSetZero(&ABt);
  for(int i = 0; i < number_of_correspondences; i++) {
    double * pc = pcs + 3 * i;
    double * pw = pws + 3 * i;

    for(int j = 0; j < 3; j++) {
      abt[3 * j    ] += (pc[j] - pc0[j]) * (pw[0] - pw0[0]);
      abt[3 * j + 1] += (pc[j] - pc0[j]) * (pw[1] - pw0[1]);
      abt[3 * j + 2] += (pc[j] - pc0[j]) * (pw[2] - pw0[2]);
    }
  }

  cvSVD(&ABt, &ABt_D, &ABt_U, &ABt_V, CV_SVD_MODIFY_A);

  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      R[i][j] = dot(abt_u + 3 * i, abt_v + 3 * j);

  const double det =
    R[0][0] * R[1][1] * R[2][2] + R[0][1] * R[1][2] * R[2][0] + R[0][2] * R[1][0] * R[2][1] -
    R[0][2] * R[1][1] * R[2][0] - R[0][1] * R[1][0] * R[2][2] - R[0][0] * R[1][2] * R[2][1];

  // 
  if (det < 0) {
    R[2][0] = -R[2][0];
    R[2][1] = -R[2][1];
    R[2][2] = -R[2][2];
  }

  t[0] = pc0[0] - dot(R[0], pw0);
  t[1] = pc0[1] - dot(R[1], pw0);
  t[2] = pc0[2] - dot(R[2], pw0);
}

void PnPsolver::print_pose(const double R[3][3], const double t[3])
{
  cout << R[0][0] << " " << R[0][1] << " " << R[0][2] << " " << t[0] << endl;
  cout << R[1][0] << " " << R[1][1] << " " << R[1][2] << " " << t[1] << endl;
  cout << R[2][0] << " " << R[2][1] << " " << R[2][2] << " " << t[2] << endl;
}

void PnPsolver::solve_for_sign(void)
{
  if (pcs[2] < 0.0) {
    for(int i = 0; i < 4; i++)
      for(int j = 0; j < 3; j++)
	ccs[i][j] = -ccs[i][j];

    for(int i = 0; i < number_of_correspondences; i++) {
      pcs[3 * i    ] = -pcs[3 * i];
      pcs[3 * i + 1] = -pcs[3 * i + 1];
      pcs[3 * i + 2] = -pcs[3 * i + 2];
    }
  }
}

double PnPsolver::compute_R_and_t(const double * ut, const double * betas,
			     double R[3][3], double t[3])
{
  // 通过Betas和特征向量，得到机体坐标系下四个控制点
  compute_ccs(betas, ut);
  // 根据控制点和相机坐标系下每个点与控制点之间的关系，恢复出所有3D点在相机坐标系下的坐标
  compute_pcs();

  // 随便取一个相机坐标系下3D点，如果z < 0，则表明3D点都在相机后面，则3D点坐标整体取负号
  solve_for_sign();

  // 3D-3D svd方法求解ICP获得R，t
  estimate_R_and_t(R, t);

  // 获得R，t后计算所有3D点的重投影误差平均值
  return reprojection_error(R, t);
}

// betas10        = [B00 B01 B11 B02 B12 B22 B03 B13 B23 B33]
// betas_approx_1 = [B00 B01     B02         B03]
// 获得B0、B1、B2、B3的粗略解
void PnPsolver::find_betas_approx_1(const CvMat * L_6x10, const CvMat * Rho,
			       double * betas)
{
  double l_6x4[6 * 4], b4[4];
  CvMat L_6x4 = cvMat(6, 4, CV_64F, l_6x4);
  CvMat B4    = cvMat(4, 1, CV_64F, b4);

  for(int i = 0; i < 6; i++) {
    cvmSet(&L_6x4, i, 0, cvmGet(L_6x10, i, 0));
    cvmSet(&L_6x4, i, 1, cvmGet(L_6x10, i, 1));
    cvmSet(&L_6x4, i, 2, cvmGet(L_6x10, i, 3));
    cvmSet(&L_6x4, i, 3, cvmGet(L_6x10, i, 6));
  }

  cvSolve(&L_6x4, Rho, &B4, CV_SVD);

  // 由于：B00 = B0 * B0一定为正
  if (b4[0] < 0) {
    // 如果B00为负，则整体取负
    // 取根号：sqrt(B00)=B0
    betas[0] = sqrt(-b4[0]);
    betas[1] = -b4[1] / betas[0];
    betas[2] = -b4[2] / betas[0];
    betas[3] = -b4[3] / betas[0];
  } else {
    betas[0] = sqrt(b4[0]);
    betas[1] = b4[1] / betas[0];
    betas[2] = b4[2] / betas[0];
    betas[3] = b4[3] / betas[0];
  }
}

// betas10        = [B00 B01 B11 B02 B12 B22 B03 B13 B23 B33]
// betas_approx_2 = [B00 B01 B11                            ]
// 获得B0、B1、B2、B3的粗略解
void PnPsolver::find_betas_approx_2(const CvMat * L_6x10, const CvMat * Rho,
			       double * betas)
{
  double l_6x3[6 * 3], b3[3];
  CvMat L_6x3  = cvMat(6, 3, CV_64F, l_6x3);
  CvMat B3     = cvMat(3, 1, CV_64F, b3);

  for(int i = 0; i < 6; i++) {
    cvmSet(&L_6x3, i, 0, cvmGet(L_6x10, i, 0));
    cvmSet(&L_6x3, i, 1, cvmGet(L_6x10, i, 1));
    cvmSet(&L_6x3, i, 2, cvmGet(L_6x10, i, 2));
  }

  cvSolve(&L_6x3, Rho, &B3, CV_SVD);

  // B00 = B0*B0, B11 = B1*B1，B00和B11都为正
  // 如果无法满足B00和B11同时为正，则将B11置为0
  // 取根号：sqrt(B00)=B0
  if (b3[0] < 0) {
    betas[0] = sqrt(-b3[0]);
    betas[1] = (b3[2] < 0) ? sqrt(-b3[2]) : 0.0;
  } else {
    betas[0] = sqrt(b3[0]);
    betas[1] = (b3[2] > 0) ? sqrt(b3[2]) : 0.0;
  }

  // 上述步骤中求解的B0和B1都大于0，但是如果B01 < 0，说明B0和B1异号，则将B0或B1其中一个取反
  if (b3[1] < 0) betas[0] = -betas[0];

  betas[2] = 0.0;
  betas[3] = 0.0;
}

// betas10        = [B00 B01 B11 B02 B12 B22 B03 B13 B23 B33]
// betas_approx_3 = [B00 B01 B11 B02 B12                    ]
// 获得B0、B1、B2、B3的粗略解
void PnPsolver::find_betas_approx_3(const CvMat * L_6x10, const CvMat * Rho,
			       double * betas)
{
  double l_6x5[6 * 5], b5[5];
  CvMat L_6x5 = cvMat(6, 5, CV_64F, l_6x5);
  CvMat B5    = cvMat(5, 1, CV_64F, b5);

  for(int i = 0; i < 6; i++) {
    cvmSet(&L_6x5, i, 0, cvmGet(L_6x10, i, 0));
    cvmSet(&L_6x5, i, 1, cvmGet(L_6x10, i, 1));
    cvmSet(&L_6x5, i, 2, cvmGet(L_6x10, i, 2));
    cvmSet(&L_6x5, i, 3, cvmGet(L_6x10, i, 3));
    cvmSet(&L_6x5, i, 4, cvmGet(L_6x10, i, 4));
  }

  cvSolve(&L_6x5, Rho, &B5, CV_SVD);

  // B00 = B0*B0, B11 = B1*B1，B00和B11都为正
  // 如果无法满足B00和B11同时为正，则将B11置为0
  // 取根号：sqrt(B00)=B0
  if (b5[0] < 0) {
    betas[0] = sqrt(-b5[0]);
    betas[1] = (b5[2] < 0) ? sqrt(-b5[2]) : 0.0;
  } else {
    betas[0] = sqrt(b5[0]);
    betas[1] = (b5[2] > 0) ? sqrt(b5[2]) : 0.0;
  }
  if (b5[1] < 0) betas[0] = -betas[0];
  betas[2] = b5[3] / betas[0];
  betas[3] = 0.0;
}

// 计算并填充矩阵L
void PnPsolver::compute_L_6x10(const double * ut, double * l_6x10)
{
  // ut为12*12的特征向量矩阵，ut每一行为一组特征向量解（u的每一列为一组特征向量解）
  // 由于svd按照特征值大小降序排列，因此越往下排列的特征向量越优
  // 论文中是分别讨论了N=1, N=2, N=3, N=4四种情况进行求解，这里实现时按照最一般的情况进行求解，即N=4
  // 因此这里使用v[0], v[1], v[2], v[3]分别取出最优的四组特征向量解
  const double * v[4];

  v[0] = ut + 12 * 11;
  v[1] = ut + 12 * 10;
  v[2] = ut + 12 *  9;
  v[3] = ut + 12 *  8;

  // dv三维数组
  // 4代表4个svd中的最优四个特征向量，
  // 6代表4个控制点之间的向量差（距离）：[0, 1], [0, 2], [0, 3], [1, 2], [1, 3], [2, 3]之间的向量差
  // 3代表向量差的x, y, z三个轴
  double dv[4][6][3];

  for(int i = 0; i < 4; i++) {
    int a = 0, b = 1;
    for(int j = 0; j < 6; j++) {
      dv[i][j][0] = v[i][3 * a    ] - v[i][3 * b];
      dv[i][j][1] = v[i][3 * a + 1] - v[i][3 * b + 1];
      dv[i][j][2] = v[i][3 * a + 2] - v[i][3 * b + 2];

      // 4个相机坐标系下控制点有6个距离
      // [j=0, a=0, b=1], [j=1, a=0, b=2], [j=2, a=0, b=3], [j=3, a=1, b=2], [j=4, a=1, b=3], [j=5, a=2, b=3]
      b++;
      if (b > 3) {
        a++;
        b = a + 1;
      }
    }
  }

  // Beta用B简写, ||x||表示对x取平方和（二范数），ij表示从4个控制点中选择其中2个，总共有6中选择（用k表示）
  // Lij = ||(B0*v0[i]+B1*v1[i]+B2*v2[i]+B3*v3[i]) - (B0*v0[i]+B1*v1[i]+B2*v2[i]+B3*v3[i])||
  //     = ||B0*(v0[i]-v0[j]) + B1*(v1[i]-v1[j]) + B2*(v2[i]-v2[j]) + B3*(v3[i]-v3[j])||
  // Lk = ||B0*dv0[k] + B1*dv1[k] + B2*dv2[k] + B3*dv3[k]||
  //    = (dv0[k]^2 * B0^2) + (2*dv0[k]*dv1[k]*B0*B1) + (dv1[k]^2 * B1^2) + 
  //      (2*dv0[k]*dv2[k]*B0*B2) + (2*dv1[k]*dv2[k]*B1*B2) +
  //      (dv2[k]^2 * B2^2) + (2*dv0[k]*dv3[k]*B0*B3) +
  //      (2*dv1[k]*dv3[k]*B1*B3) + (2*dv2[k]*dv3[k]*B2*B3) + (dv3[k]^2 * B3^2)
  // 简写为： B00简写为B0*B0，dv0[k]^2简写为dv00，其它的以此类推
  // dv00*B00 + 2*dv01*B01 + dv11*B11 + 2*dv02*B02 + 2*dv12*B12 + dv22*B22 + 2*dv03*B03 + 2*dv13*B13 + 2*dv23*B23 + dv33*B33
  // 即：L * Betas
  // Lk: |dv00, 2*dv01, dv11, 2*dv02, 2*dv12, dv22, 2*dv03, 2*dv13, 2*dv23, dv33|, 1*10
  // Betas：[B00 B01 B11 B02 B12 B22 B03 B13 B23 B33]
  // 四个控制点共6个距离，因此L为6*10
  for(int i = 0; i < 6; i++) {
    double * row = l_6x10 + 10 * i;

    row[0] =        dot(dv[0][i], dv[0][i]);  // dv0^2
    row[1] = 2.0f * dot(dv[0][i], dv[1][i]);  // 2*dv0*dv1
    row[2] =        dot(dv[1][i], dv[1][i]);  // dv1^2
    row[3] = 2.0f * dot(dv[0][i], dv[2][i]);  // 2*dv0*dv2
    row[4] = 2.0f * dot(dv[1][i], dv[2][i]);  // 2*dv1*dv2
    row[5] =        dot(dv[2][i], dv[2][i]);  // dv2^2
    row[6] = 2.0f * dot(dv[0][i], dv[3][i]);  // 2*dv0*dv3
    row[7] = 2.0f * dot(dv[1][i], dv[3][i]);  // 2*dv1*dv3
    row[8] = 2.0f * dot(dv[2][i], dv[3][i]);  // 2*dv2*dv3
    row[9] =        dot(dv[3][i], dv[3][i]);  // dv3^2
  }
}

// 计算四个控制点任意两点间的距离，总共6个距离
// (0, 1)、(0, 2)、(0, 3)、(1, 2)、(1, 3)、(2, 3)
void PnPsolver::compute_rho(double * rho)
{
  rho[0] = dist2(cws[0], cws[1]);
  rho[1] = dist2(cws[0], cws[2]);
  rho[2] = dist2(cws[0], cws[3]);
  rho[3] = dist2(cws[1], cws[2]);
  rho[4] = dist2(cws[1], cws[3]);
  rho[5] = dist2(cws[2], cws[3]);
}

void PnPsolver::compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho,
					double betas[4], CvMat * A, CvMat * b)
{
  for(int i = 0; i < 6; i++) {
    const double * rowL = l_6x10 + i * 10;
    double * rowA = A->data.db + i * 4;

    // A为6*4矩阵
    // 详细公式见：compute_L_6x10 函数中的注释
    // （dv00*B00 + 2*dv01*B01 + dv11*B11 + 2*dv02*B02 + 2*dv12*B12 + dv22*B22 + 2*dv03*B03 + 2*dv13*B13 + 2*dv23*B23 + dv33*B33）
    // rowL: |dv00, 2*dv01, dv11, 2*dv02, 2*dv12, dv22, 2*dv03, 2*dv13, 2*dv23, dv33|
    // wubo 这里雅克比求错了，不需要乘以2
    rowA[0] = 2 * rowL[0] * betas[0] +     rowL[1] * betas[1] +     rowL[3] * betas[2] +     rowL[6] * betas[3];  // 对B0求导
    rowA[1] =     rowL[1] * betas[0] + 2 * rowL[2] * betas[1] +     rowL[4] * betas[2] +     rowL[7] * betas[3];  // 对B1求导
    rowA[2] =     rowL[3] * betas[0] +     rowL[4] * betas[1] + 2 * rowL[5] * betas[2] +     rowL[8] * betas[3];  // 对B2求导
    rowA[3] =     rowL[6] * betas[0] +     rowL[7] * betas[1] +     rowL[8] * betas[2] + 2 * rowL[9] * betas[3];  // 对B3求导

    // b为6*1矩阵
    // Betas: [B00 B01 B11 B02 B12 B22 B03 B13 B23 B33]
    // l_6x10 * Betas_10x1得到机体坐标系下控制点距离平方
    // rho记录了世界坐标系下控制点之间距离平方
    // b为机体坐标系下控制点平方距离 与 世界坐标系下控制点平方距离 的残差
    cvmSet(b, i, 0, rho[i] -
	   (
	    rowL[0] * betas[0] * betas[0] +
	    rowL[1] * betas[0] * betas[1] +
	    rowL[2] * betas[1] * betas[1] +
	    rowL[3] * betas[0] * betas[2] +
	    rowL[4] * betas[1] * betas[2] +
	    rowL[5] * betas[2] * betas[2] +
	    rowL[6] * betas[0] * betas[3] +
	    rowL[7] * betas[1] * betas[3] +
	    rowL[8] * betas[2] * betas[3] +
	    rowL[9] * betas[3] * betas[3]
	    ));
  }
}

void PnPsolver::gauss_newton(const CvMat * L_6x10, const CvMat * Rho,
			double betas[4])
{
  const int iterations_number = 5;

  double a[6*4], b[6], x[4];
  CvMat A = cvMat(6, 4, CV_64F, a);
  CvMat B = cvMat(6, 1, CV_64F, b);
  CvMat X = cvMat(4, 1, CV_64F, x);

  for(int k = 0; k < iterations_number; k++) {
    // 构造Ax = B中的A和B，A为目标函数关于待优化变量（B0、B1、B2、B3）的雅克比，
    // B为目标函数当前残差（相机坐标系下控制点之间的平方距离与世界坐标系下控制点之间的平方距离之差）
    compute_A_and_b_gauss_newton(L_6x10->data.db, Rho->data.db,
				 betas, &A, &B);
    qr_solve(&A, &B, &X);

    for(int i = 0; i < 4; i++)
      betas[i] += x[i];
  }
}

void PnPsolver::qr_solve(CvMat * A, CvMat * b, CvMat * X)
{
  static int max_nr = 0;
  static double * A1, * A2;

  const int nr = A->rows;
  const int nc = A->cols;

  if (max_nr != 0 && max_nr < nr) {
    delete [] A1;
    delete [] A2;
  }
  if (max_nr < nr) {
    max_nr = nr;
    A1 = new double[nr];
    A2 = new double[nr];
  }

  double * pA = A->data.db, * ppAkk = pA;
  for(int k = 0; k < nc; k++) {
    double * ppAik = ppAkk, eta = fabs(*ppAik);
    for(int i = k + 1; i < nr; i++) {
      double elt = fabs(*ppAik);
      if (eta < elt) eta = elt;
      ppAik += nc;
    }

    if (eta == 0) {
      A1[k] = A2[k] = 0.0;
      cerr << "God damnit, A is singular, this shouldn't happen." << endl;
      return;
    } else {
      double * ppAik = ppAkk, sum = 0.0, inv_eta = 1. / eta;
      for(int i = k; i < nr; i++) {
	*ppAik *= inv_eta;
	sum += *ppAik * *ppAik;
	ppAik += nc;
      }
      double sigma = sqrt(sum);
      if (*ppAkk < 0)
	sigma = -sigma;
      *ppAkk += sigma;
      A1[k] = sigma * *ppAkk;
      A2[k] = -eta * sigma;
      for(int j = k + 1; j < nc; j++) {
	double * ppAik = ppAkk, sum = 0;
	for(int i = k; i < nr; i++) {
	  sum += *ppAik * ppAik[j - k];
	  ppAik += nc;
	}
	double tau = sum / A1[k];
	ppAik = ppAkk;
	for(int i = k; i < nr; i++) {
	  ppAik[j - k] -= tau * *ppAik;
	  ppAik += nc;
	}
      }
    }
    ppAkk += nc + 1;
  }

  // b <- Qt b
  double * ppAjj = pA, * pb = b->data.db;
  for(int j = 0; j < nc; j++) {
    double * ppAij = ppAjj, tau = 0;
    for(int i = j; i < nr; i++)	{
      tau += *ppAij * pb[i];
      ppAij += nc;
    }
    tau /= A1[j];
    ppAij = ppAjj;
    for(int i = j; i < nr; i++) {
      pb[i] -= tau * *ppAij;
      ppAij += nc;
    }
    ppAjj += nc + 1;
  }

  // X = R-1 b
  double * pX = X->data.db;
  pX[nc - 1] = pb[nc - 1] / A2[nc - 1];
  for(int i = nc - 2; i >= 0; i--) {
    double * ppAij = pA + i * nc + (i + 1), sum = 0;

    for(int j = i + 1; j < nc; j++) {
      sum += *ppAij * pX[j];
      ppAij++;
    }
    pX[i] = (pb[i] - sum) / A2[i];
  }
}



void PnPsolver::relative_error(double & rot_err, double & transl_err,
			  const double Rtrue[3][3], const double ttrue[3],
			  const double Rest[3][3],  const double test[3])
{
  double qtrue[4], qest[4];

  mat_to_quat(Rtrue, qtrue);
  mat_to_quat(Rest, qest);

  double rot_err1 = sqrt((qtrue[0] - qest[0]) * (qtrue[0] - qest[0]) +
			 (qtrue[1] - qest[1]) * (qtrue[1] - qest[1]) +
			 (qtrue[2] - qest[2]) * (qtrue[2] - qest[2]) +
			 (qtrue[3] - qest[3]) * (qtrue[3] - qest[3]) ) /
    sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

  double rot_err2 = sqrt((qtrue[0] + qest[0]) * (qtrue[0] + qest[0]) +
			 (qtrue[1] + qest[1]) * (qtrue[1] + qest[1]) +
			 (qtrue[2] + qest[2]) * (qtrue[2] + qest[2]) +
			 (qtrue[3] + qest[3]) * (qtrue[3] + qest[3]) ) /
    sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

  rot_err = min(rot_err1, rot_err2);

  transl_err =
    sqrt((ttrue[0] - test[0]) * (ttrue[0] - test[0]) +
	 (ttrue[1] - test[1]) * (ttrue[1] - test[1]) +
	 (ttrue[2] - test[2]) * (ttrue[2] - test[2])) /
    sqrt(ttrue[0] * ttrue[0] + ttrue[1] * ttrue[1] + ttrue[2] * ttrue[2]);
}

void PnPsolver::mat_to_quat(const double R[3][3], double q[4])
{
  double tr = R[0][0] + R[1][1] + R[2][2];
  double n4;

  if (tr > 0.0f) {
    q[0] = R[1][2] - R[2][1];
    q[1] = R[2][0] - R[0][2];
    q[2] = R[0][1] - R[1][0];
    q[3] = tr + 1.0f;
    n4 = q[3];
  } else if ( (R[0][0] > R[1][1]) && (R[0][0] > R[2][2]) ) {
    q[0] = 1.0f + R[0][0] - R[1][1] - R[2][2];
    q[1] = R[1][0] + R[0][1];
    q[2] = R[2][0] + R[0][2];
    q[3] = R[1][2] - R[2][1];
    n4 = q[0];
  } else if (R[1][1] > R[2][2]) {
    q[0] = R[1][0] + R[0][1];
    q[1] = 1.0f + R[1][1] - R[0][0] - R[2][2];
    q[2] = R[2][1] + R[1][2];
    q[3] = R[2][0] - R[0][2];
    n4 = q[1];
  } else {
    q[0] = R[2][0] + R[0][2];
    q[1] = R[2][1] + R[1][2];
    q[2] = 1.0f + R[2][2] - R[0][0] - R[1][1];
    q[3] = R[0][1] - R[1][0];
    n4 = q[2];
  }
  double scale = 0.5f / double(sqrt(n4));

  q[0] *= scale;
  q[1] *= scale;
  q[2] *= scale;
  q[3] *= scale;
}

} //namespace ORB_SLAM
