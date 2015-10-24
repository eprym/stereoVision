#include <iostream>
#include <fstream>
#include<exception>
#include <time.h>
#include<cstdlib>
#include <string>
#include "cv.h"
#include "highgui.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/contrib/contrib.hpp"

#include "pcl/common/common.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <matrix.h>

using namespace std;
using namespace cv;
using namespace pcl;
//using namespace Eigen;

class myStereoVision
{
public:

    int numImage;
    StereoBM bm;
    StereoSGBM sgbm;
    StereoVar var;
    string sphereID = "sphere";
    char basename[256];
    //Mat left, right;
    Mat disp, reproject_image, Q;
    Eigen::Matrix4f visoData;
    Eigen::Vector4f start{0, 0, 0, 1};
    Eigen::Vector4f destination;
    PointCloud<PointXYZI>::Ptr myPointCloud{new PointCloud<PointXYZI>};
    PointCloud<PointXYZI>::Ptr myPointCloud_filtered{new PointCloud<PointXYZI>};
    PointCloud<PointXYZI>::Ptr outputCloud{new PointCloud<PointXYZI>};
    PointCloud<PointXYZI>::Ptr wholePointCloud{new PointCloud<PointXYZI>};
    VoxelGrid<PointXYZI> sor;
    //visualization::CloudViewer viewer{"stitch point cloud"};
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer {new pcl::visualization::PCLVisualizer ("Stitch Point Cloud")};

    myStereoVision(int numImage, Mat Q);
    ~myStereoVision();
    void initStereo();
    //int readData();
    void StereoMatching(Mat left, Mat right, int method);
    void reprojectTo3D(Mat left);
    void SparsePointCloud();
    void MatrixToMatrix4f(Matrix src);
    void stitchPointCloud(Matrix* Data, int imgNum);
    void calculateTrajectory(Eigen::Matrix4f trajectory, int imgNum);
    void showPointCloud();


};

//Initial the myStereoVision class
myStereoVision::myStereoVision(int numImage, Mat Q)
{
    this->numImage = numImage;
    this->Q = Q;
    reproject_image.create(960, 1280, CV_32FC3);
    sor.setLeafSize(200.0f, 200.0f, 200.0f);
}

//Initial the parameters of the stereo vision system
void myStereoVision::initStereo()
{
    assert(bm.state != 0);
	bm.state->preFilterSize = 11;
	bm.state->preFilterCap = 15;
	bm.state->SADWindowSize = 7;
	bm.state->minDisparity = 0;
	bm.state->numberOfDisparities = 64;
	bm.state->textureThreshold = 5;
	bm.state->uniquenessRatio = 8;
	bm.state->speckleWindowSize = 100;
	bm.state->speckleRange = 32;
	bm.state->disp12MaxDiff = 1;

	sgbm.preFilterCap = 7;
	sgbm.SADWindowSize = 9;
	sgbm.P1 = 8*3*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.P2 = 32*3*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.minDisparity = 0;
	sgbm.numberOfDisparities = 32;
	sgbm.uniquenessRatio = 15;
	sgbm.speckleWindowSize = bm.state->speckleWindowSize;
	sgbm.speckleRange = 2;
	sgbm.disp12MaxDiff = 1;
	sgbm.fullDP =true;

	var.levels = 3; // ignored with USE_AUTO_PARAMS
	var.pyrScale = 0.5; // ignored with USE_AUTO_PARAMS
	var.nIt = 25;
	var.minDisp = -64;
	var.maxDisp = 0;
	var.poly_n = 5;
	var.poly_sigma = 1.1;
	var.fi = 15.0f;
	var.lambda = 0.03f;
	var.penalization = var.PENALIZATION_TICHONOV; // ignored with USE_AUTO_PARAMS
	var.cycle = var.CYCLE_V; // ignored with USE_AUTO_PARAMS
	var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY |var.USE_MEDIAN_FILTERING ;
}

//Using the specified method to match left and right images
void myStereoVision::StereoMatching(Mat left, Mat right, int method)
{
    //Mat vdisp;
    //clock_t begin, end;
    switch(method)
	{
	case 0:
		//cvFindStereoCorrespondenceBM(&left, &right, &disp, bm.state);
		//begin = clock();
		bm(left, right, disp);
		//end = clock();
		//normalize(disp, vdisp, 255, 0, NORM_MINMAX);
		//cvNamedWindow("disparity");
		//imshow("disparity",vdisp);
		//waitKey();
		//imwrite("disparity.png",vdisp);
		//cout<<"Time lasted for "<<(end-begin)/1000.0<<" seconds"<<endl;
		cout<<"successfully save the disparity map"<<endl;
		break;
	case 1:
		//begin = clock();
		sgbm(left, right, disp);
		//end = clock();
		//normalize(disp, vdisp, 255, 0, NORM_MINMAX);
		//imshow("disparity", vdisp);
		//waitKey();
		//imwrite("disparity.png", vdisp);
		//cout<<"Time lasted for "<<(end-begin)/1000000.0<<" seconds"<<endl;
		cout<<"successfully save the disparity map"<<endl;
		break;
	case 2:
		//begin = clock();
		var(left, right, disp);
		//end = clock();
		//normalize(disp, vdisp, 255, 0, NORM_MINMAX);
		//imshow("disparity", vdisp);
		//waitKey();
		//imwrite("disparity.tif", vdisp);
		//cout<<"Time lasted for "<<(end-begin)/1000.0<<" seconds"<<endl;
		cout<<"successfully save the disparity map"<<endl;
		break;
	default:
		cout<<"cannot find the specified method"<<endl;
		break;
	}
}

//Reproject from disparity image to 3D point cloud
	void myStereoVision::reprojectTo3D(Mat left)
	{
	    reprojectImageTo3D(disp, reproject_image, Q);
	    PointXYZI point;
        Point3f cvPoint;
	    for (int j = 0; j<reproject_image.rows; j++)
			{
				//printf("The %dth row\n",j);
				for (int k =0; k<reproject_image.cols; k++)
				{
					cvPoint = reproject_image.at<Point3f>(j,k);
					if (abs(cvPoint.z) < 500 && abs(cvPoint.x) < 200 && abs(cvPoint.y) < 200)
					{

						point.x = cvPoint.x * 16;
						point.y = cvPoint.y * 16;
						point.z = cvPoint.z * 16;
						point.intensity = left.at<unsigned char>(j,k);
//						point.b = left_rectified.at<Vec3b>(j,k)[0];
//						point.g = left_rectified.at<Vec3b>(j,k)[1];
//						point.r = left_rectified.at<Vec3b>(j,k)[2];
						if (point.intensity > 0 ) myPointCloud->points.push_back(point);
						//cout<<(int)myPointCloud->points.size()<<endl;
					}

				}
			}
			myPointCloud->width = (int) myPointCloud->points.size();
			myPointCloud->height = 1;
	}


//Sparse the point cloud using specified parameters
    void myStereoVision::SparsePointCloud()
    {
        sor.setInputCloud(myPointCloud);
        sor.filter(*myPointCloud_filtered);
    }


    void myStereoVision::MatrixToMatrix4f(Matrix src)
    {
        for(int i=0; i<4; i++)
        {
            for(int j=0; j<4; j++)
            {
                visoData(i,j) = src.val[i][j];
            }
        }

        visoData(0,3) *= 1000;
        visoData(1,3) *= 1000;
        visoData(2,3) *= 1000;
    }

//Calculate and show the trajectory
    void myStereoVision::calculateTrajectory(Eigen::Matrix4f trajectory, int imgNum)
    {
        destination = trajectory * start;
        PointXYZ center;
	    center.x = destination(0,0);
	    center.y = destination(1,0);
	    center.z = destination(2,0);
	    sprintf(basename, "%04d", imgNum);
	    sphereID = sphereID + basename;
	    viewer->addSphere(center, 100, 255, 0, 0, sphereID);

    }

    //Stitch multiple point cloud frames
	void myStereoVision::stitchPointCloud(Matrix* Data, int imgNum)
	{

        while(Data[imgNum].val == 0x00)
            {sleep(1);}
        MatrixToMatrix4f(Data[imgNum]);
        calculateTrajectory(visoData, imgNum);
	    transformPointCloud(*myPointCloud_filtered, *outputCloud, visoData);
        *wholePointCloud += *outputCloud;
        myPointCloud->clear();
        myPointCloud_filtered->clear();
        outputCloud->clear();


//		viewer.showCloud(wholePointCloud);
//		while(!viewer.wasStopped())
//		{
//		}
//		waitKey();
	}

//Show the point cloud
	void myStereoVision::showPointCloud()
	{

        viewer->setBackgroundColor (0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZI> (wholePointCloud, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->addCoordinateSystem (1000);
        viewer->initCameraParameters ();
		while(!viewer->wasStopped())
		{
		    viewer->spinOnce (100);
		}
		//waitKey();
	}



