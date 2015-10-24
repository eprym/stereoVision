#include <string>
#include <vector>
#include <flycapture/FlyCapture2.h>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc//imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/contrib/contrib.hpp"

using namespace std;
using namespace FlyCapture2;
using namespace cv;

class myImage
{
public:
    CvMat* cvcamera_matrix1, *cvcamera_matrix2;
    CvMat* cvcamera_matrix3, *cvcamera_matrix4;
    CvMat* cvdistortions1, *cvdistortions2;
    CvMat* cvdistortions3, *cvdistortions4;
    CvMat* cvR_tmp, *cvR_tmp2;
    CvMat* cvT, *cvT2;
    Mat camera_matrix1, camera_matrix2;
    Mat camera_matrix3, camera_matrix4;
    Mat distortions1, distortions2;
    Mat distortions3, distortions4;
    Mat R_tmp,T,R,Rl,Rm,Pl,Pm,Q;
    Mat R_tmp2,T2,R2,Rl2,Rm2,Pl2,Pm2,Q2;
    Mat mapx1, mapy1, mapx2, mapy2;
    Mat mapx3, mapy3, mapx4, mapy4;


    Error error;
	BusManager busMgr;
	PGRGuid guid;
	Camera cam;
	int numImage;
	int numBegin;
	Format7ImageSettings format;
	int width = 1280;
	int height = 960;
	Image rawImage;
	Mat imageLeft;
    Mat imageMid;
    Mat imageRight;
    Mat imageLeft_rec;
    Mat imageMid_rec;
    Mat imageRight_rec;
	unsigned int fps = 480; // 480 means one frame per second


	bool success = true;
	char basename[256];
	string dir = "2015_05_12_test_fps1";
	string imgfilename;



	myImage(int numImage, int numBegin);
	~myImage(){};
    void ImageCapture();
    void ImageSeparate();
    void getRecParameter();
    void ImageRectify(int imgCnt);
    void PrintError()
    {
        error.PrintErrorTrace();
        success = false;
    }

    Mat FlycaptureToOpencv()
    {
        unsigned int rowBytes = (double)rawImage.GetReceivedDataSize()/(double)rawImage.GetRows();
        Mat cvImage = Mat(rawImage.GetRows(), rawImage.GetCols(), CV_8UC3, rawImage.GetData(), rowBytes);
        return cvImage;
    }
};

//Initial the myImage class
myImage::myImage(int numImage, int numBegin)
{
    this->numImage = numImage;
    this->numBegin = numBegin;
    R.create(3, 3, CV_64F);
    Rl.create(3, 3, CV_64F);
    Rm.create(3, 3, CV_64F);
    Pl.create(3, 3, CV_64F);
    Pm.create(3, 3, CV_64F);
    Q.create(4, 4, CV_32F);

    R2.create(3, 3, CV_64F);
    Rl2.create(3, 3, CV_64F);
    Rm2.create(3, 3, CV_64F);
    Pl2.create(3, 3, CV_64F);
    Pm2.create(3, 3, CV_64F);
    Q2.create(4, 4, CV_32F);

    imageLeft.create(height, width, CV_8U);
    imageMid.create(height, width, CV_8U);
    imageRight.create(height, width, CV_8U);
    imageLeft_rec.create(height, width, CV_8U);
    imageMid_rec.create(height, width, CV_8U);
    imageRight_rec.create(height, width, CV_8U);

    error = busMgr.GetCameraFromIndex(0, &guid);
    if (error != PGRERROR_OK)
    {
       PrintError();
       return;
    }

	error = cam.Connect(&guid);
    if (error != PGRERROR_OK)
    {
        PrintError();
        return;
    }
    format.mode = MODE_3;
	format.pixelFormat = PIXEL_FORMAT_RGB8;
	format.width = width;
	format.height = height;
	error = cam.SetFormat7Configuration(&format, fps);
	if (error != PGRERROR_OK)
    {
        PrintError();
        return;
    }
}


//Separate the left, middle and right images from the original image
void myImage::ImageSeparate()
{
    Mat tmp = FlycaptureToOpencv();
    string imgfilename;
	for (int row = 0; row < tmp.rows; row++)
	{
		for (int col = 0; col < tmp.cols; col++)
		{
            imageLeft.at<uchar>(row, col) = tmp.at<Vec3b>(row, col)[2];
            imageMid.at<uchar>(row, col) = tmp.at<Vec3b>(row, col)[1];
            imageRight.at<uchar>(row, col) = tmp.at<Vec3b>(row, col)[0];

		}
	}
//	sprintf(basename, "/left%02d.png", imgNum);
//	imgfilename = dir + basename;
//	imwrite(imgfilename,left);
//	sprintf(basename, "/mid%02d.png", imgNum);
//	imgfilename = dir + basename;
//	imwrite(imgfilename,mid);
//	sprintf(basename, "/right%02d.png", imgNum);
//	imgfilename = dir + basename;
//	imwrite(imgfilename,right);
}


//Capture the image from the camera
void myImage::ImageCapture()
{
    if (success == false)
    {
        cout<<"The class Image is initialized unsuccessfully"<<endl;
        return;
    }
    cam.StartCapture();
	if (error != PGRERROR_OK)
    {
        PrintError();
        return;
    }
    for ( int imageCnt= numBegin; imageCnt < numImage; imageCnt++ )
    {
        error = cam.RetrieveBuffer(&rawImage);
        if (error != PGRERROR_OK)
        {
            printf("Error grabbing image %u\n", imageCnt);
            continue;
        }
        else
        {

            printf("Grabbed image %u\n", imageCnt);

        }

        //vecImages[imageCnt].DeepCopy(&rawImage);
		//seperateImage(imageCnt, rawImage, imageLeft, imageMid, imageRight);
		ImageSeparate();
		ImageRectify(imageCnt);
    }

    error = cam.StopCapture();
    if (error != PGRERROR_OK)
    {
        PrintError();
        return;
    }

    // Disconnect the camera
    error = cam.Disconnect();
    if (error != PGRERROR_OK)
    {
        PrintError();
        return;
    }
}

//Get the rectified parameter of the lens
    void myImage::getRecParameter()
    {
        cvcamera_matrix1 = (CvMat*)cvLoad("intrinsic_left.xml");
		cvcamera_matrix2 = (CvMat*)cvLoad("intrinsic_mid.xml");
		cvdistortions1 = (CvMat*)cvLoad("distortions_left.xml");
		cvdistortions2 = (CvMat*)cvLoad("distortions_mid.xml");
		cvR_tmp = (CvMat*)cvLoad("om.xml");
		cvT = (CvMat*)cvLoad("translation.xml");

		cvcamera_matrix3 = (CvMat*)cvLoad("intrinsic_left2.xml");
		cvcamera_matrix4 = (CvMat*)cvLoad("intrinsic_right2.xml");
		cvdistortions3 = (CvMat*)cvLoad("distortions_left2.xml");
		cvdistortions4 = (CvMat*)cvLoad("distortions_right2.xml");
		cvR_tmp2 = (CvMat*)cvLoad("om2.xml");
		cvT2 = (CvMat*)cvLoad("translation2.xml");
		cout<<"successfully read parameters"<<endl;

		camera_matrix1 = Mat(cvcamera_matrix1, false);
		camera_matrix2 = Mat(cvcamera_matrix2, false);
		distortions1 = Mat(cvdistortions1, false);
		distortions2 = Mat(cvdistortions2, false);
		R_tmp = Mat(cvR_tmp, false);
		T = Mat(cvT, false);

		camera_matrix3 = Mat(cvcamera_matrix3, false);
		camera_matrix4 = Mat(cvcamera_matrix4, false);
		distortions3 = Mat(cvdistortions3, false);
		distortions4 = Mat(cvdistortions4, false);
		R_tmp2 = Mat(cvR_tmp2, false);
		T2 = Mat(cvT2, false);

		Rodrigues( R_tmp, R);
		Rodrigues( R_tmp2, R2);
		cout<<"successfully made the Rodrigues transformation"<<endl;
		stereoRectify(camera_matrix1, distortions1, camera_matrix2, distortions2, Size(width, height),
				R, T, Rl, Rm, Pl, Pm, Q, CV_CALIB_ZERO_DISPARITY);
        stereoRectify(camera_matrix3, distortions3, camera_matrix4, distortions4, Size(width, height),
				R2, T2, Rl2, Rm2, Pl2, Pm2, Q2, CV_CALIB_ZERO_DISPARITY);
		cout<<"successfully get rectified parameters"<<endl;
		initUndistortRectifyMap(camera_matrix1, distortions1, Rl, Pl, Size(width, height),
				CV_32FC1, mapx1, mapy1);
		initUndistortRectifyMap(camera_matrix2, distortions2, Rm, Pm, Size(width, height),
				CV_32FC1, mapx2, mapy2);
        initUndistortRectifyMap(camera_matrix3, distortions3, Rl2, Pl2, Size(width, height),
				CV_32FC1, mapx3, mapy3);
		initUndistortRectifyMap(camera_matrix4, distortions4, Rm2, Pm2, Size(width, height),
				CV_32FC1, mapx4, mapy4);
    }


//Rectify images using rectified parameters
    void myImage::ImageRectify(int imgNum)
    {
        remap(imageLeft, imageLeft_rec, mapx1, mapy1, INTER_LINEAR);
        remap(imageMid, imageMid_rec, mapx2, mapy2, INTER_LINEAR);
        remap(imageRight, imageRight_rec, mapx4, mapy4, INTER_LINEAR);
        sprintf(basename, "/left%04d.png", imgNum);
        imgfilename = dir + basename;
        imwrite(imgfilename,imageLeft_rec);
        remap(imageLeft, imageLeft_rec, mapx3, mapy3, INTER_LINEAR);
        sprintf(basename, "/rleft%04d.png", imgNum);
        imgfilename = dir + basename;
        imwrite(imgfilename, imageLeft_rec);
        sprintf(basename, "/mid%04d.png", imgNum);
        imgfilename = dir + basename;
        imwrite(imgfilename,imageMid_rec);
        sprintf(basename, "/right%04d.png", imgNum);
        imgfilename = dir + basename;
        imwrite(imgfilename,imageRight_rec);
        printf("Rectify and save the image %d\n", imgNum);
    }

