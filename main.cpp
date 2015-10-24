#include <iostream>
#include <string>
#include <pthread.h>
#include "myImage.h"
#include "myStereoVision.h"
#include "myVisualOdometry.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

struct parameter
{
    int threadNum;
    myImage* image;
    myStereoVision* msv;
    myVisualOdometry* mvo;
};

void myImageFunc(myImage* image)
{
    image->ImageCapture();
}

void myStereoVisionFunc(myStereoVision* msv, myVisualOdometry* mvo)
{
    string dir = "2015_05_12_test_fps1/";
    string imgfilename;
    char basename[256];
    Mat left, right;
    for(int i=0; i < msv->numImage; i++)
    {
        sprintf(basename, "left%04d.png", i);
        imgfilename = dir + basename;
        left = imread(imgfilename, 0);
        sprintf(basename, "mid%04d.png", i);
        imgfilename = dir + basename;
        right = imread(imgfilename, 0);
        msv->StereoMatching(left, right, 1);
        msv->reprojectTo3D(left);
        msv->SparsePointCloud();
        msv->stitchPointCloud(mvo->Data, i);
    }
    msv->showPointCloud();
}


void myVisualOdometryFunc(myVisualOdometry* mvo)
{
    mvo->runVisualOdometry();
}
void* thr_fn(void* args)
{
    struct parameter* fnargs = (struct parameter*)args;
    int x = fnargs->threadNum;
    switch(x)
    {
        case 0 : myImageFunc(fnargs->image);break;
        case 1 : myVisualOdometryFunc(fnargs->mvo);break;
        case 2 : myStereoVisionFunc(fnargs->msv, fnargs->mvo);break;
        default : cout<<"invalid value of x"<<endl;
    }
    pthread_exit(0);
}

int main()
{
    struct parameter* args;
    int numImage = 488;
    int numBegin = 0;
    myImage* image = new myImage(numImage, numBegin);
    image->getRecParameter();
    myStereoVision* msv = new myStereoVision(image->numImage, image->Q);
    msv->initStereo();
    myVisualOdometry* mvo = new myVisualOdometry(numImage);
    args->image = image;
    args->msv = msv;
    args->mvo = mvo;
    pthread_t tid[4];
    for (int i = 0; i < 3; i++)
    {
        args->threadNum = i;
        int err = pthread_create(&tid[i], NULL, thr_fn,(void*)args);
		if (err != 0){
			printf("cannot create the thread %d\n", i+1);
			exit(1);
		}
		if(i == 0)  sleep(3);
		else        sleep(1);
    }
    pthread_join(tid[2], NULL);
    return 0;
}
