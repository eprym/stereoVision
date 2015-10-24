#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>
#include <pthread.h>
#include <viso_stereo.h>
#include <png++/png.hpp>
#include <fstream>
#include <math.h>
using namespace std;


class myVisualOdometry
{
public:
    int numImage;
    int32_t width = 1280;
    int32_t height = 960;
    VisualOdometryStereo::parameters param;
    VisualOdometryStereo* viso;
    Matrix* Data;
    Matrix* Data_motion;
    Matrix pose = Matrix::eye(4);
    Matrix motion = Matrix::eye(4);
    string dir = "2015_05_12_test_fps1/";
    string imgfilename;
    char basename[256];
    uint8_t* left_img_data;
    uint8_t* right_img_data;
    int32_t dims[3] = {width,height,width};
    ofstream fout{"./data.txt"};
    ofstream fout1{"./position.txt"};
    double eulerAngle[4];
    double eulerAngleMotion[4];
    double quaternion[4];
    double quaternionMotion[4];

    myVisualOdometry(int numImage);
    ~myVisualOdometry();
    void readImage(int img);
    void computeOdometry(int img);
    void runVisualOdometry();
    void writeOdometry(int img);
    void deComposeMatrix(int img);
    void eulerToQuaternion();
};

//Initial the myVisualOdometry class
myVisualOdometry::myVisualOdometry(int numImage)
{
    this->numImage = numImage;
    this->param.calib.f = 1047.83;
    this->param.calib.cu = 644.944;
    this->param.calib.cv = 484.225;
    this->param.base = 0.12;
    //this->param.bucket.max_features = 5;
//    this->param.calib.f = 1040.57;
//    this->param.calib.cu = 647.51;
//    this->param.calib.cv = 475.69;
//    this->param.base = 0.24;
    viso = new VisualOdometryStereo(this->param);
    Data = new Matrix[numImage];
    Data[0] = pose;
    Data_motion = new Matrix[numImage];
    Data_motion[0] = motion;
    left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
    right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
}

//Read images
void myVisualOdometry::readImage(int img)
{
    sprintf(basename, "left%04d.png", img);
    imgfilename = dir + basename;
    png::image< png::gray_pixel > left_img(imgfilename);
    sprintf(basename, "mid%04d.png", img);
    imgfilename = dir + basename;
    png::image< png::gray_pixel > right_img(imgfilename);

    //int32_t width  = left_img.get_width();
    //int32_t height = left_img.get_height();

    // convert input images to uint8_t buffer
    //uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
    //uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
    int32_t k=0;
    for (int32_t v=0; v<height; v++) {
    for (int32_t u=0; u<width; u++) {
        left_img_data[k]  = left_img.get_pixel(u,v);
        right_img_data[k] = right_img.get_pixel(u,v);
        k++;
        }
    }
}

//Compute visual odometry matrix using libviso2
void myVisualOdometry::computeOdometry(int img)
{
    //cout<<"Processing frame : "<<img;
    printf("Processing frame %d\n", img);
    if (viso->process(left_img_data,right_img_data,dims)) {

        // on success, update current pose
        motion = viso->getMotion();
        pose = pose * Matrix::inv(motion);

        // output some statistics
        double num_matches = viso->getNumberOfMatches();
        double num_inliers = viso->getNumberOfInliers();
        cout << ", Matches: " << num_matches;
        cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
        cout << pose << endl << endl;
        Data[img] = pose;
        Data_motion[img] = motion;

      } else {
        //cout << " ... failed!" << endl;
        printf("...failed!\n");
        if (img >= 2)
        {   Data[img] = Data[img-1] * 2 - Data[img-2];
            Data_motion[img] = Data_motion[img-1] * 2 - Data_motion[img-2];
        }
        if (img == 1)
        {   Data[img] = Data[0];
            Data_motion[img] = Data_motion[0];
        }
      }
}



//Run the computation process of the visual odometry
void myVisualOdometry::runVisualOdometry()
{
    for(int i= 0; i<numImage; i++)
    {
        try{
            readImage(i);
            computeOdometry(i);
            writeOdometry(i);
        }
        catch(exception ex){printf("cannot compute the odometry\n");sleep(1); i--;}

    }
    fout.close();
    fout1.close();
}

//Compute three euler angles from odometry matrix
void myVisualOdometry::deComposeMatrix(int img)
{
    eulerAngle[0] = atan2(Data[img].val[0][1],Data[img].val[0][0]);
    eulerAngle[1] = atan2(-Data[img].val[0][2],
                          sqrt(Data[img].val[1][2] * Data[img].val[1][2] + Data[img].val[2][2] * Data[img].val[2][2]));
    eulerAngle[2] = atan2(Data[img].val[0][1],Data[img].val[0][0]);
    eulerAngle[3] = 1;
    eulerAngleMotion[0] = atan2(Data_motion[img].val[0][1],Data_motion[img].val[0][0]);
    eulerAngleMotion[1] = atan2(-Data_motion[img].val[0][2],
                          sqrt(Data_motion[img].val[1][2] * Data_motion[img].val[1][2] + Data_motion[img].val[2][2] * Data_motion[img].val[2][2]));
    eulerAngleMotion[2] = atan2(Data_motion[img].val[0][1],Data_motion[img].val[0][0]);
    eulerAngleMotion[3] = 1;
}

//Change from euler form to quaternion form
void myVisualOdometry::eulerToQuaternion()
{
    double roll = eulerAngle[0], pitch = eulerAngle[1],  yaw = eulerAngle[2];
    double rollmotion = eulerAngleMotion[0], pitchmotion = eulerAngleMotion[1],
            yawmotion = eulerAngleMotion[2];
    quaternion[0] = sin(roll/2) * cos(pitch/2) * cos(yaw/2) -
                    cos(roll/2) * sin(pitch/2) * sin(yaw/2);
    quaternion[1] = cos(roll/2) * sin(pitch/2) * cos(yaw/2) +
                    sin(roll/2) * cos(pitch/2) * sin(yaw/2);
    quaternion[2] = cos(roll/2) * cos(pitch/2) * sin(yaw/2) -
                    sin(roll/2) * sin(pitch/2) * cos(yaw/2);
    quaternion[3] = cos(roll/2) * cos(pitch/2) * cos(yaw/2) +
                    sin(roll/2) * sin(pitch/2) * sin(yaw/2);

    quaternionMotion[0] = sin(rollmotion/2) * cos(pitchmotion/2) * cos(yawmotion/2) -
                    cos(rollmotion/2) * sin(pitchmotion/2) * sin(yawmotion/2);
    quaternionMotion[1] = cos(rollmotion/2) * sin(pitchmotion/2) * cos(yawmotion/2) +
                    sin(rollmotion/2) * cos(pitchmotion/2) * sin(yawmotion/2);
    quaternionMotion[2] = cos(rollmotion/2) * cos(pitchmotion/2) * sin(yawmotion/2) -
                    sin(rollmotion/2) * sin(pitchmotion/2) * cos(yawmotion/2);
    quaternionMotion[3] = cos(rollmotion/2) * cos(pitchmotion/2) * cos(yawmotion/2) +
                    sin(rollmotion/2) * sin(pitchmotion/2) * sin(yawmotion/2);

}

//Write the odometry to txt file, which will be used by g2o later
void myVisualOdometry::writeOdometry(int img)
{
    if (!fout){
		cerr<<"cannot write into the file"<<endl;
		return;
	}
	if (!fout1){
		cerr<<"cannot write into the file"<<endl;
		return;
	}
	deComposeMatrix(img);
	eulerToQuaternion();
	fout<<"VERTEX_SE3:QUAT"<<" "<<img<<" "<<Data[img].val[0][3]<<" "<<Data[img].val[1][3]
	<<" "<<Data[img].val[2][3]<<" "<<quaternion[0]<<" "<<quaternion[1]<<" "
	<<quaternion[2]<<" "<<quaternion[3]<<"\n";
	if(img == 0) fout<<"FIX 0\n";
	if(img > 0)
    {
       fout<<"EDGE_SE3:QUAT"<<" "<<img-1<<" "<<img<<" "<<Data_motion[img].val[0][3]<<" "<<Data_motion[img].val[1][3]
	<<" "<<Data_motion[img].val[2][3]<<" "<<quaternionMotion[0]<<" "<<quaternionMotion[1]<<" "
	<<quaternionMotion[2]<<" "<<quaternionMotion[3]<<" "<<1<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "
	<<0<<" "<<1<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<1<<" "<<0<<" "<<0<<" "<<0<<" "
	<<1<<" "<<0<<" "<<0<<" "<<1<<" "<<0<<" "<<1<<"\n";
    }
    fout1<<img<<"   "<<Data[img].val[0][3]<<"   "<<Data[img].val[2][3]<<"\n";
    cout<<"save the position"<<img<<endl;

}
