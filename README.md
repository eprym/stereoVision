# stereoVision
This is the program I developed in my bachelor graduation project, which is about stereoVision. The main goal of this program is to reconstruct the 3D environment during the motion of the mobile robot and track the motion trajectory, which is part of a SLAM problem. The program is based on the Bumblebee XB3 stereo camera produced by PointGrey Research. In the program, several open source libraries are used, including OpenCV, PCL, Matlab camera calibration toolbox, Libviso2 and g2o. The are also some other files that the program depends on, including camera calibration and distortion information files, sample images, g2o data files and so on. You may refer to the following if you get confused about the camera information files.

1. distortions_left.xml and distortions_mid.xml save the distortions parameters of left and middle lens.
2. distortions_left2.xml and distortions_right2.xml save the distortions parameters of left and right lens.
3. intrinsic_left.xml and intrinsic_mid.xml save the intrinsic parameter of left and middle lens.
4. intrinsic_left2.xml and intrinsic_right2.xml save the intrinsic parameter of left and right lens.
5. om.xml and translation.xml save the rotation and translation parameters between left and middle lens.
6. om2.xml and translation2.xml save the rotation and translation parameters between left and right lens.
