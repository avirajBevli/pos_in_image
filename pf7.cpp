//I have decided the world cordinate frame to lie on the corner of the table
//I have assumed we know the position of the door in the world coordinate frame
//To reach the camera coordinate frame from the world coordinate frame, we will have to translate and rotate
//This information has been encoded in the extrinsic matrix

//The webcamera had been calibrated by "CameraCalibration.cpp" and the result has been saved in the file "The_camera_calibration_matrix"
//The result obtained from calibrating the web camera have been used in this program(The instrinsic matrix)

//In this program, we will feed in the 4 corners of the door in the world coordinate frame
//We then display the outline of the door in the webcam frame(the bounding blue box)

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"

#include<bits/stdc++.h> 
#include <iostream>
#include <fstream>//inlcude the fstream class
#include <math.h> 
#include <vector>
#include <stdlib.h>
#include <stdio.h> 

#include <sstream>

using namespace std;
using namespace cv;

void matmul1(double K[3][3], double E[3][4], double P[3][4])
{
	int i, j, k;
	for(i=0;i<3;i++)
	{
		for(j=0;j<4;j++)
		{
			P[i][j] = 0;
			for(k=0;k<3;k++)
			{
				P[i][j] = P[i][j] + K[i][k]*E[k][j];
			}
		}
	}
}

void matmul2(double P[3][4], double c[4][1], double cd[3][1])
{
	int i, j, k;
	for(i=0;i<3;i++)
	{
		cd[i][0] = 0;
		for(k=0;k<4;k++)
		{
			cd[i][0] = cd[i][0] + P[i][k]*c[k][0];
		}
	}

	for(i=0;i<3;i++)
	{
		cd[i][0] = cd[i][0]/cd[2][0];
	}
}

void printMat(double P[3][4])
{
	int i, j;
	for(i=0;i<3;i++)
	{
		for(j=0;j<4;j++)
		{
			cout<<P[i][j]<<" ";
		}
		cout<<endl;
	}

	cout<<endl<<endl;
}

int main()
{
	//from camera calibration, we got the values of fx, fy, cx, cy(file with these values: "The_camera_calibration_matrix" )
	double fx = 735.677;
	double cx = 236.822;
	double fy = 733.533;
	double cy = 254.936;

	double K[3][3] = { {fx, 0, cx}, {0, fy, cy}, {0, 0, 1} };
	//This is the instrinsic matrix...this was found by caibrating the webcam of my laptop using "CameraCalibration.cpp" 
	//and the result had been saved in the file "The_camera_calibration_matrix"

	//the 3d vector represents the translational vector from the world to the camera coordinates(webcam of my laptop)
	double tx = 18.5;
	double ty = 25.3;
	double tz = 0;

	double t[3][1] = { {tx}, {ty}, {tz} };//this is the 3 dimensional translation vector(in non-homogenous coordinates)
	
	
	//Since, the world coordinate frame does not lie along the same axes as the camera coordinate system,
	//we calculate the rotation matrix first and concatenate the rotation matrix with the translation vector, 
	//to obtain the 3*4 extrinsic matrix, E[3][4]
	double E[3][4] = { {0, -1, 0, tx}, {0, 0, -1, ty}, {1, 0, 0, tz} };

	double P[3][4];//this is the final projection matrix, formed by matrix multiplication of the K and the E matrices
	matmul1(K, E, P);//this function multiplies E and K matrices and populates the P(projection) matrix

	//printMat(P);

	//the 4 points in the world coordinate system(may represent the 4 corners of the door)
	//c1, c2, c3, c4 are the corners of the door in a cyclic order
	double c1[4][1] = { {56.3}, {0}, {20.5}, {1} };
	double c2[4][1] = { {56.3}, {0}, {38.5}, {1} };
	double c3[4][1] = { {56.3}, {29.5}, {38.5}, {1} };
	double c4[4][1] = { {56.3}, {29.5}, {20.5}, {1} };

	////the corresponding 4 points in the image
	double c1d[3][1];
	double c2d[3][1];
	double c3d[3][1];
	double c4d[3][1];

	//The function matmul2 will populate the vectors c1d, c2d, c3d, c4d
	matmul2(P, c1, c1d);
	matmul2(P, c2, c2d);
	matmul2(P, c3, c3d);
	matmul2(P, c4, c4d);

	cout<<"The 4 points in the image have the coordinates: "<<endl;
	cout<<"c1d: ("<<c1d[0][0]<<","<<c1d[1][0]<<","<<c1d[2][0]<<")"<<endl;
	cout<<"c2d: ("<<c2d[0][0]<<","<<c2d[1][0]<<","<<c2d[2][0]<<")"<<endl;
	cout<<"c3d: ("<<c3d[0][0]<<","<<c3d[1][0]<<","<<c3d[2][0]<<")"<<endl;
	cout<<"c4d: ("<<c4d[0][0]<<","<<c4d[1][0]<<","<<c4d[2][0]<<")"<<endl;

	//VideoCapture is the class and cap is the object of that class

	VideoCapture cap(0);
	//opens the defaut video camera

	if(cap.isOpened() == false)
	{
		cout << "cannot open the video camera" <<endl;
		cin.get(); //wait for any key press
        return -1;
	}

	int frame_width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));  //get the width of frames of the video
 
	int frame_height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));

	Size frame_size(frame_width, frame_height);
    int frames_per_second = 10;

    String window_name = "sample_video.mp4";

    VideoWriter oVideoWriter(window_name, VideoWriter::fourcc('M', 'J', 'P', 'G'), frames_per_second, frame_size, true); 

    Mat imgTmp;
    cap.read(imgTmp); 
    Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );

    //To draw the blue rectangle in the image, bounding the corners of the door
	line(imgLines, Point(c1d[0][0], c1d[1][0]), Point(c2d[0][0], c2d[1][0]), Scalar(255,0,0), 5);
	line(imgLines, Point(c2d[0][0], c2d[1][0]), Point(c3d[0][0], c3d[1][0]), Scalar(255,0,0), 5);
	line(imgLines, Point(c3d[0][0], c3d[1][0]), Point(c4d[0][0], c4d[1][0]), Scalar(255,0,0), 5);
	line(imgLines, Point(c4d[0][0], c4d[1][0]), Point(c1d[0][0], c1d[1][0]), Scalar(255,0,0), 5);

   	while (true)
	{
	    Mat frame;
	    bool isSuccess = cap.read(frame); // read a new frame from the video camera 

	    //Breaking the while loop if frames cannot be read from the camera
	    if (isSuccess == false)
	    {
	        cout << "Video camera is disconnected" << endl;
	        cin.get(); //Wait for any key press
	        break;
	    }

	    //write the video frame to the file
	    oVideoWriter.write(frame); 
	    
	    //so that the webcam displays the door as well on the frame obtained from the webcam 
	    frame = frame + imgLines;

	    //show the frame in the created window
	    imshow(window_name, frame);

	    //Wait for for 10 milliseconds until any key is pressed.  
	    //If the 'Esc' key is pressed, break the while loop.
	    //If any other key is pressed, continue the loop 
	    //If any key is not pressed within 10 milliseconds, continue the loop 
	    if (waitKey(10) == 27)
	    {
	        cout << "Esc key is pressed by the user. Stopping the video" << endl;
	        break;
	    }
	}

  //Flush and close the video file
    oVideoWriter.release();
    
    return 0;
}

