//Assumed we have been given the following information- 
		//The robot's pose in the 3D world(x,y,theeta)
		//The orientation of the camera wrto the robot
		//We have the 3D coordinates of the corners of the doors(all doors have the same height)
		//We want to find the outlines of all the doors in the image

//Proposed Algorithm: Since it is relatively difficult to detect door in the image, we will instead try to detect doors in the 3D coordinates itself 
//We have taken the world coordinate in such a way that the doors lie along the x axis...
//Hence we sort the coordinates of the door according to their x coordinates and then according to their z coordinates
//Then sets of 4 points will belong to the same door and we can hence draw the boundries of the door after projecting the points on the image
#include<bits/stdc++.h> 
#include <iostream>
#include <fstream>//inlcude the fstream class
#include <math.h> 
#include <vector>
#include <stdlib.h>
#include <stdio.h> 
#include <sstream>
#include <algorithm>//for the sort function in

using namespace std;
using namespace cv;

//function to obtain the final projection matrix from K, Mrc, Mwr matrices
double** matMul3(double K[3][4], double Mrc[4][4], double Mwr[4][4])
{
	int i, j, k;
	double temp[4][4];
	for(i=0;i<4;i++)
	{	
		temp[i][j] = 0;
		for(j=0;j<4;j++)
		{
			for(k=0;k<4;k++)
			{
				temp[i][j] += Mrc[i][k]*Mwr[k][j];
			}
		}
	}

	double output[3][4];
	for(i=0;i<3;i++)
	{	
		output[i][j] = 0;
		for(j=0;j<4;j++)
		{
			for(k=0;k<4;k++)
			{
				output[i][j] += K[i][k]*temp[k][j];
			}
		}
	}

	return output;
}

typedef struct 3D_point
{
	double x, y, z;
}3D_point;

typedef struct 2D_point
{
	double x, y;
}2D_point;

2D_point project_to_image(double P[3][4], 3D_point point)
{
	double temp[3][1];
	temp[0][0] = point.x;
	temp[1][0] = point.y;
	temp[2][0] = point.z;
	double result[3][1];
	int i, j;
	for(i=0;i<3;i++)
	{
		result[i][0] = 0;
		for(j=0;j<4;j++)
		{
			result[i][0] += P[i][j]*temp[j][0]; 
		}
	}

	2D_point to_return;
	to_return.x = result[0][0]/result[2][0];	
	to_return.y = result[1][0]/result[2][0];	

	return to_return;
}

bool compare3d_points(3D_point p1, 3D_point p2) 
{ 
    if(p1.x == p2.x)
    	return (p1.z < p2.z);
    return (p1.x < p2.x);
} 

int main()
{
	//To get from the world to robot coordinate frame
	//The translation vector and Rotation matrix for world to Robot coordinate frame is - trw(3*1), Rrw(3*3)
	//The translation vector and Rotation matrix for Robot to Camera coordinate frame is - tcr(3*1), Rcr(3*3)

	//from Camera Calibration, we got the values of fx, fy, cx, cy(file with these values: "The_camera_calibration_matrix" )
	double fx = 735.677;
	double cx = 236.822;
	double fy = 733.533;
	double cy = 254.936;

	double K[3][4] = { {fx, 0, cx, 0}, {0, fy, cy, 0}, {0, 0, 1, 0} };//the Intrinsic matrix
	
	double x0, y0, theeta;//we get the robot pose in the world coordinate frame as an input
	//double Twr[3][1] = { {-x0*cos(theeta) + y0*sin(theeta)}, {x0*sin(theeta)-y0*cos(theeta)}, {0} };
	double Twr[3][1] = { {-x0*cos(theeta) - y0*sin(theeta)}, {x0*sin(theeta)-y0*cos(theeta)}, {0} };
	double Trc[3][1] = { {tx}, {ty}, {tz} };//this is fixed and known to us
	
	double Mrc[4][4];//is fixed all the time and known to us
	double Mwr[4][4];//keeps on changing with time
	Mwr = { {cos(theeta), sin(theeta), 0, Twr[0][0]} , {-sin(theeta), cos(theeta), 0, Twr[1][0]} , {0, 0, 1, Twr[2][0]} };

	double P[3][4] = matMul3(K,Mrc,Mwr);
	vector<3D_point> 3D_world_corners;//the list of cornerss of the door is known
	//All the doors have the same height from the floor
	vector<2D_point> corners_in_image;//vector of the door corners as points in the image
	
	//We need to detect rectangles from the list of 3D points "3D_world_corners"
	int n = 3D_world_corners.size();

	//Assumed that the doors lie in the plane of the wall and the wall lies on the X axis
	sort(3D_world_corners, 3D_world_corners+n, compare3d_points);
	//Now the points in sets of 4 form doors in increasing order of their x axis
	//Hence, points with increasing	x coordinates(in sets of 4) will give the doors (from the 4 corners of the doors)
	//the points are sorted in inreasing order of x coordinates and in case of tie in increasing order of their z coordinates

	int i;
	for(i=0;i<n;i++)
	{
		2D_point temp = project_to_image(P,3D_world_corners[i]);
		corners_in_image.push_back(temp);
	}
	//now the vector "corners_in_image" contain the corners of the doors in sets of 4

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

    //To draw the blue rectangles in the image, bounding the corners of the doors
    i=0;
    while(i!=n)//for all the (n/4) number of doors
    {
    	line(imgLines, Point(corners_in_image[i].x, corners_in_image[i].y), Point(corners_in_image[i+1].x, corners_in_image[i+1].y), Scalar(255,0,0), 5);
   		line(imgLines, Point(corners_in_image[i+1].x, corners_in_image[i+1].y), Point(corners_in_image[i+3].x, corners_in_image[i+3].y), Scalar(255,0,0), 5);
   		line(imgLines, Point(corners_in_image[i+3].x, corners_in_image[i+3].y), Point(corners_in_image[i+2].x, corners_in_image[i+2].y), Scalar(255,0,0), 5);
   		line(imgLines, Point(corners_in_image[i].x, corners_in_image[i].y), Point(corners_in_image[i+2].x, corners_in_image[i+2].y), Scalar(255,0,0), 5);	
   		i=i+4;
    }
   	
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