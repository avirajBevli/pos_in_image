//I found ideas for this code online, to quickly calibrate the web camera
//This code saves the camera calibration matrix into a new file(The_camera_caliration_matrix)
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"

#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

const float calibrationSquareDimension = 0.0245f;//meters
const float arucoSquareDimension = 0.1016f; //meters
const Size chessboardDimensions = Size(9,6);
//9*6 is not the number of squares, but it is the number of intersections of squares

void createArucoMarkers()
{
    Mat outputMarker;//mat object to insert aruco marker image into

    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
    //ptr is a pointer

    for (int i = 0; i < 50; i++)
    {
        aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
        ostringstream convert;
        string imageName = "4x4Marker_";
        convert << imageName << i << ".jpg";
        imwrite(convert.str(), outputMarker);
    }
}

//this function will create points in the real world distane scale, on the planar board
void createKnownBoardPosition(Size boardsize, float squareEdgeLength, vector<Point3f>& corners)
{
    for (int i = 0; i < boardsize.height; i++)
    {
        for (int j = 0; j < boardsize.width; j++)
        {
            corners.push_back(Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));//last argument is the z value, that is alsways zero(flat plane)
        }
    }
}
//corners is the vector of 3d points in the plane of the paper, with the z coordinate equals 0

//function to find and visualize the corners from the chessboard
void getchessboardcorners(vector<Mat> images, vector<vector<Point2f>>& allfoundcorners, bool showresults = false)
{
    //iterator is present in c++
    for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
    {
        vector<Point2f> pointBuf;
        //to hold all the points found in the image

        bool found = findChessboardCorners(*iter, Size(9, 6), pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

        if(found)
        {
            allfoundcorners.push_back(pointBuf);
            //allfoundcorners was passed to this function
        }

        if (showresults)
        {
            drawChessboardCorners(*iter, Size(9, 6), pointBuf, found);
           //this function takes in the corners and draws them on the image
            imshow("Looking for Corners", *iter);
           //we passed the particular image as the iterator for imshow 
            waitKey(0);
        }
    }
}


//the function takes in the good images, that is the calibration images that we want to use
//this function acrually calibrates the camera, that is it populates the camera matrix and the distance coefficient matrices
void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoefficients)
{
    vector<vector<Point2f>> checkerboardImageSpacePoints;
    //the below function is passed the good images and it returns to us all the different points in the chess board
    getchessboardcorners(calibrationImages, checkerboardImageSpacePoints, false);
   
   //this is the list of the know corners in the world coordinates
    vector<vector<Point3f>> worldSpaceCornerPoints(1);

    createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
    worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);

    vector<Mat> rVectors, tVectors;
    distanceCoefficients = Mat::zeros(8, 1, CV_64F);

    calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors );
    //this is an opencv function only
}

//this function takes in a name that you want to call the file, takes in the matric, distance coefficients and returns true or false
//this function actually prints the cameraMatrix and distortionCoefficientMatrix in the file "The_camera_calibration_matrix"
//first, cameraMatrix values are written down in the order of rows....then the distortionCoefficient Matrix values are written down in the order of rows
bool saveCameraCalibration(string name, Mat cameraMatrix, Mat distanceCoefficients)
{
    ofstream outStream(name);
    //ofstream: Stream class to write on files
    if (outStream)
    {
        uint16_t rows = cameraMatrix.rows;
        uint16_t columns = cameraMatrix.cols;

        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < columns; c++)
            {
                double value = cameraMatrix.at<double>(r, c);
                outStream << value << endl;
            }
        }

        rows = distanceCoefficients.rows;
        columns = distanceCoefficients.cols;

        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < columns; c++)
            {
                double value = distanceCoefficients.at<double>(r, c);
                outStream << value << endl;
            }

        }

        outStream.close();
        return true;
    }
    return false;
}

int main(int argv, char** argc)
{
    Mat frame;
    Mat drawToFrame;

    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    //identuty matrix

    Mat distanceCoefficients;

    vector<Mat> savedImages;
    //if we see a good calibration, it can e saved automatically

    vector<vector<Point2f>> markerCorners, rejectedCandidates;

    VideoCapture vid(0);

    if (!vid.isOpened())
    {
        return 0;
    }
    
    int framesPersecond = 20;
    namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

    while(true)
    {
        if (!vid.read(frame))
            break;

        vector<Vec2f> foundPoints;
        //to store the points found
        bool found = false;

        found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE );
        frame.copyTo(drawToFrame);
        //if the corners are found, the drawToFrame will be displayed instead of the original frame

        drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);
       
        if (found)
            imshow("Webcam", drawToFrame);
        else
            imshow("Webcam", frame);
        
        char character = waitKey(1000 / framesPersecond);
//now what character the user presses, will decide what to do next       
//if we see a valid checkerboard, we will save it by pressing the space key
//if we now have enough images and want to start the camera calibration we will use the enter key
//escape key to exit        

        switch(character)
        {
        case ' ':
            //if space bar is pressed, means we want to be saving the image
            if(found)
            {
                Mat temp;
                frame.copyTo(temp);//save the current frame of the webcam
                savedImages.push_back(temp);//push the current frame into savedImages
            }
            break;
        case 13:
            //enter key: start calibration(press enter when you are satisfied with the images you have saved and now want to start the process of camera calibration)
            if (savedImages.size() > 15)
            {
                cameraCalibration(savedImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix, distanceCoefficients);
                //this function will populate the cameraMatrix
                saveCameraCalibration("The_camera_calibration_matrix", cameraMatrix, distanceCoefficients);
            //The_camera_calibration_matrix will be the name of the file where the camera matrix will be saved
            }

            break;
        case 27:
            //escape key: exit
            return 0;
            break;

        }

    }
    return 0;

}