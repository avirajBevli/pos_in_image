# pos_in_image
"CameraCalibration.cpp" is used to calibrate the web camera(estimate the instrinsic paramteres)

The result of the calibration has been saved in the file "The_camera_calibration_matrix".

We use this result in "pf7.cpp" to get the intrinsic matrix of the webcam. 

"pf7.cpp" takes in the 4 corners of a door in the world coordinate frame and computes the projection matrix. 
This projection matrix is then used to calculate the position of the 4 corners in the image. 

The 4 corners in the image have been bounded by a blue box in the image. This blue box signifies where the door will lie in the image.

The image "situation.jpg" is the representation of the world coordinate frame and the camera coordinate frame and the 4 corners in the world coordinate frame

The image "result.jpg" is the result I obtained where the corners of the notebook on top had been taken as the 4 points(representing the 4 corners of the door) in the world coordinate frame. The corners of the blue box are the projected coordinates of the 4 corners.

