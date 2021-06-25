# Pos_in_image
You can use this project to predict the position of any object in the image captured by your webcam(or any camera for that matter)


##Steps:
* You need to first "Calibrate" your camera. 
* Refer to this link to know more about what is camera calibration ->https://boofcv.org/index.php?title=Tutorial_Camera_Calibration#:~:text=Camera%20calibration%20is%20the%20process,and%20orientation%20in%20the%20world"
* Once you have calibrated your camera, we will have calculated the camera's intrinstic(things like camera's focal length, skew distortion, position of the image centre), extrinsic parameters(position of the camera with respect to the word). 
* Using these calculated parameters, we can then predict where each point in the world would map to, in the image captured by our camera.


## CameraCalibration.cpp
* Used to calibrate the web camera(estimate the instrinsic paramteres)
* The result of the calibration has been saved in the file "The_camera_calibration_matrix".
* We use this result in "pf7.cpp" for the intrinsic matrix of the webcam. 

## pf7.cpp
* "pf7.cpp" takes in the 4 corners of a door in the world coordinate frame and computes the projection matrix. 
* This projection matrix is then used to calculate the position of the 4 corners in the image. 
* The 4 corners in the image have been bounded by a blue box in the image. This blue box signifies where the door will lie in the image.

## situation.jpg
The image "situation.jpg" is the representation of the world coordinate frame and the camera coordinate frame and the 4 corners in the world coordinate frame

## result.jpg
The image "result.jpg" is the result I obtained where the corners of the notebook on top had been taken as the 4 points(representing the 4 corners of the door) in the world coordinate frame. The corners of the blue box are the projected coordinates of the 4 corners.
