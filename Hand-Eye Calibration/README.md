# Hand-eye Calibration
This project is for hand-eye calibration of the panda robot in a hand-to-eye scenario. The camera and the base of robot are fixed respectively, and the calibration board is held by the gripper of the robot. The goal of calibration is to get the transform matice from camera to the base ```T_cam_base```.

## Collecting Data
To do the hand-eye calibration, 1. images of the robot holding the calibration board and 2. transform matrices from the end-effector to the base ```T_ee_base``` need to be collected simultaneously.
### Collect Images
To collect images, run the camera_handler.py in this directory:
``` 
cd ./Hand-Eye Calibration
python camera_handler.py
```
At each position, press ```s``` to save an image in this directory ```/Hand-Eye Calibration```. After enough images are collected, press ```c``` to exit the program.
### Collect Transform Matrices
Transfrom matices ```T_ee_base``` are obtained from the computer connected to panda. To collect transform matrices, run handeye.py:
```
python handeye.py
```
At each position, enter ```s``` to append the current ```T_ee_base``` to the list. After enough matrices are collected, enter ```d``` to save them in a txt file and exit the program.

## Calibration
Before calibration, folder of images and the txt file of matrices need to be put in this directory. ```MARKER_SIDE_LENGTH_MM``` needs to be specified in calibraion.py. To calculate the ```T_cam_base```:
```
cd ./Hand-Eye Calibration
python calibration.py --pose <pose_name.txt> --image <image_folder_name> --mode s
```
The ```mode``` can be chosen between ```s``` for a single aurco marker and ```b``` for a aurco marker board. After running this program, ```T_cam_base``` is calculated and saved in the txt file T_cam_base.txt or T_cam_base_board.txt. 

Tha transform matrix is calculated using opencv function [calibrateHandEye](https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html). In the official example, this function is initially used for hand-in-eye calibration. For hand-to-eye calibration in this project, the parameters are:
```
InputArrayOfArrays 	R_gripper2base,
InputArrayOfArrays 	t_gripper2base,
InputArrayOfArrays 	R_cam2target,
InputArrayOfArrays 	t_cam2target,
OutputArray 	R_target2gripper,
OutputArray 	t_target2gripper,
HandEyeCalibrationMethod 	method = CALIB_HAND_EYE_TSAI 
)	
```
Then, ```T_cam_base``` is calculated as:
```
T_cam2base = T_gripper2base*T_target2gripper*T_cam2target
```

## Validation
### Validation for ```T_cam2target```
In ```calibration.py```, ```T_cam2target``` is obtained from aruco markers with function ```estimatePoseSingleMarkers``` or ```estimatePoseBoard```. To validate if the result is correct, we compare ```t_cam2target``` to the 3d coordinate of the marker in the camera frame which is obtained directly from realsense depth camera. The comparison shows that the two matche with each other.

### Validation for ```T_gripper2base```
In ```calibration.py```, ```T_gripper2base``` is read directly from the robot, and thus should not need to be validated.

### Validation for calibration
We validate if the final ```T_cam_base``` is correct by picking a point in the streaming of the camera, and checking if its 3D coordinate in the base frame calculated with ```T_cam_base``` is consistent with the result read from the robot:
```
python validation.py
```
This program will allow you to pick a point and print its corresponding coordinates in the base reference. Before running the validation, you need to specify the variable ```transform_matrix``` as the outcome of the previous calibration pocess ```T_cam_base```.
The pixel coordinate of the picked point as well as its 3D coordinate in the camera frame are also saved to ``` test_node.txt```.

On the other hand, move the end-effector to the selected point, and read the last column of the current ```T_ee_base``` as the position of the point in the reference of the base.

The position of camera in the base coordinate is roughly (0.55, 0.875, 0.605).

## Trouble 
The return of calibrateHandEye ```R_target2gripper``` and ```t_target2gripper``` are quite different from each other when calculated from different (image, T_gripper2base) groups, and is not consistent with the result from validation or measurement.

