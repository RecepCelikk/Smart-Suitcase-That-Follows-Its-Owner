# Smart-Suitcase-That-Follows-Its-Owner
 In this project, the suitcase project was automatically followed by the owner. The luggage is capable of following a person and avoiding obstacles. After the image of the colored object on the person is taken with the camera, the image captured by the image processing method is converted to the HSV color space and the color range of the object is determined and the object is detected. Finally, by determining the size and the center of the object, the direction of the suitcase will be determined. When the object to be followed with Raspberry Pi is detected by image processing method, depending on the position of the object, the motor driver card communicates to determine which direction the suitcase will go.