# Event Camera Kalman Filtered Corner Tracker
By Joseph Seklawy

This project utilises the corner event extraction suite developed by Mueggler, Bartolozzi & Scaramuzza, and builds ontop of it by deploying a kalman filter to aid in filtering and tracking corners through the camera's frame. Some assumptions and hueristics are leveraged in order to select events from the feature events to be taken as measurments for the kalman filterting steps.

Testing and results were conducting and obtained by using the shapes_translation data set ROS bag created by (Mueggler et al. 2017)

The MATLAB scripts included here were devloped for working with pre-recorded ROS bags taken from a DAVIS camera. There currently exists no functionality to run with live data from ROS, however the script can be altered to do so easily. The only required input from the live stream would be that of the feature_event topic and the image_raw topics.




# Instructions - Supplied data sets
1. For running the system with the data set used in testing, run the corresponding scripts for their respective data sets as named accordinly DataSet1.m and            DataSet2.m


# Instructions - Other data sets
Setup
1. Extract the times and positions of the feature events into a matlab variable or .mat file. This may require some coding outside of MATLAB such as C++ as     the amount of data within the ROS bag can be so much that the extraction processes in MATLAB takes far too long.
      a) the positions should be stored in a cell. Where the rows correspond to the ROS messages and the columns contain all the x,y and time values in           cells. Refer to Featuresxyt.mat as an example
2. Run a Harris Corner detection pass over the first standard image frame obtained from the DAVIS sesnor and select corners of the same object you wish to    track. E.g. the 6 corners of a hexagon
3. Set those positions as the initial measurment and initial guess in a script following the structure as shown in the DataSet1.m script
4. Set process noise, measurment noise and velocities as desired. These may need to be altered during testing depending on the data set being used
5. Following the examples of the DataSet1.m and DataSet2.m, store the variables and pass them to an object of class corners and simply set up a loop to go    through each message in the data set and perform tracking  

# Notes
Some other variables you may need to change within the corner class depending on use case
1. The criteria set in the getPointsInEllisp function, i.e. the time limit
2. The measurment noise value set in the kalman gain step in the case of no measurment selected. Line 239.
