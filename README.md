# Investigation of Adaptive Filtering Techniques for Vehicle Tracking in Videos
This is code for the ECE 251B Project in the Spring 2021 quarter.

This project focuses on adaptive filtering algorithms used for tracking individual vehicles as they move through a surveillance-style traffic video.

# Data
The dataset being used for this project is the University at Albany DEtection and TRACking (UA-DETRAC) image set. This is available through their website at: https://detrac-db.rit.albany.edu/ and https://sites.google.com/view/daweidu/projects/ua-detrac.

# File Structure
The "Video Functions" folder contains the code to verify that the dataset downloaded correctly and simple display the videos with or without bounding boxes.

The "Tracking Functions" folder contains code to track a single vehicle or all vehicles in the frame using one of the adaptive filters that have been tuned for this dataset, and using one of the motion models for the vehicles. Current motion models are "Constant Velocity" and "Constant Turn."

The "Tuning Functions" folder contains code to perform the parameter tuning on all implemented filters for the motion models they are compatible with.
