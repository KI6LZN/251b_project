# Investigation of Adaptive Filtering Techniques for Vehicle Tracking in Videos
This is code for the ECE 251B Project in the Spring 2021 quarter.

This project focuses on adaptive filtering algorithms used for tracking individual vehicles as they move through a surveillance-style traffic video. The project paper and presentation slides are available in the Documents folder.

# Data
The dataset being used for this project is the University at Albany DEtection and TRACking (UA-DETRAC) image set. This is available through their website at: https://detrac-db.rit.albany.edu/ and https://sites.google.com/view/daweidu/projects/ua-detrac.

# File Structure
The "Video Functions" folder contains the code to verify that the dataset downloaded correctly and simply display the videos with or without bounding boxes.

The "Tracking Functions" folder contains code to track a single vehicle or all vehicles in the frame using one of the adaptive filters that have been tuned for this dataset, and using one of the motion models for the vehicles.

The "Tuning Functions" folder contains code to perform the parameter tuning on all implemented filters for the motion models they are compatible with.

Currently implemented filters are: 
- Kalman Filter with Constant Velocity motion model
- Unscented Kalman Filter with Constant Velocity motion model
- Unscented Kalman Filter with Constant Turn motion model
- Particle Filter with Constant Velocity motion model
- Particle Filter with Constant Turn motion model

# To Use
- Change the video sequence that is being run with the seq_num variable at the top of the file.
- Change the filter being used with the filt_name variable at the top of the file.
- Change the filter parameters in the createNewTracks function at the bottom of the file. 
  - Each filter has different parameters to change. 
  - The optimized values that result in the outputs shown in the chart in the paper are in the comments.
