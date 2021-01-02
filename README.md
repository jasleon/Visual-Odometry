# Visual Odometry for Localization in Autonomous Driving

This software pipeline implements visual odometry to estimate the trajectory of a self-driving car. It processes images taken with a monocular camera set up on the vehicle.

This project is the programming assignment for *Module 2: Visual Features - Detection, Description and Matching* in the [Visual Perception for Self-Driving Cars](https://www.coursera.org/learn/visual-perception-self-driving-cars?) course. The [University of Toronto](https://www.utoronto.ca/) provided the starter code of this project.

**The steps of this project are the following:**

- Extract features from the photographs taken with a camera setup on the vehicle.
- Use the extracted features to find matches between the features in different photographs.
- Use the found matches to estimate the camera motion between subsequent photographs.
- Use the estimated camera motion to build the vehicle trajectory.

![result](output/visual-odometry.gif)

## Preliminaries

Here is a definition of **visual odometry** from [Wikipedia](https://en.wikipedia.org/wiki/Visual_odometry).

> In robotics and computer vision, visual odometry is the process of determining the position and orientation of a robot by analyzing the associated camera images. It has been used in a wide variety of robotic applications, such as on the Mars Exploration Rovers.

The algorithm implementation is divided into four parts [[1]](http://publications.lib.chalmers.se/records/fulltext/246134/246134.pdf).

1. Acquire an image and extract features using a *feature detector*.
2. Find corresponding features in another image with *feature matching* or *feature tracking*.
3. Determine the camera pose from the *Perspective-n-Point* solution using the *RANSAC* scheme.
4. Propagate the vehicle trajectory from the camera pose estimation.

## Loading and Visualizing the Data

The starter code provides a convenient dataset handler class to read and iterate through samples taken from the [CARLA](https://carla.org/) simulator.

```python
dataset_handler = DatasetHandler()
```

The dataset handler contains 52 data frames. Each frame contains an RGB image and a depth map taken with a setup on the vehicle and a grayscale version of the RGB image which will be used for computation. Furthermore, the camera calibration matrix `K` is also provided in the dataset handler.

Upon creation of the dataset handler object, all the frames will be automatically read and loaded. The frame content can be accessed by using `images`, `images_rgb`, `depth_maps` attributes of the dataset handler object along with the index of the requested frame.

**Notes about Depth Maps**

The maximum depth distance is 1000. This value of depth shows that the selected pixel is at least 1000m (1km) far from the camera however, the exact distance of this pixel from the camera is unknown. Having these points in further trajectory estimation might affect precision.

## Feature Extraction

The purpose of this section is to implement a function to extract features from an image. A **feature** is a point of interest in an image defined by its image pixel coordinates. In contrast, a **descriptor** is an n-dimensional vector that provides a summary of the image information around the detected feature. The following image illustrates these feature properties.

<p align="center">
<img src="output\features.png" />
</p>

Here is a list of common feature detectors:

- **S**cale-**I**nvariant **F**eature **T**ransform (**SIFT**)
- **S**peeded-**U**p **R**obust **F**eatures (**SURF**)
- **F**eatures from **A**ccelerated **S**egment **T**est (**FAST**)
- **B**inary **R**obust **I**ndependent **E**lementary **F**eatures (**BRIEF**)
- **O**riented FAST and **R**otated **B**RIEF (**ORB**)

The OpenCV documentation provides implementation [examples](https://docs.opencv.org/3.4.3/db/d27/tutorial_py_table_of_contents_feature2d.html) of these detectors.

```python
def extract_features(image):
    """
    Find keypoints and descriptors for the image

    Arguments:
    image -- a grayscale image

    Returns:
    kp -- list of the extracted keypoints (features) in an image
    des -- list of the keypoint descriptors in an image
    """
    # Initiate SURF detector
    surf = cv2.xfeatures2d.SURF_create(400)
        
    # Find keypoints and descriptors directly
    kp, des = surf.detectAndCompute(image, None)
    
    return kp, des
```

Feature Matching

Trajectory Estimation

References

