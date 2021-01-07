# Visual Odometry for Localization in Autonomous Driving

This project implements visual odometry to estimate the trajectory of a self-driving car. It processes images taken with a monocular camera set up on the vehicle.

This project is the programming assignment for *Module 2: Visual Features - Detection, Description and Matching* in the [Visual Perception for Self-Driving Cars](https://www.coursera.org/learn/visual-perception-self-driving-cars?) course. The [University of Toronto](https://www.utoronto.ca/) provided the starter code of this project.

**The steps of this project are the following:**

- Extract features from the photographs taken with a camera setup on the vehicle.
- Use the extracted features to find matches between the features in different photographs.
- Use the found matches to estimate the camera motion between subsequent photographs.
- Use the estimated camera motion to build the vehicle trajectory.

<p align="center">
<img src="output/camera-motion.gif" width=67% height=67%/>
</p>



## Preliminaries

Here is a definition of **visual odometry** from [Wikipedia](https://en.wikipedia.org/wiki/Visual_odometry).

> In robotics and computer vision, visual odometry is the process of determining the position and orientation of a robot by analyzing the associated camera images. It has been used in a wide variety of robotic applications, such as on the Mars Exploration Rovers.

The algorithm implementation is divided into four parts.

1. Acquire an image and extract features using a *feature detector*.
2. Find corresponding features in another image with *feature matching* or *feature tracking*.
3. Determine the camera pose from the *Perspective-n-Point* solution using the *RANSAC* scheme.
4. Propagate the vehicle trajectory from the camera pose estimation.

This [thesis](http://publications.lib.chalmers.se/records/fulltext/246134/246134.pdf) covers each part of the algorithm in greater detail.

## Loading and Visualizing the Data

The starter code provides a convenient dataset handler class to read and iterate through samples taken from the [CARLA](https://carla.org/) simulator.

```python
dataset_handler = DatasetHandler()
```

The dataset handler contains 52 data frames. Each frame contains an RGB image and a depth map taken with a setup on the vehicle and a grayscale version of the RGB image which will be used for computation. Furthermore, the camera calibration matrix `K` is also provided in the dataset handler.

Upon creation of the dataset handler object, all the frames will be automatically read and loaded. The frame content can be accessed by using `images`, `images_rgb`, `depth_maps` attributes of the dataset handler object along with the index of the requested frame.

Here is an example of the grayscale images:

<p align="center">
<img src="output/image.png" width=67% height=67%/>
</p>

Here is an example of the RGB images:

<p align="center">
<img src="output/image-rgb.png" width=67% height=67%/>
</p>

Here is an example of the depth maps:

<p align="center">
<img src="output/depth-maps.png" width=67% height=67%/>
</p>


**Notes about Depth Maps**

The maximum depth distance is 1000. This value of depth shows that the selected pixel is at least 1000m (1km) far from the camera however, the exact distance of this pixel from the camera is unknown. Having these points in further trajectory estimation might affect precision.

## Feature Extraction

The purpose of this section is to implement a function to extract features from an image. A **feature** is a point of interest in an image defined by its image pixel coordinates. A **descriptor** is an n-dimensional vector that summarizes the image information around the detected feature. The following figure illustrates these properties.

<p align="center">
<img src="output/feature-slide.png" />
</p>

Here is a list of common feature detectors:

- Scale-Invariant Feature Transform (**SIFT**)
- Speeded-Up Robust Features (**SURF**)
- Features from Accelerated Segment Test (**FAST**)
- Binary Robust Independent Elementary Features (**BRIEF**)
- Oriented FAST and Rotated BRIEF (**ORB**)

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
    # Initiate ORB detector
    orb = cv2.ORB_create(nfeatures=1500)
    
    # Find the keypoints and descriptors with ORB
    kp, des = orb.detectAndCompute(image, None)
    
    return kp, des
```

Here is an example of the extracted features:

<p align="center">
<img src="output/feature-extraction.png" width=67% height=67%/>
</p>

## Feature Matching

The purpose of this section is to implement a function to match features in a sequence of images. **Feature matching** is the process of establishing correspondences between two images of the same sequence.

<p align="center">
<img src="output/matcher-slide.png" />
</p>

OpenCV provides two techniques to match different descriptors: Brute-Force matcher and FLANN based matcher.

The **Brute-Force** matcher compares the descriptor of one feature in the first image to all other features in the second image. The algorithm then matches the descriptor with the shortest distance to the descriptor in the first image.

**FLANN** stands for Fast Library for Approximate Nearest Neighbors. It contains a collection of algorithms optimized for fast neighbor search in datasets and for high dimensional features.

```python
def match_features(des1, des2):
    """
    Match features from two images

    Arguments:
    des1 -- list of the keypoint descriptors in the first image
    des2 -- list of the keypoint descriptors in the second image

    Returns:
    match -- list of matched features from two images. Each match[i] is k or less matches for the same query descriptor
    """
    # Define FLANN parameters
    FLANN_INDEX_LSH = 6
    index_params = dict(algorithm = FLANN_INDEX_LSH,
                        table_number = 6,
                        key_size = 12,
                        multi_probe_level = 1)
    search_params = dict(checks = 50)
    
    # Initiate FLANN matcher
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    
    # Find matches with FLANN
    match = flann.knnMatch(des1, des2, k=2)
    
    return match
```

Here is an example of the matched features:

<p align="center">
<img src="output/feature-matching.png" />
</p>

Trajectory Estimation

References

