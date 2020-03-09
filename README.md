# Vision-based approach for GPS Localization Improvement in Semi-Obstructed Areas

by Alireza Ahmadi - July 2018
***

## What is the Problem?
Performance of Global Navigation Satellite System (GNSS) positioning in urban environments is hindered by poor satellite availability because there are many man-made and natural objects in urban environments that obstruct satellite signals.

**NLOS (Non-Line of Sight)**
(one of the error sources in GNSS observation due to receiving signals from satellites which antenna is blind to them)

<div align="center">
	<img src="/doc/NLOS1.png" alt="nlos" width="400" title="nlos"/>
</div>

### Our solution
Our approach involves with using a landscape calibrated camera and through image processing algorithms tries to classify the scene to non-sky and free-sky regions, also tries to determine whether the satellites are located in free-sky or not. And finally generates an Elevation map of the environment

<div align="center">
	<img src="/doc/nlos2.png" alt="nlos" width="700" title="nlos"/>
</div>

### Advantages of Integration of vision:

* Low-cost with respect to laser scanner
* Able to  get  a dense data of surrounded  area at  minimum time  
* Real-time capability for processing data
* The results are more sensible and intuitive for human, in case of having any type of visualization

## Main Pipeline
<div align="center">
	<img src="/doc/pipe.png" alt="pipe" width="700" title="pipe"/>
</div>

### Step1: NMEA Package(Parser)

Receiving the NMEA package from GPS receiver and extracting Satellites’ information (PRN-Position,...)

### Step2: Changing Image Color System to Gray (3 channel RGB to 1 channel intensity)
Once the data (image) acquisition is made, the second step which consists of transforming the acquired image in the most appropriate color space (which guarantees a better detection of objects of interest). In this case, the GRAY scale color system is chosen that decreases the process required to extract the features from the scene. 

<div align="center">
	<img src="/doc/gray.png" alt="gray" width="700" title="gray"/>
</div>

### Step3: Smoothing the image(Gaussian Blur)
The third step concerns with the image smoothing by Gaussian blur operator provided by the OpenCV library, in order to simplify the acquired image and eliminate unnecessary details and noises.


<div align="center">
	<img src="/doc/blur.png" alt="blur" width="400" title="blur"/>
</div>

### Step4: Edge Detection(Canny operator)
The Next step would be finding the most top edges of buildings in the scene and horizon line of the scene. First, all the edges captured by the camera should reveal sharply in a sense of violent variations in values of neighbored pixels, which Canny operator is used to exploits them, 

<div align="center">
	<img src="/doc/cany.png" alt="cany" width="400" title="cany"/>
</div>

### Step5: Morphological Operators(Dilation and Erosion)
Morphological transformations are some simple operations used to modify extracted feature from image like:

<div align="center">
	<img src="/doc/morph.png" alt="morph" width="400" title="morph"/>
</div>

### Step6: Line Detection (through Hough Transformation)
in general, a line can be detected by finding the number of intersections between curves in Hough space. The more curves intersecting means that the line represented by that intersection have more points. In general, we can define a threshold of the minimum number of intersections needed to detect a line.

<div align="center">
	<img src="/doc/line.png" alt="line" width="400" title="line"/>
</div>

### Step7: Watershed Segmentation
Segmentation: The goal of segmentation is to simplify change the representation of an image into something that is more meaningful and easier to analyze. In our case we are interested having the scene classified into sky and no-sky regions.

<div align="center">
	<img src="/doc/seg.png" alt="seg" width="700" title="seg"/>
</div>

### Auto Sampling the scene for the segmentation

<div align="center">
	<img src="/doc/auto.png" alt="auto" width="700" title="auto"/>
</div>

### Problems of the Segmentation

<div align="center">
	<img src="/doc/prob.png" alt="prob" width="700" title="prob"/>
</div>

### Merging results of Line detection and Segmentation

<div align="center">
	<img src="/doc/resmix.png" alt="resmix" width="700" title="resmix"/>
</div>

### Step8: Satellite Repositioning in images:
Satellites’ positions can be projected into the images through Projection matrix:
* Camera position and External points (Satellites’ poses) should be described in same coordinate system. (Earth Centered Earth Fixed->ECEF, WGS84) 
-> filtering the ones which place in the image size and marking them in image.

<div align="center">
	<img src="/doc/pin.png" alt="pin" width="700" title="pin"/>
    <img src="/doc/elevationMap.png" alt="elevationMap" width="700" title="elevationMap"/>
</div>

## Final Results

* A satellite falling into the non-sky areas (e.g., buildings, trees) will be rejected. 
* We can generate a grid map or inclination map of the environment in 360 degrees, which can be used in receivers or in post processing procedures.

<div align="center">
	<img src="/doc/res.png" alt="res" width="800" title="res"/>
</div>

### multiple satellites in the scene

<div align="center">
	<img src="/doc/finalres.png" alt="finalres" width="800" title="finalres"/>
</div>

--- 
 by: Alireza Ahmadi                                     
 University of Bonn- Robotics & Geodetic Engineering
 
 Alireza.Ahmadi@uni-bonn.de                             
 [www.AliezaAhmadi.xyz](https://www.AlirezaAhmadi.xyz)
