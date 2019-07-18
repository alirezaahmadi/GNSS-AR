# Vision-based approach for GPS Localization Improvement in Semi-Obstructed Areas

by Alireza Ahmadi - July 2018
***

## What is the Problem?
Performance of Global Navigation Satellite System (GNSS) positioning in urban environments is hindered by poor satellite availability because there are many man-made and natural objects in urban environments that obstruct satellite signals.

**NLOS (Non-Line of Sight)**
(one of the error sources in GNSS observation due to receiving signals from satellites which antenna is blind to them)

<div align="center">
	<img src="/doc/NLOS1.png" alt="nlos" width="300" title="nlos"/>
</div>

### Our solution
Our approach involves with using a landscape calibrated camera and through image processing algorithms tries to classify the scene to non-sky and free-sky regions, also tries to determine whether the satellites are located in free-sky or not. And finally generates an Elevation map of the environment

<div align="center">
	<img src="/doc/nlos2.png" alt="nlos" width="300" title="nlos"/>
</div>

### Advantages of Integration of vision:

* Low-cost with respect to laser scanner
* Able to  get  a dense data of surrounded  area at  minimum time  
* Real-time capability for processing data
* The results are more sensible and intuitive for human, in case of having any type of visualization

## Main Pipeline
<div align="center">
	<img src="/doc/pipe.png" alt="pipe" width="300" title="pipe"/>
</div>