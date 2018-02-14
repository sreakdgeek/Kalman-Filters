# Extended Kalman Filters

[//]: # (Image References) 
[image1]: ./images/KalmanFilter_Algorithm.png
[image2]: ./images/KalmanFilter_Equations.png
[image3]: ./images/Test1_Graph.png
[image4]: ./images/Test2_Graph.png
[image5]: ./images/Results.png

Extended Kalman filter is an extension to the non-linear version of Kalman filter, which linearizes an estimate of current mean and covariance (https://en.wikipedia.org/wiki/Extended_Kalman_filter).
Kalman filter uses prior knowledge of the system to make predictions of a state of the system and fuses with measurements (often not very reliable) of the state to accurately estimate the state of a
system. Below is the schematic of Extended Kalman Filters:

![alt text][image1]

---

### Kalman Filters Equations

Below are Kalman filter Equations:

![alt text][image2]


---

### Results

#### Graph for Dataset 1

![alt text][image3]

#### Graph for Dataset 2

![alt text][image4]

#### RMSE

![alt text][image4]
