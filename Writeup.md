# Steps of implementation #
Below I have explained how I tackled all the points, implemented the code and tuned the parameters for better performance.

Outline:

- [Step 1: Sensor Noise](#1.-Sensor-noise)
- [Step 2: Attitude Estimation](#2.-Attitude-Estimation)
- [Step 3: Prediction Step](#3.-Prediction-step)
- Step 4: Magnetometer Update
- Step 5: Closed Loop + GPS Update
- Step 6: Adding Your Controller

### 1. Sensor noise ###
1. Started running the simulator with senario 6_noisysensors. 
2. Recorded the GPS and Accelerometer data in graph1 and graph2 files respectively.
3. Calculated the mean and standard deviation using below mathematical formulas:
<pre>
<math>
mean = Sum of all points / No of points
Std_dev = pow(sum(pow(point -mean,2))/N,0.5)
</math>
</pre>
4. changed the values of `MeasuredStdDev_GPSPosXY` and `MeasuredStdDev_AccelXY` in `config/6_Sensornoise.txt`.
5. After running the simulator again, graphs turned green after few sec and results were passed.

### 2. Attitude Estimation ###
Used the quaternion class as per suggested and took help from section 7.2.1 Nonliear Complementary Filter of 
[Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj). Below is code snippet:  

<pre><code>
  Quaternion qt_bar = Quaternion::FromEuler123_RPY(rollEst,pitchEst,ekfState(6));
  qt_bar.IntegrateBodyRate(gyro,dtIMU);

  float predictedPitch = qt_bar.Pitch();
  float predictedRoll = qt_bar.Roll();
  ekfState(6) = qt_bar.Yaw();

</code>
</pre>

### 3. Prediction step ###
