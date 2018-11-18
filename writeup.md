### Step 1: Sensor Noise ###

I used python and numpy to calculate standard deviations ([scenario6.py](scenario6.py)):

```python
import numpy as np

gps = np.loadtxt(open("config/log/Graph1.txt", "rb"), delimiter=",", skiprows=1, dtype='Float64')
print("GPS std: ", np.std(gps[:,1]))

acc = np.loadtxt(open("config/log/Graph2.txt", "rb"), delimiter=",", skiprows=1, dtype='Float64')
print("ACC std: ", np.std(acc[:,1]))
```

Result:
```
GPS std:  0.704375122559
ACC std:  0.509920269748
```

![scenario 6](./images/Scenario6.png)

### Step 2: Attitude estimation ###

I used `Quaternion<float>` class `FromEuler123_RPY()` for creating quaternion from Euler Roll/Pitch/Yaw
and then used `IntegrateBodyRate()` to get updated attitude.

```c++
Quaternion<float> attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
Quaternion<float> attitude_update = attitude.IntegrateBodyRate(gyro, dtIMU);

float predictedPitch = attitude_update.Pitch();
float predictedRoll = attitude_update.Roll();
ekfState(6) = attitude_update.Yaw();
```

Result:

![scenario 7](./images/Scenario7.png)

### Step 3: Prediction step

#### Predict mean ####

#### Predict covariance ####


### Step4: Magnetometer update ###

First I updated `QYawStd` parameter to value `0.07`, so it would approximately captures the magnitude of the drift.

I then implemented magnetometer update step using formulas from `Estimation for Quadrotors` chapter `7.3.2 Magnetometer`:

![magnetometer formulas measurement](./images/magnetometer-formulas1.png)

![magnetometer update](./images/magnetometer-formulas2.png)

I also normalized the difference between measured and estimated yaw. 

This was the result I got:

![magnetometer update](./images/magnetometer.png)


### Step 6: Adding you controller

Replaced `QuadController.cpp` and `QuadControlParams.txt` and run scenario `11_GPSUpdate`. Surprisingly quad flied within 
allowed error limits without needed any de-tuning. Although the flight path was a little bit chaotic:

Result:

![custom controller](./images/custom-controller.png)

```
Simulation #4 (../config/11_GPSUpdate.txt)
PASS: ABS(Quad.Est.E.Pos) was less than 1.000000 for at least 20.000000 seconds
```