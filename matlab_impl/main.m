% UKF Setup and running it!
format longG

%% Set up the sensor classes
Accelerometer = Sensor("../data/Accelerometer.csv", 1);
Gyroscope     = Sensor("../data/Gyroscope.csv", 1);
Magnetometer  = Sensor("../data/Magnetometer.csv", 1e-6);

%% Set up the biases
Accel_bias    = SensorBias("../data/accel_steady_run.csv", 0, "Accelerometer", "../images/AccelBiasLineofBestFig.png", 1);
Gyro_bias     = SensorBias("../data/gyro_steady_run.csv", 0, "Gyroscope", "../images/GyroBiasLineofBestFit.png", 1);

%% Set up UKF Class
dt = 0.01;

%% Hard coded values from iPhone 14 Page on phyphox
% Set up measurement covariances, using values from collected values listed at phyphox
% Values found from: https://phyphox.org/sensordb/ which is self reported data
Racc = diag([0.016, 0.016, 0.016]) .^2 ;
Rgyr  = diag([0.0039, 0.0039, 0.0039]) .^ 2;
Rmag = diag([0.24, 0.24, 0.24]) .^ 2;
 
ukf = Unscented(Accelerometer, Gyroscope, Magnetometer, Accel_bias, Gyro_bias, dt, Racc, Rgyr, Rmag);
