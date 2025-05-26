% UKF Setup and running it!
format longG

%% Set up the sensor classes
Accelerometer = Sensor("../data/Accelerometer.csv", 1);
Gyroscope     = Sensor("../data/Gyroscope.csv", 1);
Magnetometer  = Sensor("../data/Magnetometer.csv", 1e-6);

%% Set up the biases
Accel_bias    = SensorBias("../data/accel_steady_run.csv", 1, "Accelerometer", "../images/AccelBiasLineofBestFig.png", 1);
Gyro_bias    = SensorBias("../data/gyro_steady_run.csv", 1, "Gyroscope", "../images/GyroBiasLineofBestFit.png", 1);

%% Set up IMU class
imu = IMU(Accelerometer, Gyroscope, Magnetometer, Accel_bias, Gyro_bias);
imu.get_new_data()
imu.get_prefixes()

%% Pass this into the Unscented Kalman Filter (UKF) Class

