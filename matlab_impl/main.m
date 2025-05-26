% UKF Setup and running it!

%% Set up the sensor classes
Accelerometer = Sensor("../data/Accelerometer.csv");
Gyroscope     = Sensor("../data/Gyroscope.csv");
Magnetometer  = Sensor("../data/Magnetometer.csv");

%% Set up the biases
Accel_bias    = SensorBias("../data/accel_steady_run.csv", 1, "Accelerometer", "../images/AccelBiasLineofBestFig.png");
Gyro_bias    = SensorBias("../data/gyro_steady_run.csv", 1, "Gyroscope", "../images/GyroBiasLineofBestFit.png");

%% Set up IMU class
imu = IMU(Accelerometer, Gyroscope, Magnetometer, Accel_bias, Gyro_bias);
imu.get_new_data()

%% Pass this into the Unscented Kalman Filter (UKF) Class

