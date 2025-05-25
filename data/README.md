## Data Folder
Format we're expecting is  
- Accelerometer.csv, Gyroscope.csv, Magnoteometer.csv &
- bias_accel.csv, bias_gyro.csv, and bias_magno.csv

Line 1 refers to the actual data we're estimating orientation from and line 2 refers to the data we're using to estimate the steady state sensor drift of the individual sensor. The `Sensor` class takes in line 1 and the `IMUBias` class takes in line 2.
