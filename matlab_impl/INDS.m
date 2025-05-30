classdef INDS
  properties (Constant)
    % Indices for States
    POSITION    = 1:3;   
    VELOCITY    = 4:6;
    ORIENTATION = 7:10;
    ACCEL_BIAS  = 11:13;
    GYRO_BIAS   = 14:16;

    % Indices for Measurements
    ACCEL = 1:3;
    GYRO = 4:6;
    MAGNO = 7:9;
  end
end
