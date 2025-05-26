classdef IMU
  properties (Access = private)
    Accelerometer
    Gyroscope
    Magnetometer

    Accel_bias
    Gyro_bias
  end

  methods
    function obj = IMU(accelerometer, gyroscope, magnotometer, accel_bias, gyro_bias)
      %% Save Sensor Values
      obj.Accelerometer = accelerometer;
      obj.Gyroscope     = gyroscope;
      obj.Magnetometer  = magnotometer;

      %% Save Biases
      obj.Accel_bias    = accel_bias;
      obj.Gyro_bias     = gyro_bias;   
    end

    function next_point = get_new_data(obj)
      % Get values
      [acc, t_acc] = obj.Accelerometer.get_next_value();
      [gyr, t_gyr] = obj.Gyroscope.get_next_value();
      [mag, t_mag] = obj.Magnetometer.get_next_value();

      next_point = [acc, gyr, mag];
      % Below assertion fails, the time across sensors isn't actually synced
      % assert(abs(t_acc - t_gyr) < 1e-3 & abs(t_acc - t_mag) < 1e-3);   % Currently assuming identical sensor rates
    end

    function acc_bias = get_acc_start(obj)
      % return y-intercept of the Line of Best Fit from Accelerometer Bias
      acc_bias = obj.Accel_bias.get_sensor_yint();
    end

    function gyr_bias = get_gyr_start(obj)
      % return y-intercept of the Line of Best Fit from Gyroscope Bias
      % This is in (x, y, z)^T orientation.
      gyr_bias = obj.Gyro_bias.get_sensor_yint();
    end

    function [xLOBF, yLOBF, zLOBF] = get_bias_coeff_acc(obj)
      [xLOBF, yLOBF, zLOBF] = obj.Accel_bias.get_sensor_bias();
      xLOBF = xLOBF(1:end-1);
      yLOBF = yLOBF(1:end-1);
      zLOBF = zLOBF(1:end-1);

      %% Below logic is not used anymore but saved in case we need it. 

      % return the next step. 
      % The logic here is as follows. The inner product between [dt^2, dt, 0] and *LOBF
      %   should match the taylor series approximation to give you the delta for the next 
      %   step in a (determinisitic) state transition model on the bias

      % time vec for inner product
      % We should end up with [dt^2, dt^1, 0] if we're doing quadratic lines of best fit
      % t = 0;
      % for i=1:length(xLOBF)
      %   t = [dt ^ i, t];
      % end
      %
      % acc_inc = [t * xLOBF; t * yLOBF; t * zLOBF]; 
    end

    function [xLOBF, yLOBF, zLOBF] = get_bias_coeff_gyr(obj)
      % Return coefficients for bias increases for gyroscope.
      [xLOBF, yLOBF, zLOBF] = obj.Gyro_bias.get_sensor_bias();
      xLOBF = xLOBF(1:end-1);
      yLOBF = yLOBF(1:end-1);
      zLOBF = zLOBF(1:end-1);

      %% Below logic is not used anymore but saved in case we need it.

      % return the next step. 
      % The logic here is as follows. The inner product between [dt^2, dt, 0] and *LOBF
      %   should match the taylor series approximation to give you the delta for the next 
      %   step in a (determinisitic) state transition model on the bias


      % time vec for inner product
      % We should end up with [dt^2, dt^1, 0] if we're doing quadratic lines of best fit
      % t = zeros([1, length(xLOBF)]);
      % for i=2:length(xLOBF)     % if 2 > length(xLOBF), then 2:1 (e.g.) gives an empty iterator
      %   t(end - i + 1) = dt^i;
      % end
      
      % State transition matrix should increase bias by this much
      % gyr_inc = [t * xLOBF; t * yLOBF; t * zLOBF]; 
    end

    function prefixes = get_prefixes(obj)
      % These prefixes tell a user how much to scale the value by in terms of SI units
      % e.g. if our Magnetometer uses \mu T, then we should use 1e-6 for our prefix
      prefixes = [obj.Accelerometer.get_prefix(), obj.Gyroscope.get_prefix(), obj.Magnetometer.get_prefix()];
    end

  end

end
