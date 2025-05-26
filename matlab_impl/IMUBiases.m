% Measure IMU Data Biases
classdef IMUBiases < handle
  properties (Access = private)
    steadyAccel
    steadyGyro
    steadyMagno

    imu_obj
  end

  properties (Constant)
    ACCEL_DATA = 1;
    GYRO_DATA = 2;
    MAGNO_DATA = 3;
  end

  methods 
    %{ 
      These are the steady state file inputs that will be used to calculate the bias of the filter.
    %}
    function obj = IMUBiases(accel, gyro, magno)
      obj.steadyAccel = accel;
      obj.steadyGyro = gyro;
      obj.steadyMagno = magno;

      % IMU Data Setup
      obj.imu_obj = IMUData(accel, gyro, magno);
    end

    function [dtAvg, xLoBFCoeff, yLoBFCoeff, zLoBFCoeff] = calculate_gyro_bias(obj)
      [dtAvg, xLoBFCoeff, yLoBFCoeff, zLoBFCoeff] = obj.calc_sensor_bias(obj.GYRO_DATA, "Gyroscope");
    end

    function [dtAvg, xLoBFCoeff, yLoBFCoeff, zLoBFCoeff] = calculate_accel_bias(obj)
      [dtAvg, xLoBFCoeff, yLoBFCoeff, zLoBFCoeff] = obj.calc_sensor_bias(obj.ACCEL_DATA, "Accelerometer");
    end

    function [dtAvg, xLoBFCoeff, yLoBFCoeff, zLoBFCoeff] = calculate_magno_bias(obj)
      [dtAvg, xLoBFCoeff, yLoBFCoeff, zLoBFCoeff] = obj.calc_sensor_bias(obj.MAGNO_DATA, "Magnotometer");
    end

    function next_data = get_next_data(obj, DATA_SRC_IND)

      switch DATA_SRC_IND
        case obj.ACCEL_DATA
         next_data = obj.imu_obj.get_next_accel();
        case obj.GYRO_DATA
          next_data = obj.imu_obj.get_next_gyro();
        case obj.MAGNO_DATA
          next_data = obj.imu_obj.get_next_magno();
        otherwise
          % disp("Bug!")
          next_data = NaN;
      end 

    end

    function [dtAvg, xLoBFCoeff, yLoBFCoeff, zLoBFCoeff] = calc_sensor_bias(obj, DATA_SRC_IND, sensorName)
      % Get Data length
      l = obj.imu_obj.get_length();
      winSize = 100;
      offset = 50;    % Data points to not plot from the beginning 
      
      % Will plot sensor's bias
      data_src = zeros([l, 4]);
      avged_data = zeros([l, 3]);

      for i=1:l
        data_src(i, :) = obj.get_next_data(DATA_SRC_IND);
        
        % Store averages
        avged_data(i, 1) = sum(data_src(max(1, i-winSize):i, 1)) / (i - max(1, i-winSize));
        avged_data(i, 2) = sum(data_src(max(1, i-winSize):i, 2)) / (i - max(1, i-winSize));
        avged_data(i, 3) = sum(data_src(max(1, i-winSize):i, 3)) / (i - max(1, i-winSize));
      end

      % Extract time and set up return value
      t = data_src(:, 4);
      dtAvg = (t(end) - t(1) ) / length(t);

      % Plotting the sensor drift
      figure;
      ax = gca;
      hold(ax, 'all');

      % Plots
      %
      % X Axis
      %
      % Fit sensor noise to Line of best fit with drift.
      tFit = t(offset:end-offset);
      xLoBFCoeff = polyfit(tFit, avged_data(offset:end-offset, 1), FIT_ORDER);
      xLoBFPlot = polyval(xLoBFCoeff, tFit);

      % % display values
      % disp("Bias for stationary " + sensorName + " data in X axis: ")
      % disp("The y-intercept is: " + xLoBFCoeff(2));
      % disp("The slope is: " + xLoBFCoeff(1));
      % disp(newline);

      % Visually plot line of best fit for data
      subplot(3, 1, 1)
      plot(t(offset:end), avged_data(offset:end, 1), 'b-', 'LineWidth', 2);
      hold on;
      
      % Plot data on top 
      plot(tFit(offset:end), xLoBFPlot(offset:end), 'r-', 'LineWidth', 1);
      
      title("X Axis " + sensorName + " Plot for Steady State")
      xlabel("Time (s)")
      hold off;

      %
      % Y axis
      %
      % Fit sensor noise to Line of best fit with drift.
      yLoBFCoeff = polyfit(tFit, avged_data(offset:end-offset, 2), FIT_ORDER);
      yLoBFPlot = polyval(yLoBFCoeff, tFit);

      % % display values
      % disp("Bias for stationary " + sensorName + " data in Y axis: ")
      % disp("The y-intercept is: " + yLoBFCoeff(2));
      % disp("The slope is: " + yLoBFCoeff(1));
      % disp(newline);

      % Visually plot line of best fit for data
      subplot(3, 1, 2); 
      plot(t(offset:end), avged_data(offset:end, 2), 'b-', 'LineWidth', 2);
      hold on;  

      % Plot data on top 
      plot(tFit(offset:end), yLoBFPlot(offset:end), 'r-', 'LineWidth', 1);
      title("Y Axis " + sensorName + " Plot for Steady State")
      xlabel("Time (s)") 

      hold off;

      %
      % Z Axis
      %
      % Fit sensor noise to Line of best fit with drift.
      zLoBFCoeff = polyfit(tFit, avged_data(offset:end-offset, 3), FIT_ORDER);
      zLoBFPlot = polyval(zLoBFCoeff, tFit);

      % % display values
      % disp("Bias for stationary " + sensorName + " data in Z axis: ")
      % disp("The y-intercept is: " + zLoBFCoeff(2));
      % disp("The slope is: " + zLoBFCoeff(1));
      % disp("Average value is: " + mean(avged_data(offset:end-offset, 3)))
      % disp(newline);

      % Visually plot line of best fit for data
      subplot(3, 1, 3);
      plot(t(offset:end), avged_data(offset:end, 3), 'b-', 'LineWidth', 2);
      hold on;

      % Plot data on top 
      plot(tFit(offset:end), zLoBFPlot(offset:end), 'r-', 'LineWidth', 1); 
      title("Z Axis " + sensorName + " Plot for Steady State")
      xlabel("Time (s)")

      if (DATA_SRC_IND == obj.ACCEL_DATA)
        ylim([9.88, 9.92]);
      end

      hold off

      % Test - Visually, line of best fit seems awful but the overall MSE is best!
      %     it was only bad for a few sensors.
      % errorLoBF = mean( ( avged_data(offset:end-offset, 3) - zLoBFPlot ) .^ 2 );
      % errorGuess = mean ( ( avged_data(offset:end-offset, 3) - 9.81 ) .^ 2);

      % % disp(newline)
      % % disp("MSE for Line of Best Fit is: " + errorLoBF);
      % % disp("MSE for Guess: " + errorGuess);  
   end
  end

end
