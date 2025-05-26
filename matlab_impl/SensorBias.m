classdef SensorBias < handle
  properties (Access = private)
    sensor Sensor

    bias_x
    bias_y
    bias_z
    
    ind       % Simple index to see if we've gone through a function before
  end

  methods
    function obj = SensorBias(data_file, WILL_PLOT, sensor_string, LOBF_filename)
      % Parameters, can change later. Used in calculate_bias
      WINSIZE = 100;
      OFFSET  = 500;
      FIT_ORDER = 2;

      %% Treat the file as a sensor class at first
      obj.sensor = Sensor(data_file);

      %{ 
        Calculate the bias. The bias here is represented as a line of best fit,
          the y-intercept will be the initial bias, and then we'll model sensor
          drift using the other coefficients. 

          This is going to be part of the state transition model as well, though 
          the y-intercept will be used as a method to ensure proper initialization.

        Will be split up into each axis as well 
      %} 

      x_data = obj.sensor.get_x_data();
      y_data = obj.sensor.get_y_data();
      z_data = obj.sensor.get_z_data();

      % Calculate each bias
      [averaged_x, obj.bias_x] = obj.calculate_bias(x_data, WINSIZE, OFFSET, FIT_ORDER);                                      
      [averaged_y, obj.bias_y] = obj.calculate_bias(y_data, WINSIZE, OFFSET, FIT_ORDER);
      [averaged_z, obj.bias_z] = obj.calculate_bias(z_data, WINSIZE, OFFSET, FIT_ORDER);
      
      % Plot as a form of sanity check
      if (WILL_PLOT) 
        % get time data
        t_data = obj.sensor.get_t_data();
        t_data = t_data(OFFSET:end - OFFSET);

        % Set up LOBF plot for all axes
        xLOBF = polyval(obj.bias_x, t_data);
        yLOBF = polyval(obj.bias_y, t_data);
        zLOBF = polyval(obj.bias_z, t_data);

        % Set up the plots
        fig = figure;
        ax = gca;
        hold(ax, 'all');

        subplot(3, 1, 1)        
        plot(t_data, averaged_x, 'b-', 'LineWidth', 2);
        hold on

        plot(t_data, xLOBF, 'r-', 'LineWidth', 1);
        title("X Axis " + sensor_string + " Plot for Steady State, with Line of Best Fit")

        subplot(3, 1, 2)
        plot(t_data, averaged_y, 'b-', 'LineWidth', 2);
        hold on

        plot(t_data, yLOBF, 'r-', 'LineWidth', 1);
        title("Y Axis " + sensor_string + " Plot for Steady State, with Line of Best Fit")

        subplot(3, 1, 3)        
        plot(t_data, averaged_z, 'b-', 'LineWidth', 2);
        hold on

        plot(t_data, zLOBF, 'r-', 'LineWidth', 1);
        title("Z Axis " + sensor_string + " Plot for Steady State, with Line of Best Fit")

        % Save fig
        saveas(fig, LOBF_filename, "png");
      end

      % Set up ind == 0 to make sure we check that get_sensor_bias() is using column vectors
      obj.ind = 0;
    end

    function [averaged_data, bias] = calculate_bias(obj, data, WINSIZE, OFFSET, FIT_ORDER)
      % Set up parameters on data
      len = obj.sensor.get_data_length(); 
      dt = obj.sensor.get_dt();
      time_data = obj.sensor.get_t_data();
    
      % Set up averages
      averaged_data = zeros([len, 1]);
      
      for ind=1:len
        averaged_data(ind) = sum(data(max(1, ind - WINSIZE):ind)) / (ind - max(1, ind - WINSIZE));
      end
      
      % Calculate line of best fit
      tFit = time_data(OFFSET:end - OFFSET);
      averaged_data = averaged_data(OFFSET:end - OFFSET);
      bias = polyfit(tFit, averaged_data, FIT_ORDER); % Array of coefficients w/ LOBF params      
    end

    function acc_bias = get_sensor_yint(obj)
      % return start point
      acc_bias = [obj.bias_x(end), obj.bias_y(end), obj.bias_z(end)]';
    end

    function [x_bias, y_bias, z_bias] = get_sensor_bias(obj)
      % return bias
      [x_bias, y_bias, z_bias] = [obj.x_bias, obj.y_bias, obj.z_bias];  

      % Asserts to make sure all the dimensions are right
      if (obj.ind == 0)
        s = size(x_bias);
        assert(s == size(y_bias) && s == size(z_bias))
        assert(s(1) >= s(2));   % This makes sure we're dealing with a column vector

        % Signify that we've passed through this loop
        obj.ind = 1;
      end
    end

  end
end
