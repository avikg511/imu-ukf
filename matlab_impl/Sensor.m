% @brief Class that handles Sensor I/O simulation, allowing us to ingest 1 data point, manage
%           the sampling period, etc. etc.
classdef Sensor < handle
  properties (Access = private)
    file_path
    time_values
    sample_period
    x_data
    y_data
    z_data  

    % Values constantly incremented
    ind
  end

  methods (Access = public)
    function obj = Sensor(file_path)
      obj.file_path = file_path;
      obj.ind = 1;

      % Access data
      data = readtable(file_path, VariableNamingRule='preserve');

      % Here, we presume knowledge of the data file. Using the format (T, X, Y, Z)
      data_arr = cell(1, 4);
      [data_arr{1:4}] = obj.data_separator(data);
      [obj.time_values, obj.x_data, obj.y_data, obj.z_data] = deal(data_arr{:}); 
      
      % Other parameters
      obj.sample_period = mean(obj.time_values(2:end) - obj.time_values(1:end - 1));
    end

    function [data_pt, cur_time] = get_next_value(obj)
      % Increment the index and move on to the next value
      % x = randn(1, 34);
      data_pt = [obj.x_data(obj.ind), obj.y_data(obj.ind), obj.z_data(obj.ind)];
      cur_time = obj.time_values(obj.ind);

      % Increment index 
      obj.ind = obj.ind + 1;
    end

    % Separator for the data
    function [tdata, xdata, ydata, zdata] = data_separator(~, data_table)
      % make array
      data_array = table2array(data_table);

      % Separate data into xyzt
      t_separator = [1, 0, 0, 0];
      x_separator = [0, 1, 0, 0];
      y_separator = [0, 0, 1, 0];
      z_separator = [0, 0, 0, 1];

      % get data
      tdata = data_array(:, :) * t_separator';
      xdata = data_array(:, :) * x_separator';
      ydata = data_array(:, :) * y_separator';
      zdata = data_array(:, :) * z_separator';
    end

    % Set up data retreival
    function xdata = get_x_data(obj)
      % Get X data
      xdata = obj.x_data; 
    end
    
    function ydata = get_y_data(obj)
      % Get Y data
      ydata = obj.y_data; 
    end
    
    function zdata = get_z_data(obj)
      % Get Z data
      zdata = obj.z_data; 
    end

    function tdata = get_t_data(obj)
      % Get time data
      tdata = obj.time_values;
    end

    function dt = get_dt(obj)
      % Get dt
      dt = obj.sample_period;
    end

    function len = get_data_length(obj)
      % Get length of processing for this sensor, used to set up say a for loop
      len = min([length(obj.x_data), length(obj.y_data), length(obj.z_data), length(obj.time_values)]);
    end
  end
end
