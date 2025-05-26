% Unscented Kalman Filter
classdef Unscented
  properties (Access = private)
    imu IMU
    measurement_model MeasurementModel
    state_transition
    states

    % Seems unnecessary for now
    % ac
    % gy
    % mg
  end

  methods
    function obj = Unscented(accel_sensor, gyro_sensor, magno_sensor, accel_scale, gyro_scale, accel_bias, gyro_bias, dt, R_accel, R_gyro, R_magno)
      % Given sensors, lets make the IMU class
      obj.imu = IMU(accel_sensor, gyro_sensor, magno_sensor, accel_bias, gyro_bias);

      obj.measurement_model = MeasurementModel(R_accel, R_gyro, R_magno, [0, 0, -9.81]', dt);

      % Set up state transition matrix
      obj.state_transition = StateTransition(obj.imu.get_acc_bias(), obj.imu.get_gyr_bias(), obj.imu.get_bias_coeff_acc(), obj.imu.get_bias_coeff_gyr()); 

      % Save data
      len = obj.imu.get_data_length();       
      obj.states = zeros([len, 16])
      obj.states(1, :) = obj.state_transition.get_init_state();
      last_t = 0;       % Convert to dt later with IMU

      % Run the Unscented Kalman Filter steps
      % obj.run()

    end

  function run(obj)
      %% Run the UKF

      % 1. Generate sigma points

      % 2. Propagate them through the system dynamics, estimate the new noise/covariance

    
  end

  function plotter(obj)
      len = 2;
      for i=1:len
        % Increment states
        [t, meas] = obj.imu.get_new_data();
        obj.states(i+1, :) = obj.state_transition.calc_next_state(obj.states(i, :), meas, t - last_t); 
        last_t = t;

        obj.measurement_model.predict_acceleration(obj.states(i+1, :))
        obj.measurement_model.predict_gyro(obj.states(i+1, :))
        obj.measurement_model.predict_magno(obj.states(i+1, :))
      end

      trajectory = obj.states(:, 1:3); 
      %% Plot trajectory - Mostly GPT
      % Extract coordinates
      X = trajectory(:,1);
      Y = trajectory(:,2);

          Z = trajectory(:,3);

      N = length(X);

      % Set up figure
      fig = figure;
      axis equal;
      grid on;
      xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
      title('3D Trajectory Animation');
      hold on;

      % Plot the entire trajectory path as background (optional)
      plot3(X, Y, Z, 'k--', 'LineWidth', 0.5); % Dotted black trail for reference
      view(3);

      % Initialize animated point and trail
      h_point = plot3(X(1), Y(1), Z(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
      h_trail = plot3(X(1), Y(1), Z(1), 'b-', 'LineWidth', 2);

      % set up file name
      gif_filename = "../images/NoProcessUpdateDecimated.gif";

      % Animation loop
      for i = 1:N
        % Update current point
        set(h_point, 'XData', X(i), 'YData', Y(i), 'ZData', Z(i));
        
        % Update trail
        set(h_trail, 'XData', X(1:i), 'YData', Y(1:i), 'ZData', Z(1:i));
        
        drawnow;

        % Capture current frame
        frame = getframe(fig);
        img = frame2im(frame);
        [imind, cm] = rgb2ind(img, 256);

        % Write to GIF
        if i == 1
            imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.03);
        elseif (mod(i, 10) == 0)
            imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.03);
        end
      end

  end
end



end
