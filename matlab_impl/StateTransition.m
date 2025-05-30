% State Transition Class
classdef StateTransition
  properties (Access = private)
    accel_biasCoeff
    gyro_biasCoeff

    st_covar
    init_state
    cur_time
  end
  
  methods 
    function obj = StateTransition(initial_bias_acc, inital_bias_gyr, bias_coeff_acc, bias_coeff_gyr)
      % Here, we can assume SI units for everything
      obj.accel_biasCoeff = bias_coeff_acc;
      obj.gyro_biasCoeff = bias_coeff_gyr;

      % initial state
      pos = [0, 0, 0];
      vel = [0, 0, 0];
      orient = [1, 0, 0, 0];
      acc_bias = initial_bias_acc';
      gyr_bias = inital_bias_gyr';

      % Combine it all into the initial state, return this
      obj.init_state = [pos, vel, orient, acc_bias, gyr_bias]';
      obj.cur_time = 0;

      % State covar
      % Paper suggests large values along the diagonal! Probably should change this to match the scale of
      % the incoming data itself
      obj.st_covar = diag(ones(size(obj.init_state)) * 10);
    end

    function init_state = get_init_state(obj)
      init_state = obj.init_state;
    end

    function [next_state, st_covar] = calc_next_state(obj, cur_state, meas, dt)
      %% Incremetation of each step will be outlined below
      obj.cur_time = obj.cur_time + dt;

      % Biases first
      c_acc_bias = cur_state(INDS.ACCEL_BIAS);
      n_acc_bias = c_acc_bias + obj.bias_increm(obj.accel_biasCoeff, dt);

      c_gyr_bias = cur_state(INDS.GYRO_BIAS);
      n_gyr_bias = c_gyr_bias + obj.bias_increm(obj.gyro_biasCoeff, dt);

      % Increment Velocity
      c_vel = cur_state(INDS.VELOCITY);
      n_vel = c_vel + dt * ( meas(INDS.ACCEL)' - n_acc_bias);

      % Increment Position
      c_pos = cur_state(INDS.POSITION);
      n_pos = c_pos + dt * n_vel + (dt^2) * meas(INDS.ACCEL)' / 2;

      % Increment Orientation
      c_orient = cur_state(INDS.ORIENTATION);
      n_orient = QuaternionMath.oRotateQuaternionbyEul(c_orient, meas(INDS.GYRO), dt);
                  %obj.quat_rotate(c_orient, meas(4:6), dt)';

      %% Combine into next_state
      next_state = [n_pos; n_vel; n_orient; n_acc_bias; n_gyr_bias];

      %% State Covariance estimate
      obj.st_covar(INDS.ACCEL_BIAS, INDS.ACCEL_BIAS) = obj.st_covar(INDS.ACCEL_BIAS, INDS.ACCEL_BIAS) + dt * diag(obj.bias_increm(obj.accel_biasCoeff, dt));
      obj.st_covar(INDS.GYRO_BIAS, INDS.GYRO_BIAS) = obj.st_covar(INDS.GYRO_BIAS, INDS.GYRO_BIAS) + dt * diag(obj.bias_increm(obj.gyro_biasCoeff, dt));

      % Return it
      st_covar = obj.st_covar;
    end

    function bias_incr = bias_increm(obj, bias_coeff, dt)
      %% Keeping the bias incrementation method separate from the next_state
      %% so we can change this on the fly
      
      % bias_coeff is a stacked matrix, the [x; y; z] coefficients are stacked.
      s = size(bias_coeff);
      len = s(1);
      dt = zeros([1, len]) + dt;

      % Add in scaling for taylor series approximation
      dt = dt .^ (1:len) ./ factorial(1:len);
      bias_incr = bias_coeff * dt';
    end

  end

end
