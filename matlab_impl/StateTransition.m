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
      %{ 
           Cur State Breakdown:
            cur_state(1:3)    == pos
            cur_state(4:6)    == vel
            cur_state(7:10)   == orient
            cur_state(11:13)  == acc_bias
            cur_state(14:16)  == gyr_bias

           Measurement breakdown:
            meas(1:3)         == accel
            meas(4:6)         == gyro
            meas(7:9)         == magno
      %}

      %% Incremetation of each step will be outlined below
      obj.cur_time = obj.cur_time + dt;

      % Biases first
      c_acc_bias = cur_state(11:13);
      n_acc_bias = c_acc_bias + obj.bias_increm(obj.accel_biasCoeff, dt);

      c_gyr_bias = cur_state(14:16);
      n_gyr_bias = c_gyr_bias + obj.bias_increm(obj.gyro_biasCoeff, dt);

      % Increment Velocity
      c_vel = cur_state(4:6);
      n_vel = c_vel + dt * ( meas(1:3)' - n_acc_bias);

      % Increment Position
      c_pos = cur_state(1:3);
      n_pos = c_pos + dt * n_vel + (dt^2) * meas(1:3)' / 2;

      % Increment Orientation
      c_orient = cur_state(7:10);
      n_orient = QuaternionMath.oRotateQuaternionbyEul(c_orient, meas(4:6), dt)'; 
                  %obj.quat_rotate(c_orient, meas(4:6), dt)';

      %% Combine into next_state
      next_state = [n_pos; n_vel; n_orient; n_acc_bias; n_gyr_bias];

      %% State Covariance estimate
      obj.st_covar(11:13, 11:13) = obj.st_covar(11:13, 11:13) + dt * diag(obj.bias_increm(obj.accel_biasCoeff, dt));
      obj.st_covar(14:16, 14:16) = obj.st_covar(14:16, 14:16) + dt * diag(obj.bias_increm(obj.gyro_biasCoeff, dt));

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
      % assert(1 == 0);
    end

  end

end
