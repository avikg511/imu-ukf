% Measurement Model
classdef MeasurementModel
  properties (Access = private)
    last_vel
    last_pos
    last_quat

    R_accel
    R_gyro
    R_magno

    steady_g
    dt
  end 

  methods 
    function obj = MeasurementModel(R_accel, R_gyro, R_magno, g_bias, dt)
      obj.R_accel = R_accel;
      obj.R_gyro = R_gyro;
      obj.R_magno = R_magno;

      obj.steady_g = g_bias;

      % Initialize previous values
      obj.last_vel = [0, 0, 0];
      obj.last_pos = [0, 0, 0];
      obj.last_quat = [1, 0, 0, 0];
      obj.dt = dt;
    end

    function R = get_measurement_covariance(obj)
      I = eye(size(obj.R_accel));     % Assuming same sizes for everything.
      R = [obj.R_accel, I, I; I , obj.R_gyro, I; I, I, obj.R_magno];
    end

    function pred_acc = predict_acceleration(obj, cur_state)
      % get_accel
      % This prediction does in fact include g because I'm using the raw accelerometer data 
      % from my phone rather than the g-subtracted one.
      cur_vel     = cur_state(4:6);
      cur_orient  = cur_state(7:10);
      cur_acc_b   = cur_state(11:13);
      a_true = (cur_vel' - obj.last_vel) / obj.dt;

      pred_acc = QuaternionMath.cQuatToRotationMat(cur_orient)' * a_true';
                % obj.rotation_mat(cur_orient)' * a_true';
      pred_acc = pred_acc + cur_acc_b;
    end

    function pred_gyr = predict_gyro(obj, cur_state)
      % Get quaternion delta first
      cur_orient = cur_state(7:10);

      % The below formula should be confirmed, but given 2 complex numbers with phases phi1 phi2,
      % it does make sense that to find the difference, multiply one by the inverse of the other (polar form)
      delq = QuaternionMath.oSubtractQuaternions(cur_orient, obj.last_quat); 
            %obj.quat_mult(cur_orient, obj.quat_inv(obj.last_quat));

      % Now, let's estimate the euler angles
      pred_gyr = QuaternionMath.cQuatToEul(delq) / obj.dt 
                % obj.quatToEul(delq) / obj.dt;

    end

    function pred_magno = predict_magno(obj, cur_state)
      % Not sure how important the magnotometer is for this function
      % Will just be using a simple model and hardcoding it.
      mWorld = [-2.6367e1, -3.16e0, -4.86e1];     % Sample from some of the steady state data of the phone laying flat
      
      cur_orient = cur_state(7:10);
      pred_magno = QuaternionMath.cQuatToRotationMat(cur_orient)' * mWorld; 
                  % obj.rotation_mat(cur_orient)' * mWorld';
    end
  end

end
