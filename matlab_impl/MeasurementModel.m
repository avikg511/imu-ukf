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
      % This prediction does in fact include g because I'm using the raw accelerometer data 
      % from my phone rather than the g-subtracted one.
      cur_vel     = cur_state(INDS.VELOCITY);
      cur_orient  = cur_state(INDS.ORIENTATION);
      cur_acc_b   = cur_state(INDS.ACCEL_BIAS);
      a_true = (cur_vel' - obj.last_vel) / obj.dt;

      pred_acc = QuaternionMath.cQuatToRotationMat(cur_orient)' * a_true';
      pred_acc = pred_acc + cur_acc_b;
    end

    function pred_gyr = predict_gyro(obj, cur_state)
      % Get quaternion delta first
      cur_orient = cur_state(INDS.ORIENTATION);
      delq = QuaternionMath.oSubtractQuaternions(cur_orient, obj.last_quat); 

      % Now, let's estimate the euler angles
      pred_gyr = QuaternionMath.cQuatToEul(delq) / obj.dt;
    end

    function pred_magno = predict_magno(obj, cur_state)
      % Not sure how important the magnotometer is for this function
      % Will just be using a simple model and hardcoding it.
      mWorld = [-2.6367e1, -3.16e0, -4.86e1]';           
      
      cur_orient = cur_state(INDS.ORIENTATION);
      pred_magno = QuaternionMath.cQuatToRotationMat(cur_orient)' * mWorld; 
    end
  end

end
