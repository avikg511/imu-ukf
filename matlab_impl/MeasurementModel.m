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

    function pred_acc = predict_acceleration(obj, cur_state)
      % get_accel
      % This prediction does in fact include g because I'm using the raw accelerometer data 
      % from my phone rather than the g-subtracted one.
      cur_vel     = cur_state(4:6);
      cur_orient  = cur_state(7:10);
      cur_acc_b   = cur_state(11:13);
      a_true = (cur_vel - obj.last_vel) / obj.dt;

      pred_acc = obj.rotation_mat(cur_orient)' * a_true' +  cur_acc_b';
    end

    function pred_gyr = predict_gyro(obj, cur_state)
      % Get quaternion delta first
      cur_orient = cur_state(7:10);

      % The below formula should be confirmed, but given 2 complex numbers with phases phi1 phi2,
      % it does make sense that to find the difference, multiply one by the inverse of the other (polar form)
      delq = obj.quat_mult(cur_orient, obj.quat_inv(obj.last_quat));

      % Now, let's estimate the euler angles
      pred_gyr = obj.quatToEul(delq) / obj.dt;

    end

    function pred_magno = predict_magno(obj, cur_state)
      % Not sure how important the magnotometer is for this function
      % Will just be using a simple model and hardcoding it.
      mWorld = [-2.6367e1, -3.16e0, -4.86e1];     % Sample from some of the steady state data of the phone laying flat
      
      cur_orient = cur_state(7:10);
      pred_magno = obj.rotation_mat(cur_orient)' * mWorld';
    end

    %% Helper Functions

    %% Tested, copy pasted from other code. Should ideally have a quaternion class managing this later
    function product = quat_mult(~, qcur, qdel)
      % This is to multiply quaternions together, which is how we rotate by a quaternion
      % Implementation of quaternion multiplication is here: https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html

      % Not completely convinced on the direction of qcur and qdel in this multiplication. I've seen some conflicting things.
      % Hone in on what axes everything is working on later on but get the physics working first. If it switches what direction it's going
      % in, it's easier to tell than if it's going at the wrong magnitudes.
      t0 = qdel(1) * qcur(1) - qdel(2) * qcur(2) - qdel(3) * qcur(3) - qdel(4) * qcur(4);  %11-22-33-44
      t1 = qdel(1) * qcur(2) + qdel(2) * qcur(1) - qdel(3) * qcur(4) + qdel(4) * qcur(3);  %12+21-34+43
      t2 = qdel(1) * qcur(3) + qdel(2) * qcur(4) + qdel(3) * qcur(1) - qdel(4) * qcur(2);  %13+24+31-42
      t3 = qdel(1) * qcur(4) - qdel(2) * qcur(3) + qdel(3) * qcur(2) + qdel(4) * qcur(1);  %14-23+32+41

      % return quaternion
      product = [t0, t1, t2, t3]; 
      product = product / norm(product);
    end 

    function eul = quatToEul(obj, quat)
      % Extract values
      q0 = quat(1);
      q1 = quat(2);
      q2 = quat(3);
      q3 = quat(4);

      % Extract angles
      roll    = atan2(2 * (q0 * q1 + q2 * q3), q0^2 - q1^2 - q2^2 + q3^2);
      pitch   = asin(2 * (q0 * q2 - q1 * q3));
      yaw     = atan2(2 * (q0 * q3 + q1 * q2), q0^2 + q1^2 - q2^2 - q3^2);

      % return
      eul = [roll, pitch, yaw]';

    end

    %% UNTESTED
    function inv = quat_inv(~, quat)
      inv = [quat(1), -1 * quat(2:4)];
    end

    function rot_mat = rotation_mat(~, quat)
      quat = quat / norm(quat);

      % Rotation matrix from quat has 2 possible solutions, because the 
      %   mapping from Quaternions to the space of rotation matrices covers it twice.
      q0 = quat(1);
      q1 = quat(2);
      q2 = quat(3);
      q3 = quat(4);

      row1 = [q0^2 + q1^2 - q2^2 - q3^2, 2 * q1 * q2 - 2 * q0 * q3, 2 * q1 * q3 + 2 * q0 * q2];
      row2 = [2 * q1 * q2 + 2 * q0 * q3, q0^2 - q1^2 + q2^2 - q3^2, 2 * q2 * q3 - 2 * q0 * q1];
      row3 = [2 * q1 * q3 - 2 * q0 * q2, 2 * q2 * q3 + 2 * q0 * q1, q0^2 - q1^2 - q2^2 + q3^2];

      rot_mat = [row1; row2; row3];
    end

  end

end
