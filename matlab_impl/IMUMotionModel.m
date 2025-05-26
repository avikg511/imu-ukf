% IMU motion Model Class
classdef IMUMotionModel < handle
  properties (Access = private)
    last_est
    last_time = 0;     % Current time set to 0. Could be an input for the NextStatePrediction

    accel_statematrix = struct('xBiasUpdate', 0, 'yBiasUpdate', 0, 'zBiasUpdate', 0, 'dtAvg', 0)
    gyro_statematrix = struct('xBiasUpdate', 0, 'yBiasUpdate', 0, 'zBiasUpdate', 0, 'dtAvg', 0)

    gyro_statemat_coeff
  end

  methods
    function obj = IMUMotionModel()
      obj.last_est = 0; 
    end

    %{ 
        For now, the input_vec will have the [a_x, a_y, a_z, \omega_x, \omega_y, \omega_z, ...
                                                magno_x, magno_y, magno_z]

          and the output state is [pos_x, pos_y, pos_z, v_x, v_y, v_z, ...
                                    quat_1, quat_2, quat_3, quat_4, ...
           aBias_x, aBias_y, aBias_z, gBias_x, gBias_y, gBias_z]

        Below is the calculation x(k+1) = NextStatePrediction(x(k), input_vec, time) + ProcNoise(time);
                  and from the Uhlmann-Julier paper, we're defining f.          

        In terms of a UKF, this will be ran on individual \sigma points.
    %}    

    function est = NextStatePrediction(obj, cur_state, input_vec, current_time)
      delta = zeros(size(cur_state));                   % The deltas are set to be 0 for now
      estCovar = zeros(size(cur_state * cur_state' ));  % This should give us a square matrix of N x N
      dt = current_time - obj.last_time;
      obj.last_time = current_time;

      % Motion Model - These are all hard coded, be careful here and double/triple check this!
      % Literally all the physics fails if this doesn't work

      % Separate out current state c<NAME> is current
      cPos = cur_state(1:3);
      cVel = cur_state(4:6);
      cQuat = cur_state(7:10);
      cAccel_bias = cur_state(11:13);
      cGyro_bias = cur_state(14:16);

      % Separate input data vector - i<NAME> is input
      iAccel = input_vec(1:3);
      iGyro = input_vec(4:6);
      iMagno = input_vec(7:9);

      % Model values - next<NAME> is the next value. First thought about storing deltas
      % but they will be calculated from `next<NAME> - c<NAME>`
      nextPos = zeros(size(pos));
      nextVel = zeros(size(vel));
      nextQuat = zeros(size(quat));

      nextAccel_bias = zeros(size(accel_bias));
      nextGyro_bias = zeros(size(gyro_bias));
      
      % Calculate next state
      adjusted_accel = iAccel - cAccel_bias;
      adjusted_gyro = iGyro - cGyro_bias;

      % Pose Estimation
      nextQuat = adjust_quat(cQuat, adjusted_gyro, dt);

      % State vector
      est = [nextPos, nextVel, nextQuat, nextAccel_bias, nextGyro_bias];
    end

    function nextQuat = adjust_quat(cQuat, iGyro, dt)
      % Return back adjusted gyro deltas
      % Following quaternion operation guide here:

      % Get delTheta vector
      delTheta = iGyro * dt;
      rotQuat = eulToQuat(delTheta);
      nextQuat = quat_mult(cQuat, rotQuat);
    end

    function quat = eulToQuat(eulVec) 
      % Euler Angles to Quaternion
      % This is used to convert gyroscope deltas to quaternions. We don't actually use the full angles
      % so pre-dividing everything by 2 for simpler notations. Again, the notation and implementation is from
      % https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html

      % The notation here is slightly off though compared to https://danceswithcode.net/engineeringnotes/quaternions/conversion_tool.html
      %   (same author). Instead of (Yaw, Pitch, Roll) being defined as rotation around x/y/z axes respectively, I'm doing
      %   a different convention which is Roll, Pitch, Yaw being defined about x/y/z axes. This is likely some sort of issue with
      %   NorthEastDown/alternative non-body frame representations.
      uH = eulVec(1) / 2; vH = eulVec(2) / 2; wH = eulVec(3) / 2;

      % Quaternion values
      q1 = cos(uH) * cos(vH) * cos(wH) + sin(uH) * sin(vH) * sin(wH);
      q2 = sin(uH) * cos(vH) * cos(wH) - cos(uH) * sin(vH) * sin(wH);
      q3 = cos(uH) * sin(vH) * cos(wH) + sin(uH) * cos(vH) * sin(wH);
      q4 = cos(uH) * cos(vH) * sin(wH) - sin(uH) * sin(vH) * cos(wH);

      % Return quat
      quat = [q1, q2, q3, q4]';
      quat = quat / norm(quat);
    end

    function product = quat_mult(qcur, qdel)
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
      product = [t0, t1, t2, t3]'; 
      product = product / norm(product);
   end 

    function state = get_initial_guess(obj)
      % The state prediction is going to be starting out at 0 for the displacement, but 
      % we can at least utilize the biases we're going to be working with from the data itself.
      pos = zeros([1, 3]);        % Position starts as zero because of displacement(xyz) = (0, 0, 0)^T
      vel = zeros([1, 3]);        % Velocity starts as zero because assuming stationary
      quat = [1, zeros([1, 3])];       % Quaternion shouldn't start at 4 zeros because it'd be invalid. Now
                                       % the object is at least aligned perfectly with the body frame.

      % Bias is now remaining, and something we're calculating from the data itself.
      % We're going to be using the IMUBiases class which ingests long running data and plots a line of best fit
      % of the object's IMU output while stationary. From this, we calculate overall bias.

      % Adds the magnotometer file even if we don't use it. Might change this organization later.
      % Actually, really should change this organization later on so we don't have to have hardcoded paths here :(
      biases = IMUBiases("./data/accel_steady_run.csv", "./data/gyro_steady_run.csv", "./data/magno_steady_run.csv");
      
      [dtAvgAcc, xAccLoBFCoeff, yAccLoBFCoeff, zAccLoBFCoeff] = biasTest.calculate_accel_bias()
      [dtAvgGyr, xGyrLoBFCoeff, yGyrLoBFCoeff, zGyrLoBFCoeff] = biasTest.calculate_gyro_bias()
      
      % Now, add the averages. The line of best fit ordering is set up as (a_n x^n + a_{n-1} x^{n-1} + \dots + a_0 x^0)
      % so the average starting bias itself starts with the last elemtn. Tehrefore, let's extract that
      accel_biases = [xAccLoBFCoeff(end), yAccLoBFCoeff(end), zAccLoBFCoeff(end)];
      gyros_biases = [xGyrLoBFCoeff(end), yGyrLoBFCoeff(end), zGyrLoBFCoeff(end)];

      % Then combining them all into the state
      state = [pos, vel, quat, accel_biases, gyros_biases]';      % State should be a collumn

      % From here, store the coefficients for when we need them later in the state transition matrix setup
      obj.accel_statematrix = struct('xBiasUpdate', xAccLoBFCoeff, 'yBiasUpdate', yAccLoBFCoeff, 'zBiasUpdate', zAccLoBFCoeff, 'dtAvg', dtAvgAcc);
      obj.gyro_statematrix = struct('xBiasUpdate', xGyrLoBFCoeff, 'yBiasUpdate', yGyrLoBFCoeff, 'zBiasUpdate', zGyrLoBFCoeff, 'dtAvg', dtAvgGyr);
    end
  end
end
