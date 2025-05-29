classdef QuaternionMath 
  %{ 
    The naming might be a little confusing but there is a difference between o<> and c<>
     
    If the method is prefixed by a c<> (e.g. cEulerAngleToQuat), it is ONLY changing the 
     representation rather than the underlying data itself. Converting Euler Angles to 
     Quaternions will inevitably change some of the underlying data, but the goal is to
     have it represent the same information

      in other words, c<> will 'convert' the data.

    If the method is prefixed by a o<> (e.g. oRotateQuaternionbyEul), it IS changing the 
      underlying data or performing an 'operation' on the data. It is rotating, inverting,
      subtracting, etc. the quaternions provided. Usually is inputed at least 1 quaternion
      and additional information

      in other words, c<> will 'operate' on the data.

  %}
  methods(Static)
    function rotatedQuat = oRotateQuaternionbyEul(curOrientQ, gyro_meas, dt)
        % Quaternion check
        curOrientQ = QuaternionMath.cQuatCheck(curOrientQ);

        % Convert Euler to Quaternion
        dQuat = QuaternionMath.cEulerAngleToQuat(gyro_meas * dt);
        rotatedQuat = QuaternionMath.oQuatMult(curOrientQ, dQuat);
    end

    function quatDiff = oSubtractQuaternions(quat1, quat2)
      % This is 'subtraction' of quaternions and very poorly defined. Really, the goal
      % is to find some sort of delta between the two. This will be replaced in the future
      % with the multiplicative filter though.
      q2Inv = QuaternionMath.oQuatInvert(quat2);
      quatDiff = QuaternionMath.oQuatMult(quat1, q2Inv);
    end

    function quat = cEulerAngleToQuat(dOmega)
        % Convert Euler Angle dOmega to Quaternion
        % The logic for this is largely from https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html

        % The notation is slightly different than the notes above and the conversion tool
        % I verified with here (https://danceswithcode.net/engineeringnotes/quaternions/conversion_tool.html). 
        % Instead of (Yaw, Pitch, Roll) being defined around the xyz axes, I'm doing Roll, Pitch, then Yaw. I've seen both around though.
        
        % Halving the angles to make the formulas simpler.
        uH = dOmega(1) / 2; vH = dOmega(2) / 2; wH = dOmega(3) / 2;

        % Quaternion values
        q1 = cos(uH) * cos(vH) * cos(wH) + sin(uH) * sin(vH) * sin(wH);
        q2 = sin(uH) * cos(vH) * cos(wH) - cos(uH) * sin(vH) * sin(wH);
        q3 = cos(uH) * sin(vH) * cos(wH) + sin(uH) * cos(vH) * sin(wH);
        q4 = cos(uH) * cos(vH) * sin(wH) - sin(uH) * sin(vH) * cos(wH);

        % Return quat, normalized
        quat = [q1, q2, q3, q4];
        quat = quat / norm(quat);
    end

    function eul = cQuatToEul(quat1)
      % Check if quaternion is valid
      quat1 = QuaternionMath.cQuatCheck(quat1);

      % Apply angle formula to calculate one of the Euler Angle Representations
      % In the future, confirm which one this is using and how to justify which one is the 
      %   expected one

      % Extract values
      q0 = quat1(1); q1 = quat1(2); q2 = quat1(3); q3 = quat1(4);

      % Extract angles
      roll    = atan2(2 * (q0 * q1 + q2 * q3), q0^2 - q1^2 - q2^2 + q3^2);
      pitch   = asin(2 * (q0 * q2 - q1 * q3));
      yaw     = atan2(2 * (q0 * q3 + q1 * q2), q0^2 + q1^2 - q2^2 - q3^2);

      % return
      eul = [roll, pitch, yaw]';
    end

    function rotationMat = cQuatToRotationMat(quat1)
      % First normalize and confirm it's valid
      quat1 = QuaternionMath.cQuatCheck(quat1);

      % Now, extract values and return math. Much of this is again from danceswithcode (linked in other functions)

      % This has two possible solutions with how the SO(3) space works, 
      % I'm not sure exactly which one to pick but here is one of them. Confirm in the future
      q0 = quat1(1); q1 = quat1(2); q2 = quat1(3); q3 = quat1(4);

      row1 = [q0^2 + q1^2 - q2^2 - q3^2, 2 * q1 * q2 - 2 * q0 * q3, 2 * q1 * q3 + 2 * q0 * q2];
      row2 = [2 * q1 * q2 + 2 * q0 * q3, q0^2 - q1^2 + q2^2 - q3^2, 2 * q2 * q3 - 2 * q0 * q1];
      row3 = [2 * q1 * q3 - 2 * q0 * q2, 2 * q2 * q3 + 2 * q0 * q1, q0^2 - q1^2 - q2^2 + q3^2];

      rotationMat = [row1; row2; row3];
    end
    
    function product = oQuatMult(quat1, quat2)
      % Multiply Quaternions. Again a lot of the math/implementation details are from
      %  https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html

      % Multiplication here is not commutative, so order does matter. Please confirm the order
      %   is the proper one while using this function.

      % Set up the commonly used variables to simplify the formula below
      q11 = quat1(1); q12 = quat1(2); q13 = quat1(3); q14 = quat1(4);
      q21 = quat2(1); q22 = quat2(2); q23 = quat2(3); q24 = quat2(4);
      product = zeros(size(quat1));

      % Multiply
      product(1) = q21 * q11 - q22 * q12 - q23 * q13 - q24 * q14;  %11-22-33-44
      product(2) = q21 * q12 + q22 * q11 - q23 * q14 + q24 * q13;  %12+21-34+43
      product(3) = q21 * q13 + q22 * q14 + q23 * q11 - q24 * q12;  %13+24+31-42
      product(4) = q21 * q14 - q22 * q13 + q23 * q12 + q24 * q11;  %14-23+32+41

      % Return quaternion, normalized
      product = product / norm(product);
    end

    function inverse = oQuatInvert(quat1)
      % Calculate inverse
      inverse = zeros(size(quat1));
      inverse(1) = quat1(1);
            
      % Invert without changing shape (which is why I didn't do this all on one line)
      inverse(2) = -1 * quat1(2);
      inverse(3) = -1 * quat1(3);
      inverse(4) = -1 * quat1(4);
    end

    function quat1 = cQuatCheck(quat1)
      % Confirm the quaternion is normalized enough
      if abs(norm(quat1) - 1) > 1e-3
        warning('Quaternion is not normalized: norm: %.6f', norm(quat1));
        quat1 = quat1 / norm(quat); 
      end

      % Confirm there are no NaN values
      if (sum(isnan(quat1))) ~= 0 || ((sum(isinf(quat1)))) ~= 0
        error('Quaternion does not have proper values. Please confirm it is valid: ' + quat1)
      end

    end
  end
end
