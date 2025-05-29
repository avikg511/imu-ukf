classdef Unscented < handle
  properties (Access = public)
    imu IMU
    measurement_model MeasurementModel
    state_transition

    states
    Pst   % State covariance matrix
    Q     % Process covariance matrix
    dt

    index

    % Seems unnecessary for now
    % ac
    % gy
    % mg
  end

  methods
    % Units are oopsied - thank god they're 1 so far for this set
    % actually, I shouldn't need scale anymore right? bias and everything encorporates: accel_scale, gyro_scale, 
    function obj = Unscented(accel_sensor, gyro_sensor, magno_sensor, accel_bias, gyro_bias, dt, R_accel, R_gyro, R_magno)
      % Given sensors, lets make the IMU class
      obj.imu = IMU(accel_sensor, gyro_sensor, magno_sensor, accel_bias, gyro_bias);
      obj.measurement_model = MeasurementModel(R_accel, R_gyro, R_magno, [0, 0, -9.81]', dt);
      obj.dt = dt;

      % Set up state transition matrix
      obj.state_transition = StateTransition(obj.imu.get_acc_bias(), obj.imu.get_gyr_bias(), obj.imu.get_bias_coeff_acc(), obj.imu.get_bias_coeff_gyr()); 
      obj.index = 0;
 
      % Run the Unscented Kalman Filter steps
      obj.run();

      % Plot it!
      obj.plotter("../images/UKFImplementationPath1.gif");
    end

    function run(obj)
      %% Run the UKF

      % initialize
      [len, N] = obj.initialize();
      alpha = 1e-3; beta = 2; kappa = 0;

      % Some book keeping
      % N = size of state
      % Nsig = number of sigma points
      Nsig = 2 * N + 1;
      
      for i=2:len
        % disp("Made it to " + i + " iterations!")
        % 0. Get measurements
        [~, meas] = obj.imu.get_new_data();

        % 1. Generate sigma points
        mu = obj.states(i - 1, :);      % Initial point
        [sigmas, weightsmu, weightscov, Nsig] = obj.generate_sigma_points(mu, alpha, kappa, beta, N, obj.Pst);

        % 2. Propagate them through the system dynamics, estimate the new noise/covariance
        transformed = zeros(size(sigmas));
        for j=1:Nsig
          % Get new sigmas from state transition model 
          transformed(:, j) = obj.state_transition.calc_next_state(sigmas(:, j), meas, obj.dt);
        end

        % 3. Predicted mean and covariances (wow quaternions kind of suck)
        muPrime = obj.calculate_new_mean(transformed, weightsmu, N);
        [st_dels, obj.Pst] = obj.calculate_new_covar(muPrime, transformed, weightscov, obj.Q, Nsig);

        % 4. Sigma points through measurement model
        predictions = obj.predict_measurements(transformed, Nsig);

        % 5. Update Measurement model mean and covariance
        meas_mu = obj.measure_mean(predictions, weightsmu);
        meas_P = obj.calculate_measurement_covar(predictions, meas_mu, weightscov, Nsig);

        % 6. Cross covariance for measurement and state
        % using st_dels instead of muPrime because I don't want to do the quaternion subtraction system again
        cross_covar = obj.calculate_meas_state_covar(transformed, predictions, weightscov, Nsig, st_dels, meas_mu);        

        % 7. Next Step with Kalman
        obj.kalman_step(cross_covar, meas_P, meas, meas_mu, muPrime, i);

        % 7.1 debugger step
        obj.index = obj.index + 1;
      end
    end

    function [len, order] = initialize(obj)
      % Save data
      len = obj.imu.get_data_length();
      obj.states = zeros([len, 16]);
      
      init_state = obj.state_transition.get_init_state();
      obj.states(1, :) = init_state;      
      obj.Pst = 0.1 * eye(length(init_state));
      obj.Q = 1e-5 * eye(length(init_state));

      % Return order as well, so size of state vector
      order = length(init_state);
    end

    function [sigmas, weightsmu, weightscov, Nsig] = generate_sigma_points(obj, mu, alpha, beta, kappa, N, Pst)
      % Get lambda param
      lambda = alpha^2 * (N + kappa) - N;
      Nsig = 2 * N + 1;

      % Square root factor for the 1:N poitns using \mu_x + (\sqrt{L + \lamda ) * P_x})
      % and using cholesky for square root
      sqrt_factor = N + lambda;

      % if any(~isfinite(Pst), 'all')
      %   disp("Pst contains non-finite values!");
      %   Pst = max(min(Pst, 1e4), 1e-8); 
      % end

      % matSqrt = sqrtm(sqrt_factor * Pst);

      try
          matSqrt = chol(sqrt_factor * Pst, 'lower');
      catch
          warning('Cholesky failed. Adding jitter...');
          Pst = max(min(Pst, 1e4), 1e-8)

          jitter = 1e-6 * eye(size(Pst));
          matSqrt = chol(sqrt_factor * (Pst + jitter), 'lower');
      end

      % mean matrix to add to
      pos_ind_sigmas = repmat(mu', 1, N) + matSqrt;
      neg_ind_sigmas = repmat(mu', 1, N) - matSqrt;
      zero_ind_sigma = mu';

      % Sigma Matrix
      sigmas = [neg_ind_sigmas, zero_ind_sigma, pos_ind_sigmas]; 

      %{ 
        We're also going to set up the weight vector here because it multiplies with
          the sigma matrix above.
        
        From the Uhlmann-Julier Paper, we have W_o = \frac{\kappa}{N + \kappa} and 
          the rest at \frac{1}{2 * (N + \kappa)}
      %}
      zero_weight   = kappa / (1 * (N + kappa));
      rest_weights  =     1 / (2 * (N + kappa));

      weightsmu = [(zeros([1, N]) + rest_weights), zero_weight, (zeros([1, N]) + rest_weights)]';

      % the above weights are for mean. For the covariance weights, adjust center point
      cov_zero_weight = zero_weight + (1 - alpha^2 + beta);
      weightscov = [(zeros([1, N]) + rest_weights), cov_zero_weight, (zeros([1, N]) + rest_weights)]';

      % Just outputtings tate at this point
      % if (mod(obj.index, 10) == 0)
        disp("Currently on iteration: " + obj.index);
      % end
    end 

    function [mu] = calculate_new_mean(obj, transformed_pts, weights, N, Q)
        % Get new mean
        % Will be split up for position, velocity, biases vs quaternions
        pos_chunk   = transformed_pts(1:3, :);
        vel_chunk   = transformed_pts(4:6, :);
        quat_chunk  = transformed_pts(7:10, :);
        biases      = transformed_pts(11:16, :);  % Treatment of biases is the same
        
        % Weights should be a column vector. Assert if that's not true
        s = size(weights);
        assert(s(1) > s(2));

        % Now calculate regular means
        pos_mean = pos_chunk * weights;
        vel_mean = vel_chunk * weights;
        bias_mean = biases * weights;

        % Quaternion must be handled differently
        % Averages here (plugging in online methods basically) should be the weighted
        % really outer product's strongest eigenvector, and then normalize that I guess
        outProd = zeros(4, 4);  % Outer prod mat
        for i=1:N
          quat = quat_chunk(:, i);
          outProd = outProd + weights(i) * (quat * quat'); % Outer product weighted and summed up along all quats
        end

        [eigenvectors, eigenvals] = eig(outProd);
        [~, ind] = max(diag(eigenvals));
        quat_mean = eigenvectors(:, ind);
        
        % normalize
        quat_mean = quat_mean / norm(quat_mean);

        % Combine into the state mean
        mu = [pos_mean; vel_mean; quat_mean; bias_mean]';
    end

    function [DiffMat, P] = calculate_new_covar(obj, mu, transformed_pts, weights, Q, Nsig)
        % Now get new covariance

        % Weights should be a column vector. Assert if that's not true
        s = size(weights);
        assert(s(1) > s(2));

        P = zeros(size(Q));

        % Store the diffs in a matrix for later in the cross covariance calculations
        DiffMat = 0;

        % Now, do the covariance deltas for each type
        % This gets a bit weird because the covariance matrix for quaternions has only 
        % 3 Degrees of freedom, so using an online method
        for i=1:Nsig
          % pos vel biases
          posvel = weights(i) * (transformed_pts(1:6, i) - mu(1:6)');
          biases = weights(i) * (transformed_pts(11:16, i) - mu(11:16)');

          % quats don't do the differences because of its DoF issue, so instead, you work
          % with the setup separately
          quat = transformed_pts(7:10, i);
          muQuat = mu(7:10);
          if norm(muQuat) > 1.2 || norm(muQuat) < 0.9 || sum(isnan(mu)) ~= 0
            muQuat
            error("Normalize your quaternions");
          end

          quat_diff =  weights(i) * (obj.quat_mult(obj.quat_inv(muQuat), quat));

          % now, we don't actually use all these points because of the 3DoFs so we ignore the
          % norm/weight at the beginning
          % This also ruins basically any good system to keep up the 4 variable quaternion in
          % the state. Therefore, ignoring that for now!
          quat_diff = quat_diff / norm(quat_diff);
          quat_diff = quat_diff(1:4)';   % The 2 * is an approximation GPT recommended, need to look into this later.


          % combine them all
          diff = [posvel; quat_diff; biases];

          % Store diffs
          if DiffMat == 0
            DiffMat = zeros([length(diff), Nsig]);
            DiffMat(:, i) = diff;
          else
            DiffMat(:, i) = diff;
          end
          
          % Covariance
          P = P + (diff * diff');     % weights already included
        end 
      P = P + Q;
    end

    function pred = predict_measurements(obj, transformed_pts, N)
        % Predict measurements
        pred = zeros([9, N]);
        for i=1:N
          sigma = transformed_pts(:, i);
          
          acc_pred = obj.measurement_model.predict_acceleration(sigma);
          gyr_pred = obj.measurement_model.predict_gyro(sigma);
          mag_pred = obj.measurement_model.predict_magno(sigma);

          pred(:, i) = [acc_pred; gyr_pred; mag_pred]';
        end

        if sum(pred > 1e5) > 0
          acc_pred
          gyr_pred
          mag_pred

          transformed_pts
          error("You fucked up")
        end
    end

    function cross_covar = calculate_meas_state_covar(obj, transformed, predictions, weights, Nsig, st_dels, meas_mu)
      % Cross covariance
      l1 = length(st_dels(:, 1));
      l2 = length(predictions(:, 1));

      cross_covar = zeros([l1, l2]);    % so state is first, then multiplied by measurements

      for i=1:Nsig
        % Covariance
        state_del = st_dels(:, i);    % saved from old covariance estimate
        meas_del  = predictions(:, i) - meas_mu;

        % state_del has weights already appliecd, and since we only need one multiplicatoin
        % of the weights, no need for weights anymore
        
        s = size(state_del);
        assert(s(1) > s(2)) % more rows than columns so it's a column vector

        % Set up covar
        cross_covar = cross_covar + state_del * meas_del';
        % cross_covar1 = cross_covar(1:3, 1:3)
      end
    end
    
    function kalman_step(obj, cross_covar, meas_P, meas, meas_mu, mu_est, ind)
      % K kalman gain
      K = cross_covar * pinv(meas_P); %cross_covar * inv(meas_P);    % fingers crossed it's invertible!

      % mu_new for state
      mu_new = mu_est' + K * (meas' - meas_mu);
      mu_new = mu_new / norm(mu_new + 1e-12)

      if sum(isnan(mu_new)) ~= 0
        mu_new
        meas
        meas_mu
        K
        error("You fucked up! We're on iteration " + ind);
      end
      
      % Store
      obj.states(ind, :) = mu_new;
      
      % Pnew for state covar
      Pnew = obj.Pst - K * meas_P * K';
      obj.Pst = Pnew;
    end

    function m = measure_mean(obj, pred, weights)
      m = pred * weights;

      if norm(m) > 1e6
        % m
        % weights
        % pred
        error("Why is this kinda big tho? Currently on index: " + obj.index);
      end
    end

    function measCovar = calculate_measurement_covar(obj, pred, mu, weights, Nsig)
      % contains the measurement covariance only
      s = size(pred(:, 1));
      assert(s(1) > s(2));  % column vector of predictions

      % Get measurement covariance matrix
      R = obj.measurement_model.get_measurement_covariance();
      measCovar = R;

      % add covariance from estimate
      for i=1:Nsig
        % Add individual covariances
        meas = pred(:, i);
        measCovar = measCovar + weights(i) * (pred(:, i) - mu) * (pred(:, i) - mu)';
      end      
    end

    %% HELPER
    %% UNTESTED
    function inv = quat_inv(~, quat)
      inv = [quat(1), -1 * quat(2), -1 * quat(3), -1 * quat(4)];
    end

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

    %% Currently broken, just will do other stuff before fixing
    function plotter(obj, gifFileName)
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
              imwrite(imind, cm, gifFileName, 'gif', 'Loopcount', inf, 'DelayTime', 0.03);
          elseif (mod(i, 10) == 0)
              disp("Displayed the " + i + "th frame so far!")
              imwrite(imind, cm, gifFileName, 'gif', 'WriteMode', 'append', 'DelayTime', 0.03);
          end
        end

    end
  end
end
