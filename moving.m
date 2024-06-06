function moving()

    function ellipse_params = fit_ellipse_direct(x, y)
        % Construct matrix A for linear equations: [x^2, xy, y^2, x, y, 1]
        A = [x.^2, x.*y, y.^2, x, y, ones(size(x))];

        % Solve the linear equations to obtain ellipse parameters
        [~, ~, V] = svd(A, 0);
        params = V(:, end);

        % Extract ellipse parameters from the solution
        ellipse_params = struct();
        ellipse_params.A = params(1);
        ellipse_params.B = params(2);
        ellipse_params.C = params(3);
        ellipse_params.D = params(4);
        ellipse_params.E = params(5);
        ellipse_params.F = params(6);
    end

    % Load data from ROS bag file
    bag = rosbag('/home/veditha/lab4/src/data/data1.bag');
    imu = select(bag, 'Topic', '/imu');
    imudata = readMessages(imu, 'DataFormat', 'struct');
    gps = select(bag, 'Topic', '/gps');
    gpsdata = readMessages(gps, 'DataFormat', 'struct');

    % Extract data fields
    magnetic_field = cell2mat(cellfun(@(m) [m.MagField.MagneticField_.X m.MagField.MagneticField_.Y m.MagField.MagneticField_.Z], imudata, 'UniformOutput', false));
    quaternion = cell2mat(cellfun(@(m) [m.IMU.Orientation.X m.IMU.Orientation.Y m.IMU.Orientation.Z m.IMU.Orientation.W], imudata, 'UniformOutput', false));
    angular_velocity = cell2mat(cellfun(@(m) [m.IMU.AngularVelocity.X m.IMU.AngularVelocity.Y m.IMU.AngularVelocity.Z], imudata, 'UniformOutput', false));
    linear_acceleration = cell2mat(cellfun(@(m) [m.IMU.LinearAcceleration.X m.IMU.LinearAcceleration.Y m.IMU.LinearAcceleration.Z], imudata, 'UniformOutput', false));
    time = cellfun(@(m) str2double(strcat(num2str(m.Header.Stamp.Sec - imudata{1, 1}.Header.Stamp.Sec), '.', num2str(m.Header.Stamp.Nsec))), imudata);

    % Extract UTM easting and northing values
    utm = cell2mat(cellfun(@(m) [m.UtmEasting m.UtmNorthing], gpsdata, 'UniformOutput', false));
    %% 
    gps_time = cellfun(@(m) str2double(strcat(num2str(m.Header.Stamp.Sec-gpsdata{1,1}.Header.Stamp.Sec),'.',num2str(m.Header.Stamp.Nsec))),gpsdata);

    %% Yaw
    mag_x = magnetic_field(:,1);
    mag_y = magnetic_field(:,2);
    mag_z = magnetic_field(:,3);

    eular = quat2eul(quaternion);
    eular_yaw = eular(:,1);

    % Raw yaw from uncalibrated magnetic field
    mag_yaw = transpose(atan2(mag_x, mag_y));

    % Calibrate magnetometer using Direct Least Squares Fitting of Ellipses method
    ellipse_params = fit_ellipse_direct(mag_x, mag_y);

    % Extract ellipse parameters
    A = ellipse_params.A;
    B = ellipse_params.B;
    C = ellipse_params.C;
    D = ellipse_params.D;
    E = ellipse_params.E;
    F = ellipse_params.F;

    % Calculate calibrated_x, calibrated_y, and further calculations using ellipse parameters
    calibrated_x = (C* D - 2* B* E) / (4* A* C - B^2);
    calibrated_y = (B* D - 2* A* E) / (4* A* C - B^2);

    calibrated_yaw = atan2(calibrated_y, calibrated_x);

    angular_yaw = angular_velocity(:,3);
    % Cumulative trapezoidal numerical integration --- cumtrapz()
    angle_yaw = cumtrapz(time, angular_yaw);
    % Selected Alpha value for Complementary Filter II
    a = 0.03;
    % Lecture Powerpoint Complementary Filter II
    filter = a * unwrap(calibrated_yaw) + (1 - a) * (angle_yaw + calibrated_yaw(1));
    % Wrap angle from radius to Pi
    filtered = wrapToPi(filter);


%% Forward acceleration
linear_acceleration_x = linear_acceleration(:,1);
figure(1)
hold on
plot(time, linear_acceleration_x, '.');
grid on
xlabel("Time (s)")
ylabel("Linear Acceleration in x direction (m/s^2)")
title("Forward Acceleration")

%% Forward velocity
imu_velocity = cumtrapz(time, linear_acceleration_x);


gps_x = gradient(utm(:,1)) ./ gradient(gps_time);
gps_y = gradient(utm(:,2)) ./ gradient(gps_time);
gps_velocity = sqrt(gps_x .^2 + gps_y .^2);

figure(2)
hold on
grid on
plot(time, imu_velocity, '.')
plot(gps_time, gps_velocity, '.')
title("Forward Velocity from IMU and  GPS")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
legend("IMU data", "GPS data")


%% Calibrate velocity
mean_imu_acc = mean(linear_acceleration_x);
imu_calibrated = linear_acceleration_x - mean_imu_acc;

i = smoothdata(linear_acceleration_x, 'gaussian', 200);
j = gradient(i);
for counter = 1 : length(j)
    if 0.0004 > abs(j(counter))
        result(counter) = 0;
    else
        result(counter) = imu_calibrated(counter);
    end
end

imu_result = transpose(cumtrapz(time, result));

figure(3)
subplot(2,1,1)
grid on
hold on
%plot(time, imu_result, '.')
plot(gps_time, gps_velocity,'green')
title("Forward Velocity with GPS & Calibrated IMU")
ylabel("Velocity (m/s)")
legend("GPS")
hold off

subplot(2,1,2)
grid on
hold on
plot(time, imu_result, 'red')
%plot(gps_time, gps_velocity, '.')
%title("Forward Velocity with Calibrated IMU")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
legend("IMU")
hold off

%% Dead Reckoning with IMU
imu_position = cumtrapz(time,imu_result);
gps_position = cumtrapz(gps_time, gps_velocity);

gps_position_x = utm(:,1) - utm(1,1);
gps_position_y = utm(:,2) - utm(1,2);

figure(4)
grid on
hold on
plot(time, imu_position)
plot(gps_time, gps_position)
title("Displacement from IMU & GPS")
xlabel("Time (s)")
ylabel("Displacement (m)")
legend("IMU", "GPS")
hold off

omega = angular_velocity(:,3);
x_diff = gps_velocity;
omegax_diff = omega .* imu_velocity;
y_2diff = linear_acceleration(:,2);

figure(5)
grid on
hold on
plot(time, y_2diff, '.')
plot(time, omegax_diff, '.')
title("Comparison of Acceleration")
xlabel("Time (s)")
ylabel("Acceleration (m/s^2)")
legend("y''", '\omegaX')
hold off

% Calculate imu_east_velocity and imu_north_velocity using filtered
imu_east_velocity = -imu_velocity .* cos(filtered);
imu_north_velocity = imu_velocity .* sin(filtered);
imu_easting = cumtrapz(time, imu_east_velocity);
imu_northing = cumtrapz(time, imu_north_velocity);

figure(6)
grid on
hold on
subplot(2,1,1)
plot(gps_position_x, gps_position_y, '.')
legend("GPS")
title("Route Taken")

ylabel("Northing (m)")
subplot(2,1,2)
plot(imu_easting,imu_northing,'.')
xlabel("Easting (m)")
ylabel("Northing (m)")
legend("IMU")
hold off

%% IMU Offset
J = omegax_diff;
K = y_2diff(1:end) - omegax_diff(1:end);
offset = linsolve(J, K);
end