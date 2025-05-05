% Track initial conditions - from project 2

% Track parameters
R = 200;           % Radius
L = 900;           % Length of straight sections
w = 15;            % Width of track

% Define waypoints for the centerline
% Half-circle points
theta = linspace(0, pi, 300);

% First straight section (Start to first curve)
x1 = linspace(0, L, 300);
y1 = zeros(size(x1));

% First curve (counterclockwise from right to top)
x2 = L + R * sin(theta);
y2 = R * (1 - cos(theta));

% ADJUST FOR SMOOTH LINE
x2 = x2(2:end);
y2 = y2(2:end);

% Second straight section (Top to left)
x3 = linspace(L, 0, 300);
y3 = R * 2 * ones(size(x3));

% ADJUST FOR SMOOTH LINE
x3 = x3(2:end);
y3 = y3(2:end);

% Second curve (counterclockwise from left to bottom)
x4 = -R * sin(theta);
y4 = R * (1 + cos(theta));

% ADJUST FOR SMOOTH LINE
x4 = x4(2:end);
y4 = y4(2:end);

% --- Smooth the transition from the first curve to the second straight ---
n_transition_points = 20; % Number of transition points
last_curve_x = x2(end-n_transition_points+1:end);
last_curve_y = y2(end-n_transition_points+1:end);
first_straight_x = x3(1:n_transition_points);
first_straight_y = y3(1:n_transition_points);

transition_x = linspace(last_curve_x(1), first_straight_x(1), n_transition_points);
transition_y = linspace(last_curve_y(1), first_straight_y(1), n_transition_points);

% Adjust the original curve and straight to exclude the overlapping parts
x2_adjusted = x2(1:end-n_transition_points);
y2_adjusted = y2(1:end-n_transition_points);
x3_adjusted = x3(n_transition_points+1:end);
y3_adjusted = y3(n_transition_points+1:end);

% Concatenate the centerline points with the transition
x_center = [x1 x2_adjusted transition_x x3_adjusted x4];
y_center = [y1 y2_adjusted transition_y y3_adjusted y4];

% --- End of transition smoothing ---

% Get dy and dx
dy = diff(y_center);
dx = diff(x_center);

% Gets angle of the car/track based on dx and dy coordinate
theta = atan2(dy, dx);

% Define track boundaries
x_outer = x_center - (w/2) * [-sin(theta) 0];
y_outer = y_center + (w/2) * [cos(theta) 1];
x_inner = x_center + (w/2) * [-sin(theta) 0];
y_inner = y_center - (w/2) * [cos(theta) 1];

% % Plot the track
% figure; hold on;
% axis equal;
% plot(x_outer, y_outer, 'k', 'LineWidth', 0.5);
% plot(x_inner, y_inner, 'k', 'LineWidth', 0.5);
% 
% % Centerline
% plot(x_center, y_center, 'b--', 'LineWidth', 1);
% title('Race Track and Moving Vehicle');
% xlabel('X (meters)');
% ylabel('Y (meters)');

% Vehicle simulation
vehicle_length = 10;
vehicle_width = 5;

% Create a figure for combined plot and animation
figure('Name', 'Race Animation and Trajectory'); hold on; axis equal;
% Add padding for view window
padding = 100;
xlim([min(x_center) - padding, max(x_center) + padding]);
ylim([min(y_center) - padding, max(y_center) + padding]);
% Plot the static track
plot(x_outer, y_outer, 'k', 'LineWidth', 0.5);
plot(x_inner, y_inner, 'k', 'LineWidth', 0.5);
plot(x_center, y_center, 'b--', 'LineWidth', 1);
title('Race Track and Vehicle Trajectory/Animation');
xlabel('X (meters)');
ylabel('Y (meters)');
% Recreate car shape
x_car_dim = [-vehicle_length/2 vehicle_length/2 vehicle_length/2 -vehicle_length/2];
y_car_dim = [-vehicle_width/2 -vehicle_width/2 vehicle_width/2 vehicle_width/2];
race_car = patch(x_car_dim, y_car_dim, 'w');
% Line to show car's motion
car_motion = animatedline('Color', 'r', 'LineWidth', 1.5);

% Initial conditions for car - from project 3
% Drag coefficients
carData.C1 = 0.00041; % N/(m/s)
carData.C0 = 0.00004; % Sim N/(u/s)

% Parameters for calculation of C2
rho = 1.225;     % kg/m^3 (air density)
A = 2.16;        % m^2 (projected area)
Cd = 0.36;       % Aerodynamic drag coefficient
carData.C2 = 0.5 * rho * A * Cd;

% Initial Conditions - Vehicle
carData.Inertia = 1600;  % kg·m^2
carData.Mass = 1000;     % kg

% Initial Conditions
carData.init.x0 = 0;          % meters - initial position in x direction
carData.init.y0 = 0;          % meters - initial position in y direction

% Desired Velocities
 carData.init.vx0 = 25;        % m/s - velocity in x of the car
 carData.init.vy0 = 0;         % m/s - velocity in y of the car
carData.desired.v = 12;       % m/s Tuned for current performance
carData.init.omega0 = 0;      % rad/s - initial yaw rate of the car
carData.init.psi0 = 0;        % rad - initial heading of the car

% Tire conditions
carData.Calphaf = 40000;     % N/rad - front tire coefficient
carData.Calphar = 40000;     % N/rad - rear tire coefficient
carData.fymax = 0.7 * 4000 * 9.81; % max front tire force
carData.fymaxR = 0.7 * 4000 * 9.81; % max rear tire force
carData.lr = 1.5;             % Distance from center line to rear axis
carData.lf = 1.5;             % Distance from center line to front axis
carData.radius = 0.3;         % Radius of tires
carData.maxAlpha = 2.7 / 180 * pi; % max alpha value for cal

carData.s = carData.fymax / (carData.radius)^2;

carData.C_lambda = 50;        % Longitudinal stiffness
carData.lambda_max = 0.1;     % Max tire slip ratio
carData.tire_mu = 1.0;

carData.init.omega = carData.init.vx0 / carData.radius;

% Transmission - Gear ratios
carData.gearRatio1 = 10;
carData.gearRatio2 = 3;
carData.gearRatio = 1;

carData.FDRatio = 7.5;
carData.Eta = 0.95;

% Brakes
carData.maxBrakeTorque = 50000;

% Track Data
track.radius = 200;   % Radius of curves
track.width = 15;     % Width of track
track.straight = 900; % Length straightaway

% PID Controller
% k_proport = 1;
% k_integral = 0;
% k_deriv = 0;

scaleFactor = 0.75;

% Electric Motor Data
motorData.rpm = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 11000, 12000];

motorData.maxtorque = [...
    280, 280, 270, 260, 140, 110, 95, 70, 65, 50, 40, 0, 0; 
    280, 280, 275, 255, 240, 180, 125, 95, 75, 50, 40, 0, 0;
    280, 280, 275, 250, 240, 180, 140, 125, 95, 70, 50, 0, 0;
    280, 280, 275, 250, 240, 200, 175, 140, 120, 100, 75, 0, 0] * scaleFactor;

motorData.vbus = [150, 250, 600, 700] * scaleFactor;

max_torque = max(motorData.maxtorque(:));  % Get max torque from the matrix
motorData.eta_torque = linspace(0, max_torque, 14);

motorData.eta_speed = (0:500:10000);
motorData.eta_speed(1) = 10;

motorData.eta_val = [0.740000000000000,0.740000000000000,0.740000000000000,0.740000000000000,0.740000000000000,0.740000000000000,0.740000000000000,0.740000000000000,0.740000000000000,0.740000000000000,0.740000000000000,0.740000000000000,0.740000000000000,0.740000000000000,0.740000000000000,0.740000000000000,0.740000000000000,0.740000000000000,0.740000000000000,0.740000000000000,0.740000000000000;
    0.860000000000000,0.940000000000000,0.96000000000000,0.940000000000000,0.940000000000000,0.940000000000000,0.920000000000000,0.920000000000000,0.900000000000000,0.900000000000000,0.880000000000000,0.860000000000000,0.860000000000000,0.860000000000000,0.860000000000000,0.840000000000000,0.820000000000000,0.800000000000000,0.780000000000000,0.760000000000000,0.740000000000000;
    0.840000000000000,0.940000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.960000000000000,0.940000000000000,0.940000000000000,0.940000000000000,0.940000000000000,0.920000000000000,0.920000000000000,0.920000000000000,0.920000000000000,0.920000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.880000000000000,0.880000000000000;
    0.840000000000000,0.920000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.940000000000000,0.940000000000000,0.940000000000000,0.940000000000000,0.940000000000000,0.940000000000000,0.920000000000000,0.920000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.880000000000000;
    0.820000000000000,0.900000000000000,0.940000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.940000000000000,0.940000000000000,0.940000000000000,0.920000000000000,0.900000000000000,0.900000000000000,0.880000000000000,0.880000000000000,0.880000000000000;
    0.820000000000000,0.880000000000000,0.940000000000000,0.940000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.940000000000000,0.940000000000000,0.920000000000000,0.920000000000000,0.900000000000000,0.900000000000000,0.880000000000000,0.860000000000000,0.860000000000000;
    0.800000000000000,0.880000000000000,0.920000000000000,0.940000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.940000000000000,0.940000000000000,0.920000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000;
    0.800000000000000,0.860000000000000,0.900000000000000,0.940000000000000,0.940000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.940000000000000,0.920000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000;
    0.780000000000000,0.860000000000000,0.900000000000000,0.920000000000000,0.940000000000000,0.940000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.96000000000000,0.940000000000000,0.920000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000;
    0.780000000000000,0.860000000000000,0.900000000000000,0.920000000000000,0.920000000000000,0.940000000000000,0.940000000000000,0.940000000000000,0.96000000000000,0.940000000000000,0.940000000000000,0.920000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000;
    0.760000000000000,0.860000000000000,0.880000000000000,0.900000000000000,0.920000000000000,0.940000000000000,0.940000000000000,0.940000000000000,0.940000000000000,0.940000000000000,0.920000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000;
    0.740000000000000,0.840000000000000,0.860000000000000,0.900000000000000,0.920000000000000,0.920000000000000,0.940000000000000,0.940000000000000,0.940000000000000,0.940000000000000,0.920000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.9000000000000000;
    .740000000000000,0.840000000000000,0.860000000000000,0.880000000000000,0.900000000000000,0.920000000000000,0.920000000000000,0.940000000000000,0.920000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000;
    0.720000000000000,0.820000000000000,0.860000000000000,0.880000000000000,0.900000000000000,0.900000000000000,0.920000000000000,0.920000000000000,0.920000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000,0.900000000000000];


motorData.eta_val = motorData.eta_val .* motorData.eta_val;

motorData.inertia = 0.5;
% Used to have:
% engineInertia = 0.1;

% Converting MPH to MPS (Meter/second)
mph2mps = 1600 / 3600;

% Battery data
BattData.SOC = [0, .1, .2, .3, .4, .5, .6, .7, .8, .9, 1];
BattData.OCV = [0, 1.55,1.95,3.5,3.76,3.78,3.9,3.95,3.98,4.05,4.15];
BattData.Rint = 0.13; % Internal resistance per cell
BattData.C = 25;      % Amp hr total battery capacity
BattData.numSeries = 96;
BattData.numParallel = 74;

% Getting the simulation data
% Calling the model
model = 'Project4_MODEL_FINAL';

% Set parameters
set_param(model, 'Solver', 'ode45');

% Track length
track_length = 2 * L + 2 * pi * R;

% Time to complete one lap
estimated_lap_time = track_length / carData.desired.v;

% Tolerance to stop because car doesn’t go perfectly on center line
tol = 1.3;

% Simulation end time
set_param(model, 'StopTime', num2str(estimated_lap_time + tol));
set_param(model, 'StopTime', num2str(600));

% Loading the model
load_system(model);
simout = sim(model);

% Get data from Simulink simulation
X_sim_data = simout.X.Data;
Y_sim_data = simout.Y.Data;
Time_data = simout.X.Time;

% Heading angle
psi_sim_data = simout.psi.Data;

car_motion = animatedline('Color', 'r');

% Project 2 results
for i = 1:length(X_sim_data)-1
    % Update vehicle position and rotation
    % 2x2 matrix to rotate car
    Rotation = [cos(psi_sim_data(i)) -sin(psi_sim_data(i)); 
                sin(psi_sim_data(i))  cos(psi_sim_data(i))];

    vehicle_shape = Rotation * [x_car_dim; y_car_dim];

    % Updates graph with the car position and rotation
    % Adds x and y center to make car stay in the middle
    set(race_car, 'XData', vehicle_shape(1,:) + X_sim_data(i), ...
                  'YData', vehicle_shape(2,:) + Y_sim_data(i));
    addpoints(car_motion, X_sim_data(i), Y_sim_data(i));
    pause(0.001);
end

% Function to track car and display laps completed, lap time, and if car
% stays in track
function racestats(X_data, Y_data, time_data, track)
    % Track parameters
    width = track.width;
    tolerance = 0;
    vehicle_length = 10;
    vehicle_width = 5;

    % Initialize variables
    num_loops = 0;
    out_of_bounds = false;
    lap_times = [];
    last_lap_time = time_data(1);
    
    % Lap detection parameters
    lap_zone_y_threshold = 20;   % Only detect laps near center of track (adjust as needed)
    lap_triggered = false;
    lap_cooldown = 10;           % Seconds before next lap can be counted

    for i = 2:length(X_data)
        % Find the closest centerline point
        distances = sqrt((X_data(i) - track.x_center).^2 + (Y_data(i) - track.y_center).^2);
        [~, min_idx] = min(distances);

        % Get direction of the track at the closest point
        if min_idx < length(track.x_center)
            dx = track.x_center(min_idx + 1) - track.x_center(min_idx);
            dy = track.y_center(min_idx + 1) - track.y_center(min_idx);
        else
            dx = track.x_center(min_idx) - track.x_center(min_idx - 1);
            dy = track.y_center(min_idx) - track.y_center(min_idx - 1);
        end

        % Track direction angle
        theta_local = atan2(dy, dx);

        % Compute the normal vector
        normal_x = -sin(theta_local);
        normal_y = cos(theta_local);

        % Distance from centerline (center point of car)
        car_to_center_x = X_data(i) - track.x_center(min_idx);
        car_to_center_y = Y_data(i) - track.y_center(min_idx);
        dist_to_center = abs(car_to_center_x * normal_x + car_to_center_y * normal_y);

        if dist_to_center > (width / 2) + tolerance
            out_of_bounds = true;
            disp(['Car is OUT OF BOUNDS at iteration ', num2str(i)]);
            break;
        end

        % Corner check
        corners = [...
            X_data(i) - vehicle_width / 2, Y_data(i) + vehicle_length / 2;
            X_data(i) + vehicle_width / 2, Y_data(i) + vehicle_length / 2;
            X_data(i) - vehicle_width / 2, Y_data(i) - vehicle_length / 2;
            X_data(i) + vehicle_width / 2, Y_data(i) - vehicle_length / 2];

        for j = 1:4
            corner_x = corners(j, 1);
            corner_y = corners(j, 2);

            corner_to_center_x = corner_x - track.x_center(min_idx);
            corner_to_center_y = corner_y - track.y_center(min_idx);
            dist_to_center_corner = abs(corner_to_center_x * normal_x + corner_to_center_y * normal_y);

            if dist_to_center_corner > (width / 2) + tolerance
                out_of_bounds = true;
                disp(['Car corner is OUT OF BOUNDS at iteration ', num2str(i)]);
                break;
            end
        end

        if out_of_bounds
            break;
        end

        % --- Revised Lap Detection ---
        if X_data(i) > 0 && X_data(i-1) < 0 && abs(Y_data(i)) < lap_zone_y_threshold && ...
                ~lap_triggered && (time_data(i) - last_lap_time) > lap_cooldown

            num_loops = num_loops + 1;
            current_lap_time = time_data(i) - last_lap_time;
            lap_times = [lap_times, current_lap_time];
            last_lap_time = time_data(i);

            lap_triggered = true;

            disp(['Lap Completed! Total Laps: ', num2str(num_loops)]);
            disp(['Lap time: ', num2str(current_lap_time), ' seconds']);
        end

        % Reset trigger once the car is far from the lap line
        if abs(X_data(i)) > 50
            lap_triggered = false;
        end
    end

    % Final race status
    if out_of_bounds
        disp('The vehicle went out of bounds.');
    elseif num_loops == 0
        disp('The vehicle did not complete a lap yet.');
    else
        disp('The vehicle stayed within the track.');
    end
end


% Track parameters
track.radius = 200;
track.width = 15;
track.l_straightaways = 900;

% Define track centerline
track.x_center = x_center;
track.y_center = y_center;

% Run the race statistics function 
racestats(X_sim_data, Y_sim_data, Time_data, track);