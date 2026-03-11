%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
% slam, explore data association algorithms
% MODIFIED: Added SINGLES, JCBB experiments, map maintenance,
%           map.seen / map.last_seen tracking counters.
%-------------------------------------------------------
clear all;
close all;
randn('state', 0);
rand('state', 0);
addpath 'tools';

% determines execution and display modes
global configuration;

configuration.ellipses       = 1;
configuration.tags           = 0;
configuration.noise          = 1;
configuration.alpha          = 0.99;

% -------------------------------------------------------
% EXPERIMENT SELECTOR
% -------------------------------------------------------
% Set 'algorithm' to one of:  'NN'  |  'SINGLES'  |  'JCBB'
algorithm = 'NN';

% Set to 1 to pause after each step, 0 for continuous run
configuration.step_by_step = 0;

% Set to 1 to add rogue measurements (people walking)
configuration.people = 0;

% Set to 1 to use odometry, 0 to disable it
configuration.odometry = 1;

% -------------------------------------------------------
% Map Maintenance toggle
% Set to 1 to activate elimination of features seen only
% once, more than 2 steps ago.
% -------------------------------------------------------
use_map_maintenance = 1;

% -------------------------------------------------------

% figure numbers
configuration.ground           = 1;
configuration.map              = 2;
configuration.observations     = 3;
configuration.compatibility    = 4;
configuration.ground_hypothesis = 5;
configuration.hypothesis       = 6;
configuration.tables           = 7;

% variables you need in several places
global map ground sensor people chi2 results;

load 'data/chi2';

% -------------------------------------------------------
% SENSOR PARAMETERS
% Try increasing srho / stita to see effect on ellipses
% -------------------------------------------------------
sensor.range    = 5;
sensor.minangle = -pi/2;
sensor.maxangle =  pi/2;
sensor.srho     = 0.01;        % range std dev      (try: 0.05, 0.10)
sensor.stita    = 0.125*pi/180;% angle std dev (rad) (try: 1*pi/180)

% generate the experiment data
[ground, people] = generate_experiment;

% start with a fresh map
[map, ground] = new_map(map, ground);

% Initialise map maintenance counters
% map.seen(f)      = number of times feature f has been observed
% map.last_seen(f) = last step at which feature f was observed
map.seen      = [];
map.last_seen = [];

% plot ground
draw_ground(ground);
pause

if configuration.people
    people.x = [];
    people.y = [];
end

% ok, here we go
step = 1;
observations = get_observations(ground, sensor, step);
draw_observations (observations, ground, step);

GT = zeros(1, observations.m);
H  = zeros(1, observations.m);

map = add_features(map, observations);

% Initialise counters for the first batch of features
for f = 1:map.n
    map.seen(f)      = 1;
    map.last_seen(f) = step;
end

results.total          = [];
results.true.positives = [];
results.true.negatives = [];
results.false.positives = [];
results.false.negatives = [];

results = store_results (results, observations, GT, H);

% plot map
configuration.name = '';
draw_map (map, ground, step);

steps = length(ground.motion);
for step = 2 : steps,

    disp('--------------------------------------------------------------');
    disp(sprintf('Step: %d', step));

    %  EKF prediction step
    motion    = ground.motion(step - 1);
    ground    = move_vehicle (ground, motion);
    odometry  = get_odometry (motion);
    map       = EKF_prediction (map, odometry);

    % sense
    observations = get_observations(ground, sensor, step);

    % individual compatibility
    prediction    = predict_observations (map, ground);
    compatibility = compute_compatibility (prediction, observations);

    disp(sprintf('Hypothesis: %d', compatibility.HS));
    disp(['IC: ' sprintf('%2d   ', compatibility.AL)]);
    disp(' ');

    % ground truth
    GT = ground_solution(map, observations);
    disp(['GT: ' sprintf('%2d   ', GT)]);
    disp(' ');

    % --------------------------------------------------
    % DATA ASSOCIATION — select algorithm via 'algorithm'
    % --------------------------------------------------
    if strcmp(algorithm, 'NN')
        H = NN (prediction, observations, compatibility);

    elseif strcmp(algorithm, 'SINGLES')
        H = SINGLES (prediction, observations, compatibility);

    elseif strcmp(algorithm, 'JCBB')
        H = JCBB (prediction, observations, compatibility);

    else
        error('Unknown algorithm. Choose: NN | SINGLES | JCBB');
    end
    % --------------------------------------------------

    draw_map (map, ground, step);
    draw_observations (observations, ground, step);
    draw_compatibility (prediction, observations, compatibility);

    disp(['H : ' sprintf('%2d   ', H)]);
    disp(['    ' sprintf('%2d   ', GT == H)]);
    disp(' ');

    draw_hypothesis (prediction, observations, H,  [algorithm ':'], 'b-');
    draw_hypothesis (prediction, observations, GT, 'JCBB:', 'b-');
    draw_tables (compatibility, GT, H);

    % update EKF step
    map = EKF_update (map, prediction, observations, H);

    % --------------------------------------------------
    % Update observation counters for matched features
    % --------------------------------------------------
    for i = 1:observations.m
        if H(i) > 0
            fid = H(i);
            if fid <= length(map.seen)
                map.seen(fid)      = map.seen(fid) + 1;
                map.last_seen(fid) = step;
            end
        end
    end

    % --------------------------------------------------
    % Add new features (unmatched observations with no
    % individually compatible neighbours in map)
    % --------------------------------------------------
    new = find((H == 0) & (compatibility.AL == 0));

    if nnz(new)
        disp(['NEW: ' sprintf('%2d   ', new)]);
        n_before = map.n;
        map = add_features(map, observations, new);
        n_after  = map.n;
        % Initialise counters for newly added features
        for f = (n_before + 1):n_after
            map.seen(f)      = 1;
            map.last_seen(f) = step;
        end
    end

    % --------------------------------------------------
    % MAP MAINTENANCE
    % Eliminate features seen only once, more than 2 steps ago
    % --------------------------------------------------
    if use_map_maintenance
        to_remove = [];
        for f = 1:map.n
            if map.seen(f) == 1 && (step - map.last_seen(f)) > 2
                to_remove = [to_remove, f];
            end
        end
        if ~isempty(to_remove)
            disp(['REMOVING features: ' sprintf('%d ', to_remove)]);
            map = remove_features(map, to_remove);
        end
    end
    % --------------------------------------------------

    draw_map (map, ground, step);
    results = store_results(results, observations, GT, H);

    if configuration.step_by_step
        wait
    else
        drawnow;
    end

end

show_results(map, ground, results);
