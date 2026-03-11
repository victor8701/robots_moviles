function map = remove_features(map, indices)
%-------------------------------------------------------
% remove_features  Remove features from the EKF map by index.
%
% Inputs:
%   map     - current EKF map structure
%   indices - vector of feature indices (1-based) to remove
%
% The map state vector is:
%   map.x = [robot(3); feat1(2); feat2(2); ... featN(2)]
% So feature f occupies rows: 3 + (f-1)*2 + 1  to  3 + f*2
%
% map.P is the corresponding covariance matrix.
%-------------------------------------------------------

if isempty(indices)
    return;
end

% Sort descending so we can remove safely without index shift
indices = sort(indices, 'descend');

for f = indices
    % State vector rows for feature f (2D point = 2 rows)
    rows = (3 + (f-1)*2 + 1) : (3 + f*2);

    % Remove from state vector
    map.x(rows)   = [];

    % Remove rows and columns from covariance
    map.P(rows, :) = [];
    map.P(:, rows) = [];

    % Remove from ground_id list
    map.ground_id(f) = [];

    % Remove from maintenance counters
    map.seen(f)      = [];
    map.last_seen(f) = [];

    % Decrement feature count
    map.n = map.n - 1;
end

end
