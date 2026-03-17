function H = SINGLES (prediction, observations, compatibility)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
% SINGLES (Only Neighbour) Data Association Algorithm
%
% An observation i is matched to feature j ONLY IF:
%   1. Observation i has exactly ONE compatible feature (j)
%   2. Feature j has exactly ONE compatible observation (i)
%
% This ensures strictly unambiguous (bijective) matches.
% Uses compatibility.ic(i,j) = 1 if observation i is a
% neighbour of feature j (individually compatible).
%-------------------------------------------------------
global chi2;
global configuration;

H = zeros(1, observations.m);

for i = 1:observations.m

    % Find all features that are individually compatible with observation i
    compatible_features = find(compatibility.ic(i, :));

    if length(compatible_features) == 1
        j = compatible_features(1);

        % Find all observations that are individually compatible with feature j
        compatible_observations = find(compatibility.ic(:, j));

        if length(compatible_observations) == 1
            % Biunivocal unique match: observation i <-> feature j
            H(i) = j;
        end
        % else: feature j is ambiguous (multiple obs can reach it) -> no match
    end
    % else: observation i is ambiguous (multiple features) -> no match

end

configuration.name = 'ONLY NEIGHBOUR';
