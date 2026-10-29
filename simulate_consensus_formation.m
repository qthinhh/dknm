function results = simulate_consensus_formation()
% SIMULATE_CONSENSUS_FORMATION
% Simplified reproduction of:
% "Consensus-Based Formation Control for Heterogeneous Multi-Agent
% Systems in Complex Environments", Drones, 2025, 9, 175.
%
% The script uses only basic MATLAB functions. It simulates:
%   1) consensus of 6 first-order and 4 second-order agents;
%   2) dynamic formation change: circle -> line -> triangle, with obstacles;
%   3) an extended case with 25 agents divided into 4 groups.
%
% Run from the MATLAB command window:
%   results = simulate_consensus_formation;
%
% This is a pedagogical reproduction, not the authors' original source code.

clc;
close all;
rng(7);

cfg.dt = 0.03;
cfg.T = 24;
cfg.commR = 11.0;
cfg.commCore = 0.60 * cfg.commR;
cfg.safeAgent = 0.80;
cfg.agentInfluence = 3.00;
cfg.obstacleInfluence = 3.50;
cfg.kFormation = 1.35;
cfg.kConsensus = 0.55;
cfg.kVelocity = 1.80;
cfg.kObstacle = 3.80;
cfg.kAgentRepulsion = 1.25;
cfg.maxSpeed = 2.60;
cfg.maxAcceleration = 5.50;
cfg.integrationDamping = 0.92;

fprintf('Running simplified consensus-based formation simulations...\n');
fprintf('No toolbox is required. MATLAB R2016b+ or GNU Octave 7+ is recommended.\n\n');

results.consensus10 = runScenario(cfg, 10, 6, 3, 'consensus');
results.formation10 = runScenario(cfg, 10, 6, 3, 'formation');
results.formation25 = runScenario(cfg, 25, 15, 4, 'large');

plotScenario(results.consensus10, 'Scenario 1: Heterogeneous consensus, 10 agents');
plotScenario(results.formation10, 'Scenario 2: Formation change and obstacle avoidance');
plotScenario(results.formation25, 'Scenario 3: 25 agents and four groups');

fprintf('\nSimulation finished.\n');
printSummary(results.consensus10, '10 agents - consensus');
printSummary(results.formation10, '10 agents - dynamic formation');
printSummary(results.formation25, '25 agents - four groups');

end

function result = runScenario(cfg, n, nFirst, nGroups, mode)
dt = cfg.dt;
steps = floor(cfg.T / dt);
time = (0:steps) * dt;

firstOrder = false(n, 1);
firstOrder(1:nFirst) = true;
secondOrder = ~firstOrder;

group = assignGroups(n, nGroups);
omegaValues = linspace(1.0, 1.2, nGroups);
omega = omegaValues(group).';

% Initial positions are intentionally perturbed so convergence is visible.
offset0 = desiredOffsets(n, 'circle', 2.8);
[pLeader0, ~] = leaderState(0, mode);
x = pLeader0 + offset0 + 0.75 * randn(2, n);
v = zeros(2, n);
v(:, secondOrder) = 0.25 * randn(2, sum(secondOrder));

X = zeros(2, n, steps + 1);
V = zeros(2, n, steps + 1);
U = zeros(2, n, steps + 1);
targetHistory = zeros(2, n, steps + 1);
formationError = zeros(1, steps + 1);
targetError = zeros(1, steps + 1);
connectivity = zeros(1, steps + 1);

X(:, :, 1) = x;
V(:, :, 1) = v;

for k = 1:steps + 1
    t = time(k);
    [pLeader, vLeader] = leaderState(t, mode);
    shape = shapeAtTime(t, mode);
    offsets = desiredOffsets(n, shape, formationSpacing(mode));
    heading = atan2(vLeader(2), vLeader(1));
    Rheading = [cos(heading), -sin(heading); sin(heading), cos(heading)];
    target = pLeader + Rheading * offsets;

    targetHistory(:, :, k) = target;
    formationError(k) = pairwiseFormationError(x, offsets, Rheading, group);
    targetError(k) = mean(sqrt(sum((x - target).^2, 1)));
    connectivity(k) = connectedEdgeRatio(x, cfg.commR);

    if k == steps + 1
        break;
    end

    obstacles = obstaclesAtTime(t, mode);
    u = zeros(2, n);

    for i = 1:n
        % Formation tracking plus feed-forward leader velocity.
        eTarget = target(:, i) - x(:, i);
        command = vLeader + cfg.kFormation * omega(i)^2 * eTarget;

        % Local graph-consensus term. The relative desired offset is removed
        % so consensus supports, rather than destroys, the formation.
        for j = 1:n
            if j == i
                continue;
            end
            dvec = x(:, j) - x(:, i);
            d = norm(dvec);
            w = communicationWeight(d, cfg.commCore, cfg.commR);
            if w > 0
                desiredRelative = Rheading * (offsets(:, j) - offsets(:, i));
                command = command + cfg.kConsensus * omega(i) * w * ...
                    (dvec - desiredRelative);
            end
        end

        % Repulsion from neighboring agents.
        repulsion = zeros(2, 1);
        for j = 1:n
            if j == i
                continue;
            end
            delta = x(:, i) - x(:, j);
            d = norm(delta);
            if d < cfg.agentInfluence
                if d < 1e-6
                    delta = [1; 0];
                    d = 1e-6;
                end
                direction = delta / d;
                strength = cfg.kAgentRepulsion * ...
                    (cfg.agentInfluence - d) / cfg.agentInfluence;
                if d < cfg.safeAgent
                    strength = strength + 2.0 * (cfg.safeAgent - d) / cfg.safeAgent;
                end
                repulsion = repulsion + strength * direction;
            end
        end

        % Repulsion from static/moving environmental obstacles.
        obstacleRepulsion = zeros(2, 1);
        for q = 1:numel(obstacles)
            [clearance, normal] = obstacleDistance(x(:, i), obstacles(q));
            if clearance < cfg.obstacleInfluence
                effectiveClearance = max(clearance, -0.25);
                strength = cfg.kObstacle * ...
                    (cfg.obstacleInfluence - effectiveClearance) / cfg.obstacleInfluence;
                if clearance < 0
                    strength = strength + 5.0 * (-clearance + 0.1);
                end
                obstacleRepulsion = obstacleRepulsion + strength * normal;
            end
        end

        command = command + repulsion + obstacleRepulsion;
        command = command + cfg.kVelocity * (vLeader - v(:, i));
        u(:, i) = command;
    end

    % Integrate heterogeneous dynamics.
    for i = 1:n
        if firstOrder(i)
            v(:, i) = limitVector(u(:, i), cfg.maxSpeed);
            x(:, i) = x(:, i) + dt * v(:, i);
        else
            acceleration = limitVector(u(:, i), cfg.maxAcceleration);
            v(:, i) = cfg.integrationDamping * v(:, i) + dt * acceleration;
            v(:, i) = limitVector(v(:, i), cfg.maxSpeed);
            x(:, i) = x(:, i) + dt * v(:, i);
        end
    end

    X(:, :, k + 1) = x;
    V(:, :, k + 1) = v;
    U(:, :, k + 1) = u;
end

result.time = time;
result.X = X;
result.V = V;
result.U = U;
result.target = targetHistory;
result.formationError = formationError;
result.targetError = targetError;
result.connectivity = connectivity;
result.group = group;
result.firstOrder = firstOrder;
result.mode = mode;
result.n = n;
result.nGroups = nGroups;
result.cfg = cfg;

end

function group = assignGroups(n, nGroups)
group = zeros(1, n);
for i = 1:n
    group(i) = min(nGroups, floor((i - 1) * nGroups / n) + 1);
end
end

function [p, v] = leaderState(t, mode)
switch mode
    case 'consensus'
        p = [0.45 * t; 0.8 * sin(0.22 * t)];
        v = [0.45; 0.8 * 0.22 * cos(0.22 * t)];
    case 'formation'
        p = [0.42 * t; 0.55 * sin(0.32 * t)];
        v = [0.42; 0.55 * 0.32 * cos(0.32 * t)];
    otherwise
        p = [0.35 * t; 0.7 * sin(0.20 * t)];
        v = [0.35; 0.7 * 0.20 * cos(0.20 * t)];
end
end

function shape = shapeAtTime(t, mode)
if strcmp(mode, 'formation') || strcmp(mode, 'large')
    if t < 8
        shape = 'circle';
    elseif t < 16
        shape = 'line';
    else
        shape = 'triangle';
    end
else
    shape = 'circle';
end
end

function spacing = formationSpacing(mode)
if strcmp(mode, 'large')
    spacing = 2.3;
else
    spacing = 2.5;
end
end

function offsets = desiredOffsets(n, shape, spacing)
offsets = zeros(2, n);
switch shape
    case 'circle'
        radius = max(2.3, spacing * sqrt(n) / 2.0);
        angles = (0:n-1) * 2*pi/n;
        offsets = [radius * cos(angles); radius * sin(angles)];
    case 'line'
        offsets(2, :) = ((1:n) - (n + 1) / 2) * spacing;
        offsets(1, :) = 0;
    otherwise
        % Rows of a triangular lattice, centered around the origin.
        row = 1;
        col = 0;
        for i = 1:n
            offsets(:, i) = [(col - row / 2) * spacing; -row * 0.78 * spacing];
            col = col + 1;
            if col > row
                row = row + 1;
                col = 0;
            end
        end
        offsets = offsets - mean(offsets, 2);
end
end

function w = communicationWeight(d, core, radius)
if d < core
    w = 1.0;
elseif d < radius
    w = 0.5 * (1 + cos(pi * (d - core) / (radius - core)));
else
    w = 0.0;
end
end

function value = pairwiseFormationError(x, offsets, Rheading, group)
n = size(x, 2);
sumError = 0;
count = 0;
for i = 1:n
    for j = i+1:n
        if group(i) == group(j)
            actual = x(:, i) - x(:, j);
            desired = Rheading * (offsets(:, i) - offsets(:, j));
            sumError = sumError + norm(actual - desired)^2;
            count = count + 1;
        end
    end
end
if count == 0
    value = 0;
else
    value = sqrt(sumError / count);
end
end

function ratio = connectedEdgeRatio(x, radius)
n = size(x, 2);
total = n * (n - 1) / 2;
edges = 0;
for i = 1:n
    for j = i+1:n
        if norm(x(:, i) - x(:, j)) < radius
            edges = edges + 1;
        end
    end
end
ratio = edges / max(total, 1);
end

function obstacles = obstaclesAtTime(t, mode)
% Obstacles include circular, rectangular and elongated shapes.
switch mode
    case 'consensus'
        obstacles = emptyObstacles();
    case 'formation'
        obstacles = emptyObstacles();
        obstacles(1) = makeObstacle('circle', [5.5, 0.2], 1.25, [0, 0], 0, 0);
        obstacles(2) = makeObstacle('rect', [8.7, -1.5], [1.0, 2.2], [0, 0], 0, 0);
        obstacles(3) = makeObstacle('capsule', [6.9, 3.0], 2.8, [0, 0], 0, 0.38);
    otherwise
        obstacles = emptyObstacles();
        obstacles(1) = makeObstacle('triangle', [4.5, 0.7], 1.35, [0.0, -0.15], 0, 0);
        obstacles(2) = makeObstacle('rect', [8.0, -1.0], [1.0, 1.8], [-0.03, 0.06], 0, 0);
        obstacles(3) = makeObstacle('capsule', [10.4, 2.7], 3.2, [-0.03, 0], 0.30, 0.32);
end
for q = 1:numel(obstacles)
    obstacles(q).center = obstacles(q).center0 + obstacles(q).velocity * t;
end
end

function obstacles = emptyObstacles()
obstacles = struct('type', {}, 'center0', {}, 'center', {}, ...
    'size', {}, 'radius', {}, 'velocity', {}, 'angle', {});
end

function obstacle = makeObstacle(type, center, sizeOrRadius, velocity, angle, radius)
obstacle.type = type;
obstacle.center0 = center(:);
obstacle.center = center(:);
obstacle.size = sizeOrRadius;
if strcmp(type, 'circle') || strcmp(type, 'triangle')
    obstacle.radius = sizeOrRadius;
else
    obstacle.radius = radius;
end
obstacle.velocity = velocity(:);
obstacle.angle = angle;
end

function [clearance, normal] = obstacleDistance(p, obstacle)
% Returns signed clearance and outward unit normal. Positive means outside.
c = obstacle.center;
switch obstacle.type
    case 'circle'
        delta = p - c;
        d = norm(delta);
        if d < 1e-9
            normal = [1; 0];
        else
            normal = delta / d;
        end
        clearance = d - obstacle.radius;
    case 'triangle'
        % A conservative circular envelope for a triangular obstacle.
        delta = p - c;
        d = norm(delta);
        if d < 1e-9
            normal = [1; 0];
        else
            normal = delta / d;
        end
        clearance = d - obstacle.radius;
    case 'rect'
        halfSize = obstacle.size(:);
        q = p - c;
        outside = max(abs(q) - halfSize, 0);
        outsideDistance = norm(outside);
        inside = all(abs(q) <= halfSize);
        if inside
            margin = halfSize - abs(q);
            [~, axis] = min(margin);
            normal = [0; 0];
            normal(axis) = sign(q(axis));
            if normal(axis) == 0
                normal(axis) = 1;
            end
            clearance = -margin(axis);
        else
            if outsideDistance < 1e-9
                normal = [1; 0];
            else
                normal = outside / outsideDistance;
                normal = normal .* sign(q);
                normal = normal / max(norm(normal), 1e-9);
            end
            clearance = outsideDistance;
        end
    otherwise % capsule: elongated obstacle with circular ends
        halfLength = obstacle.size / 2;
        ca = cos(obstacle.angle);
        sa = sin(obstacle.angle);
        Q = [ca, sa; -sa, ca];
        local = Q * (p - c);
        nearest = [min(max(local(1), -halfLength), halfLength); 0];
        delta = local - nearest;
        d = norm(delta);
        if d < 1e-9
            localNormal = [0; 1];
        else
            localNormal = delta / d;
        end
        normal = [ca, -sa; sa, ca] * localNormal;
        clearance = d - obstacle.radius;
end
end

function value = limitVector(value, limit)
normValue = norm(value);
if normValue > limit
    value = (limit / normValue) * value;
end
end

function plotScenario(result, plotTitle)
figure('Color', 'w', 'Name', plotTitle);
subplot(1, 2, 1);
hold on;
colors = lines(max(result.nGroups, 4));
hAgents = [];
for i = 1:result.n
    g = result.group(i);
    if i == 1
        hAgents = plot(squeeze(result.X(1, i, :)), squeeze(result.X(2, i, :)), ...
            'Color', colors(g, :), 'LineWidth', 1.0);
    else
        plot(squeeze(result.X(1, i, :)), squeeze(result.X(2, i, :)), ...
            'Color', colors(g, :), 'LineWidth', 1.0, 'HandleVisibility', 'off');
    end
end
hTarget = plot(squeeze(result.target(1, 1, :)), squeeze(result.target(2, 1, :)), ...
    'k--', 'LineWidth', 1.2);
hInitial = plot(squeeze(result.X(1, :, 1)), squeeze(result.X(2, :, 1)), ...
    'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 4);
hFinal = plot(squeeze(result.X(1, :, end)), squeeze(result.X(2, :, end)), ...
    'ks', 'MarkerFaceColor', 'g', 'MarkerSize', 4);
grid on;
axis equal;
xlabel('x (m)');
ylabel('y (m)');
title('Quỹ đạo tác tử');

if ~strcmp(result.mode, 'consensus')
    initialObstacles = obstaclesAtTime(0, result.mode);
    for q = 1:numel(initialObstacles)
        drawObstacle(initialObstacles(q));
    end
end
legend([hAgents, hTarget, hInitial, hFinal], ...
    'Agent trajectories', 'Target of agent 1', 'Initial', 'Final', ...
    'Location', 'bestoutside');

subplot(1, 2, 2);
plot(result.time, result.targetError, 'b', 'LineWidth', 1.2);
hold on;
plot(result.time, result.formationError, 'r', 'LineWidth', 1.2);
plot(result.time, result.connectivity, 'k--', 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Metric');
title('Sai số và mức kết nối');
legend('Target error (m)', 'Intra-group formation error (m)', ...
    'Connected edge ratio', 'Location', 'best');

if strcmp(result.mode, 'formation') || strcmp(result.mode, 'large')
    yLimits = ylim;
    plot([8, 8], yLimits, ':k', 'LineWidth', 1.0);
    plot([16, 16], yLimits, ':k', 'LineWidth', 1.0);
end
end

function drawObstacle(obstacle)
c = obstacle.center;
switch obstacle.type
    case {'circle', 'triangle'}
        th = linspace(0, 2*pi, 80);
        r = obstacle.radius;
        plot(c(1) + r*cos(th), c(2) + r*sin(th), 'm-', 'LineWidth', 1.2);
    case 'rect'
        halfSize = obstacle.size(:);
        xx = c(1) + halfSize(1) * [-1, 1, 1, -1, -1];
        yy = c(2) + halfSize(2) * [-1, -1, 1, 1, -1];
        plot(xx, yy, 'm-', 'LineWidth', 1.2);
    otherwise
        th = linspace(0, 2*pi, 50);
        r = obstacle.radius;
        halfLength = obstacle.size / 2;
        ca = cos(obstacle.angle);
        sa = sin(obstacle.angle);
        endpoints = [c + [ca; sa] * halfLength, c - [ca; sa] * halfLength];
        plot(endpoints(1, :), endpoints(2, :), 'm-', 'LineWidth', 3.0);
        plot(c(1) + r*cos(th), c(2) + r*sin(th), 'm:');
end
end

function printSummary(result, label)
window = max(1, round(0.10 * numel(result.time))):numel(result.time);
fprintf('%s: final target error = %.3f m, final formation error = %.3f m, ', ...
    label, result.targetError(end), result.formationError(end));
fprintf('mean connected-edge ratio (last 10%%) = %.2f\n', ...
    mean(result.connectivity(window)));
end

