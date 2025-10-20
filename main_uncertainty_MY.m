clear all; clc; close all;

% Simulation configurations
Time = 17;
T    = 100e-3;
t    = 0:T:Time;
N    = length(t);

% The maximum ranging radius
D_B   = 10.0;
D     = 0.48;

% Initial states
p0   = [-2.0; 0.5];
R0   = [1, 0; 0, 1];
z0   = zeros(2, 1);

% States buffers
p    = zeros(2, N);
R    = zeros(2, 2, N);

% The environments
env.obs  = imread('sim_map.bmp');
env.N    = 100;
env.p_e  = [-2; 3];

% Graphics
H = graphic_MY([], 0, p0, R0, zeros(6, 1), env, D_B, 0);

for ith_controller = 1:3
    x = [p0; R0(:); z0];

    for i = 1:N
        % Solve the differential equations
        [dx, v] = closed_loop_dyanmics(x, t(i), env, D_B, ith_controller);
        x  = x + dx*T;

        % update the states
        p(:, i)    = x(1:2,   1);
        [U, ~, V]  = svd(reshape(x(3:6, 1), 2, 2));
        R(:, :, i) = U * V';
        x(3:6, 1)  = reshape(R(:, :, i), [], 1);

        % Update the graphic
        H = graphic_MY(H, t(i), p(:, i), R(:, :, i), dx, env, D_B, ith_controller);
    end
end

H.fig11_obs_or_re(1).Visible = false;
H.fig11_obs_or_re(2).Visible = false;
H.fig11_obs_or_re(3).Visible = false;


figure(11);
set(H.fig11_gca(1), 'xlim', [-10, 10], 'ylim', [-2.1, 3.1]);
H_fig11_gca1_zoom = copyobj(H.fig11_gca(1), gcf);
axis(H_fig11_gca1_zoom, [-0.3, 0.7, -0.5, 0.5]); 
set(H_fig11_gca1_zoom, 'position', [0.1,0.705,0.3,0.26], 'XLabel', [], 'YLabel', [], 'XAxisLocation', 'origin', 'YAxisLocation', 'origin', 'XTick', [], 'YTick', []); 
H.fig11_gca(1).XLabel.Position(2)=-1.1;

set(H.fig11_gca(2), 'xlim', [-10, 10], 'ylim', [-2.1, 3.1]);
H_fig11_gca2_zoom = copyobj(H.fig11_gca(2), gcf);
axis(H_fig11_gca2_zoom, [-0.3, 0.7, -0.5, 0.5]); 
set(H_fig11_gca2_zoom, 'position', [0.1,0.4,0.3,0.26], 'XLabel', [], 'YLabel', [], 'XAxisLocation', 'origin', 'YAxisLocation', 'origin', 'XTick', [], 'YTick', []); 
H.fig11_gca(2).XLabel.Position(2)=-1.1;

set(H.fig11_gca(3), 'xlim', [-10, 10], 'ylim', [-2.1, 3.1]);
H_fig11_gca3_zoom = copyobj(H.fig11_gca(3), gcf);
axis(H_fig11_gca3_zoom, [-0.3, 0.7, -0.5, 0.5]); 
set(H_fig11_gca3_zoom, 'position', [0.1,0.095,0.3,0.26], 'XLabel', [], 'YLabel', [], 'XAxisLocation', 'origin', 'YAxisLocation', 'origin', 'XTick', [], 'YTick', []); 
H.fig11_gca(3).XLabel.Position(2)=-1.1;

function [dx, v] = closed_loop_dyanmics(x, t, env, D_B, ith_controller)
% reformulate the state form x.
n = 2;
p = x(1:n,   1);
R = reshape(x((n+1):6, 1), n, n);
z = x(7:end,   1);

% Nominal controller 
v_n    = [3; 0];

if(ith_controller == 1)
    % All-obstacle avoidance safety controller
    [r, l] = measurements_2D(p, eye(2), env, D_B);      % Feedback
    v      = safety_controller_all(r, l, v_n);          % Safety controller
elseif(ith_controller == 2)
    % Safety controller with Moreau-Yosida regularization
    [r, l] = measurements_2D(p, R, env, D_B);           % Feedback
    v      = safety_controller_MY(r, l, v_n);           % Safety controller
elseif(ith_controller == 3)
    % Proposed safety controller
    [r, l] = measurements_2D(p, R, env, D_B);           % Feedback
    v      = safety_controller_mine(r, l, v_n);         % Safety controller
else
    error('No such ith_controller.');
end


dz  = m_speed_dynamics(z, v);
va  = m_speed_output(z, v);

% Angular velocity matrix, which is control input and unkown/uncontrollable.
% theta_e = -2 * ( atan2(-R(1,2), R(1, 1)) - 0*atan2(v(2), v(1))+  pi/2 );
% omega = atan2(sin(theta_e), cos(theta_e));
omega = 0;
Omega = [0, -1; 1, 0] * omega;
tmp   = R*Omega;

% Translational and rotational kinematics
dx    = [R*va; tmp(:); dz];

end

function y = alpha(s)
y = s;
end

function v = safety_controller_all(r, l_w, v_n)
persistent opt
D = evalin('base', 'D');
if(isempty(opt))
    opt   = optimoptions('quadprog', 'display', 'off');
end

A   = l_w';
b   = alpha(r' - D);

[v, ~,exitflag,~,~] = quadprog(eye(2), -v_n, A, b, [],[],[],[],[],opt);

if(exitflag == -2)
    v = v_n*0;
end
end


function v = safety_controller_mine(r, l, v_n)
persistent opt A_L c_A

D = evalin('base', 'D');


n_L = 9;
theta_bias = pi/n_L;
if(isempty(opt))
    opt   = optimoptions('quadprog', 'display', 'off');
    angle = linspace(0, 2*pi, n_L+1);
    A_L   = [cos(angle(2:end)+theta_bias); sin(angle(2:end)+theta_bias)]';
    c_A   = cos(2*pi/n_L);
end

r_L = regularization(A_L, r, l, c_A);
b   = alpha(r_L - D);

[v, ~,exitflag,~,~] = quadprog(eye(2), -v_n, A_L, b, [],[],[],[],[],opt);


% H = evalin('base', 'H');
% H.fig1_feasible_set = half_plane(H.fig1_feasible_set,  A_L, b, [0;0]);

if(exitflag == -2)
    v = v_n*0;
end

end





function v = safety_controller_MY(r, l, v_n)
persistent opt A_L c_A
D = evalin('base', 'D');

n_L = 9;
theta_bias = pi/n_L;
if(isempty(opt))
    opt   = optimoptions('quadprog', 'display', 'off');
    angle = linspace(0, 2*pi, n_L+1);
    A_L   = [cos(angle(2:end)+theta_bias); sin(angle(2:end)+theta_bias)]';
    c_A   = cos(2*pi/n_L);
end

r_L = regularization_MY(A_L, r, l);
b   = alpha(r_L - D);

[v, ~,exitflag,~,~] = quadprog(eye(2), -v_n, A_L, b, [],[],[],[],[],opt);

% H = evalin('base', 'H');
% H.fig1_feasible_set = half_plane(H.fig1_feasible_set,  A_L, b, [0;0]);

if(exitflag == -2)
    v = v_n*0;
end

end


function dz = m_speed_dynamics(z, v_r)
T        = 0.2;
coupling =  -8;
A = [-1/T,    0; -coupling, -1/T];
B = [ 1/T,    0;  coupling,  1/T];
dz = A*z + B*v_r;
end

function v = m_speed_output(z, v_r)
C = eye(2);
D = zeros(2);
v = C*z + D*v_r;
end