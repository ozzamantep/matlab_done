function params = BlueROV2Params()
% BLUEROV2PARAMS Returns 6-DOF physical and hydrodynamic parameters for BlueROV2 Heavy.
%   Matches the Fossen 6-DOF marine craft model:
%   M * nu_dot + C(nu)*nu + D(nu)*nu + g(eta) = tau + tau_current

    params.mass = 11.5;                 % kg
    params.gravity = 9.81;              % m/s^2
    params.rho = 1025.0;                % kg/m^3 (seawater)
    params.volume = 0.0114;             % m^3 (slightly positive buoyancy ~0.18 kgf)
    params.W = params.mass * params.gravity;
    params.B = params.rho * params.gravity * params.volume;

    % Center of gravity and buoyancy in body frame (NED/FRD)
    params.rg = [0; 0; 0.02];           % m
    params.rb = [0; 0; -0.02];          % m (buoyancy above CG for passive stability)

    % Rigid-body inertia tensor (kg*m^2)
    params.Ixx = 0.16;
    params.Iyy = 0.16;
    params.Izz = 0.28;

    % Added mass diagonal terms (kg, kg*m^2)
    params.Ma = diag([5.5, 12.7, 14.57, 0.12, 0.12, 0.12]);

    % Rigid-body mass matrix
    m = params.mass;
    params.Mrb = diag([m, m, m, params.Ixx, params.Iyy, params.Izz]);
    params.M = params.Mrb + params.Ma;
    params.Minv = inv(params.M);

    % Linear hydrodynamic damping (skin friction)
    params.D_lin = diag([4.03, 6.22, 5.18, 0.07, 0.07, 0.07]);

    % Quadratic hydrodynamic damping (form drag)
    params.D_quad = diag([18.18, 21.66, 36.99, 1.55, 1.55, 1.55]);

    % Thruster limits (6-DOF vectored BlueROV2 Heavy)
    params.max_force_xy = 40.0;         % N per axis
    params.max_force_z  = 35.0;         % N vertical
    params.max_torque   = 12.0;         % N*m

    % Max linear/angular speeds for reference
    params.max_speed = 2.2;             % m/s
end
