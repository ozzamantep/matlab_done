function [eta_next, nu_next] = FossenDynamics(eta, nu, tau, current_world, params, dt, bounds)
% FOSSENDYNAMICS 6-DOF AUV hydrodynamic simulation step using RK4 integration.
%   eta = [x; y; z; phi; theta; psi]  (World ENU: x=East, y=North, z=Up, z<=0 underwater)
%   nu  = [u; v; w; p; q; r]          (Body-frame linear & angular velocities)
%   tau = [Fx; Fy; Fz; K; M; N]       (Body-frame thruster forces & torques)
%   current_world = [cx; cy; cz]      (World-frame ocean current velocity, m/s)

    if nargin < 7 || isempty(bounds)
        bounds = [-60, 60, -60, 60, -28, -0.5];
    end

    [k1_eta, k1_nu] = deriv(eta, nu, tau, current_world, params);
    [k2_eta, k2_nu] = deriv(eta + 0.5*dt*k1_eta, nu + 0.5*dt*k1_nu, tau, current_world, params);
    [k3_eta, k3_nu] = deriv(eta + 0.5*dt*k2_eta, nu + 0.5*dt*k2_nu, tau, current_world, params);
    [k4_eta, k4_nu] = deriv(eta + dt*k3_eta, nu + dt*k3_nu, tau, current_world, params);

    eta_next = eta + (dt / 6.0) * (k1_eta + 2*k2_eta + 2*k3_eta + k4_eta);
    nu_next  = nu  + (dt / 6.0) * (k1_nu  + 2*k2_nu  + 2*k3_nu  + k4_nu);

    % Wrap Euler angles to [-pi, pi]
    eta_next(4:6) = atan2(sin(eta_next(4:6)), cos(eta_next(4:6)));
    % Prevent gimbal lock
    eta_next(5) = max(min(eta_next(5), 1.2), -1.2);

    % Arena boundary enforcement (soft bounce)
    if eta_next(1) < bounds(1), eta_next(1) = bounds(1); nu_next(1) = abs(nu_next(1))*0.2; end
    if eta_next(1) > bounds(2), eta_next(1) = bounds(2); nu_next(1) = -abs(nu_next(1))*0.2; end
    if eta_next(2) < bounds(3), eta_next(2) = bounds(3); nu_next(2) = abs(nu_next(2))*0.2; end
    if eta_next(2) > bounds(4), eta_next(2) = bounds(4); nu_next(2) = -abs(nu_next(2))*0.2; end
    if eta_next(3) < bounds(5), eta_next(3) = bounds(5); nu_next(3) = max(0, nu_next(3)); end
    if eta_next(3) > bounds(6), eta_next(3) = bounds(6); nu_next(3) = min(0, nu_next(3)); end
end

function [eta_dot, nu_dot] = deriv(eta, nu, tau, current_world, params)
    phi   = eta(4);
    theta = eta(5);
    psi   = eta(6);

    R = eul2rotm_zyx(phi, theta, psi);

    % Transform ocean current to body frame for relative velocity
    v_curr_body = R' * current_world(:);
    nu_rel = nu;
    nu_rel(1:3) = nu(1:3) - v_curr_body;

    % Coriolis + Centripetal approximation (rigid body)
    m = params.mass;
    v = nu_rel(1:3);
    w = nu_rel(4:6);
    I_w = [params.Ixx*w(1); params.Iyy*w(2); params.Izz*w(3)];
    C_nu = [m * cross(w, v); cross(w, I_w)];

    % Linear + quadratic hydrodynamic drag
    D_nu = (params.D_lin + params.D_quad * diag(abs(nu_rel))) * nu_rel;

    % Restoring forces (Archimedes buoyancy & gravity in ENU z-up)
    % Net upward force in world frame = (B - W)
    net_buoy_world = [0; 0; params.B - params.W];
    f_rest_body = R' * net_buoy_world;

    % Restoring pitch/roll torque due to metacentric height (BG = rb - rg)
    bg_z = params.rb(3) - params.rg(3); % negative in NED, positive separation stabilizes
    k_restore = abs(bg_z) * params.B * 3.5;
    tau_rest_body = [-k_restore * sin(phi); -k_restore * sin(theta); 0];
    g_eta = [-f_rest_body; -tau_rest_body];

    % Clamp thruster forces/torques
    tau_clamped = tau;
    tau_clamped(1) = max(min(tau(1), params.max_force_xy), -params.max_force_xy);
    tau_clamped(2) = max(min(tau(2), params.max_force_xy), -params.max_force_xy);
    tau_clamped(3) = max(min(tau(3), params.max_force_z),  -params.max_force_z);
    tau_clamped(4:6) = max(min(tau(4:6), params.max_torque), -params.max_torque);

    nu_dot = params.Minv * (tau_clamped - C_nu - D_nu - g_eta);

    % Kinematics: eta_dot = J(eta) * nu
    T_ang = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
             0, cos(phi),           -sin(phi);
             0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    eta_dot = [R * nu(1:3); T_ang * nu(4:6)];
end

function R = eul2rotm_zyx(phi, theta, psi)
    cp = cos(phi); sp = sin(phi);
    ct = cos(theta); st = sin(theta);
    cy = cos(psi); sy = sin(psi);

    R = [cy*ct, cy*st*sp - sy*cp, cy*st*cp + sy*sp;
         sy*ct, sy*st*sp + cy*cp, sy*st*cp - cy*sp;
         -st,   ct*sp,            ct*cp];
end
