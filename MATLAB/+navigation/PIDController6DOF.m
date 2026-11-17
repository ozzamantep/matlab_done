classdef PIDController6DOF
% PIDCONTROLLER6DOF 6-DOF AUV motion controller (Surge, Sway, Heave, Roll, Pitch, Yaw).
%   Converts desired world-frame velocity/target into body-frame thruster forces/torques.

    properties
        Kp_vel = [32.0; 32.0; 28.0]
        Ki_vel = [4.0; 4.0; 5.0]
        Kp_att = [8.0; 8.0; 10.0]
        Kd_att = [2.5; 2.5; 3.5]
        IntErr (3,1) double = zeros(3,1)
    end

    methods
        function [tau, obj] = computeControl(obj, eta, nu, v_des_world, yaw_des, dt)
            phi   = eta(4);
            theta = eta(5);
            psi   = eta(6);

            R = eul2rotm_zyx(phi, theta, psi);
            v_des_body = R' * v_des_world(:);

            % Velocity error in body frame
            e_vel = v_des_body - nu(1:3);
            obj.IntErr = max(min(obj.IntErr + e_vel * dt, 5.0), -5.0);

            f_body = obj.Kp_vel .* e_vel + obj.Ki_vel .* obj.IntErr;

            % Attitude stabilization (Roll=0, Pitch=0, Yaw=yaw_des)
            e_roll  = wrapAngle(-phi);
            e_pitch = wrapAngle(-theta);
            e_yaw   = wrapAngle(yaw_des - psi);

            tau_att = obj.Kp_att .* [e_roll; e_pitch; e_yaw] - obj.Kd_att .* nu(4:6);
            tau = [f_body; tau_att];
        end
    end
end

function a = wrapAngle(a)
    a = atan2(sin(a), cos(a));
end

function R = eul2rotm_zyx(phi, theta, psi)
    cp = cos(phi); sp = sin(phi);
    ct = cos(theta); st = sin(theta);
    cy = cos(psi); sy = sin(psi);
    R = [cy*ct, cy*st*sp - sy*cp, cy*st*cp + sy*sp;
         sy*ct, sy*st*sp + cy*cp, sy*st*cp - cy*sp;
         -st,   ct*sp,            ct*cp];
end
