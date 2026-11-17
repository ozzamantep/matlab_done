classdef EKFTargetTracker
% EKFTARGETTRACKER 3D Constant-Velocity Kalman/EKF Target Tracker for Hider AUV.
%   State x = [px; py; pz; vx; vy; vz] in world ENU coordinates.

    properties
        x (6,1) double = zeros(6,1)
        P (6,6) double = eye(6) * 25.0
        Q (6,6) double
        R (3,3) double = diag([0.6^2, 0.6^2, 0.5^2])
        Initialized logical = false
        Confidence double = 0.0
        TimeSinceLastMeas double = 999.0
    end

    methods
        function obj = EKFTargetTracker()
            obj.Q = diag([0.05, 0.05, 0.03, 0.35, 0.35, 0.15]);
        end

        function obj = predict(obj, dt)
            if ~obj.Initialized
                obj.TimeSinceLastMeas = obj.TimeSinceLastMeas + dt;
                return;
            end

            F = [eye(3), dt * eye(3);
                 zeros(3), 0.98 * eye(3)]; % slight velocity damping prior
            obj.x = F * obj.x;
            obj.P = F * obj.P * F' + obj.Q * dt;

            % Clamp estimated Z to underwater bounds
            obj.x(3) = max(min(obj.x(3), -0.5), -28.0);

            obj.TimeSinceLastMeas = obj.TimeSinceLastMeas + dt;
            obj.Confidence = max(0.0, obj.Confidence * exp(-0.18 * dt));
        end

        function obj = update(obj, z_meas)
            z_meas = z_meas(:);
            if any(isnan(z_meas))
                return;
            end

            if ~obj.Initialized
                obj.x = [z_meas; 0; 0; 0];
                obj.P = diag([1.0, 1.0, 1.0, 2.0, 2.0, 1.0]);
                obj.Initialized = true;
                obj.Confidence = 0.75;
                obj.TimeSinceLastMeas = 0.0;
                return;
            end

            H = [eye(3), zeros(3)];
            y = z_meas - H * obj.x;
            S = H * obj.P * H' + obj.R;
            K = (obj.P * H') / S;

            obj.x = obj.x + K * y;
            I6 = eye(6);
            obj.P = (I6 - K * H) * obj.P;

            obj.Confidence = min(1.0, obj.Confidence + 0.35);
            obj.TimeSinceLastMeas = 0.0;
        end
    end
end
