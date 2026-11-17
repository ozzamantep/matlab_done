classdef DepthIMUDVL
% DEPTHIMUDVL Simulates hydrostatic Depth/Pressure, 9-DOF IMU, and 4-beam DVL.

    properties
        DepthNoiseStd = 0.05;     % m
        VelNoiseStd   = 0.03;     % m/s (DVL)
        AttNoiseStd   = 0.008;    % rad (IMU)
        RateNoiseStd  = 0.01;     % rad/s (Gyro)
    end

    methods
        function meas = read(obj, eta, nu)
            meas.depth = -eta(3) + obj.DepthNoiseStd * randn();
            meas.pressure_kpa = 101.325 + 1025.0 * 9.81 * max(0, meas.depth) / 1000.0;
            meas.orientation = eta(4:6) + obj.AttNoiseStd * randn(3, 1);
            meas.angular_vel = nu(4:6) + obj.RateNoiseStd * randn(3, 1);
            meas.dvl_vel_body = nu(1:3) + obj.VelNoiseStd * randn(3, 1);
        end
    end
end
