function params = parameters()
% PARAMETERS Physical parameters for the Inverted Pendulum on a Cart.
%
% This function returns a struct containing all physical constants,
% geometric dimensions, friction coefficients, actuator constraints,
% and track boundary limits for the benchmark inverted pendulum system.
%
% Outputs:
%   params - Struct containing:
%       .M      : Cart mass [kg]
%       .m      : Pendulum rod mass [kg]
%       .L      : Total pendulum rod length [m]
%       .l      : Distance from pivot to pendulum center of mass [m]
%       .I      : Pendulum moment of inertia about COM [kg*m^2]
%       .b      : Cart viscous friction coefficient [N*s/m]
%       .g      : Acceleration due to gravity [m/s^2]
%       .Fmax   : Actuator peak force saturation limit [N]
%       .track_lim : Maximum horizontal rail travel (+/-) [m]
%       .cart_w : Cart visualization width [m]
%       .cart_h : Cart visualization height [m]
%       .wheel_r: Cart visualization wheel radius [m]

    % Cart mass (kg) - Assumed laboratory bench cart
    params.M = 1.0;

    % Pendulum mass (kg) - Assumed aluminum rod
    params.m = 0.1;

    % Total length of pendulum rod (m)
    params.L = 0.5;

    % Pivot to Center-of-Mass distance (m) (Uniform rod: l = L / 2)
    params.l = params.L / 2.0;

    % Moment of inertia of uniform rod about its COM (kg*m^2):
    % I = (1/12) * m * L^2
    params.I = (1.0 / 12.0) * params.m * (params.L^2);

    % Viscous friction coefficient of the cart on the linear rail (N*s/m)
    params.b = 0.1;

    % Gravitational acceleration (m/s^2)
    params.g = 9.81;

    % Actuator saturation limit (N) - Max continuous motor force
    params.Fmax = 15.0;

    % Track physical travel limits (+/- meters from center)
    params.track_lim = 1.0;

    % Graphical rendering dimensions (meters)
    params.cart_w = 0.20;     % Cart width
    params.cart_h = 0.10;     % Cart height
    params.wheel_r = 0.025;   % Wheel radius
end
