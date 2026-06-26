function [sys,x0,str,ts] = sf_guidance(t, x, u, flag, waypoints, Ts)
% sf_guidance  S-Function Level-1 do Guidance (discrete, Ts = sample time).
%
%   Parametros (mask/workspace):
%       waypoints — struct com .x e .y (constante, trajetoria)
%       Ts        — sample time [s]
%
%   Inputs  (3): U = [x, y, psi]
%   Outputs (8): Y = [lateralError, psiError, alpha, curvature, lineIndex, status, t, s]

persistent state

switch flag
    case 0  % Init
        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = 0;
        sizes.NumOutputs     = 8;
        sizes.NumInputs      = 3;
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;
        sys = simsizes(sizes);
        x0  = [];
        str = [];
        ts  = [Ts 0];
        state = guidance_init(waypoints);

    case 3  % Output
        pose = [u(1), u(2), u(3)];
        [out, state] = guidance_step(pose, state);
        sys = [out.lateralError; out.psiError; out.alpha; ...
               out.curvature; out.lineIndex; out.status; out.t; out.s];

    otherwise
        sys = [];
end
end
