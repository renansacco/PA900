function state = guidance_init(waypoints, varargin)
% GUIDANCE_INIT  Inicializa o estado interno do Guidance (B-spline).
%
%   state = guidance_init(waypoints)
%   state = guidance_init(waypoints, 'searchRadius', 10, 'wpDistance', 3)
%
%   Input:
%       waypoints — struct com campos:
%           .x (Nx1) — coordenadas x dos waypoints [m]
%           .y (Nx1) — coordenadas y dos waypoints [m]
%
%   Parametros opcionais (name-value):
%       searchRadius       — raio de busca inicial [m] (default: 5.0)
%       wpDistance          — distancia entre waypoints [m] (default: 3.0)
%       divergenceCountMax  — biseccoes falhadas consecutivas (default: 10)
%       divergencePsiMax    — limite de |psiError| [rad] (default: pi/2)
%
%   Output:
%       state — struct com estado interno
%
%   Data: 2026-06-09 | Autor: Renan / Claude

    p = inputParser;
    addRequired(p, 'waypoints');
    addParameter(p, 'searchRadius', 5.0);
    addParameter(p, 'wpDistance', 3.0);
    addParameter(p, 'divergenceCountMax', 10);
    addParameter(p, 'divergencePsiMax', pi/2);
    parse(p, waypoints, varargin{:});

    state.wps = waypoints;
    state.n   = numel(waypoints.x);
    state.wpIndex = 0;          % 0 = not initialized
    state.lastT   = 0.5;        % bisection cache
    state.countDiv = 0;
    state.splineValid = false;
    state.xCoefs = zeros(1,4);  % [a, b, c, d] for x
    state.yCoefs = zeros(1,4);  % [a, b, c, d] for y
    state.isReady = false;
    state.status  = 0;          % 0=Inactive, 1=Active, 2=EndOfPath, 3=Diverged

    % Config
    state.cfg.searchRadius      = p.Results.searchRadius;
    state.cfg.wpDistance         = p.Results.wpDistance;
    state.cfg.divergenceCountMax = p.Results.divergenceCountMax;
    state.cfg.divergencePsiMax   = p.Results.divergencePsiMax;

    if state.n < 2
        return;
    end

    state.isReady = true;
    state.status  = 1;  % Active
end
