function [A, B] = linearizar_veiculo(Xe, Ue, params)
% LINEARIZAR_VEICULO  Numerical linearization of the vehicle plant model.
%
%   [A, B] = linearizar_veiculo(Xe, Ue, params)
%
%   Computes the Jacobian matrices A and B of dinamica_veiculo around the
%   equilibrium point (Xe, Ue) using central finite differences.
%
%       dx/dt = f(X, U, params)
%       A = df/dX |_(Xe,Ue)    (7x7)
%       B = df/dU |_(Xe,Ue)    (7x2)
%
%   Inputs:
%       Xe     - equilibrium state vector  (7x1)
%       Ue     - equilibrium input vector  (2x1)
%       params - parameter struct (see dinamica_veiculo)
%
%   Outputs:
%       A - state Jacobian  (7x7)
%       B - input Jacobian  (7x2)
%
%   Date:   2026-06-09
%   Author: Renan / Claude

    Xe = Xe(:);
    Ue = Ue(:);

    nx = numel(Xe);
    nu = numel(Ue);

    eps_x = 1e-10;
    eps_u = 1e-10;

    % --- State Jacobian A ---
    A = zeros(nx, nx);
    for i = 1:nx
        dX = zeros(nx, 1);
        dX(i) = eps_x;
        A(:, i) = (dinamica_veiculo(Xe + dX, Ue, params) ...
                  - dinamica_veiculo(Xe - dX, Ue, params)) / (2 * eps_x);
    end

    % --- Input Jacobian B ---
    B = zeros(nx, nu);
    for i = 1:nu
        dU = zeros(nu, 1);
        dU(i) = eps_u;
        B(:, i) = (dinamica_veiculo(Xe, Ue + dU, params) ...
                  - dinamica_veiculo(Xe, Ue - dU, params)) / (2 * eps_u);
    end

end
