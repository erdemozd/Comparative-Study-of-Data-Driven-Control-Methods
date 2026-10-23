function [Sx,Su] = make_lifted_state_map(A,B,N)
% MAKE_LIFTED_STATE_MAP
%
% Builds a stage-aligned trajectory
%
% X = [x0; x1; ...; x_(N-1)] = Sx*x0 + Su*U
%
% U = [u0; ...; u_(N-1)]
%
% The final u_(N-1) does not affect the stored state sequence because
% x_N is not included; this matches a stage cost k=0,...,N-1.

nx = size(A,1);
nu = size(B,2);

Sx = zeros(nx*N,nx);
Su = zeros(nx*N,nu*N);

Sx(1:nx,:) = eye(nx);

for i = 2:N
    rows = (i-1)*nx + (1:nx);
    prev = (i-2)*nx + (1:nx);

    Sx(rows,:) = A*Sx(prev,:);
    Su(rows,:) = A*Su(prev,:);

    ucols = (i-2)*nu + (1:nu);
    Su(rows,ucols) = Su(rows,ucols) + B;
end

end
