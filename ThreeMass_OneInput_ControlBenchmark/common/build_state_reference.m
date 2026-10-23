function xRef = build_state_reference(rPreview,rDotPreview)
% BUILD_STATE_REFERENCE
%
% Coordinated equilibrium:
%   [q1 v1 q2 v2 q3 v3] = [r 0 r 0 r 0]
%
% q1/q2 position weights are zero by default, but keeping the physically
% correct equilibrium reference makes the formulation transparent.

N = numel(rPreview);
xRef = zeros(6*N,1);

for i = 1:N
    rows = (i-1)*6 + (1:6);
    xRef(rows) = [ ...
        rPreview(i);
        0;
        rPreview(i);
        0;
        rPreview(i);
        rDotPreview(i)];
end

end
