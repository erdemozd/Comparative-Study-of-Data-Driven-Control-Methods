function r = build_reference(t)
% BUILD_REFERENCE Common piecewise-constant q3 reference.

r = zeros(size(t));

r(t >= 2  & t < 8 ) =  0.60;
r(t >= 8  & t < 14) = -0.45;
r(t >= 14 & t < 20) =  0.75;
r(t >= 20 & t < 26) = -0.50;
r(t >= 26) = 0.00;

end
