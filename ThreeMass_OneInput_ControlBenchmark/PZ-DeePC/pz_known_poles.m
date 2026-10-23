function prior = pz_known_poles(p,nKnown)
% PZ_KNOWN_POLES Select 2 or 4 nominal discrete poles closest to unit circle.

[A,B,C,D] = three_mass_nominal_linear(p);
sysd = c2d(ss(A,B,C,D),p.Ts,'zoh');

allPoles = eig(sysd.A);

[~,idx] = sort(abs(allPoles),'descend');

if ~ismember(nKnown,[2 4])
    error('nKnown must be 2 or 4.');
end

known = allPoles(idx(1:nKnown));

% Real polynomial coefficients:
%
% prod(z-lambda_i) = z^sp + a1*z^(sp-1)+...+asp

polyCoeff = real(poly(known));

prior.allPoles = allPoles;
prior.knownPoles = known;
prior.sp = nKnown;
prior.polyCoeff = polyCoeff;

end
