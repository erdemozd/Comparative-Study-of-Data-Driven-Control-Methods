function [M,d] = pz_unfilter_map(yPast,polyCoeff,N,ny)
% PZ_UNFILTER_MAP
%
% Inverse recursion:
%
% y_k = y'_k - a1*y_(k-1)-...-asp*y_(k-sp)

yPast = yPast(:);

p = numel(yPast)/ny;
sp = numel(polyCoeff)-1;

if p < sp
    error('Past horizon must be at least sp.');
end

Ypast = reshape(yPast,ny,p);

nF = ny*N;

M = zeros(nF,nF);
d = zeros(nF,1);

% Store affine representations for last sp raw samples.
Rhist = cell(sp,1);
dhist = cell(sp,1);

for j = 1:sp
    Rhist{j} = zeros(ny,nF);
    dhist{j} = Ypast(:,p-sp+j);
end

for k = 1:N

    rows = (k-1)*ny + (1:ny);

    E = zeros(ny,nF);
    E(:,rows) = eye(ny);

    Rk = E;
    dk = zeros(ny,1);

    % polyCoeff(j+1) multiplies y_(current-j).
    for j = 1:sp

        histIndex = sp-j+1;

        Rk = Rk - polyCoeff(j+1)*Rhist{histIndex};
        dk = dk - polyCoeff(j+1)*dhist{histIndex};
    end

    M(rows,:) = Rk;
    d(rows) = dk;

    if sp > 1
        Rhist(1:end-1) = Rhist(2:end);
        dhist(1:end-1) = dhist(2:end);
    end

    Rhist{end} = Rk;
    dhist{end} = dk;
end

end
