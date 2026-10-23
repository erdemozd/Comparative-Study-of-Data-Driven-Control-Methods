function Qy = pz_filter_matrix(ny,L,polyCoeff)
% PZ_FILTER_MATRIX
%
% If
%   P(z)=z^sp+a1*z^(sp-1)+...+asp
%
% then
%   y'_k = y_k + a1*y_(k-1)+...+asp*y_(k-sp).

sp = numel(polyCoeff)-1;

Qscalar = zeros(L-sp,L);

for row = 1:(L-sp)

    k = row+sp;

    % Current sample coefficient = 1.
    Qscalar(row,k) = 1;

    for j = 1:sp
        Qscalar(row,k-j) = polyCoeff(j+1);
    end

end

Qy = kron(Qscalar,eye(ny));

end
