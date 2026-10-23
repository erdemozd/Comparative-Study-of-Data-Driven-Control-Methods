function H = block_hankel(data,L)
% BLOCK_HANKEL Standard block Hankel matrix.

[nSignal,T] = size(data);

if L < 1 || L > T
    error('Invalid Hankel depth.');
end

nCol = T-L+1;
H = zeros(nSignal*L,nCol);

for i = 1:L
    rows = (i-1)*nSignal + (1:nSignal);
    H(rows,:) = data(:,i:i+nCol-1);
end

end
