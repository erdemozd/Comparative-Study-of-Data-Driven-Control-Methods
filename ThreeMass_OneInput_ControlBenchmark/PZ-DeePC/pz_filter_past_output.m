function yPrime = pz_filter_past_output(yPast,polyCoeff,ny)

yPast = yPast(:);

p = numel(yPast)/ny;
sp = numel(polyCoeff)-1;

Y = reshape(yPast,ny,p);

Yp = zeros(ny,p-sp);

for k = sp+1:p

    val = Y(:,k);

    for j = 1:sp
        val = val + polyCoeff(j+1)*Y(:,k-j);
    end

    Yp(:,k-sp) = val;
end

yPrime = Yp(:);

end
