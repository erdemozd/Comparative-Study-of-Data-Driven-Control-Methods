function uData = generate_excitation(Tdata,amplitude)
% GENERATE_EXCITATION Deterministic multilevel PE-oriented signal.

levels = [ ...
     1.00, -1.00, 0.50, -0.50, ...
     0.80, -0.20, 0.30, -0.90, ...
     0.65, -0.35, 0.15, -0.70];

dwell = [2 5 3 7 4 1 6 3 8 2 4 5];

base = [];

for i = 1:numel(levels)
    base = [base, levels(i)*ones(1,dwell(i))]; %#ok<AGROW>
end

uData = zeros(1,Tdata);

idx = 1;
cycle = 1;

while idx <= Tdata

    if mod(cycle,2)==1
        sig = base;
    else
        sig = -base;
    end

    nCopy = min(numel(sig),Tdata-idx+1);

    uData(idx:idx+nCopy-1) = sig(1:nCopy);

    idx = idx+nCopy;
    cycle = cycle+1;
end

envelope = linspace(0.8,1.2,Tdata);

uData = amplitude*envelope.*uData;

end
