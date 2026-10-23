%% Quick HDeePC Gfull consistency test

Gfull = ctrl.Gfull;
Yf    = ctrl.Yf;
Uf    = ctrl.Uf;

Ak = ctrl.Ak;
Ay = ctrl.Ay;
Bk = ctrl.Bk;

N  = ctrl.N;

fprintf('\n');
fprintf('Gfull consistency test\n');
fprintf('----------------------\n');

%% 1. Check unknown/data-driven part

maxYuError = 0;

for i = 1:N

    rowsFull = (i-1)*6 + (1:6);
    rowsYu   = (i-1)*4 + (1:4);

    err = norm( ...
        Gfull(rowsFull(3:6),:) ...
        - Yf(rowsYu,:), ...
        'fro');

    maxYuError = max(maxYuError,err);

end

fprintf('Max Yf/Gfull mismatch      : %.3e\n',maxYuError);

%% 2. Check known-state recursion

maxRecursionError = 0;

for i = 1:N-1

    rowsFull_i   = (i-1)*6 + (1:6);
    rowsFull_ip1 = i*6     + (1:6);

    rowsYu = (i-1)*4 + (1:4);

    Ki = ...
        Gfull(rowsFull_i(1:2),:);

    Kip1 = ...
        Gfull(rowsFull_ip1(1:2),:);

    Yui = ...
        Yf(rowsYu,:);

    Ui = ...
        Uf(i,:);

    Kip1_expected = ...
        Ak*Ki ...
        + ...
        Ay*Yui ...
        + ...
        Bk*Ui;

    err = ...
        norm(Kip1-Kip1_expected,'fro');

    maxRecursionError = ...
        max(maxRecursionError,err);

end

fprintf('Max recursion mismatch     : %.3e\n',maxRecursionError);

%% 3. Show first few block norms

fprintf('\nFirst few Gfull blocks\n');
fprintf('----------------------\n');

for i = 1:min(3,N)

    rowsFull = (i-1)*6 + (1:6);

    Ki = ...
        Gfull(rowsFull(1:2),:);

    Yui = ...
        Gfull(rowsFull(3:6),:);

    fprintf( ...
        'Stage %d: ||K_i||_F = %.6e, ||Y_u,i||_F = %.6e\n', ...
        i, ...
        norm(Ki,'fro'), ...
        norm(Yui,'fro'));

end