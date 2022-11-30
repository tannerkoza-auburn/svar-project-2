%% State Variable Project - Individual Work

clear
clc
close all

%% Problem 1

A = [-1 1 1; 2 -1 2; -2 -1 -4];
B = [0; -1; 1];

% Part 1
s = sym('s');
W = sym('W', [3 3]);

sIA = s * eye(3) - A;
ceOL = det(sIA);
eigs = solve(ceOL);

eqn = A * W + W * A' == -B * B';
W = solve(eqn);

% Part 2
[V, D] = eig(A');
N = null(B');

syms a b

for i = 1:size(V,1)

eqn = a*N(:,1) + b*N(:,2) == V(:,i);

[a_, b_] = solve(eqn);

if ~isempty(a_) || ~isempty(b_)
    disp('Eigenvector in nullspace! Not Controllable.')
else
    disp('Eigenvector is not in nullspace!')
end

end

%% Problem 2

A = [-1 0 0 0; 0 2 0 0; 0 0 -3 0; 0 0 0 4];
B = [0; 1; 0; 1];
C = [0 1 0 1];

[V, D] = eig(A);
co = ctrb(A,B);

ob = obsv(A, C)
