M = mmread('matrix.mtx');
n = size(M, 1);
[V,D] = eigs(M, min(20, n), 'LM');
D = diag(D); % make it a vector
mmwrite('eigvecs.mtx', sparse(V));
mmwrite('eigvals.mtx', sparse(D));
