function r_L = regularization(A_L, r, l, c_A)
N   = size(A_L, 1);
r_L = zeros(  N, 1);
for i = 1:N
    r_L(i, 1) = min(r + 2*limit(c_A - A_L(i, :)*l, 0, inf));
end
end

function  res = limit(res, lower, upper)
res = min(max(res, lower), upper);
end