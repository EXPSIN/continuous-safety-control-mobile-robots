function r_L = regularization_MY(A_L, r, l)
N   = size(A_L, 1);
r_L = zeros(  N, 1);
for i = 1:N
    r_L(i, 1) = min(r + vecnorm(A_L(i, :)'-l).^2 );
end
end

function  res = limit(res, lower, upper)
res = min(max(res, lower), upper);
end