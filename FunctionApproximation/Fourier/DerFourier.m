function [ad, bd] = DerFourier(a, b, T)
ad = zeros(size(a));

k = (1:length(b))';

ad(2:end) = 2*pi*k/T.*b;
bd = -2*pi*k/T.*a(2:end);

end