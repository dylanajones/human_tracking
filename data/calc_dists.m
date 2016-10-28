% Function to compute the distance from a line

function [dists] = calc_dists(y, dist)
    
    n = length(y);
    dists = zeros(n,1);

    for i = 1:n
        dists(i) = abs(y(i) - dist);
    end

end