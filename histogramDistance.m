function distance = histogramDistance(h1, h2)

    if length(h1) ~= length(h2)
        error('Histograms must be of the same length.');
    end

    Nh1 = h1 / sum(h1);
    Nh2 = h2 / sum(h2);

    distance = sum(abs(Nh1 - Nh2));
end
