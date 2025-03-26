function dist = bhattacharyya(hist1, hist2)
    dist = sqrt(max(0, 1 - sum(sqrt(hist1 .* hist2))));
end