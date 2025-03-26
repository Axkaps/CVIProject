function dist = bhattacharyya(hist1, hist2)
    dist = sqrt(1 - sum(sqrt(hist1 .* hist2)));
end