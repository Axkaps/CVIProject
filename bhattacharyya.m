function distance = bhattacharyya(hist1, hist2)

    hist1 = hist1 / sum(hist1);
    hist2 = hist2 / sum(hist2);
    
    bc_coeff = max(sum(sqrt(hist1 .* hist2)), eps);
    distance = -log(bc_coeff);

end