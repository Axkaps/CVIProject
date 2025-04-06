function plotSuccessRates(thresholds, successRates)
    figure;
    plot(thresholds, successRates, 'LineWidth', 2);
    grid on;
    xlabel('IoU Threshold');
    ylabel('Success Rate');
    title('Success Plot for Pedestrian Detection');
    
end