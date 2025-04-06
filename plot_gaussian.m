function plot_gaussian(mu, Sigma, color)
    theta = linspace(0, 2*pi, 100);
    circle = [cos(theta); sin(theta)];  % unit circle

    [V, D] = eig(Sigma);  % eigen decomposition
    ellipse = V * sqrt(D) * circle;

    ellipse = bsxfun(@plus, ellipse, mu(:));  % shift to mean
    plot(ellipse(1,:), ellipse(2,:), 'Color', color, 'LineWidth', 2);
end