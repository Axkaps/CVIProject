function [mu, Sigma, pi_k] = m_step(X, gamma)
    [N, D] = size(X);
    K = size(gamma, 2);

    mu = zeros(K, D);
    Sigma = zeros(D, D, K);
    pi_k = zeros(1, K);

    for k = 1:K
        Nk = sum(gamma(:, k));
        mu(k, :) = sum(gamma(:, k) .* X) / Nk;

        diff = X - mu(k, :);
        Sigma_k = zeros(D, D);
        for n = 1:N
            Sigma_k = Sigma_k + gamma(n, k) * (diff(n, :)' * diff(n, :));
        end
        Sigma(:, :, k) = Sigma_k / Nk + 1e-6 * eye(D);  % regularization
        pi_k(k) = Nk / N;
    end
end