function [mu, Sigma, pi_k, gamma, log_likelihoods] = em_gmm(X, K, max_iters, mu_init, idx)
    [N, D] = size(X);
    rng(1);
    
    % Initialize
    mu = mu_init;
    Sigma = zeros(D, D, K);       % DxDxK
    for k = 1:K
        Sigma(:, :, k) = cov(X(idx == k, :)) + 1e-3 * eye(D); % Use k-means clusters
    end
    pi_k = ones(1, K) / K;
    gamma = zeros(N, K);
    log_likelihoods = zeros(max_iters, 1);

    for iter = 1:max_iters
        % E-step
        for k = 1:K
            gamma(:, k) = pi_k(k) * mvnpdf(X, mu(k, :), Sigma(:, :, k));
        end
        gamma = gamma ./ sum(gamma, 2);

        % M-step
        [mu, Sigma, pi_k] = m_step(X, gamma);

        % Log-likelihood
        ll = 0;
        for n = 1:N
            p = 0;
            for k = 1:K
                p = p + pi_k(k) * mvnpdf(X(n, :), mu(k, :), Sigma(:, :, k));
            end
            ll = ll + log(p + eps);
        end
        log_likelihoods(iter) = ll;

        % Convergence
        if iter > 1 && abs(ll - log_likelihoods(iter-1)) < 1e-4
            fprintf("Converged at iteration %d\n", iter);
            break;
        end
    end
end