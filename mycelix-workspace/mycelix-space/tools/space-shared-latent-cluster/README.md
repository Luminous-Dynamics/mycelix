# Space shared-latent cluster estimator

A dependency-free Rust reference for estimating collision probability marginalized over shared environmental uncertainty, while preserving the dependence created by a common latent environment draw.

## Hierarchical estimand

For each independent cluster k, draw one shared environment Z_k (for example a global atmospheric-density forecast error). Conditional on Z_k, generate a primary trajectory ensemble and a secondary trajectory ensemble under their declared conditional sampling distributions.

For the E0 estimator these two residual ensembles must be conditionally independent given Z_k and IID within their respective ensemble. The cluster contribution is:

    U_k = (1/(m_k*n_k)) * sum_i sum_j h(X_ki, Y_kj; Z_k)

where h is a fixed high-fidelity continuous-time collision kernel over the declared full risk horizon.

The target is the **environment-marginalized probability**:

    Pc = E_Z [ E[h(X,Y;Z) | Z] ]

Under the declared unbiased conditional sampling design, each U_k is in [0,1] and has expectation Pc. With K independent clusters, the estimator is:

    Pc_hat = (1/K) * sum_k U_k

The independent statistical units are the K shared-environment clusters. The within-cluster m_k*n_k pair outcomes are dependent and must not be counted as independent Bernoulli trials.

## One-sided Hoeffding bound

For independent cluster contributions U_k in [0,1], Hoeffding's inequality gives:

    UCB = min(1, Pc_hat + sqrt(ln(1/delta)/(2*K)))

This is a fixed-sample bound across K clusters. It is conservative and distribution-free under the independent-cluster and common-target assumptions. The implementation reports the total propagated trajectories and pair comparisons as computational quantities, separately from the cluster count.

## Conditional versus marginalized risk

This estimator targets risk marginalized over the declared distribution of shared environmental uncertainty. It does not automatically estimate Pc conditional on the one actual unknown density forecast error realized in operations. That conditional risk is a different estimand.

If primary and secondary residuals remain dependent after conditioning on Z (for example additional common random errors, unmodeled cross-correlation or a shared source estimation error), E0 rejects the design. A joint conditional sampler or a further clustered estimator is required.

Caller-supplied IID and independence flags are evidence declarations, not automatic proofs. The shared-latent distribution, conditional sampler, common-cause decomposition, event oracle and sample provenance all require separate qualification.

## Evidence contract

Each cluster retains:
- unique cluster ID and shared-latent draw ID;
- exact shared-latent artifact SHA-256;
- primary and secondary sample-set IDs and trajectory IDs;
- primary and secondary sample-order digests;
- pair-outcome matrix digest;
- pair matrix shape and high-fidelity collision indicators.

The global evidence binds:
- shared-environment model ID/revision/distribution and IID cluster-sampling evidence;
- conditional sampler ID/revision and evidence;
- explicit conditional independence model/evidence for primary and secondary residual ensembles;
- fixed event definition/horizon and high-fidelity continuous-time event-oracle ID/revision;
- cluster-order digest and claim ceiling.

## Benchmarks

- Analytically integrable shared-scalar Gaussian nuisance and 1D/2D collision events.
- Common-mode density bias with same-sign/opposing responses.
- Independent product case, which must agree in expectation with PR #6468's U-statistic target when the shared latent is degenerate.
- Compare independent cluster Hoeffding bound against brute-force fully joint sampling at matched outer and inner propagation budget.
- Vary K, m_k and n_k while preserving the distinction between outer independent clusters and inner dependent pair reuse.
- Adversarial latent reuse, correlated clusters, missing common-cause factors, non-IID outer sampling, conditional residual dependence, duplicate trajectory IDs and event-oracle drift.
- At least 10,000 independent hierarchical repetitions per synthetic stratum, with uncertainty on the empirical coverage estimate.

## Limitations

This is an E0 cluster-level Hoeffding bound. It does not yet implement:
- multi-factor latent distributions with fitted dependence;
- conditional primary-secondary dependence beyond a shared latent;
- adaptive/importance-sampled outer latent distributions;
- a tighter between-/within-cluster empirical-Bernstein bound;
- sequential acquisition across clusters;
- physical orbit propagation or the high-fidelity event oracle.

The result is conditional on the declared environment distribution, independent outer clusters, correct conditional trajectory sampling and a fixed high-fidelity event kernel. It does not validate orbit truth, covariance realism, the actual realized environmental state, complete encounter search or maneuver safety.
