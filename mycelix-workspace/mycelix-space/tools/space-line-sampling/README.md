# Gaussian line-sampling reference

A dependency-free Rust reference for **conditional Gaussian line integration** in a whitened standard-normal state space.

This crate integrates supplied collision intervals along a fixed Gaussian direction. It does not discover intervals by propagating orbital states; a separately qualified high-fidelity line-event solver must supply every interval.

## Conditional decomposition

For Z~N(0,I_d), let a be a unit line direction and B an orthonormal basis for its orthogonal complement. Then:

    Z = B V + a T
    V ~ N(0,I_(d-1)), T ~ N(0,1), V independent of T

For a transverse draw v, suppose the collision set along T is a union of all disjoint intervals:

    A_v = union_l [lower_l(v), upper_l(v)]

The conditional event probability is:

    g(v) = sum_l [Phi(upper_l(v)) - Phi(lower_l(v))]

The unconditional collision probability is:

    Pc = E_V[g(V)]

The reference computes each g(v) by one-dimensional Gaussian integration. It can handle multiple disconnected intervals on the same line, including one-sided and unbounded intervals.

## Statistical estimator

Given K IID transverse samples V_k drawn from the standard-normal complement distribution:

    Pc_hat = (1/K) sum_k g(V_k)

Each g(V_k) lies in [0,1]. A one-sided fixed-sample Hoeffding upper bound is:

    UCB = min(1, Pc_hat + sqrt(ln(1/delta)/(2*K)))

The independent statistical unit is the transverse line, not the number of intervals, interval endpoints, or root evaluations.

This is a Rao-Blackwellized estimator relative to sampling T and recording a binary collision indicator, provided the complete conditional interval set is supplied. The reduction in variance follows from conditional expectation; the actual computational advantage depends on the cost of complete interval discovery.

## Interval completeness is the critical model obligation

A single sign-change bracket can miss:
- tangencies with no sign change;
- narrow collision intervals;
- multiple disconnected collision intervals;
- local minima between coarse time samples;
- intervals introduced by discontinuous dynamics or maneuvers.

The evidence therefore requires that the interval solver enumerate all disconnected intervals, declare the interval sets complete and report no unresolved lines. If this cannot be established, the line must not be silently treated as a non-collision. Boundary sentinels from #6466 should qualify the high-fidelity interval solver separately.

## Numerical normal-tail calculation

The standard-normal survival function is evaluated using an erf power series near the center and a Laplace continued fraction in the tails. Interval masses are calculated as differences of survival probabilities to avoid cancellation between CDF values close to one.

Finite tail arguments above 37 are rejected rather than silently underflowed to zero. This E0 implementation is qualified only over its tested numerical range; it is not a certified interval-arithmetic implementation.

## Evidence contract

The report binds:
- whitening transform ID/revision;
- line direction and complement basis IDs/digests;
- direction-training and transverse-evaluation split IDs;
- transverse sampling design/evidence and sample-order digest;
- line interval-solver ID/revision;
- high-fidelity continuous-time event-oracle ID/revision;
- event definition and full risk horizon;
- every line contribution and the fixed-sample confidence bound.

The direction/basis must be frozen before the independent transverse evaluation set. The direction-training and evaluation split IDs must differ. IID sampling and interval completeness flags are caller attestations, not automatic proofs.

## Benchmarks

- 2D isotropic Gaussian disk with exact probability Pc = 1 - exp(-R^2/2).
- Offset and anisotropic disks with independent quadrature references.
- No-event, full-line event, one-interval and multiple-disjoint-interval fixtures.
- Tail intervals such as [5,6] that would be harmed by naive CDF subtraction.
- Matched variance comparison with direct pointwise indicator Monte Carlo.
- Compare direction choices learned from a training split, fixed geometry directions and multiple direction mixtures.
- Full end-to-end cost includes all high-fidelity boundary searches and interval completeness sentinels.

## Relation to other rare-event methods

Compare under matched target and compute budget with:
- crude Monte Carlo;
- defensive Gaussian-mixture importance sampling (#6465);
- cross-entropy GMM methods (Ketzner & Elgohary, 2026): https://doi.org/10.2514/1.G009204
- ESA/UC3M adaptive importance sampling and multi-level splitting (2023): https://conference.sdo.esoc.esa.int/proceedings/neosst2/paper/46
- subset simulation and line sampling for near-Earth-object impact probability (adjacent, not directly satellite conjunction): https://doi.org/10.1007/s10569-020-09981-5

Subset simulation samples along MCMC/SMC chains are dependent. Its conditional level counts must not be fed into IID binomial intervals without a valid MCMC error analysis.

## Claim ceiling

The E0 estimator qualifies integration of the supplied complete interval sets and statistical behavior across IID transverse lines. It does not prove that the line solver found all collision intervals, that the orbital target distribution is correct, or that the high-fidelity event oracle is physically true. It does not establish operational safety or flight readiness.
