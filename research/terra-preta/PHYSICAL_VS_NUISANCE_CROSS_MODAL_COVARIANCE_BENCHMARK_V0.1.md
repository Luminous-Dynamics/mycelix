# Physical vs Nuisance Cross-Modal Covariance Benchmark v0.1

**Status:** statistical design specification and synthetic controls. No empirical covariance was estimated and no physical sample/instrument study is authorized.

**Tracker:** [TERRA-COV-001 / #5651](https://github.com/Luminous-Dynamics/mycelix/issues/5651)

**Dependencies:** #5632 (covariance-aware inference/SBC), #5633 (multi-world paired design), #5643 (CT/tracer support operators), #5649 (correlated sequential histories), #5648 (robust/posterior-SBC qualification), #5650 (CT-to-tensor physical validation).

## 1. The attribution problem

A joint covariance matrix can correctly reproduce dependence while assigning that dependence to the wrong cause.

For CT and tracer measurements, dependence may arise because both modalities observe the same physical pore system. It may also arise because the measurements share a specimen, preparation batch, instrument, calibration, reconstruction method, operator, location or time window.

These mechanisms have different consequences for experimental design:

- Shared physical structure can make a complementary cross-modal pair scientifically informative.
- Shared nuisance effects can make the pair statistically redundant or create false confidence about the physical mechanism.
- An unrestricted full covariance matrix may predict the pair well but cannot by itself distinguish the two explanations.

The benchmark must score *total predictive dependence* and *mechanism attribution* separately.

## 2. Conceptual hierarchical observation model

For modality (m), specimen (i), batch (b), instrument (j), spatial support (s) and time (t):

```text
Y_m(i,b,j,s,t)
 = H_m(Z_i(s,t); support_m, operator_m)
 + U_physical(i,s,t)
 + B_batch(b)
 + I_instrument(m,j)
 + O_operator_m(operator_m)
 + S_spatial(s)
 + T_temporal(t)
 + E_m(i,b,j,s,t)
```

This is a decomposition template, not an assumption that every term is additive or Gaussian. Each component must declare:
- physical meaning / nuisance role;
- modality loading or incidence matrix;
- specimen, batch, instrument, spatial and temporal support;
- units and variance/covariance parameterization;
- whether it is shared or modality-specific;
- parameter-identifiability assumptions.

Do not collapse all dependence into one pairwise correlation and then call it physical complementarity.

## 3. Synthetic worlds

### Required mechanism families

1. **Physical shared state only:** CT and tracer depend on a common latent physical feature; nuisance terms are independent.
2. **Batch-only dependence:** no shared physical signal, but the same sample-preparation batch affects both measurements.
3. **Instrument-only drift:** repeated measurements on one instrument correlate over time; separate instruments have independent drift.
4. **Physical plus batch:** both mechanisms coexist.
5. **Operator/reconstruction dependence:** shared segmentation/reconstruction errors affect CT descriptors; tracer has distinct measurement error.
6. **Spatial-support mismatch:** same core ID but non-coincident CT volume and tracer collection region.
7. **Nested repetition:** multiple slices/timepoints within one core are repeated observations, not independent treatment units.
8. **Crossed design:** specimens are measured across instruments/operators/batches, permitting selected variance components to be separated.
9. **Single-pair non-identifiability:** one CT/tracer pair cannot separate physical and nuisance covariance without added assumptions or replication.
10. **Omitted physics:** a missing transport mechanism must not be silently absorbed into a batch random effect.

Include both positive and negative covariance loadings and deliberately misspecified nuisance structures. Hidden synthetic component realizations remain evaluator-only.

## 4. Replication and identifiability designs

Compare the following data-collection structures:

- one CT–tracer pair on one core;
- repeat measurements on the same core and same instrument;
- same core measured by crossed instruments/operators;
- multiple independent cores from a shared source batch;
- specimens crossed across preparation batches, where scientifically feasible;
- spatially separated supports within one assignment unit;
- independent assignment units.

For each design, calculate the covariance-component design matrix/rank and assess parameter identifiability. Report:
- identified components;
- weakly identified components;
- confounded components;
- non-identifiable components;
- condition number and posterior parameter correlations.

A full-rank estimated covariance matrix does not establish that the physical and nuisance sources are separately identified.

Use exact linear-Gaussian reference cases, profile likelihood or posterior recovery, and simulation studies with independent worlds.

## 5. Comparator ladder

- unrestricted full covariance (predictive baseline, attribution-limited);
- correctly specified shared-effect decomposition;
- diagonal independence;
- physical-state covariance only;
- batch/instrument nuisance covariance only;
- wrong-sign/wrong-loading and omitted-component ablations;
- hierarchical covariance parameters with uncertainty;
- exact small-case likelihood and posterior oracle.

A model that predicts total covariance accurately but attributes it to the wrong cause must not count as a physical mechanism-discovery success.

## 6. Sequential-design consequence

Connect the decomposition to the conditional information-gain API:

```text
I(M_physical; Y_next | Y_history)
I(M_nuisance; Y_next | Y_history)
I(M_physical; Y_next | Y_history, nuisance)
I(M_nuisance; Y_next | Y_history, physical)
```

The policy should be able to recognize that another CT/tracer pair may be highly predictive but adds little *new physical* information when its covariance is dominated by already-observed batch/instrument effects.

Compare acquisition under:
- true decomposition;
- full-covariance-only attribution;
- omitted batch term;
- omitted physical term;
- wrong nuisance loading.

Report physical information gain, nuisance information gain, decision EVSI and regret separately. Information about a nuisance factor can still help calibration/decision uncertainty, but it must not be relabelled as mechanism identification.

## 7. Required outcomes

- variance-component bias and recovery;
- posterior intervals/coverage for each component;
- attribution confusion matrix: physical, batch, instrument, operator, unresolved;
- false physical-complementarity certification under nuisance-only controls;
- missed physical signal under true shared-state controls;
- held-out log predictive density and proper calibration by specimen, batch, instrument, support and modality;
- posterior SBC on covariance and derived component quantities;
- posterior predictive checks by each dependence group;
- model-discrepancy alerts and abstention;
- design-matrix rank, condition number and parameter posterior correlations;
- acquisition information attributed to physical vs nuisance effects;
- policy regret, experiment/sample cost and extra replication required for identification.

## 8. Proposed gates (targets, not results)

- Zero false physical-complementarity certificates on deterministic batch-only and instrument-only null controls at the preregistered test resolution.
- Correct decomposition recovers identifiable variance components with calibrated uncertainty on independent synthetic worlds.
- Known non-identifiability returns an unresolved state/identified set; no forced attribution.
- Shared nuisance-only dependence does not increase posterior probability of the physical mechanism beyond calibrated false-positive limits.
- Physical shared-state dependence is detected with preregistered sensitivity at matched false-positive rate.
- Crossed instrument/batch replication improves identification only where the design matrix predicts that it should.
- Calibration and false-attribution rate reported by support/instrument/batch; pooled scores cannot mask a failed subgroup.
- No empirical support-to-covariance model enters adaptive design without independent-unit, repeated-measurement, provenance and qualified-review gates.

These are qualification targets, not achieved results or guarantees.

## 9. Integration sequence

1. Create deterministic synthetic linear-Gaussian decomposition cases with analytical covariance ground truth.
2. Implement covariance-component design-matrix and rank/identifiability audit.
3. Implement exact posterior for Gaussian random-effect reference cases where solvable.
4. Add parameter/covariance SBC and held-out support calibration.
5. Compare full covariance, diagonal and decomposed models under nuisance-only/physical-only controls.
6. Feed posterior component mixtures into #5649 conditional action information and #5633 paired multi-world evaluation.
7. Extend to CT/tracer support operators from #5643 only after the hierarchy is qualified.

## 10. References and non-claims

- López-Fidalgo & Wong (2026), *Optimal Designs for Correlated Data*, Annual Review of Statistics and Its Application 13:245–270, DOI [10.1146/annurev-statistics-042324-012947](https://doi.org/10.1146/annurev-statistics-042324-012947).
- Säilynoja et al. (2026), *Posterior SBC*, Statistics and Computing 36, 78, DOI [10.1007/s11222-026-10825-9](https://doi.org/10.1007/s11222-026-10825-9).
- Modrák et al. (2025), Bayesian Analysis, DOI [10.1214/23-BA1404](https://doi.org/10.1214/23-BA1404).

The benchmark is statistical-methods software and synthetic data only. It authorizes no sampling, instrument deployment, sample handling, tracer injection, CT scan, field trial, land access or amendment.


## 11. Executable structural covariance identifiability reference — 2026-11-01

The Rust library now contains `tools/sequential-bayes-design/src/covariance_identifiability.rs`. It constructs covariance basis matrices for shared physical support, preparation batch, instrument and independent residual effects; computes structural rank and a Gram spectral condition diagnostic; determines individually identifiable components; and performs noiseless exact coefficient recovery for synthetic covariance matrices only.

The reference controls show one CT/tracer pair sharing support/batch/instrument is rank 2/4, while a crossed support × batch × instrument × modality design is rank 4/4 and recovers variance coefficients [0.7, 0.4, 0.2, 0.1] to floating-point tolerance. A batch-only pair identifies the batch component but does not make physical covariance identifiable. This is an exact synthetic design oracle, not a statistical variance estimator or empirical soil result.
