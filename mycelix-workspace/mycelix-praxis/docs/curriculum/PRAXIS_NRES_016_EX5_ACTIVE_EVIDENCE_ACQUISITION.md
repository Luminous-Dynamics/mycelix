# PRAXIS-NRES-016-EX5 — Active evidence acquisition and decision-dependent observations

**Status:** synthetic public-development specification and arithmetic oracle  
**Activation:** inactive; no operational use  
**Parent:** NRES-016 EX1 exact nested-risk benchmark  
**Tracking:** https://github.com/Luminous-Dynamics/mycelix/issues/5583

## 1. Purpose

EX5 makes inspection and evidence acquisition explicit decisions. It measures whether the expected downstream decision improvement justifies the information action's acquisition cost and resource burden.

NRES-016 already permits VERIFY / OBSERVE / REQUEST-REVIEW actions. EX5 adds the missing quantitative contract: acquisition actions select an observation process; emitted evidence arrives at a declared time; evidence is usable only after the specified integrity and availability checks; and the terminal action is selected from accepted observable evidence only.

The information-discovery problem family is established. EX5 is not a claim to invent decision-dependent information discovery. Its intended contribution is an evidence-aware, multistage benchmark combining information action, temporal availability, custody-versus-accuracy semantics, exact policy evaluation, and inherited resource/service invariants.

## 2. Formal model

At an information state \(h_t\), choose acquisition action \(u_t\). The observation kernel is

\[
K_{u_t}(y,\tau,v\mid z,h_t),
\]

where:
- \(z\) is a latent physical condition used only by the evaluator;
- \(y\) is the signal emitted by the observation process;
- \(\tau\) is its declared availability time;
- \(v\) is its evidence-integrity / verification status.

Acquisition has explicit cost \(c_u\) and resource use \(r_u\). The terminal policy may condition only on accepted evidence available by the decision deadline:

\[
a_t=\pi_t(\mathcal I_t^{accepted,available}).
\]

A signal is not ground truth. A valid signature or custody receipt may establish source identity and byte integrity; it does not establish the physical accuracy of the sensor. Sensitivity, specificity, provenance, integrity, availability, staleness and missingness are separate properties.

## 3. Exact synthetic micro-oracle

Binary latent condition:
- **H**: high disruption, prior probability 0.20;
- **L**: low disruption, prior probability 0.80.

The latent condition is never passed to the operational policy.

Terminal choices:
- **PROTECT**: deterministic cost 30.
- **DEFER**: cost 100 if H, cost 0 if L.
- Both terminal choices satisfy the fixture's service floor. DEFER in state H leaves service at 75 with a floor of 70; its cost captures emergency-response / remediation burden, not a concealed floor breach.

Acquisition options:

| Action | Acquisition cost | Sensitivity | Specificity | Inspection units |
| --- | ---: | ---: | ---: | ---: |
| NONE | 0 | — | — | 0 |
| BASIC | 5 | 0.80 | 0.90 | 1 |
| ADVANCED | 12 | 0.95 | 0.95 | 2 |

Total inspection capacity is 2 units. Signals are available before the terminal decision in this first micro-instance. Follow-on families add delay, missingness, and physical-state changes.

### Exact arithmetic

**NONE**
- P(H)=0.20.
- PROTECT costs 30; DEFER has expected cost 20.
- Select DEFER; total expected cost = **20**.

**BASIC**
- P(ALERT)=0.20(0.80)+0.80(0.10)=0.24.
- P(H | ALERT)=0.16/0.24=2/3, so select PROTECT.
- P(CLEAR)=0.76.
- P(H | CLEAR)=0.04/0.76=1/19, so select DEFER.
- Expected terminal decision cost = 0.24(30)+0.04(100)=11.2.
- Acquisition cost = 5; total expected cost = **16.2**.
- Gross EVSI = 20−11.2=8.8; net value after acquisition cost = **3.8**.

**ADVANCED**
- P(ALERT)=0.20(0.95)+0.80(0.05)=0.23.
- P(H | ALERT)=0.19/0.23, so select PROTECT.
- P(CLEAR)=0.77.
- P(H | CLEAR)=0.01/0.77, so select DEFER.
- Expected terminal decision cost = 0.23(30)+0.01(100)=7.9.
- Acquisition cost = 12; total expected cost = **19.9**.
- Gross EVSI = 20−7.9=12.1; net value after acquisition cost = **0.1**.

The optimal acquisition is BASIC, not the most accurate sensor. Better information can be economically inferior after its cost and resource burden are included.

Perfect information at zero acquisition cost gives expected cost 0.20(30)=**6**. It is a non-deployable lower bound, never an eligible policy.

All values are synthetic arithmetic checks, not sensor-performance or operational-effectiveness results.

## 4. Required comparisons

1. No-information adaptive policy.
2. Fixed PROTECT.
3. Fixed acquisition schedule without adaptive terminal actions.
4. Joint acquisition-and-action policy (EX5).
5. Perfect-information lower bound, clearly marked non-deployable.
6. Myopic information-value heuristic using the same tree.
7. Future: robust / ambiguity-aware acquisition when observation quality, latency and outage rates are uncertain.

Report gross EVSI, net EVSI, acquisition cost, expected terminal cost, resource use and perfect-information gap separately.

## 5. Evidence / temporal integrity contract

- Latent condition \(z\) is excluded from observations, action payloads, policy keys and state identifiers.
- A signal is not usable before its declared available_at time.
- Late, stale, missing, unauthenticated or contradictory signals trigger the predeclared fallback (no-information policy or human review).
- event_time, observed_at, received_at, verified_at and available_at are distinct fields. Missing times remain missing; they are never silently imputed as timely.
- Source authenticity / custody and sensor measurement accuracy are separate.
- Verification proves only the declared cryptographic or custody predicate, not physical truth.
- Acquisition cost, elapsed time and resource use are charged exactly once.
- Paths with identical accepted observable histories must select the same terminal action.
- An inspection result may update beliefs only through the signal actually emitted and made available; the hidden condition itself is not revealed.

## 6. Follow-on stress families

- Sensitivity/specificity grids, including useless and inverted sensors.
- Delayed observations and hard decision deadlines.
- Sensor outage, missing-not-at-random and source dropout.
- Stale evidence, replayed records and inconsistent timestamps.
- Multiple correlated inspectors with common source failures.
- Verification cost and human-review deferral.
- Inspection action changes the physical state or future hazard kernel (true dual-control extension).
- Sequential stopping rules and repeated measurements under resource exhaustion.
- Distribution shift in sensor accuracy and calibration drift.

## 7. Correctness and benchmark targets (proposed, not results)

- Exact objective and policy agreement with independent enumeration within absolute error \(10^{-10}\).
- Zero latent-condition leakage and zero use of signals before declared availability.
- 100% acquisition-cost and inspection-resource accounting.
- 100% rejection / fallback conformance for invalid, late, stale or unauthenticated evidence.
- Outcome-label permutation and equivalent probability-splitting invariance.
- Report gross/net EVSI, acquisition burden, terminal decision cost, delay, abstention, fallback rate and perfect-information gap separately.
- Compare paired regret on independent source/regime families, not duplicated rows or time steps treated as independent.
- Publish no-information-dominant cases, negative net EVSI and infeasible acquisition options; do not force information acquisition to appear beneficial.
- Keep hard service floors and resource invariants separate from expected-cost objectives.

## 8. Research positioning

- Vayanos, Georghiou & Yu, *Robust Optimization with Decision-Dependent Information Discovery*, Management Science 72(2), 2026, DOI [10.1287/mnsc.2021.00160](https://doi.org/10.1287/mnsc.2021.00160): costly exploratory decisions control when uncertain information becomes observable.
- Paradiso et al., *Exact and Approximate Schemes for Robust Optimization Problems with Decision-Dependent Information Discovery*, INFORMS Journal on Computing 37(6), 2025, DOI [10.1287/ijoc.2023.0290](https://doi.org/10.1287/ijoc.2023.0290): exact and K-adaptability-style schemes for this problem family.

NRES's defensible research target is the integration of decision-dependent observation with strict temporal availability, custody-versus-accuracy semantics, resource/service invariants, and exact independent policy verification—not a claim to invent information discovery.

## 9. Non-claims

The first micro-oracle is synthetic, public-development-only and inactive. No real sensor accuracy, causal effect, operational readiness, emergency authority, certification, calibration guarantee or SOTA superiority is claimed.