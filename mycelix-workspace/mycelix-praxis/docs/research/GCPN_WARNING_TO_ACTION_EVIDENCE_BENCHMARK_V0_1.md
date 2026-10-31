# GCPN Warning-to-Action Evidence Benchmark v0.1

**Status:** deterministic synthetic evidence-chain benchmark; non-operational.  
**Parent:** issue #5611.  
**Package:** `benchmarks/gcpn-warning-to-action`.  
**Scope:** accountable process evidence from an authorized public warning through an externally governed early-action protocol and independently evaluated outcome evidence.

## 1. Research shift: warning delivery is not the outcome

GCPN's previous increments cover record integrity, CAP interoperability, topology-level continuity, and unique/accessibility-aware receiver reach. The next meaningful question is whether people-centred warnings are connected to timely, locally governed action.

The Early Warnings for All framework treats preparedness and response as one of its four pillars. IFRC's Early Warning–Early Action work emphasizes community engagement, pre-agreed Early Action Protocols, training, last-mile support, trust, understanding and feedback about whether warnings led to action. WFP/OCHA's multi-agency impact-evaluation window (1 October 2026) asks comparable programme questions: when to act, whom to reach, which action packages work, and at what cost.

A recent regional policy signal is especially relevant: the Mombasa Ministerial Declaration on Anticipatory Action, adopted 30 October 2026, calls for nationally agreed protocols by 2027, defined trigger thresholds, risk layering, pre-agreed multi-sector actions, clear institutional responsibilities, pre-arranged finance, simulation exercises, monitoring and activation against agreed rules.

References:
- Mombasa Ministerial Declaration (30 Oct 2026): https://igad.int/mombasa-ministerial-declaration-on-anticipatory-action-second-eastern-africa-dialogue-platform-on-anticipatory-action/
- WFP/OCHA Multi-agency Impact Evaluation Window (1 Oct 2026): https://www.wfp.org/publications/multi-agency-impact-evaluation-window-anticipatory-action
- IFRC Early Warning, Early Action: https://www.ifrc.org/early-warning-early-action
- UNDRR, anticipatory-action financial protocol principles: https://www.undrr.org/publication/anticipatory-action-aa-best-practices-and-guiding-principles-financial-protocols-aa
- UN Early Warnings for All: https://www.un.org/en/climatechange/early-warnings-for-all

## 2. The complete evidence chain

The benchmark models these distinct stages:

1. **Authorized CAP alert:** public warning is issued by a competent authority and bound to a CAP message identifier.
2. **Protocol decision:** the locally authorized institution records activate, no-action or abstain against a pre-agreed protocol version.
3. **Financing readiness:** where an action package requires financing, the funding is pre-arranged and its release is authorized and timestamped.
4. **Action authority and implementation:** the responsible actor is authorized; start and completion are recorded as separate events.
5. **Unique population reach:** unique people served are kept separate from service records and duplicate receipts.
6. **Community accountability:** grievances/feedback reported, recorded and resolved remain visible.
7. **Outcome evidence:** observations, independent evaluation and counterfactual assumptions are recorded separately from process completion.
8. **Learning:** after-action corrections are appended; previous records are never overwritten.

GCPN is only the evidence/provenance and conformance layer. It must not:
- set hazard thresholds or trigger policies;
- issue, update or cancel public alerts;
- activate an Early Action Protocol;
- release or transfer funds;
- direct public authorities or responders;
- select real people for assistance;
- establish eligibility, entitlements or legal status;
- claim that a process receipt proves impact.

All activation and resource decisions remain with competent national/local institutions and agreed protocol owners.

## 3. Synthetic benchmark package

The dependency-free Rust package uses a frozen ten-row CSV fixture with generic, fictional event IDs, one protocol version, invented receiver counts and abstract tick timestamps. It includes both valid and deliberately invalid examples:

| Case | Deliberate property under test |
|---|---|
| valid-complete-chain | Complete authorized process record |
| unauthorized-activation | Activation decision without valid decision authority |
| unapproved-fund-release | Funds released without pre-arranged and authorized financing |
| unauthorized-action-start | Action begins without action authority |
| false-completion | Completion asserted without a recorded start |
| population-overclaim | Unique people served exceeds intended population |
| grievance-suppression | Reported grievances exceed recorded grievances |
| unsupported-impact-claim | Impact claim submitted without outcome, independent evidence and identifiable counterfactual |
| legitimate-abstention | Authorized abstention with no action or release |
| chronology-violation | Completion timestamp predates action start |

This fixture is a negative-control suite. The expected benchmark result is that the evaluator detects the exact violations in the invalid cases and accepts the valid complete chain and legitimate abstention. A report with expected negative-control violations is not a successful operational protocol; passing the fixture oracle means only that the evaluator detected the fixture's known defects.

## 4. Hard integrity gates

Each dimension is reported separately and non-compensatorily:

- CAP lineage and protocol version must be present.
- All protocol decisions require competent decision authority.
- Activation must also bind to an authorized warning.
- Funds release requires an activation decision, pre-arranged finance and valid financing authority.
- Action start requires activation, action authority and the required financing release.
- Completion requires a recorded start and correct event ordering.
- Unique people served cannot exceed the intended population denominator.
- Service-record counts must equal unique people served plus explicitly recorded duplicates.
- Grievances reported, recorded and resolved must be separately accounted for; unresolved grievances remain visible.
- A submitted impact claim requires outcome observation, independent evaluation and an identified counterfactual design.
- Timestamps must be monotonic across the recorded warning/decision/finance/action sequence.

The last impact-claim gate is a **minimum evidence precondition**, not a causal inference method. Even if all three fields are true, the system has not proven the action caused a particular outcome or saved lives.

## 5. Metrics

Report separately:
- warning-to-decision latency;
- decision-to-finance-release latency;
- finance-release-to-action-start latency;
- action-start-to-completion latency;
- authorized activation, abstention and no-action counts;
- financing-readiness and release counts;
- actions started/completed;
- intended population and unique people served per case;
- duplicate service records;
- grievance reporting, recording, resolution and open counts;
- outcome observation and independent evaluation status;
- counterfactual identifiability and causal-claim submission status;
- false activation, missed activation, unauthorized release and unauthorized action counts.

The benchmark uses arbitrary synthetic ticks. They are not hours or days. The frozen fixture includes deliberately invalid rows, so aggregate sums across all rows are only regression-test arithmetic, not a population coverage estimate or a programme KPI.

Do not make a composite score that lets high delivery or action throughput conceal an authority, finance, protection, inclusion, grievance or evidence-integrity failure.


## 6.1 Frozen fixture oracle (expected negative-control counts)

The ten-row fixture should produce these exact counters:

| Counter | Expected |
|---|---:|
| Missing CAP/protocol lineage | 0 |
| Unauthorized decisions | 1 |
| Unauthorized activations | 1 |
| Unapproved fund releases | 1 |
| Unauthorized action starts | 1 |
| False completions | 1 |
| Population overclaims | 1 |
| Service-record accounting mismatches | 0 |
| Grievance suppression | 1 |
| Grievance resolution overcounts | 0 |
| Unsupported impact claims | 1 |
| Chronology violations | 1 |

It should also produce nine activation decisions, eight funding-release records, six action starts and six action-completed assertions. The last number includes one deliberately false completion, which is why completed assertions must be distinguished from valid completion latency.

The arithmetic sums across the ten test rows are 10,000 intended-person slots and 3,790 unique-person service records. These are not a unique population, programme reach, or coverage rate: fixture cases represent different fictional scenarios and deliberately include a population-overclaim negative control.

Mean synthetic timing ticks are expected to be 98 from warning to decision, 100 from decision to finance release, approximately 133.33 from finance release to action start, and 500 from action start to validly ordered completion. These are test-vector arithmetic, not real-world time measurements.

## 6. Reproduction

From the Mycelix repository root:

```bash
cargo test --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-warning-to-action/Cargo.toml
cargo run --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-warning-to-action/Cargo.toml
```

The executable prints:
- the expected-negative-control oracle status;
- violation counters;
- process counts and aggregate regression sums;
- mean synthetic timing values;
- case-level unique service coverage, duplicate records, grievances and impact-claim state.

The fixture has a pinned FNV-1a fingerprint. Tests assert the exact expected violation counts and the valid-chain / abstention cases.

## 7. Industry and SOTA comparison

Do not compare GCPN against one imaginary "best early-warning system". Compare complementary layers and common tasks.

**Existing baselines and practices:**
- UNDRR's anticipatory-action financial protocol principles emphasize pre-arranged funding, objective risk information, flexibility under uncertainty, localization, coordination, inclusion, transparency and accountability.
- OCHA's Anticipatory Action Framework describes three pre-agreed pillars: a trigger, activities and financing.
- IFRC's Early Warning–Early Action work emphasizes community engagement, trained local responders, early action protocols and feedback on receipt, trust, understanding and action.
- WFP/OCHA's 2026 impact-evaluation window aims to generate comparable evidence across countries about timing, recipient reach, action packages and cost.

GCPN should complement these approaches with exact source-to-decision-to-action evidence linkage, versioned authority and protocol references, local-first records, unique-recipient accounting, append-only grievance records and independent conformance.

A fair comparison requires identical hazards, trigger assumptions, local action packages, recipient denominators, available resources and evaluation designs. Report false/missed activation, action timing, cost, population reach, exclusions, grievance handling and outcome uncertainty separately. Do not rank named national or humanitarian programmes without permissioned data, equivalent denominators and independent evaluation.

## 8. Evaluation methodology beyond the synthetic suite

Process evaluation can establish whether the agreed protocol was followed; it cannot alone establish outcome impact.

Real-world studies, if later authorized, should distinguish:
- **process evaluation:** were warnings, finance, logistics and actions delivered as designed?
- **coverage/equity evaluation:** who was and was not reached, and why?
- **outcome evaluation:** did the action package improve the defined outcome?
- **causal impact evaluation:** what would likely have happened without the action, using an appropriate comparison/counterfactual design?

The design must document selection bias, confounding, forecast error, baseline trends, spillovers, exposure, implementation variation, missing data and uncertainty. Prefer explicit "not identifiable" over a fabricated point estimate. Humanitarian evaluation should use independent ethics, safeguarding and data-protection review, and community feedback/appeal channels.

## 9. Non-claims and safety boundary

All examples are synthetic. This package does not trigger actions, release money, allocate assistance, issue public alerts, determine hazard thresholds, identify real people, or claim causal impact. It does not measure real population coverage, real delivery time, lives saved, livelihoods protected, or cost-effectiveness. Real studies require locally legitimate authorities, community co-design, safeguarding, consent where applicable, and formal legal/operational approvals.
