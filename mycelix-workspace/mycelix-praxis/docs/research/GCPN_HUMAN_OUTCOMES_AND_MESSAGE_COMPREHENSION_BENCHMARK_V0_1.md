# GCPN Human-Usable Warning and Message-Comprehension Benchmark v0.1

**Status:** deterministic synthetic benchmark; non-operational.  
**Parent:** issue #5635.  
**Package:** benchmarks/gcpn-human-outcomes.  
**Dependencies:** Rust standard library only.  
**Fixture:** 18 paired cells, 6 interaction profiles × 3 message variants; 100 invented trials per cell.

## 1. Paradigm shift: a warning is a human-usable contract, not a packet

The existing GCPN chain distinguishes record authority, forecast evidence, dissemination, unique reach, accessible-format reach, action authorization and lifecycle cost. It does not yet measure whether the content remains understandable after it reaches a person's available interface.

A transport receipt is not comprehension. A rendered format is not comprehension. Understanding an instruction is not proof someone followed it. This benchmark therefore keeps distinct denominators and outcomes:

1. **Intended test exposure** — all synthetic task trials.
2. **Accessible representation** — trials whose supplied presentation is declared usable in the functional interaction profile.
3. **Core-message comprehension** — correct response to an abstract task about the warning's central meaning.
4. **Action-intent identification** — correct identification of the action stated in the message; not actual behaviour or advice to a real person.
5. **Uncertainty understanding** — correct interpretation of the message's uncertainty.
6. **False certainty** — incorrect interpretation of a conditional/uncertain statement as certain.
7. **Response time** — synthetic median and p90 descriptors.

The benchmark reports both conditional rates among accessible trials and **effective outcome per intended trial**. The latter prevents inaccessible cases from disappearing when analysis narrows its denominator to people who could access the message.

## 2. Frozen synthetic design

The same abstract task identity is crossed with six functional interaction profiles and three semantically equivalent presentation designs.

| Interaction profile | Constraint represented |
|---|---|
| standard smartphone | baseline small-screen text presentation |
| low-bandwidth text | constrained network/transfer budget |
| screen reader | linearized and nonvisual navigation |
| sign-language video | signed-language video availability |
| audio-only radio | spoken-only reception |
| language mismatch | supplied language not matched to receiver preference |

| Message variant | Description |
|---|---|
| dense technical | compact, jargon-heavy baseline |
| plain language | simplified text presentation |
| accessible multimodal | plain language plus suitable alternative formats |

The 18 rows each contain 100 **invented** task trials. Counts and response times are deterministic test vectors designed to exercise the scoring logic and reveal trade-offs. They are not observations, user research, a sample of any population, or forecasts of actual performance. Profiles are interface constraints, not demographic or protected-attribute groups.

## 3. Metrics and denominators

For each profile and presentation:

- Accessibility rate = accessible trials / intended trials.
- Conditional comprehension = correct comprehension / accessible trials.
- Effective comprehension = correct comprehension / intended trials.
- Conditional action-intent identification = correct action-intent responses / accessible trials.
- Effective action-intent reach = correct action-intent responses / intended trials.
- Conditional uncertainty understanding = correct uncertainty responses / accessible trials.
- Effective uncertainty understanding = correct uncertainty responses / intended trials.
- False-certainty rate = false-certainty responses / accessible trials.
- Latency = reported cell median and p90; the report uses the maximum cell p90. It does not average subgroup quantiles.

A zero accessible denominator produces an undefined conditional rate, not a fabricated zero. Effective rates remain defined using the intended-trial denominator.

## 4. Synthetic fixture oracle

The v0.1 fixture has 18 rows, 6 profiles, 3 variants and 1,800 total intended trials. The FNV-1a 64-bit fingerprint is 0x019e77cec7c2246c.

The accessible-multimodal synthetic profile totals are:
- 600 intended trials;
- 530 accessible trials;
- 484 correct core-message responses;
- 452 correct action-intent identifications;
- 422 correct uncertainty interpretations;
- 87 false-certainty interpretations.

Therefore effective comprehension is 484/600 = 80.67%, while comprehension among accessible trials is 484/530 = 91.32%. Reporting only the conditional figure would conceal 70 inaccessible trials.

The lowest profile effective comprehension for this variant is 55% (low-bandwidth text); the best is 94%; the cross-profile gap is 39 percentage points. The maximum profile p90 in this invented fixture is 34 seconds. These are fixture arithmetic, not estimates of real people.

## 5. Non-compensatory analysis

Do not rank variants by a single weighted score. Keep these trade-offs visible:
- format accessibility;
- effective comprehension;
- effective action-intent identification;
- uncertainty understanding;
- false certainty;
- worst-profile performance;
- profile gaps;
- latency.

An improvement in mean comprehension cannot compensate for a presentation that is unavailable to a particular interaction profile. A lower false-certainty rate cannot compensate for an inaccessible message. Report the full profile matrix and all denominators.

## 6. Parser and negative-control invariants

The dependency-free parser rejects:
- any header/schema mismatch;
- unknown profiles or message variants;
- semantic-task identity mismatch;
- duplicate profile/variant pairs;
- missing cross-product cells;
- zero intended trials;
- accessible counts above intended trials;
- any task outcome count above accessible trials;
- p90 below the median;
- wrong row cardinality.

Negative controls assert that each input mutation actually changes the frozen fixture before expecting rejection. The fixture hash pins exact bytes including its final newline.

## 7. Proposed research targets — not standards

Potential targets for later, independently designed and ethically reviewed field research:

| Measure | Aspirational target |
|---|---:|
| Accessible-format availability in each interaction stratum | ≥95% |
| Effective core-message comprehension per intended exposure | ≥90% |
| Effective action-intent identification | ≥90% |
| False-certainty interpretation among accessible exposures | ≤5% |
| Worst-to-best effective-comprehension profile gap | ≤10 percentage points |
| p90 comprehension time for short urgent messages | ≤10 seconds |

These values are hypotheses for study design, not WMO/ITU/UNDRR standards, not live-system gates, and not a claim that this synthetic fixture meets them. Real targets require co-design and validation with national warning authorities, disability organizations, language communities, broadcasters, mobile operators and human-factors researchers.

## 8. Industry and standards comparison

ITU-T X.1303 / CAP gives interoperable structure for exchanging all-hazard alerts across ICT networks. That is necessary for consistent dissemination, but the protocol's existence does not by itself demonstrate comprehension.

WMO and UNDRR's Global Status of MHEWS 2025 describes people-centred, locally led warning systems, dialogue and persistent coverage/comprehensiveness gaps. SADC's 2026 MHEWS capacity-strengthening work explicitly calls for understandable and actionable impact-based alerts, user co-production, accessibility, responsible AI/ML and alignment with national systems, SADC SHOC and AMHEWAS.

This benchmark complements, not replaces, those institutional efforts. It does not propose a second warning authority or a proprietary replacement for CAP.

## 9. Evidence ladder and non-claims

| Evidence level | v0.1 status |
|---|---|
| Contract / specification | authored |
| Synthetic fixture | frozen |
| Static arithmetic | independently recomputed outside Rust |
| Rust source | authored; not compiled in the authoring environment |
| Unit tests | authored; execution outstanding |
| Human-subject validation | not performed |
| Real-world operational validation | not performed |

No participant-level records or identifiers are present. No demographic, disability, ethnicity, nationality, precise location or other protected personal attributes are collected or inferred. No live alerting, recipient profiling, message targeting, operational thresholds, responder direction, actual protective behaviour or reduced-harm claim is included.

Any future human-subject study requires independent ethics review, informed consent, accessible participation, community governance, and data minimization. National warning authorities retain decision authority. Actual actions and causal impacts must be evaluated separately from message comprehension.

## 10. Sources

- SADC, *Consultancy to undertake capacity building arising from identified MHEWS gaps and needs from Member States* (2026): https://www.sadc.int/procurement-opportunities/consultancy-undertake-capacity-building-arising-identified-mhews-gaps-and
- WMO / UNDRR, *Global Status of Multi-Hazard Early Warning Systems 2025*: https://wmo.int/resources/publication-series/global-status-multi-hazard-early-warning-systems/global-status-multi-hazard-early-warning-systems-2025
- ITU, *Common Alerting Protocol and call to action* (CAP adopted as ITU-T X.1303): https://www.itu.int/en/ITU-D/Emergency-Telecommunications/Pages/Common-Alerting-Protocol-and-Call-to-Action.aspx
