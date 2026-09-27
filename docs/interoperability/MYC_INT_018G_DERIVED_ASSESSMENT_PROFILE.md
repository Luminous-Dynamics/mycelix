# MYC-INT-018G — Derived-Assessment Profile

Status: architecture / adapter profile only. Tracks #3233. Child of MYC-INT-018F / PR #3232.

Observed design date: 2026-09-27.

## 1. Purpose

Define a typed boundary for **derived assessments** in the productive-node showcase so source observations, analytical interpretations, standards checks, suitability determinations, verification claims, certification, and authority remain distinct.

The productive node needs to express statements such as:

- a measured value crossed a threshold;
- water met a declared drinking-water profile at a specific time;
- water met a hydroponic-process profile;
- an energy record was independently verified under a meter/evidence profile;
- a design circularity or embodied-energy value was calculated under a declared method;
- a repair appears successful under a declared outcome criterion;
- a Symthaea diagnosis or forecast was evaluated against later outcomes.

These are useful claims, but they are not raw observations.

## 2. Governing theorem

```text
Assessment
!= Observation
!= Prediction
!= Recommendation
!= Certification
!= Decision
!= Authorization
!= Effect
```

And:

```text
assessment true/positive under profile P at time T
!= universally true
!= current forever
!= valid under profile P2
```

## 3. Reuse existing owners

The profile should reuse:

- EPI `AssessmentId` semantics;
- EPI `ObservationId` / `SourceId` semantics;
- `SchemaRef` / `SemanticRef` identity;
- provenance / derivation relations;
- temporal evidence;
- EvidenceLease where its exact theorem applies;
- domain-specific currentness;
- translation receipts;
- existing Symthaea `AnalysisArtifact` boundary where analysis is the producer.

Do not create a second generic assessment identity system.

## 4. Candidate binding

Conceptually:

```text
DerivedAssessmentBindingV1 {
    assessment: AssessmentId,
    subject: SemanticRef,
    assessment_profile: SchemaRef,
    input_observations: Vec<ObservationId>,
    method_profile: SchemaRef,
    provenance_ref: SemanticRef,
    temporal_evidence_ref: SemanticRef,
    evidence_lease_ref: Option<SemanticRef>,
    disposition_ref_or_payload: profile-specific,
}
```

Exact field names and Rust ownership remain deferred until reconciled against the qualified EPI/semantic-core line.

The key requirement is that the **disposition is profile-specific**, not a universal boolean.

## 5. Why no universal `verified` or `safe`

Avoid:

```text
Assessment {
  verified: bool,
  safe: bool,
  compliant: bool,
}
```

because those terms are incomplete without a profile, method, evidence set, scope, and time horizon.

Prefer:

```text
assessment_profile = exact profile/revision
inputs = exact observations
method_profile = exact method/revision
disposition = exact profile-defined result
```

## 6. Assessment identity

Assessment identity must bind the exact inputs and profiles.

Changing any of these creates a different assessment:

- assessment profile;
- method profile;
- input observation set;
- source revision/generation where relevant;
- material parameters;
- applicable standard revision;
- evaluation temporal context where material.

```text
same human-readable result
+ different method/profile/input set
!= same assessment
```

## 7. Input closure

A derived assessment must identify the exact evidence it used.

No positive assessment should exist as:

```text
"water safe" = true
```

without an explicit input closure.

The closure may include:

- physical observations;
- calibration evidence;
- source-authentication evidence;
- source/currentness evidence;
- declared standards/profile;
- model/method revision;
- limitations/unknowns.

## 8. Missing and incomplete inputs

If an assessment profile requires a measurement/evidence item and it is missing, the evaluator must not silently treat the missing value as normal.

Candidate profile-specific outcomes may include:

```text
InsufficientEvidence
Unknown
Indeterminate
NotApplicable
```

Do not force every profile into a shared universal enum unless repeated qualified use proves the exact same semantics.

## 9. Currentness

A historical assessment remains historical evidence.

```text
assessment valid as-of T1
!= assessment current at T2
```

Currentness depends on:

- source observation currentness;
- applicable standard/profile currentness;
- method/profile currentness;
- evidence reuse horizon;
- domain-specific rules.

A valid `EvidenceLease` may constrain reuse but does not create semantic/domain currentness.

## 10. Later revisions

A later standard or method revision does not rewrite historical assessments.

Example:

```text
Standard v1 + observations O -> Assessment A1
Standard v2 + same observations O -> Assessment A2
```

A1 remains historical evidence under v1.

```text
A2
!= mutation of A1
```

## 11. Threshold findings

A threshold finding is one simple derived assessment:

```text
Observation O
+ ThresholdProfile P
-> ThresholdAssessment A
```

Examples:

- reservoir below declared low-level threshold;
- temperature above declared operating envelope;
- energy consumption above declared anomaly threshold;
- compost oxygen below declared process threshold.

The threshold value/profile must be explicit and versioned.

```text
threshold crossed
!= physical cause proven
!= effect authority
```

## 12. Water standards assessment

Current `water-purity::QualityReading` contains source-like measurements plus convenience fields such as:

```text
potability_score
meets_who_standards
meets_epa_standards
```

018G treats those convenience fields conservatively.

Until an independently qualified assessment proves their semantics:

```text
QualityReading.meets_who_standards = true
= source-domain assertion
!= qualified standards assessment
```

A stronger assessment should bind:

- exact source measurements;
- exact standards profile/revision;
- exact threshold/comparison method;
- unit conversion rules;
- missing-data rules;
- temporal/currentness evidence;
- provenance.

## 13. Water process suitability

Drinking-water standards and hydroponic-process suitability are different profiles.

```text
PotableAssessment
!= HydroponicProcessSuitabilityAssessment
```

A hydroponic suitability profile may require different dimensions such as:

- source-water chemistry;
- treatment state;
- crop/system profile;
- nutrient solution preparation assumptions;
- contaminant limits;
- operational constraints.

Do not infer process suitability from generic potability.

## 14. Admission boundary

Even a positive process-suitability assessment is not automatically an authorization/admission decision.

```text
SuitableUnderProfile
!= admitted into process
!= valve/pump command
```

A local policy/authority layer may consume the assessment when deciding whether to admit water/resource use.

## 15. Energy verification

Current `EnergyProduction` contains:

```text
verified: bool
```

018G freezes:

```text
EnergyProduction.verified
!= qualified energy verification receipt
```

A future energy verification assessment may bind:

- exact production/consumption record;
- meter identity/profile;
- meter observation(s);
- calibration/source-authentication evidence;
- interval semantics;
- verifier identity/method;
- discrepancy tolerance;
- result/disposition.

## 16. Meter-quality assessment

Meter/sensor quality itself may be assessed separately.

Examples:

```text
CalibrationCurrentUnderProfile
InstallationVerifiedUnderProfile
SourceAuthenticatedUnderProfile
DataCompletenessUnderProfile
```

These remain separate assessments.

Do not create one generic trusted-meter boolean that hides which theorem passed.

## 17. Design circularity assessment

Fabrication `Design` currently has fields such as:

```text
circularity_score
embodied_energy_kwh
```

Treat them as design metadata/claims unless bound to a declared assessment method.

A stronger circularity assessment might bind:

- exact design revision;
- material composition;
- repair/disassembly data;
- reuse/recycling assumptions;
- system boundary;
- calculation method/profile;
- evidence quality/uncertainty.

```text
calculated circularity indicator
!= actual end-of-life outcome
```

## 18. Embodied-energy assessment

Likewise:

```text
calculated embodied energy
!= metered manufacturing energy
!= lifecycle carbon impact
```

A method/profile must declare its system boundary and data sources.

If actual manufacturing energy is later measured, it remains a separate physical observation/outcome that can be compared to the prior estimate.

## 19. Repair outcome assessment

A repair chain may produce:

```text
failure observation
-> repair design
-> manufacture
-> installation
-> post-repair observations
-> repair outcome assessment
```

A repair is not successful merely because installation occurred.

Assessment criteria may include:

- restored function;
- measured flow/vibration/temperature;
- no recurrence over declared window;
- operator inspection;
- safety checks;
- remaining limitations.

The criterion/profile must be explicit.

## 20. Forecast evaluation

A forecast/prediction may later be assessed against outcome observations.

```text
Prediction
+ OutcomeObservations
+ EvaluationProfile
-> ForecastAssessment
```

Do not mutate the original prediction after the outcome is known.

Preserve:

```text
original prediction
original uncertainty
original timestamp/profile
later outcome
later evaluation
```

## 21. Symthaea diagnostic assessment

Symthaea may emit an `AnalysisArtifact` containing a diagnostic candidate.

That output is not a source observation.

A later assessment can evaluate it against:

- independent observations;
- maintenance findings;
- inspection results;
- measured outcomes.

```text
Symthaea diagnosis
!= diagnosis confirmed
```

unless an explicit evaluation/assessment says so under its profile.

## 22. Recommendation boundary

An assessment can inform a recommendation, but they are distinct.

```text
Assessment
-> Recommendation candidate
```

is permitted.

```text
Assessment
-> automatic command
```

is not implied.

## 23. Certification boundary

Certification is an authority/evidence process with its own issuer, scope, validity and rules.

```text
DerivedAssessment
!= Certification
```

A certifier may consume an assessment, but certification requires an explicit certification theorem/receipt.

Do not label an assessment `Certified` merely because it passed internal criteria.

## 24. Regulatory-compliance boundary

Likewise:

```text
assessment under encoded regulatory profile
!= legal/regulatory determination
```

unless the relevant authority/process explicitly establishes that effect.

018G can reproduce declared rules and evidence; it does not become a regulator.

## 25. Decision boundary

CDS/local governance may consume assessments as evidence.

```text
Assessment
+ other evidence
-> deliberation / decision
```

The decision remains separate and preserves rationale/evidence refs.

An assessment cannot mint governance standing or decision authority.

## 26. Authority boundary

No positive assessment may produce direct effect authority.

Examples:

```text
LowReservoirAssessment
!= permission to refill

HighTemperatureAssessment
!= permission to actuate cooling

WaterSuitableAssessment
!= permission to route water

RepairSuccessfulAssessment
!= permission to close all maintenance obligations
```

Each effect requires its normal bounded authority path.

## 27. Foreign/federated assessments

Assessments may cross federation boundaries as evidence.

Their authority does not automatically cross.

```text
foreign assessment accepted as evidence
!= foreign policy accepted
!= foreign authority recognized
!= local effect authorized
```

The receiving node decides admission/use under its own profile.

## 28. Source-domain convenience fields

018G provides the safe migration posture for fields that currently combine source record and interpretation.

Rule:

```text
legacy convenience field
-> preserve as source-domain assertion
-> optionally recompute independent DerivedAssessment
-> compare / record discrepancy
```

Do not silently rewrite historical source records.

## 29. Discrepancy evidence

If a source-domain convenience flag differs from an independently recomputed assessment, preserve both.

Example:

```text
source says meets_standard=true
independent profile says NotSatisfied
```

produces explicit discrepancy evidence.

Do not select a winner by field priority or arrival order without a separate policy.

## 30. Input provenance

A derived assessment should expose or bind enough provenance to reconstruct:

- input observation IDs;
- input source IDs;
- input profiles;
- derivation method;
- code/model/tool version where material;
- standards/profile revision;
- assessment time/currentness evidence;
- limitations.

## 31. Canonical input ordering

Where an assessment consumes a set of observations and order is not semantically meaningful, identity should use a canonical ordering rather than caller order.

Where order/time series is meaningful, the profile must say so explicitly.

```text
set semantics
!= sequence semantics
```

## 32. Partial coverage

An assessment over incomplete coverage must state its ceiling.

Example:

```text
three required water contaminants measured
one required contaminant unavailable
```

must not produce unconditional `Compliant` unless the profile explicitly allows that disposition.

Prefer explicit `InsufficientEvidence` / profile-specific outcome.

## 33. Uncertainty

Assessments may carry uncertainty/limitations, particularly for:

- model-based diagnostics;
- estimates;
- forecasts;
- inferred resource impacts;
- noisy physical measurements.

```text
confidence
!= probability unless calibrated profile says so
```

Do not coerce qualitative confidence labels into numeric probabilities.

## 34. Derived score boundary

If a profile produces a score, the score is meaningful only under that profile.

```text
score 0.8 under profile A
!= score 0.8 under profile B
```

Do not create a global score space for water safety, sustainability, reliability, circularity, or institutional value.

## 35. Multi-objective analysis

The productive node should usually preserve a vector of assessments rather than reduce everything to one scalar.

For example:

```text
water efficiency: improved
energy intensity: worsened
labor: unchanged
reliability: improved
```

No hidden weighting should turn that into `better=true`.

If weights/priorities are supplied, they are explicit input evidence with provenance and do not become universal preferences.

## 36. Assessment lifetime

Some assessments are historical snapshots; others may have a bounded reuse horizon.

Examples:

- threshold crossing at time T is a historical fact about inputs/profile;
- current suitability may expire when source data stales;
- calibration status may expire at a declared date/generation;
- forecast evaluation remains historical after outcome.

The assessment profile defines the semantics.

## 37. EvidenceLease boundary

A referenced EvidenceLease can bound reuse of qualified evidence.

```text
lease current
!= assessment semantically current
```

Domain/profile rules may shorten the effective positive horizon further.

Never widen beyond underlying evidence.

## 38. Unknown profile

Unknown assessment profiles must remain opaque/unsupported.

Do not guess that an unknown profile means:

- safety;
- compliance;
- verification;
- suitability;
- certification.

Preserve identity/bytes as allowed by the interface policy and refuse semantic use until supported.

## 39. Translation receipts

When importing an external/source-domain assessment into the interoperability profile, preserve translation class and losses.

For example:

```text
source bool -> local typed disposition
```

is not automatically an identity mapping.

The receipt should declare whether the translation was structural, lossy, derived, or unsupported according to the existing translation framework.

## 40. Privacy / sensitivity

Assessments can reveal sensitive operational facts even when raw telemetry is local.

Examples:

- repeated equipment failure;
- contamination findings;
- production shortfall;
- energy-use patterns;
- labor/process issues.

Support purpose-scoped disclosure and avoid making every assessment globally public by default.

## 41. Export / replaceability

Exports should preserve:

- assessment identity;
- subject;
- exact profile/method revisions;
- exact input observation refs;
- disposition;
- provenance;
- temporal/currentness evidence;
- uncertainty/limitations;
- translation receipt where imported.

Replacing the runtime or analytical engine must not reinterpret historical assessments.

## 42. First productive-node assessment families

Recommended initial families:

### Water

- threshold findings;
- source-quality assessment;
- drinking-water standards assessment where applicable;
- hydroponic process-water suitability.

### Energy

- interval record verification;
- meter completeness/quality assessment;
- energy-intensity derived metric.

### Hydroponics / climate

- operating-envelope assessment;
- anomaly finding;
- crop-stress diagnostic candidate evaluation.

### Compost

- process-envelope findings;
- phase/status support assessment;
- recommendation outcome assessment.

### Fabrication / repair

- design-calculation assessments;
- repair success assessment.

Keep each profile narrow and versioned.

## 43. Required negative corpus

At minimum:

1. assessment without exact input observations -> reject;
2. missing assessment profile/version -> reject;
3. missing method profile where required -> reject;
4. source-domain boolean substituted for qualified assessment -> reject;
5. assessment input replaced after identity creation -> recompute/different identity;
6. later standards revision mutates historical assessment -> reject;
7. raw observation becomes assessment without derivation -> reject;
8. assessment becomes raw observation -> reject;
9. assessment becomes certification -> reject;
10. assessment becomes legal/regulatory determination -> reject;
11. assessment becomes decision authority -> reject;
12. assessment becomes effect authority -> reject;
13. expired/stale assessment silently reused as current -> reject;
14. valid EvidenceLease becomes semantic currentness -> reject;
15. unknown profile coerced to familiar disposition -> reject;
16. incomplete required inputs still produce unconditional positive -> reject;
17. qualitative confidence coerced to probability -> reject;
18. score from one profile compared as identical to another profile -> reject;
19. source convenience flag conflicts with independent assessment and discrepancy is erased -> reject;
20. Symthaea diagnosis becomes confirmed source fact -> reject;
21. forecast is rewritten after outcome -> reject;
22. foreign assessment mints local authority -> reject;
23. hidden scalar weighting produces universal winner -> reject.

## 44. Positive corpus

At minimum:

1. exact water observations + standards profile -> reproducible derived assessment;
2. same observations under revised standard -> distinct assessment identity;
3. potable and hydroponic-suitability assessments coexist for same water observations;
4. source-domain `verified=true` remains source assertion while independent energy verification assessment exists separately;
5. design embodied-energy estimate and later metered manufacturing-energy observation remain distinct and comparable;
6. forecast evaluation preserves original prediction and later outcome;
7. repair outcome assessment binds exact post-repair observations/criteria;
8. expired assessment remains historical but excluded from live use;
9. unknown future assessment profile remains opaque and round-trippable.

## 45. Implementation gate

Architecture and fixtures may proceed now.

Executable types should only be promoted after exact reconciliation with:

- EPI `AssessmentId` ownership;
- semantic-core subject/profile identity;
- provenance/derivation;
- temporal evidence / EvidenceLease;
- domain-specific currentness;
- existing AnalysisArtifact/translation receipt owners.

No parallel evidence substrate should be introduced.

## 46. Nonclaims

MYC-INT-018G does not establish:

- universal safety;
- universal compliance;
- legal/regulatory authority;
- certification;
- trusted verification;
- sensor correctness;
- process admission;
- decision legitimacy;
- actuation authority;
- Symthaea correctness;
- universal sustainability or quality scoring.

It defines the typed boundary needed so derived claims remain explicit, reproducible, revisable by new evidence, and non-authoritative unless a separate owner grants stronger meaning.
