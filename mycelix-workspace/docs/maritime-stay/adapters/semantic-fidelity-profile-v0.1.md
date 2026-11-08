# Maritime Commons Semantic Adapter Fidelity Profile v0.1

**Status:** research profile and synthetic scenario manifest; not a normative protocol, executable verifier, provider certification or field qualification  
**Tracking:** Mycelix #5963  
**Parent profiles:** global interoperability #5945 / PR #5946; maritime network #5920; provider ingress #5925; semantic verifier #5921; profile verifier #5954  
**Compatibility:** does not modify the Stay Protocol v0.1 wire contract or the adapter mapping-ledger schema

## 1. Purpose

A syntactically valid mapping can still be semantically false. An adapter can parse JSON, satisfy a target schema and retain a payload digest while turning a port into a berth, a notification into a current reservation, an observation into a commitment, a refund request into a settled refund, or an unknown value into a positive default.

This profile makes the central interoperability requirement testable:

> Semantic interoperability is preservation of claims under representation change, not successful field matching.

It composes with the mapping-ledger schema introduced in #5946. That schema records mapping outcomes and declared loss; this profile adds semantic invariants, metamorphic properties and a reproducible test vocabulary. It does not replace that schema or duplicate provider-specific ingress verification.

## 2. Claim-preservation model

For a source assertion S, adapter transformation T, and target assertion T(S), evaluate separately:

- subject and resource scope;
- claim kind and operational stage;
- value, type, cardinality and null/absence semantics;
- unit/dimension, scale and precision;
- event time, effective time, observation time, delivery time and retrieval time;
- issuer, authority scope, recognition and purpose;
- source profile, revision, provenance and supersession lineage;
- uncertainty, ambiguity, conflict, freshness and completeness;
- privacy classification and disclosure purpose.

The adapter must not strengthen an assertion merely by changing its representation:

- output authority must not exceed source authority plus separately supplied, independently qualified recognition;
- output certainty must not exceed source certainty;
- output currentness must not exceed source currentness evidence;
- output subject/resource specificity must not exceed the bound subject/resource evidence;
- output completeness must not exceed the source fields and coverage actually observed.

These are component-wise claim-boundary constraints, not a scalar trust score or a numeric ordering over all evidence. A valid transformation may change encoding or units while preserving meaning. A lossy transformation may be used only when the loss is declared, its impact is bounded, and the dependent operation remains safe. Some claims must become Unknown, Conflict, NotEvaluable or be rejected.

## 3. Mapping classes

Every evaluated source-to-target mapping uses one of these classes:

| Class | Meaning | Required treatment |
|---|---|---|
| Exact | Same declared representation and semantics | Preserve source and target references |
| LosslessTransform | Representation changes but the declared semantic observables are preserved | Pin transform and equivalence relation; prove with vectors |
| LossyDeclared | One or more information dimensions cannot be reconstructed | Name every loss; prohibit dependent claims requiring the lost dimension |
| Unsupported | Target profile has no faithful representation | Preserve source evidence and quarantine/return unsupported |
| Ambiguous | More than one interpretation remains possible | Preserve ambiguity; do not choose a convenient default |

The existing ledger outcomes (preserved, normalized_preserving, approximated, omitted, unsupported, unknown_source_value, invented_value) remain the structural record. Do not reinterpret their existing wire meaning in this profile. A future mapping-ledger extension requires a separate compatibility review.

## 4. Non-compensatory semantic invariants

### SF-01 — Identity and subject scope
An identifier is a tuple of scheme, issuer, subject type, value, validity, jurisdiction and purpose. Names, call signs, MMSIs, port codes and organization identifiers must not be silently re-bound to a different subject type or resource scope.

### SF-02 — Authority and operational stage
Observation, notification, offer, hold, commitment, fulfilment, government submission, acknowledgement, clearance, payment attempt, settlement and refund finality are different claims. An adapter may not promote one stage into another.

### SF-03 — Currentness
Historical or cached values remain historical or cached. A notification is not a fresh availability read. A stale endpoint or source revision is not current merely because it was received recently.

### SF-04 — Uncertainty and conflict
Unknown, ambiguous, unsupported, incomplete, stale and conflicting states must survive projection. Defaults and fallback values may not convert them into positive assertions.

### SF-05 — Units and quantities
A conversion binds source and target units, dimensions, precision, rounding, uncertainty and transformation version. Similar labels do not establish dimensional equivalence.

### SF-06 — Monetary meaning
Currency code, pinned metadata digest, minor-unit exponent, effective date, amount representation, quote/obligation/invoice/refund role, tax/fee treatment and rounding are independent semantic context. Locale formatting cannot mutate an authoritative amount.

### SF-07 — Temporal meaning
Preserve source timestamp lexeme, offset, IANA zone, tzdb version, fold/gap disposition, resolved instant and distinct source/observation/delivery/retrieval times. Date-only values are not instants.

### SF-08 — Provenance and lineage
Every target assertion must link to the exact source assertion, source profile/revision and transformation implementation. Corrections and supersession must not erase historical evidence.

### SF-09 — Privacy and purpose
An adapter cannot disclose fields beyond a purpose-scoped allowlist. Hashing or pseudonymizing a field does not automatically make its use lawful or its disclosure authorized.

### SF-10 — Reversibility and drift
A round-trip claim is valid only under an explicit equivalence relation over named observables. Lossy transformations cannot be labeled exact-round-trip. Reapplying a transform must not silently accumulate semantic drift.

## 5. Metamorphic properties

The evaluator should test relations between inputs, not only isolated expected outputs:

1. Reordering source object members or changing insignificant whitespace must not change target meaning where the source format defines those changes as non-semantic.
2. Semantically equivalent source representations produce equivalent target assertions under the declared transform.
3. Semantically distinct source assertions must not collapse to the same target assertion where the differing dimension is decision-relevant.
4. A lossless round trip preserves every declared observable.
5. A lossy transformation reports the exact dimensions that cannot be reconstructed.
6. Reapplying a deterministic adapter does not create new semantic drift or duplicate logical evidence.
7. Changing subject, issuer, resource, authority stage, currency metadata digest, source revision or profile version changes the relevant binding.
8. Locale-only presentation changes do not change canonical money, quantity or time values.
9. Unknown extension fields are preserved as opaque data or explicitly quarantined according to the pinned profile; they are not silently discarded.
10. A correction creates a new linked assertion and does not rewrite the historical source event.

## 6. Conformance and industry boundaries

This is not a replacement for external standards conformance:

- DCSA Port Call conformance tests Port Call API requirements and role-specific behavior; this profile tests whether a particular adapter preserves the semantics it claims to map.
- IMO Compendium mappings remain scoped to their exact circular and dataset semantics.
- IHO S-100 product adapters must pin product specification and dependency closure; S-131 harbour/service information is not live berth inventory, and S-127 traffic-management information is not a clearance.
- IALA MCP / MRN / MSR and IPCSA NoTN are existing interoperability structures to compose with, not registries or messaging fabrics to clone.
- A successful semantic mapping does not prove source authenticity, legal authority, physical truth, current availability, provider commitment or fulfilment.

The comparison unit is a declared source-profile/target-profile/adapter-version/role combination. Coverage and correctness are reported separately. NotTested is not PASS; NotApplicable needs a reason; Unsupported remains visible.

## 7. Initial corpus

The companion adapter-semantic-fidelity-cases-v0.1.json contains 50 declarative synthetic scenarios across identity, authority, currentness, uncertainty, nullability, money, FX/refunds, units, time, localization, provenance, privacy, reversibility and profile versioning.

**Status:** scenario manifest only. It is not yet an executable byte-level corpus and no adapter has passed it. The independent oracle, materialized input/output fixtures, mutation tests and Rust evaluator are subsequent deliverables under #5963.

Every executable case must ultimately bind:
- exact source and target profile pins;
- source input bytes/digest and expected target semantic projection;
- adapter implementation/version/digest;
- exact mapping-ledger record(s);
- expected disposition and invariant(s);
- evaluator/oracle version and result digest.

Do not add real provider payloads without operator authorization, data minimization, source terms review, and rights/privacy assessment.

## 8. Measurement

Report these families separately; do not create a single aggregate winner:

- semantic agreement by field, claim type and source-profile role;
- false identity/scope, authority, currentness, certainty and completeness promotions;
- declared information loss versus independently detected loss;
- round-trip fidelity under each declared equivalence relation;
- ambiguity/unsupported-state preservation;
- cross-implementation and metamorphic agreement;
- mutation-test sensitivity and surviving counterexamples;
- privacy-field over-disclosure;
- adapter onboarding time/code and maintenance cost after source-profile revision;
- operator reconciliation burden attributable to mapping ambiguity/loss.

A system that rejects every input is not interoperable. Positive controls, supported mappings, unknown values, ambiguous values and intentionally lossy mappings must all be measured.

## 9. Acceptance sequence

1. Review this vocabulary against the #5946 mapping-ledger schema and avoid a silent schema migration.
2. Materialize the 50 scenarios into exact, deterministic input/output vectors with positive controls.
3. Build an independent oracle that does not import production adapter or verifier logic.
4. Implement a standalone, effects-disabled Rust evaluator composing with #5954 and #5921.
5. Add one abstract synthetic provider pair before any provider-specific payload is admitted.
6. Run deterministic mutations for each invariant and record seeded counterexamples.
7. Execute exact-head formatting, tests, Clippy and corpus-digest checks; record skipped/queued/stale as non-pass.
8. Only then add authorized provider samples and a matched operator study.

## 10. Claim ceiling

A PASS establishes only semantic mapping invariants for the exact pinned synthetic/source-profile fixtures and implementations. It does not establish provider partnership, source authenticity, legal authority, physical truth, booking availability, contractual commitment, fulfilment, regulatory compliance, production interoperability or field benefit.
