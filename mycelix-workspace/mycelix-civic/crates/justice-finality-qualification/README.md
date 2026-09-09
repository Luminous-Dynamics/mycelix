# Justice Finality Qualification v0.1

This crate performs pure, positive-evidence qualification of finality for exact Mycelix Justice decisions.

It exists because the active Justice runtime still exposes weak mutable convenience state (`Appeal.status`, `Decision.finalized`) and because an empty local appeal query cannot establish global absence in an eventually consistent DHT.

## Position in the authority chain

```text
authenticated Justice finality evidence
        ↓
justice-finality-qualification
        ↓
QualifiedJusticeFinalityV1
        ↓
future sealed integration into justice-resolution-verifier
```

`QualifiedJusticeFinalityV1` has private fields and no public constructor. Only this crate's qualifier can mint a positive value.

The current #339 monetary-remedy verifier has **not yet been migrated** to require this sealed token, so issue #358 remains open until that integration removes caller-declared `CurrentAppealStateV1` from the production qualification path.

## v0.1 path A — complete no-appeal coverage

`CompleteAppealCoverageEvidenceV1` is a positive completeness/checkpoint claim supplied by the owning Justice runtime. It binds:

- exact coverage artifact ref;
- exact Decision ref;
- exact authority-evidence ref;
- `justice.complete-appeal-coverage@1`;
- exact covered time interval;
- the complete observed appeal-ref set for that interval.

The qualifier requires:

- exact semantic profile/version;
- coverage starts no later than Decision rendering;
- coverage extends through qualification time;
- qualification is at/after the appeal deadline;
- every observed appeal ref is non-empty and unique;
- the exact observed set is empty.

A local `get_links(...).is_empty()` result is not sufficient authority to instantiate `CompleteAppealCoverageEvidenceV1`. The runtime still needs a separately qualified completeness/checkpoint mechanism under issue #358.

## v0.1 path B — terminal appeal affirmance

The resolved-appeal path requires:

- `AuthenticatedAppealFilingEvidenceV1` under `justice.appeal-filing@1`;
- exact Decision ref;
- exact appellant ref;
- appeal number exactly `1`;
- filing not before Decision rendering;
- filing not after the exact appeal deadline;
- filing not in the future relative to qualification;
- `AuthenticatedTerminalAppealResolutionEvidenceV1` under `justice.terminal-appeal-resolution@1`;
- exact resolution ref;
- exact appellate authority-evidence ref;
- exact appeal/Decision correspondence;
- resolution not before filing;
- resolution not after qualification.

`Affirmed` may mint positive finality.

`Changed` denies finality of the original Decision/remedy. The new current remedy must be represented through a forward Justice lineage rather than rewriting the old Decision.

Second-level/multi-level appeals are intentionally unsupported in v0.1 and require a future semantic profile/version.

## Receipt

Every positive result retains a `JusticeFinalityQualificationReceiptV1` binding:

- exact Decision ref;
- `justice.finality-qualification@1`;
- explicit qualification time;
- exact coverage evidence + authority basis, or exact Appeal + terminal resolution + authority basis.

This preserves why finality qualified instead of collapsing the conclusion into `finalized=true`.

## Authority boundary

This crate is pure. It does **not** prove:

- that any referenced DHT action exists;
- that a coverage artifact is globally complete;
- that an appellate authority is legitimate;
- that a runtime appeal resolution was authored by authorized reviewers;
- that a local negative query is complete;
- Finance execution or settlement;
- Business closure.

The owning Justice runtime must authenticate all evidence before it reaches this qualifier.

## v0.1 adversarial invariants

The corpus denies:

- blank coverage or authority refs;
- wrong coverage profile/version;
- partial coverage that starts after the Decision;
- stale coverage ending before qualification;
- qualification before the appeal deadline;
- any observed appeal on the no-appeal path;
- duplicate observed appeal identities;
- wrong Decision binding;
- late appeals;
- second-level appeals;
- resolution before filing;
- future resolution evidence;
- appeal/resolution identity mismatch;
- terminal-resolution profile/version drift;
- changed/reversed/remanded original Decision semantics.

The same exact positive basis must produce the same token and receipt.
