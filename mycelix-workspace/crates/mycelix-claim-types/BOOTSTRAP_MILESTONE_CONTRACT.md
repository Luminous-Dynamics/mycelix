# Bootstrap milestone provenance contract

This crate defines the Mycelix-side, runtime-independent schema for industrial-bootstrap milestone claims. It extends the existing shared claim-types package rather than coupling the record to Symtropy or a Holochain coordinator.

## Contract contents

A versioned BootstrapMilestone binds:

- prior and resulting bootstrap-state SHA-256 digests;
- claimed capability and closure stage;
- source repository, exact source commit and scenario-manifest digest;
- typed evidence references with content digests;
- dependency-state deltas;
- signed imported-mass and energy deltas with fixed integer units;
- typed resource statements;
- explicit failure/recovery outcomes, including rejected_safely and NotRun;
- policy identity, digest and scope;
- append-only correction/retraction links.

## Evidence rules

Synthetic software contracts, model-derived claims, ground tests, relevant-environment tests, operational measurements and physical qualification are different FixtureClass values. Validation requires evidence types appropriate to the declared class. Physical qualification requires both a qualification report and an independent review.

Resource estimates and resource observations are separate enum variants. A measured-inventory delta is a third variant tied to a batch and must reference a mass measurement, inventory reconciliation or supply-chain receipt. Synthetic and model-derived milestone claims cannot assert measured observations or mutate accepted physical inventory. No scalar trust/confidence score is used as a substitute for evidence.

Corrections and retractions are new records with a supersedes_milestone_id; the old record must not be overwritten. This data type checks the link's shape, not the existence of the prior record or immutability of a storage backend.

## Validation

From the repository root:

```sh
cargo fmt --manifest-path mycelix-workspace/crates/mycelix-claim-types/Cargo.toml -- --check
cargo test --manifest-path mycelix-workspace/crates/mycelix-claim-types/Cargo.toml
```

A dedicated GitHub Actions workflow runs these commands because the standalone claim-types package is excluded from the larger Mycelix workspace's normal member graph.

## Security and claim ceiling

The schema fails closed on unsupported schema versions, malformed digests, unknown serialized fields, duplicate dependency changes, no-op dependency deltas, conflicting correction links, incomplete recovery outcomes and inventory mutations lacking measurement evidence. Tests include a deserialization negative control for an attempted unknown authority-grant field and parse a canonical synthetic JSON fixture. The fixture uses invalid example-domain links and placeholder digests so it cannot be mistaken for evidence from a real run.

Validation does **not** fetch or hash external content, verify signatures, establish the truth of evidence, prove that source commits built the claimed artifact, authenticate the named policy issuer, or grant execution authority. Consuming services must independently resolve and hash evidence bytes, verify identities/signatures and policy, enforce append-only persistence, and check the scenario manifest and source revision. A well-formed record is a verifiable claim envelope—not proof that a robot or factory is physically qualified.

The evidence-kind vocabulary also includes calibration certificates, requirement traceability matrices, Verification Control Documents, hazard analyses, software-assurance reports, independent verification reports, standards-applicability assessments and acceptance-test reports. These artifacts support a traceable qualification package, but their presence is not proof that a standard was adopted or that a requirement passed. See the [Symtropy standards crosswalk at commit 9ed9216](https://github.com/Luminous-Dynamics/symtropy/blob/9ed9216db689bc21af1035eaea188ce4622d9ecd/docs/tech/AUTONOMOUS_SPACE_INDUSTRIAL_CELL_STANDARDS_CROSSWALK_V0_1.md) for the current G0–G8 mapping and applicability boundaries.
