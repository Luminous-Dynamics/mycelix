# NRVDEF Executable Evidence-Custody State Machine v0.1

**Status:** deterministic synthetic conformance evaluator; non-operational.  
**Parent:** issue #5613; related to NRVDEF architecture PR #5589 and evidence specification PR #5595.  
**Package:** `benchmarks/nrvdef-custody`.

## 1. Objective

This benchmark turns the NRVDEF evidence-boundary specification into a runnable evaluator for generic, synthetic claim histories. It validates event identity, sequence, predecessor references, protocol binding, role/authority boundaries, dispute preservation and summary-state claims.

It does not model a weapon, a facility, material accounting, a physical dismantlement procedure, or inspection methods. The event digest fields are **symbolic fixture tokens**: the evaluator tests equality of predecessor references, not cryptographic hashing, digital signatures, real credential validation, or physical truth.

## 2. State separation

The evaluator preserves these independent dimensions:
- record integrity status;
- actor role and abstract authority status;
- protocol version;
- claim identity binding;
- event ordering and custody references;
- externally referenced inspection attestation;
- externally referenced treaty-institution determination;
- unresolved dispute state;
- protected-data disclosure authorization.

The words `inspection_attested` and `treaty_determined` describe an imported event from an external authority in a fictional fixture. They are not generated from a claimant declaration, a hash, an evidence receipt, or an evaluator score.

## 3. Event model

Each event contains:
- history ID, claim ID and event ID;
- sequence number;
- predecessor digest token and event digest token;
- protocol version;
- actor role;
- authority status;
- event kind;
- record-integrity state;
- claim binding flag;
- external reference;
- protected-data disclosure and authorization flags;
- final reported summary state (only the final event is used for history-state comparison; intermediate fixture rows use `pending`).

Supported event kinds:
- `declaration`;
- `evidence_received`;
- `inspection_attestation`;
- `treaty_determination`;
- `dispute_opened`;
- `dispute_resolved`.

## 4. Derived state rules

The evaluator derives a history summary only from accepted events:

1. A valid declaration requires a declarant role, valid authority, valid record integrity and claim binding.
2. Evidence receipt requires a valid declaration followed by a custodian event with valid authority, integrity and claim binding.
3. Custody is complete only after both declaration and evidence receipt in a valid, uninterrupted chain.
4. Inspection attestation is accepted only from an `authorized_inspector` role with valid authority, claim binding and a non-empty external reference after custody completion.
5. Treaty determination is accepted only from a `treaty_institution` role with valid authority, claim binding and a non-empty external reference after a valid inspection attestation.
6. An unresolved dispute takes precedence in the derived public summary. It remains `disputed` until a valid treaty-institution resolution event is appended.
7. A claimant or custodian may open a dispute in this abstract fixture when their authority and claim binding are valid; only a valid treaty-institution event may resolve it.
8. Protected data disclosed without authorization is always reported as a violation.
9. The verifier never creates an inspection attestation or treaty decision. It only checks the referenced external event in the synthetic event chain.

## 5. Frozen 12-history fixture

| History | Intended test |
|---|---|
| valid-custody | Valid declaration plus evidence receipt; ends custody-complete |
| claimant-forged-inspection | Claimant attempts inspector status |
| authorized-inspection | Valid externally referenced inspector attestation |
| software-treaty | Software attempts a treaty determination after valid inspection |
| dispute-suppression | Open dispute is hidden in final summary |
| broken-custody | Incorrect predecessor token |
| replayed-event | Duplicate event ID |
| revoked-inspector | Inspector authority has been revoked |
| protected-disclosure | Protected evidence disclosed without permission |
| declaration-overclaim | Authentic declaration mislabeled as inspection-attested |
| authorized-treaty | Valid inspector and treaty-institution references |
| claim-binding-mismatch | A later event refers to a different claim ID |

There are 30 synthetic event rows. The fixture is deliberately small and has known negative controls; it is not a real evidence archive or treaty process.

## 6. Expected fixture oracle

| Counter | Expected |
|---|---:|
| Missing lineage | 0 |
| Duplicate event IDs | 1 |
| Broken custody links / sequence | 1 |
| Invalid record-integrity events | 0 |
| Claim-binding mismatches | 1 |
| Protocol-version mismatches | 0 |
| Invalid declaration events | 0 |
| Invalid evidence events | 0 |
| Unauthorized inspection attestations | 2 |
| Unauthorized treaty determinations | 1 |
| Inspection-status overclaims | 3 |
| Treaty-status overclaims | 1 |
| Custody-complete overclaims | 3 |
| Dispute suppressions | 1 |
| False dispute states | 0 |
| Protected-data disclosures | 1 |
| Final summary-state mismatches | 8 |
| Valid custody histories | 7 |
| Valid inspection-attested histories | 3 |
| Valid treaty-determined histories | 1 |

A fixture-oracle PASS means these exact negative controls were detected. It does not mean all histories are valid or that any physical event occurred.

## 7. Reproduction

From the Mycelix repository root:

```bash
cargo test --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/nrvdef-custody/Cargo.toml
cargo run --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/nrvdef-custody/Cargo.toml
```

The CSV fixture is pinned by an FNV-1a regression fingerprint in the Rust test suite. The digest values in the fixture are symbolic linkage tokens, not cryptographic proof; a production custody protocol would require a separately specified and reviewed cryptographic envelope and key lifecycle.

## 8. What the benchmark still does not prove

- It does not verify signatures, public-key validity, credential revocation against a real authority, or legal jurisdiction.
- It does not inspect physical objects or certify physical dismantlement.
- It does not determine whether a treaty obligation has been fulfilled.
- It does not guarantee the correctness or completeness of submitted evidence.
- It does not make a treaty-institution decision.
- It does not replace human inspection, safeguards, diplomacy, law, or dispute resolution.

Digital provenance is evidence about records and custody. It is not a truth oracle. A trustworthy system must retain uncertainty, dispute and authority boundaries, even when a clean status would be easier to display.
