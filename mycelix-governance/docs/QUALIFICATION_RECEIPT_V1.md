# Exact Qualification Receipt V1

`MYC-CONST-QREC-001` defines a reusable, inert evidence identity for exact hosted qualification receipts.

It exists because an opaque string such as `receipt-123` can be commitment-bound without proving which evidence authority, semantic subject, verifier, hosted run attempt, and retained artifact actually earned qualification.

## Authority-bearing tuple

A `QualificationReceiptV1` binds exactly:

- `dependency_id`
- stable `issuer_id` for the evidence authority, for example `github-repository:1176351975`
- `semantic_head`
- `verifier_head`
- non-zero hosted `run_id`
- non-zero `run_attempt`
- retained `artifact_digest` as `sha256:<64-lowercase-hex>`
- deterministic `receipt_commitment` as `blake3-256:<64-lowercase-hex>`

The receipt commitment is domain-separated by `MYCELIX-QUALIFICATION-RECEIPT\0V1\0` and commits to every authority-bearing field.

`issuer_id` is deliberately part of qualification identity. Byte-identical commits in an unauthorized fork or a different CI evidence authority must not become interchangeable with evidence emitted by the canonical issuer.

`run_attempt` is also authority-bearing because hosted CI systems can re-run one workflow under the same run ID. Attempt 1 and attempt 2 are therefore distinct evidence events even when semantic/verifier heads are identical.

## Navigation metadata is not authority

The optional navigation object may carry:

- `job_id`
- `artifact_id`
- `workflow_name`
- `evidence_label`

These values are intentionally excluded from the authority-bearing receipt commitment. They help humans and tooling locate evidence, but workflow renames, PR changes, artifact storage IDs, or labels must not redefine which qualification tuple earned authority.

A future authenticity/ingestion layer must never use mutable navigation metadata as the trust anchor. It must verify the authority-bearing tuple and retained artifact evidence independently.

## Exact dependency census

`QualificationRequirementV1` pins:

- dependency ID;
- trusted issuer ID;
- exact semantic head;
- exact verifier head.

`validate_exact_qualified_census()` rejects:

- duplicate requirements;
- duplicate evidence;
- missing evidence;
- unexpected extra evidence;
- wrong issuer;
- semantic-head mismatch;
- verifier-head mismatch;
- pending evidence;
- failed evidence;
- malformed or commitment-invalid receipts.

The requirement intentionally permits a later successful run attempt of the same exact issuer + semantic + verifier lineage. The concrete receipt still records the exact run ID, run attempt, artifact digest, and receipt commitment. A promotion profile that wants exactly one canonical hosted receipt may additionally pin the receipt commitment.

## Typed shape is not hosted authenticity

This tranche deliberately distinguishes:

```text
QualificationReceiptV1
    exact deterministic evidence identity

        !=

VerifiedQualificationReceiptV1
    receipt that crossed a separately-qualified authenticity boundary
```

`VerifiedQualificationReceiptV1` has no public constructor and does not implement `Serialize`, `Deserialize`, `Clone`, or `Copy`.

QREC-001 does **not** fetch GitHub, verify GitHub credentials, inspect Actions signatures, trust caller-provided receipts, or activate any constitutional/provider behavior. A future ingestion tranche must independently verify the canonical issuer, exact run attempt, retained artifact digest, and successful qualification result before it can construct the verified wrapper.

## Intended consumers

The first intended consumers are:

- event admission (`E1B-R1` / `E1AR2`);
- finance provider admission (`D1D-F`);
- parameter provider admission (`D1D-P`);
- refinement/activation successor profiles (`CR2/CR2A` successors).

Each consumer remains responsible for pinning the exact issuer and semantic/verifier lineage it accepts.

## Non-claims

QREC-001 does not establish:

- hosted receipt authenticity;
- issuer authenticity merely from an `issuer_id` string;
- deployment currentness;
- Holochain author authority;
- provider replay safety;
- runtime refinement;
- physical exactly-once effects;
- successful qualification of any existing semantic subject.
