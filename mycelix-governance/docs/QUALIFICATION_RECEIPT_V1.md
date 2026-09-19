# Exact Qualification Receipt V1

`MYC-CONST-QREC-001` defines a reusable, inert evidence identity for exact hosted qualification receipts.

It exists because an opaque string such as `receipt-123` can be commitment-bound without proving which semantic subject, verifier, hosted run, and retained artifact actually earned qualification.

## Authority-bearing tuple

A `QualificationReceiptV1` binds exactly:

- `dependency_id`
- `semantic_head`
- `verifier_head`
- non-zero hosted `run_id`
- retained `artifact_digest` as `sha256:<64-lowercase-hex>`
- deterministic `receipt_commitment` as `blake3-256:<64-lowercase-hex>`

The receipt commitment is domain-separated by `MYCELIX-QUALIFICATION-RECEIPT\0V1\0` and commits to every authority-bearing field.

## Navigation metadata is not authority

The optional navigation object may carry:

- `job_id`
- `artifact_id`
- `workflow_name`
- `evidence_label`

These values are intentionally excluded from the authority-bearing receipt commitment. They help humans and tooling locate evidence, but workflow renames, PR changes, artifact storage IDs, or labels must not redefine which qualification tuple earned authority.

## Exact dependency census

`validate_exact_qualified_census()` requires a closed set of `QualificationRequirementV1` rows and rejects:

- duplicate requirements;
- duplicate evidence;
- missing evidence;
- unexpected extra evidence;
- semantic-head mismatch;
- verifier-head mismatch;
- pending evidence;
- failed evidence;
- malformed or commitment-invalid receipts.

Activation consumers therefore cannot silently accept `semantic_head` alone while ignoring the verifier that produced the actual hosted evidence.

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

QREC-001 does **not** fetch GitHub, verify GitHub credentials, inspect Actions signatures, trust caller-provided receipts, or activate any constitutional/provider behavior. A future ingestion tranche must independently verify the hosted run/artifact evidence before it can construct the verified wrapper.

## Intended consumers

The first intended consumers are:

- event admission (`E1B-R1` / `E1AR2`);
- finance provider admission (`D1D-F`);
- parameter provider admission (`D1D-P`);
- refinement/activation successor profiles (`CR2/CR2A` successors).

Each consumer remains responsible for pinning the exact semantic and verifier heads it accepts.

## Non-claims

QREC-001 does not establish:

- hosted receipt authenticity;
- deployment currentness;
- Holochain author authority;
- provider replay safety;
- runtime refinement;
- physical exactly-once effects;
- successful qualification of any existing semantic subject.
