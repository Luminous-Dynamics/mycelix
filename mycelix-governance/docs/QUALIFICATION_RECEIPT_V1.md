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

## Exact dependency requirements

`QualificationRequirementV1` pins:

- dependency ID;
- trusted issuer ID;
- exact semantic head;
- exact verifier head.

The requirement intentionally permits a later successful run attempt of the same exact issuer + semantic + verifier lineage. The concrete receipt still records the exact run ID, run attempt, artifact digest, and receipt commitment. A promotion profile that wants exactly one canonical hosted receipt may additionally pin the receipt commitment.

## Presented evidence is not activation authority

QREC-001 exposes two deliberately distinct census APIs.

`validate_exact_presented_census()` accepts ordinary `QualificationEvidenceV1` values and proves only deterministic structure and lineage consistency. It rejects:

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

However, ordinary `QualificationReceiptV1` values are caller-constructible and deserializable. Passing the presented census therefore does **not** establish hosted CI authenticity and must not be treated as activation authority.

`validate_exact_verified_census()` accepts only `VerifiedQualificationReceiptV1`. It provides the activation-grade type boundary for a future consumer, but QREC-001 intentionally provides no production constructor for that wrapper. Until a separately-qualified ingestion/authenticity tranche exists, production code cannot manufacture the verified receipts required by this path.

This fail-closed split is intentional:

```text
presented receipt + exact deterministic census
        = structurally coherent evidence

presented receipt + exact deterministic census
        != hosted-authenticated qualification authority

VerifiedQualificationReceiptV1
        = reserved post-ingestion evidence type
```

## Typed shape is not hosted authenticity

`VerifiedQualificationReceiptV1` has private fields and does not implement `Serialize`, `Deserialize`, `Clone`, or `Copy`. Its only public method in QREC-001 exposes a shared reference to the underlying receipt.

QREC-001 does **not** fetch GitHub, verify GitHub credentials, inspect Actions signatures, trust caller-provided receipts, or activate any constitutional/provider behavior. A future ingestion tranche must independently verify the canonical issuer, exact run attempt, successful hosted job, and retained artifact digest before it can construct the verified wrapper.

## Intended consumers

The first intended consumers are:

- event admission (`E1B-R1` / `E1AR2`);
- finance provider admission (`D1D-F`);
- parameter provider admission (`D1D-P`);
- refinement/activation successor profiles (`CR2/CR2A` successors).

Each consumer remains responsible for pinning the exact issuer and semantic/verifier lineage it accepts. Activation-capable consumers must consume the verified-receipt path rather than treating presented receipts as trusted CI evidence.

## Non-claims

QREC-001 does not establish:

- hosted receipt authenticity;
- issuer authenticity merely from an `issuer_id` string;
- a production constructor for `VerifiedQualificationReceiptV1`;
- deployment currentness;
- Holochain author authority;
- provider replay safety;
- runtime refinement;
- physical exactly-once effects;
- successful qualification of any existing semantic subject.
