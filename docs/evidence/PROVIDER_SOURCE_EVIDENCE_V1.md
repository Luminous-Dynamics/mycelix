# EVIDENCE-CI-004E Provider Source Evidence V1

This layer binds one already-authenticated provider-source evidence identity to the raw source record consumed by MYC-EVID-001B.

It exists between acquisition/authentication and source normalization:

```text
provider acquisition/authentication
        ↓
SourceEvidenceEnvelopeV1
        ↓
provider/source/run/attempt/reference binding
        ↓
MYC-EVID-001B source normalization
```

## Authority boundary

This module does not acquire GitHub data and does not decide whether GitHub told the truth.

```text
provider said X
!= X is independently true
```

It proves only that a bounded normalization consumed the exact provider-evidence identity/lineage declared by a registered acquisition policy.

It has no HTTP client, token access, workflow dispatch, runner control, process execution, repository mutation, publication authority, or theorem-PASS authority.

## Registered policy

A policy freezes:

- provider profile;
- acquisition profile;
- source kind;
- repository ID.

The envelope cannot choose these values for itself.

## Envelope identity

`SourceEvidenceEnvelopeV1` binds:

- provider profile;
- acquisition profile;
- source kind;
- repository ID;
- exact provider object references;
- run ID;
- run attempt;
- opaque evidence ID supplied by the acquisition layer.

The module deliberately does not invent another hash algorithm for provider evidence.

## Provider object references

Provider objects are represented as bounded `{kind, reference}` pairs. Their ordering is not semantic; validation canonicalizes them by kind/reference.

For the GitHub qualification-source binding:

- every provider object reference in the envelope must also appear in the raw source's `source_references`;
- a receipt-bearing source must retain a `qualification_receipt` or `qualification_artifact` provenance object;
- any assertion stronger than `unproven` must retain a `tested_subject_proof` provenance object.

These requirements do not validate the contents of those provider objects. They ensure authority-bearing source fields cannot appear with no authenticated provenance slot at all.

## Run/attempt and source-ID binding

The raw source must agree exactly with the envelope on:

```text
repository_id
run_id
run_attempt
source_evidence_id
```

A source from another run or rerun cannot be silently relabeled under the envelope.

## Evidence-ID reuse

Within a bounded evidence set, one opaque evidence ID may either:

- identify one envelope; or
- repeat the exact same envelope.

It may not identify two different provider-source envelopes.

A later run/attempt should therefore receive a new evidence identity from the acquisition layer.

## Relationship to MYC-EVID-001B

`normalize_bound_github_qualification_source_v1(...)` first validates and binds the provider envelope, then invokes the existing MYC-EVID-001B normalizer.

The child does not change MYC-EVID-001B's theorem:

```text
exact subject + completed success + bound positive receipt -> PASS
run-level failure alone -> NOT_ASSESSED
unproven tested subject -> UNSUPPORTED
```

## Receipt boundary

This layer does not parse arbitrary receipt formats or independently prove that a receipt artifact's bytes match its declared subject/profile. The acquisition/receipt-verification layer must establish that before assigning the provider evidence identity.

The envelope only requires that a receipt-bearing source retain a receipt/artifact provenance reference and that the same source evidence identity binds the run/attempt.

## Tested-subject proof boundary

Likewise, this layer does not infer an exact checkout merely from `run_head_sha` or a caller label. A non-`unproven` exact-subject assertion must retain a `tested_subject_proof` provenance reference supplied by the authenticated acquisition layer.

A future qualified proof-profile adapter may define how checkout logs, detached replay receipts, or equivalent evidence establish that proof object.

## Canonical bytes

`canonical_bound_source_bytes_v1(...)` accepts only raw policy/envelope/source inputs and runs the full validation + #1670 normalization path before serializing the resulting bound object.

There is no public authority-bearing shortcut that accepts an arbitrary prebuilt bound dictionary.

## Claim ceiling

Even after independent qualification, this module may establish only:

`QualifiedProviderSourceEvidenceIdentityV1`

for the exact provider/profile/envelope/binding semantics.

It does not establish:

- provider incorruptibility;
- receipt contents beyond supplied qualified acquisition evidence;
- exact checkout proof beyond supplied qualified provenance;
- theorem PASS by itself;
- theorem FAIL from run-level failure;
- runner trust;
- cross-run composition;
- publication authority;
- deployment truth.
