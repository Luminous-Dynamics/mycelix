# Mycelix Accountability Evidence Bundle v1

Status: **research / interoperability contract**. This document freezes the byte-level commitment profile implemented by `mycelix-accountability-bundle`; it is not a compliance certification or a substitute for independent cryptographic review.

## 1. Evidence DAG

The v1 graph is deliberately acyclic:

1. `receipt_statement` — the canonical pre-attestation `AccessReceipt` commitment. It excludes every `AttestationRef`.
2. non-witness provider proofs — `ExecutionBinding`, `ComputationProof`, and `PolicyProof` as required by the deployment. Each binds `receipt_statement` and is cryptographically verified before its authority identity is accepted.
3. `pre_witness_evidence_bundle` — commits the exact trust-qualified non-witness evidence composition.
4. `ExternalWitness` proofs — bind `pre_witness_evidence_bundle`, **not** `receipt_statement`.
5. `finalized_evidence_bundle` — archival commitment over the pre-witness statement, witness trust policy, and complete resolved witness set.

Witness refs are excluded from step 3, preventing a witness -> bundle -> witness hash cycle.

## 2. Policy separation

`AccountabilityPolicy.required_attestation_roles` governs the semantic/non-witness receipt qualification used to create `TrustVerifiedReceipt`.

`ExternalWitnessTrustPolicy` governs the later external-witness stage.

A high-assurance deployment therefore does **not** put `ExternalWitness` into the policy used to create the pre-witness `TrustVerifiedReceipt`. The witness requirement is applied after the provider bundle exists. This distinction is intentional: a witness cannot attest a bundle that cannot be formed until the witness already exists.

The pre-witness commitment binds the exact accountability-policy digest and exact non-witness trust-policy digest. The final archival commitment additionally binds the exact external-witness trust-policy digest.

## 3. Trust identity rule

`scheme` and `verifier_profile` strings are proof claims, not roots of trust.

The commitment includes them only after a cryptographic provider has checked them and returned:

- `verifier_id` — stable fingerprint of the actual verifier/key/hardware root; and
- `trust_domain_id` — stable identifier for the independently administered trust domain derived from trusted configuration, credentials, hardware roots, or governance state.

Never derive `trust_domain_id` from an untrusted field supplied inside the proof being verified.

## 4. Role tags

Canonical encoded role tags are:

| Role | Tag |
|---|---:|
| `ExecutionBinding` | `0x01` |
| `ComputationProof` | `0x02` |
| `PolicyProof` | `0x03` |
| `ExternalWitness` | `0x04` |

The pre-witness set MUST NOT contain role `0x04`.

## 5. Primitive encodings

- All commitments and authority IDs are exactly 32 raw bytes.
- Integer counts and string lengths are unsigned 64-bit **big-endian** values.
- Strings are their exact UTF-8 bytes, prefixed by an unsigned 64-bit big-endian byte length.
- Boolean policy flags are one byte: `0x00` false, `0x01` true.
- No JSON, CBOR, bincode, protobuf, platform ABI layout, or locale-sensitive representation participates in these commitment preimages.

## 6. Verified non-witness record

Each record is encoded as the concatenation:

1. `role_tag` — 1 byte;
2. `statement_digest` — 32 bytes;
3. `proof_digest` — 32 bytes;
4. `scheme_length` — u64 BE;
5. `scheme` — UTF-8 bytes;
6. `verifier_profile_length` — u64 BE;
7. `verifier_profile` — UTF-8 bytes;
8. `verifier_id` — 32 bytes;
9. `trust_domain_id` — 32 bytes.

Before hashing, records are sorted lexicographically by the tuple:

`(role, statement_digest, proof_digest, scheme UTF-8, verifier_profile UTF-8, verifier_id, trust_domain_id)`.

Duplicate proof digests are invalid; they are not deduplicated silently.

## 7. Pre-witness evidence bundle commitment

Hash algorithm: BLAKE3-256.

Domain:

`mycelix:accountability-pre-witness-evidence-bundle:v1`

Codec label:

`mycelix-accountability-pre-witness-bundle-v1`

Preimage, in order:

1. domain bytes;
2. `0x00`;
3. codec-label bytes;
4. `0x00`;
5. `receipt_statement` — 32 bytes;
6. exact `accountability_policy_digest` — 32 bytes;
7. exact non-witness `trust_policy_digest` — 32 bytes;
8. verified-record count — u64 BE;
9. sorted verified non-witness records encoded as section 6.

The resulting 32 bytes are the statement every `ExternalWitness` must verify/sign/attest.

Changing any verified proof artifact, proof scheme/profile, verifier root, administrative trust domain, accountability policy, non-witness trust policy, or semantic receipt statement changes this digest.

Reordering the same evidence does not.

## 8. External witness trust-policy commitment

Hash algorithm: BLAKE3-256.

Domain:

`mycelix:accountability-external-witness-policy:v1`

Codec label:

`mycelix-accountability-external-witness-policy-v1`

Preimage:

1. domain bytes;
2. `0x00`;
3. codec-label bytes;
4. `0x00`;
5. `min_verified_witnesses` — u8;
6. `min_distinct_verifiers` — u8;
7. `min_distinct_trust_domains` — u8;
8. `require_verifier_disjoint_from_non_witness` — bool byte;
9. `require_trust_domain_disjoint_from_non_witness` — bool byte.

All three thresholds must be non-zero. Distinctness thresholds cannot exceed the witness-count threshold.

## 9. Final archival evidence-bundle commitment

Hash algorithm: BLAKE3-256.

Domain:

`mycelix:accountability-final-evidence-bundle:v1`

Codec label:

`mycelix-accountability-final-evidence-bundle-v1`

Preimage:

1. domain bytes;
2. `0x00`;
3. codec-label bytes;
4. `0x00`;
5. `pre_witness_evidence_bundle` digest — 32 bytes;
6. external-witness trust-policy digest — 32 bytes;
7. witness-record count — u64 BE;
8. sorted verified witness records encoded exactly as section 6, with role tag `0x04`.

This digest is archival. It is not recursively required to witness itself.

## 10. Verification invariants

A conforming verifier MUST fail closed when:

- the final receipt semantic statement differs from the base trust-qualified receipt statement;
- the accountability policy differs from the one used to qualify the base;
- the base contains `ExternalWitness` evidence;
- the final receipt adds, removes, or changes any non-witness proof ref after base qualification;
- any proof digest is duplicated in the final evidence set;
- a witness binds the receipt statement or a stale/other bundle rather than the exact pre-witness digest;
- witness cryptography fails;
- a resolved verifier or trust-domain ID is all-zero;
- witness count/distinctness thresholds are not met; or
- configured witness/non-witness verifier or trust-domain disjointness is violated.

## 11. Ordering and time non-claim

This profile proves **what evidence composition an external witness attested**. A signature alone does not prove trustworthy wall-clock time or prove that an application did not bypass its release gate.

For a durable historical claim that all required evidence existed before protected disclosure, deployments still need both:

1. an independently trustworthy order/time source in the witness proof (for example an append-only transparency log, consensus ledger, TSA, or hardware-backed monotonic source); and
2. an execution release boundary that refuses protected output until a valid `PreDisclosureVerifiedBundle` exists and durably records/anchors the release decision.

That release-gate binding is the next protocol layer; it must not be inferred merely from possession of a witness signature.
