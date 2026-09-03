# SIF Release Credential v1

Status: draft normative cross-repository profile.

Purpose: carry the fact that a Mycelix `PreDisclosureVerifiedBundle` passed semantic, cryptographic, trust-composition, and external-witness verification across the Mycelix -> Xenia boundary without requiring Xenia to parse Mycelix's complete proof graph.

## Privacy boundary

The credential MUST contain commitments only. It MUST NOT contain a subject identifier, actor/case/matter ID, purpose text, query text, legal justification text, witness names, or protected record bytes.

## Statement fields, in order

1. `credential_id` — 32 bytes, non-zero; identifies one release lineage.
2. `receipt_statement_digest` — 32 bytes.
3. `pre_witness_bundle_digest` — 32 bytes.
4. `finalized_evidence_bundle_digest` — 32 bytes.
5. `accountability_policy_digest` — 32 bytes.
6. `non_witness_trust_policy_digest` — 32 bytes.
7. `witness_policy_digest` — 32 bytes.
8. `execution_proof_digest` — 32 bytes; MUST name a trust-qualified `ExecutionBinding` in the bundle's non-witness evidence set.
9. `execution_verifier_id` — 32 bytes; resolved during proof verification, not copied from caller labels.
10. `execution_trust_domain_id` — 32 bytes; resolved from trusted authority state, not copied from caller labels.
11. result option tag: `0x00` for no result, `0x01` followed by the 32-byte `result_digest` when present.

## Canonical signature message

Authorities sign exactly:

```text
"sif:release-credential:statement:v1"
|| 0x00
|| "sif-release-credential-v1"
|| 0x00
|| "sif-release-credential-canonical-v1"
|| 0x00
|| canonical_statement_fields
```

All digests are raw 32-byte values. There are no serializer-derived bytes in the signed preimage.

## Ed25519 authority identity

The v1 helper supports `ed25519-rfc8032`. The signer key ID is:

```text
BLAKE3(
  "sif:release-credential:authority-key:v1"
  || 0x00
  || raw_32_byte_ed25519_public_key
)
```

The signature is the raw 64-byte Ed25519 signature over the canonical message.

The envelope may contain multiple signatures over the same statement. Duplicate signer key IDs MUST NOT count more than once.

## Trust policy

A signature count by itself is not a trust policy. Xenia MUST resolve each signer key ID against locally trusted release-authority configuration and apply deployment policy such as:

- minimum valid signatures;
- minimum distinct verifier/key roots;
- minimum distinct administrative trust domains;
- optional allow-list of credential/profile versions.

An unknown signer does not become trusted merely because its signature is mathematically valid.

## Release-lineage semantics

`credential_id` authorizes at most one initial Xenia release lineage. A release system MAY reuse the same credential only for an explicit retry descendant of that lineage after an `Aborted` or `Partial` terminal outcome. It MUST NOT use the same credential ID to create two unrelated initial releases.

This constraint belongs to Xenia's durable release journal because the credential itself is stateless.

## Cross-binding

Before preparing a disclosure permit, Xenia MUST require at least:

- credential `receipt_statement_digest` == execution binding `receipt_digest`;
- credential `execution_proof_digest` == the execution proof/binding identifier accepted by the Xenia adapter;
- credential `result_digest` == execution binding `result_digest`;
- the credential signature/trust policy passes;
- the credential ID is unused except for an explicitly valid retry lineage.

The Xenia permit MUST commit the credential ID and finalized evidence-bundle digest so replacing the credential after permit creation invalidates the release statement.

## Non-claims

A valid release credential does not itself prove:

- wall-clock time;
- global non-equivocation of the Mycelix or Xenia journals;
- that every application output path is gated;
- that release authorities are independently administered unless Xenia's trusted-key configuration establishes that fact.

Those are separate deployment/runtime properties and must not be inferred from possession of this credential alone.
