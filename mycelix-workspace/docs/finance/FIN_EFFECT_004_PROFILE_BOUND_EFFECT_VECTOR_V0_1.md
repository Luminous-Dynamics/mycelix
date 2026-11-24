# FIN-EFFECT-004 — Profile-bound economic effect vector projection v0.1

**Status:** pure Rust kernel implementation and synthetic conformance corpus. This is not an admission service, ledger, settlement adapter, or production finality proof.

## Purpose

FIN-EFFECT-001 provides stable obligation identity, immutable intent fingerprints and owner/role effect identities. FIN-EFFECT-004 composes those primitives into a profile-scoped projection over multiple independently owned effects.

A single economic obligation may have a payer debit, beneficiary claim, pool receipt, supplier liability, external settlement observation, resource delivery and beneficiary acceptance. These effects are not interchangeable and do not advance as one global transaction status.

The projection is a deterministic function over a caller-declared claim profile and supplied evidence. It reports only whether the evidence supports that exact profile's required-effect set.

## No second operation/effect identity

For each required slot, the kernel derives the existing FIN-EFFECT-001 effect identity:

\`EffectIdentityV1::derive(operation_id, role, effect_owner)\`

No alternate operation-ID or effect-ID algorithm is introduced.

A separate **claim-profile content digest** binds the caller's profile definition. This is not an economic operation or effect identity. It prevents an unchanged profile digest from being accidentally reused after the profile's required roles or scope change.

## Canonical claim-profile digest v1

Domain separator bytes (including the terminal NUL):

\`mycelix:economic-claim-profile:v1\0\`

The digest is BLAKE3-256 over the concatenation:

1. domain separator bytes;
2. schema version \`1\` as \`u16be\`;
3. \`profile_id\` as \`u32be(byte_length) || UTF-8 bytes\`;
4. \`claim_name\` as \`u32be(byte_length) || UTF-8 bytes\`;
5. raw 32 bytes of the FIN-EFFECT-001 operation ID;
6. raw 32 bytes of the immutable intent fingerprint;
7. required-effect count as \`u32be\`;
8. for each required effect, sorted by raw derived effect-ID bytes:
   - raw 32 bytes of that FIN-EFFECT-001 effect ID;
   - role name as \`u32be(byte_length) || UTF-8 bytes\`;
   - effect owner as \`u32be(byte_length) || UTF-8 bytes\`.

Required role/owner duplicates are rejected. Input ordering does not alter the digest. Component lengths are bounded by the existing FIN-EFFECT-001 kernel limit. The profile digest is verified again before projection.

### Frozen known-answer vector

- \`profile_id\`: \`claim:bilateral-payment:v1\`
- \`claim_name\`: \`bilateral_payment_reconciled\`
- operation ID: \`15a08e9c3bf2599c15b37cb700698aec1898cd4c66b15ebf5208568e4953f204\`
- intent fingerprint: \`6a52f1479b7a407d6de4f2650b61bf3203741c80e28f836db523f58625e62235\`
- required effects:
  - \`beneficiary_claim\`, owner \`did:mycelix:bob\`, effect ID \`35ddfe65d8d3beb9ce06fcb1245802d0cc7799d8366bef9087ee5f6d63e6f24b\`
  - \`payer_debit\`, owner \`did:mycelix:alice\`, effect ID \`984fcc30e27b1ccbb5b266f6b0960c5791be3a68ada8512aa0fb39dcf575e56a\`
- expected profile digest: \`5ad833aef24a53646218b7fbe43fb1d3314563b165a0233898e5a96b4e1f8e4c\`

This vector is also frozen in \`economic-effect-vector-corpus.v0.1.json\` and asserted by the exact-head workflow.

## Projection semantics

The kernel accepts:
- one versioned \`CompositeClaimProfileV1\`;
- the exact required-effect set derived from the profile;
- at most one \`EffectEvidenceBatchV1\` for each required effect;
- each batch's evidence-completeness declaration and owner-effect observations.

It returns one deterministic \`EffectVectorProjectionV1\` with canonical effect-ID ordering.

Aggregate precedence is:

\`Conflict > Indeterminate > Pending > SupportedWithinDeclaredScope\`

- **Conflict:** contradictory receipt, effect digest or intent fingerprint exists for any required effect.
- **Indeterminate:** a required batch is missing or the supplied evidence view is incomplete.
- **Pending:** a required effect was not observed in a view the adapter declared complete.
- **SupportedWithinDeclaredScope:** every effect required by this exact profile was observed consistently.

A complete empty view is Pending; a missing batch is Indeterminate. Neither is permission to admit a first effect. Conflict dominates incomplete evidence in another slot.

Unexpected effect batches, duplicate batches, empty profiles, duplicate required roles/owners and profile-digest mismatch are rejected.

## Meaning of SupportedWithinDeclaredScope

This result means only:

> The supplied observations are internally consistent with every required effect named by the supplied, digest-verified claim profile.

It does **not** mean:
- the profile was approved by the correct governance or policy authority;
- the profile is semantically complete for the real-world claim;
- source observations are authentic;
- a DHT/registry view is globally complete;
- the owner committed a production Holochain mutation;
- an external bank or payment rail settled;
- goods or services were physically delivered or accepted;
- legal discharge, spendability or universal completion is established.

Profile design and approval remain an explicit authority responsibility. The kernel verifies profile bytes against the supplied digest; it does not certify the profile's legitimacy.

## Tests and adversarial corpus

The pure crate tests cover:
- all required effect slots present;
- payer observed / beneficiary not observed;
- complete empty view vs missing/incomplete batch;
- conflict dominating indeterminate;
- intent-fingerprint mutation;
- duplicate required roles and duplicate evidence batches;
- unexpected effect batches;
- different claim scopes for the same operation;
- deterministic projection ordering;
- profile digest order invariance;
- profile-scope mutation rejection;
- profile-digest known-answer vector.

\`economic-effect-vector-corpus.v0.1.json\` contains 16 public synthetic adversarial cases and the canonical claim-profile digest KAT. The companion schema is closed Draft 2020-12. Public cases are development fixtures, not hidden evaluation data or performance results.

## Benchmark targets (not achieved results)

- 0 false \`SupportedWithinDeclaredScope\` results on in-profile adversarial cases.
- 100% conflict detection for conflicting required-effect evidence.
- 0 missing/incomplete slots promoted to supported.
- 0 duplicate economic effects created through repeated physical observations.
- 100% byte-identical projection and profile digest for identical frozen inputs, regardless of input order.
- Measure p50/p95/p99 CPU latency by required-effect and receipt count; report evidence-read amplification and memory use separately.
- Compare against a baseline that collapses everything into a linear transaction status, and a native-standard mapping without typed claim profiles, on the same fixtures. Report false-positive completion and review burden, not only throughput.

## Composition with the Economic OS

This is a shared pure semantic layer for SAP, TEND, Hearth, grants, procurement, payroll, external instruments, Valueflows and resource fulfilment. Each domain supplies its own reviewed claim profile and remains the canonical owner of its entries and local effects. A \`supplier_payable_supported\` profile must not be substituted for \`bank_settlement_final\`; a \`delivery_observed\` profile must not be substituted for \`beneficiary_acceptance\`.

The projection joins records but never replaces Valueflows IDs, ISO 20022 message IDs, EPCIS event IDs, Peppol invoice IDs, accounting journal IDs or owner-source-chain action hashes.

## Remaining work

- Independent full Draft 2020-12 validator over all corpus/schema bundles.
- Property-based and fuzz tests for role/owner permutations, duplicate and conflicting receipt populations, component-length boundaries and digest mutations.
- Cross-language KAT reproduction in an independently maintained reference implementation.
- Real owner-adapter tests for Holochain source-chain atomicity, forks/incomplete DHT observations, recovery and external-rail reconciliation.
- Approved claim-profile registry and review process; the pure kernel does not implement governance.

## Claim ceiling

This is a deterministic pure evidence projection and profile-integrity primitive. It does not authorize a first effect, create or settle value, establish global uniqueness, authenticate external observations, prove legal ownership, accounting compliance, bank settlement, physical delivery or production readiness.
