# FIN-EFFECT-001 — Durable Economic Operation Identity and Evidence Projection

**Version:** 0.1  
**Status:** Research contract and benchmark specification  
**Issue:** [#6588](https://github.com/Luminous-Dynamics/mycelix/issues/6588)  
**Date:** 24 November 2026

## Executive statement

An API idempotency key protects a request. A durable economic operation identity protects the **obligation and every economic effect that may fulfil it**.

Mycelix should make the economic identity survive API cache expiry, retries, process restarts, cell boundaries, asynchronous delivery, owner changes, recovery and reconciliation for the applicable accounting/legal retention interval. A response cache or webhook may expire. A completed debit must not become eligible to execute again because its response cache disappeared.

This document specifies an adapter-neutral contract. It does not claim that current Mycelix production zomes implement it, nor that a distributed system can provide global exactly-once execution without a qualified uniqueness authority. Guarantees must name their authority and failure assumptions.

## 1. Four identities, not one key

The protocol distinguishes four identities. They have different creation authorities, collision domains and retention rules.

### 1.1 Obligation identity

The obligation identity is stable before the first value-moving attempt. It is scoped by an originating authority and a namespace. It is created from an opaque stable obligation reference (for example, an originating trade ActionHash plus immutable settlement-attempt identifier, a client-supplied UUID retained by the client, or a governance-approved allocation ID).

It MUST NOT be derived solely from the full mutable request payload. Otherwise changing the amount or beneficiary would derive a new identity and evade the same-key/changed-payload conflict rule.

Conceptual derivation:

```text
operation_id = BLAKE3(
  "mycelix:economic-operation:v1\\0" ||
  u16be(schema_version = 1) ||
  lp(namespace) ||
  lp(origin_authority) ||
  lp(stable_obligation_reference)
)
```

`lp(x)` is a u32 big-endian byte length followed by exactly those UTF-8 bytes; `u16be` and `u64be` are fixed-width big-endian integers. The 32-byte terms digest is appended raw, without a length prefix. Identity-domain strings and schema versions are fixed. The stable reference is opaque and is not an authorization credential.

A production implementation must define the allowed character sets, maximum lengths, normalization rules, and collision domain for every identifier field. It must not silently Unicode-normalize or case-fold values after identity creation.

### 1.2 Intent fingerprint

The fingerprint binds the immutable economic terms, independently of the obligation identity. The source-obligation reference in the terms MUST exactly equal the stable obligation reference used to derive the operation ID; the pure kernel rejects a mismatch:

```text
fingerprint = BLAKE3(
  "mycelix:economic-intent:v1\\0" ||
  u16be(schema_version = 1) ||
  lp(settlement_profile) ||
  lp(payer_owner) ||
  lp(beneficiary_owner) ||
  lp(instrument) ||
  lp(unit) ||
  u64be(amount_minor_units) ||
  lp(purpose_code) ||
  lp(source_obligation_reference) ||
  lp(policy_revision) ||
  terms_digest_32_bytes
)
```

All value amounts are exact non-negative integer minor units with an explicit instrument and unit. Floating-point values, localized display strings, implicit FX, and rounding are forbidden in the canonical terms. FX and fees are separate typed terms with their own authority, quote identity, rounding rule and evidence.

The same operation ID + same fingerprint means the caller is referring to the original obligation. The same operation ID + different fingerprint is a conflict, not a new attempt.

Display metadata may change only when explicitly classified as non-economic and excluded from the fingerprint. The exclusion list must be versioned and reviewed; no field may be excluded merely because an implementation finds it inconvenient.

### 1.3 Effect identity

One obligation can legitimately require effects owned by different agents/services. An effect identity binds the operation ID, effect role and effect owner:

```text
effect_id = BLAKE3(
  "mycelix:economic-effect:v1\\0" ||
  u16be(schema_version = 1) ||
  operation_id_bytes ||
  lp(effect_role) ||
  lp(effect_owner)
)
```

Example roles include `payer_debit`, `beneficiary_output`, `beneficiary_claim`, `treasury_fee`, `pool_contribution`, `pool_disbursement` and `external_settlement_observation`.

Each owner can commit only its own effect within its own qualified atomic boundary. A payment is not one fictional cross-owner transaction. The protocol must prove the required effect set for its declared settlement profile.

A role must have a versioned cardinality rule. Most value-moving roles have one economic effect per operation/owner/role. Legitimate split payouts must use distinct, pre-authorized child roles or child operation identities rather than silently allowing multiple effects under one singleton role.

### 1.4 Evidence identity

Evidence identity identifies a physical fact source: Holochain ActionHash, provider receipt/reference, bank statement line, webhook event, outbox delivery, witness attestation or other observation.

Evidence references are provenance. They are not the economic operation identity. Duplicate physical observations of the same authenticated fact must not multiply value. Contradictory evidence for one effect identity produces an explicit conflict.

An evidence digest must bind the source system, source reference, schema, observed payload digest and verification profile. Never infer evidence authenticity from a reference-shaped string.

## 2. Admission and recovery rules

| Observation | Required disposition | Value mutation |
|---|---|---:|
| New operation ID + valid immutable terms | Admit under the declared authority profile | At most one per qualified effect role |
| Existing ID + identical fingerprint | Return/reconcile original lifecycle and effect references | Zero new effects |
| Existing ID + changed fingerprint | `Conflict` | Zero |
| Timeout or response loss | `Indeterminate`; query original identity | No re-keyed retry |
| Owner/authority unavailable | `Pending` or `Indeterminate` | No unauthorized mutation |
| Missing recipient claim | `ClaimPending`, not settled | No beneficiary spendability |
| Conflicting owner successors / fork evidence | `ConflictFrozen` | Freeze affected spendability |
| Duplicate outbox/webhook delivery | Reconcile by stable effect ID | Zero additional effects |
| Verified reversal/refund | New linked compensating effect | Preserve original effect history |

The operation ID is never changed to escape an unknown outcome. A retry queries and reuses the original identity. A request timeout is not proof that the original request failed.

## 3. Append-only facts; derived projections

A single mutable `completed` or `settled` boolean is not an adequate economic record. Store immutable facts and derive a typed projection from them. For example:

- `IntentObserved`
- `PayerEffectCommitted`
- `OutboxEnqueued`
- `RemoteEffectAccepted`
- `BeneficiaryOutputObserved`
- `BeneficiaryClaimed`
- `TreasuryEffectObserved`
- `ExternalSettlementConfirmed`
- `EffectDisputed`
- `CompensatingEffectCommitted`

The projection is multidimensional. Payer debit, beneficiary claim, Treasury fee and external settlement can each have a separate evidence status. `Reconciled` is profile-specific and means every effect required by that exact profile is independently evidenced.

The following claims must remain distinct:

1. API accepted the request.
2. A local ledger effect committed.
3. A beneficiary-authored claim exists.
4. A treasury/fee effect was delivered and receipted.
5. An external provider accepted an instruction.
6. External funds settled and are available to the recipient.
7. Legal finality or physical delivery is established.

No state may be promoted from one claim to another solely because an API returned HTTP 200, a webhook arrived, a reference string exists or a timeout elapsed.

Reversals and corrections are new compensating effects linked to the original operation. They do not rewrite or delete the historical effect.

## 4. Authority profiles and bounded guarantees

The operation/fingerprint format is shared; the admission authority is not.

### FinanceLedgerAtomicV1

Finance is explicitly delegated authority over both account projections. Debit, credit/payment record, effect claim and local outbox (where relevant) are committed in one local Finance source-chain transaction. Completion proves only that local Finance ledger transaction, not beneficiary-independent ownership or external settlement.

### BilateralOwnerClaimV1

The payer authors intent and debit. The beneficiary independently authors its output claim. Treasury separately owns fee delivery. The operation is settled only when the required owner legs are observed and reconciled under the profile's conflict policy.

### HearthPoolV1

A member consumes exact member-owned input value. The pool independently authors a contribution receipt under a versioned pool policy. Withdrawals require an exact allocation entitlement, pool-side disbursement, and beneficiary-authored claim. Membership is not allocation entitlement. A pool coordinator cannot write the member's balance.

### ExternalRailV1

The provider's accepted instruction, settlement receipt, recipient-available funds and any claimed legal finality are distinct evidence states. A provider reference or an accepted/processing webhook is not settlement.

For all profiles, the implementation MUST state the local serialization boundary, uniqueness authority, fork/conflict assumptions, retry behavior and nonclaims. Holochain countersigning, a locally observed link set, source-chain atomicity or an eventually consistent DHT view does not imply global uniqueness.

## 5. Retention, compaction and privacy

The obligation/effect binding persists for the applicable accounting and legal retention period, independently of API-cache lifetime.

Compaction may remove redundant delivery/observation data only if it preserves an independently verifiable tombstone with:
- operation ID and intent fingerprint;
- effect IDs and effect-owner roles;
- terminal/unknown/conflict projection and projection version;
- references or digests needed to reconstruct the accounting lineage;
- retention-policy identifier and any authorized redaction/erasure proof.

Do not promise indefinite retention. A retention profile must reconcile statutory accounting retention, privacy/deletion obligations, data minimization and the need to prevent identity reuse. Hashing low-entropy personal/economic data is not anonymization; sensitive terms should not be placed in a public identity registry. Use opaque references, access control and protected evidence storage.

If detailed evidence must be erased, retain only a legally permitted non-reversible tombstone sufficient to prevent unsafe identity reuse, or document the residual risk if even that cannot lawfully be retained. Expired or unavailable tombstones yield `Indeterminate`, never permission to create a fresh economic effect.

## 6. Cross-repository implementation sequence

1. **Pure identity kernel:** deterministic operation/effect IDs, fingerprinting, versioned canonical encoding, same-ID conflict classification and bounded input validation.
2. **Evidence projection kernel:** append-only observation admission, duplicate physical observation collapse, contradictory-fact conflict and profile-specific completion.
3. **Known-answer corpus:** stable byte vectors and digests for each canonicalization version; mutate every immutable term and expect fingerprint changes.
4. **Fault model:** response loss before/after each local commit, process restart, duplicate delivery, concurrent submissions, owner outages, stale DHT, forks, policy revocation, and compaction/retention.
5. **Hearth composition:** apply the same identity/evidence rules to FIN-SAFE-020; do not create another account/balance protocol.
6. **Holochain adapters:** integrate only after the exact conductor/HDK/HDI tuple and authority capabilities are qualified. Keep unqualified paths disabled.
7. **External rail adapter:** separately qualify provider-specific reference, settlement and reversal semantics.
8. **Independent review:** compare the pure kernel, integrity validation, coordinator and evidence artifacts against one frozen protocol version.

### 6.1 Initial pure Rust kernel

Draft [PR #6589](https://github.com/Luminous-Dynamics/mycelix/pull/6589) adds the storage-independent `finance-economic-operation` package to the Finance workspace and its public mirror. The initial slice computes the operation ID, intent fingerprint and owner/role-specific effect ID; it classifies binding and effect observations as replay, conflict, no observation, or indeterminate. It has 17 unit tests authored and exact-head format/test/Clippy plus mirror-parity steps wired into the qualification workflow.

This kernel deliberately cannot prove an observation view complete, authenticate a source author, establish a unique Holochain claim, or authorize a value-moving write. `NoBindingObserved` and `NoEffectObserved` are not admission grants. Do not promote the test count to PASS until the exact-head workflow succeeds.

### 6.2 Reuse the existing PCOE two-owner TLA+ model

Do not create a competing payment state model. Draft [PR #6556](https://github.com/Luminous-Dynamics/mycelix/pull/6556) already supplies a bounded TLA+ model, safe configuration, deliberately unsafe sender-debit mutant, and a 22-scenario coverage map. Its exact-head workflow [#45167699932](https://github.com/Luminous-Dynamics/mycelix/actions/runs/45167699932) passed on subject `1dc92170a74d08db44bacfb0a37813831256fa80`:

- 66,120 states generated; 12,105 distinct reachable states; maximum depth 19.
- Safe model: no invariant violations and no TLC specification warnings.
- Unsafe sender-debit mutant: rejected by `SenderIntentDebitAtomicity`.
- Scenario mapping: 9 bounded-modelled, 6 partially-modelled and 7 not-in-model.

This is genuine abstract model-check evidence for the two-owner protocol. It does **not** establish the new Rust kernel's correctness, uniqueness of a live Holochain claim, DHT completeness, runtime fork detection, multi-agent execution, external settlement or legal finality.

FIN-EFFECT-001 should extend and crosswalk that model rather than duplicate it. The next formal increment is to make the state variables explicit for (a) obligation identity vs immutable intent fingerprint, (b) role/owner effect identities, (c) provider/API replay windows vs durable tombstone retention, and (d) evidence completeness vs missing observations. Preserve the existing counterexample-producing negative control. Add provider API v1/v2 and Adyen expiry/scope profiles as environment parameters rather than hardcoding one provider's semantics into the core theorem. The 33-case corpus remains a language-independent conformance layer; model coverage must be recorded per case, including the 7 currently not-in-model cases, not inferred from corpus presence.

### 6.4 Canonical known-answer vectors

The v0.1 corpus now carries four permanent BLAKE3-256 known-answer vectors and the Rust kernel asserts each digest. They freeze the encoding across languages and independently authored implementations:

| Vector | Canonical object | Expected digest |
|---|---|---|
| KAT-001 | Operation identity | `15a08e9c3bf2599c15b37cb700698aec1898cd4c66b15ebf5208568e4953f204` |
| KAT-002 | Immutable intent fingerprint | `6a52f1479b7a407d6de4f2650b61bf3203741c80e28f836db523f58625e62235` |
| KAT-003 | Payer debit effect identity | `984fcc30e27b1ccbb5b266f6b0960c5791be3a68ada8512aa0fb39dcf575e56a` |
| KAT-004 | Beneficiary claim effect identity | `35ddfe65d8d3beb9ce06fcb1245802d0cc7799d8366bef9087ee5f6d63e6f24b` |

The vectors specify exact UTF-8 inputs, domain strings, schema versions, big-endian integer encodings, length-prefix rules and raw digest bytes. Independent implementations must reproduce these values byte-for-byte; normalization, case folding, locale-dependent numeric formatting, or changing a domain/schema version silently is not allowed. Any intentional encoding change requires a new version and a new vector set, not replacing v1 expected digests.

## 7. Benchmark plan

The machine-readable v0.1 corpus is in [economic-operation-identity-corpus.v0.1.json](../../mycelix-finance/benchmarks/finance/economic-operation-identity-corpus.v0.1.json) and its [Draft 2020-12 schema](../../mycelix-finance/benchmarks/finance/economic-operation-identity-corpus.v0.1.schema.json).

### Absolute safety gates

- 100 concurrent same-identity submissions: at most one accepted effect for each qualified singleton owner/effect role.
- 1,000 same-ID / changed-fingerprint attempts: 100% rejected as conflict; zero additional effects.
- 10,000 duplicate outbox/webhook/receipt deliveries: at most one effect admission per stable effect identity.
- 100,000 deterministic interleavings across timeout, restart, retries, policy rotation, DHT delay, owner refusal, forks and response loss: zero duplicate effects, false settlement or conflict-as-spendable.
- 100% of exact retries resolve to the original effect references or remain explicitly `Indeterminate`; never silently mint a new operation identity.
- Zero silent ledger deltas and zero unaccounted terminal operations.
- Every planned case is reported as pass, fail, invalid or unknown; omitted cases cannot disappear from denominators.

Targets are hypotheses until executed against a frozen subject and independently inspected evidence.

### Performance and cost, only after safety

Measure p50/p95/p99 separately for request admission, owner-effect commit, outbox dispatch, remote receipt, beneficiary claim and final reconciliation. Also measure sustainable operations/s, cost per correctly reconciled obligation, evidence-read amplification, DHT/storage growth, tombstone overhead, unresolved-outcome age, liquidity lock/claim lag and operator minutes per 1,000 ambiguous outcomes.

Compare only against matched workloads and semantic guarantees. Report baseline code, provider/rail, participant count, owner boundaries, consistency model, retention policy, retry schedule, failure injection, sample size and all failed/unknown cases. A single-database mutable SQL ledger is not an apples-to-apples comparator for bilateral owner-authored effects.

## 8. Industry comparison

- **Stripe:** API v1 stores the first response after endpoint execution begins and allows keys to be pruned after at least 24 hours; a reused pruned key can initiate a new request. API v2 is a stronger comparator: failed or partially failed work is retried without repeating successful side effects, with a 30-day replay scope for the same API and account/sandbox. Mycelix must benchmark against both profiles, not only v1. The opportunity is to bind the underlying obligation and owner-specific effects beyond provider API replay windows and across service/owner boundaries.
- **Adyen:** keys are company-account scoped, valid for 7–14 days, and simultaneous regional endpoints do not share duplicate checks. Mycelix’s identity must be scoped by its originating obligation/authority and independently reconciled across effect owners; it must not assume one provider's regional key store is a global registry.
- **Transactional outbox:** atomically links local state and event publication. Standard queue semantics can be at-least-once; duplicate-safe consumers and durable receipts remain required.
- **IETF Idempotency-Key:** use the current HTTPAPI Internet-Draft as an interoperability reference only. It remains an Internet-Draft, allows resource-defined expiry, and makes request fingerprinting optional. Do not claim RFC conformance/certification.
- **Holochain:** local zome-call atomicity is useful within its boundary. DHT observations are eventually consistent; fork/warrant capability and countersigning must be qualified on the exact shipped runtime. Do not infer global exactly-once execution.

The current [Stripe API v1 idempotency contract](https://docs.stripe.com/api/idempotent_requests) and [API v2 idempotency contract](https://docs.stripe.com/api-v2-overview#idempotency) are explicit benchmark baselines; Adyen documents its own retention and regional scope [here](https://docs.adyen.com/development-resources/api-idempotency). The differentiator is not a claim that Mycelix is faster or globally safer than incumbent processors. It is a testable, explicit contract for the economic obligation across multiple effect owners, with exact conflict and unknown-outcome handling.

## 9. Exit criteria

This research track can be promoted only when:
- canonicalization vectors are stable and independently reviewed;
- the pure kernel and all negative controls pass exact-head tests;
- the operation/effect claim has a qualified uniqueness authority for each profile;
- the evidence projection reconstructs the exact effect set without relying on mutable status flags;
- crash/restart, duplicate delivery, concurrency and conflicting-lineage tests pass;
- retention and privacy policy are specified for the actual deployment jurisdiction;
- adapters and external rails are individually qualified;
- public reports separate implemented, structurally checked, model-checked, runtime-tested, provider-tested and externally settled claims.

**Claim ceiling:** this v0.1 document and corpus define a research contract. They do not establish implementation, global exactly-once behavior, production readiness, live payments, reserve backing, external settlement or legal finality.
