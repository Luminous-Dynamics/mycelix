# Draft: Nested Community Currencies and Federation Clearing (NCF-1)

**Status:** Proposed architecture; not implemented or qualified  
**Tracking:** [FED-CUR-001 / #5464](https://github.com/Luminous-Dynamics/mycelix/issues/5464)  
**Composes with:** [CIV-ECON-001 / #4804](https://github.com/Luminous-Dynamics/mycelix/issues/4804), [MYC-ECO-002 / #3343](https://github.com/Luminous-Dynamics/mycelix/issues/3343), [MYC-ECO-003 / #3344](https://github.com/Luminous-Dynamics/mycelix/issues/3344), [FIN-SYS-023 / #4630](https://github.com/Luminous-Dynamics/mycelix/issues/4630), [FIN-ECO-007 / #1157](https://github.com/Luminous-Dynamics/mycelix/issues/1157).

## 1. Decision

Mycelix should support **optional, locally governed community currencies and explicit federation clearing**, without requiring every group to issue a currency and without making all community instruments interchangeable.

The architecture should permit a household circle, mutual-aid group, worker cooperative, neighborhood, regional federation, or federation of federations to:

1. operate with no currency, one shared instrument, a local mutual-credit instrument, or a declared combination;
2. retain authority over its own issuance, credit limits, membership, and local rules;
3. recognize evidence and trade with other groups without automatically accepting their currency;
4. establish bounded clearing or conversion agreements when there is a demonstrated need; and
5. leave a federation or retire an instrument through an explicit, auditable process.

The number of currencies is not a success metric. The target is useful local coordination with bounded risk and inexpensive interoperability.

**Terminology constraint:** the canonical Finance type defines **MYCEL** as non-transferable reputation, **SAP** as a transferable circulation medium, and **TEND** as mutual credit. Do not redefine MYCEL as a generic spendable token or use a ticker as an instrument identity. A community's spendable instrument needs its own exact, typed identity and economic semantics.

## 2. Existing foundation and actual gap

The current **mycelix-finance/zomes/currency-mint** code models DAO-created community mutual-credit currencies. A currency definition includes an ID, creator DAO DID, optional governance proposal, validated parameters, lifecycle status, and creation time; active instruments can be discovered globally. The coordinator currently documents a governance-proposal requirement for communities above a member-count threshold.

The existing [Federation Protocol](FEDERATION_PROTOCOL.md) discusses direct exchange, a federation-governed shared currency with collective backing, and external bridge currencies. The broader [Economic Framework](../economics/ECONOMIC_FRAMEWORK.md) surveys mutual credit, time banking, community tokens, and resource credits.

These are useful starting points, but they do not alone define a qualified contract for nested monetary authority and clearing. In particular, “federation-governed” and “collective backing” need machine-checkable meanings: which actor may do what, which members bear which liabilities, what evidence establishes backing, and what happens when one participant defaults or a network partitions.

This design extends the existing Finance and federation work. It must not create a competing Currency or AssetId namespace or bypass current author-binding, typed-amount, settlement/finality, and FX work.

## 3. Paradigm shift: federation as a graph of bounded contracts

Do not model federations as a single tree with transitive authority. Model organizations and currencies as nodes, and every membership, delegation, recognition, clearing, and guarantee as a separately scoped edge.

A community may belong to several federations. Membership in Federation A must not silently confer credit in Federation B. A parent may have no authority over a child's currency, or it may hold one explicitly delegated capability, such as “suspend cross-federation clearing on emergency-risk evidence.” It must not acquire all local powers simply by being the parent.

### Required separation

Keep these concepts distinct:

- community identity versus currency identity;
- currency identity versus symbol/name;
- instrument semantics versus its valuation;
- membership versus issuance authority;
- evidence recognition versus acceptance of economic liability;
- authorization versus execution;
- quote/valuation versus conversion execution;
- clearing/accounting versus settlement finality;
- local obligation versus federation guarantee.

No transitive authority, parity, collateralization, guarantee, or trust score should be inferred from graph connectivity.

## 4. Instrument profiles

Each currency must resolve to one versioned profile using the existing canonical Finance type layer where possible. Minimum semantic fields are:

| Field | Meaning |
|---|---|
| **instrument_id** | Stable, globally distinct identity; never just a ticker |
| **issuer_id** | Community/federation DID that is authoritative for the instrument |
| **instrument_family** | Explicit type: mutual credit, time credit, asset-backed money, resource credit, etc. |
| **profile_revision** | Immutable policy revision used by approvals and settlements |
| **unit_definition** | Exact denomination, scale, and canonical amount representation |
| **issuance_model** | How value/obligations are created, bounded, and retired |
| **governance_policy_ref** | Evidence identifying required decision procedure and authorization |
| **participant_scope** | Who may hold or transact, without leaking unnecessary membership data |
| **lifecycle_state** | Draft, approved, active, suspended, or retired; transitions are governed |
| **claim_ceiling** | What recorded evidence does and does not establish |
| **provenance** | Creation and amendment lineage, source, timestamp, and policy digest |

A symbol such as GARDEN or MYC is display metadata, not identity. Two instruments may share a symbol and never alias. Unknown or ambiguous profiles must fail closed.

### Do not collapse instrument families

- **Community mutual credit:** a transaction moves the participants' net positions in opposite directions. The zero-sum invariant applies within that instrument's accounting domain.
- **Time credits:** the unit and valuation rule must be explicit (for example, one accepted service hour), including whether skill differences are intentionally ignored.
- **Reserve-backed or fiat-referenced instruments:** require separate backing, redemption, legal, and external-settlement qualifications. A reserve claim is not proved by storing a URL or a signed issuer statement.
- **Resource credits:** represent a constrained resource right or quantity, not generic money.
- **MYCEL reputation:** non-transferable contribution/reputation state, not spendable balance or universal human-worth measure.
- **SAP and TEND:** preserve their existing distinct semantics; neither becomes fungible with a community instrument just because both are denominated numerically.

The current community currency factory is principally a mutual-credit design. Supporting another family requires its own invariant and qualification rather than reusing the mutual-credit zero-sum theorem indiscriminately.

## 5. Creation and lifecycle protocol

### Creation

1. A community submits a draft profile with a globally unique ID, its issuer identity, the proposed instrument family, parameters, exact units, governance rule, and evidence references.
2. The caller's authority must be bound to the **specific issuer/community** and the **create_currency** action. A valid DID or a high reputation score is insufficient.
3. Apply that community's versioned governance policy. Small circles may use a lightweight member-consent process; larger or riskier issuance may require stronger quorum or review. A member-count threshold may select the process, but must never act as an authentication bypass. Membership counts themselves must be qualified and resistant to stale or fabricated population data.
4. Record an immutable approval reference binding the exact profile digest and policy revision. Any changed parameters require a new version and the required new authorization.
5. Activate only after required checks pass. Draft, approval, activation, suspension, retirement, and recovery are separate state transitions with explicit authorizations.

### Authority rule

An authorization must bind at least the issuer, instrument ID, action, policy revision, relevant parameters or amount, recipient/scope where applicable, expiry, and a unique execution identifier. It must be consumed idempotently. A governance proposal ID that exists but authorizes different parameters does not authorize the mutation.

A child group cannot issue or amend a parent instrument. A federation cannot change a member's credit limit or mint on the member's behalf unless that member has granted an explicit, narrow, revocable capability and the applicable community policy permits it. Removing a delegation must not erase historical events.

## 6. Federation and clearing modes

Peering is opt-in and described by a versioned agreement between named parties. It must bind:

- participating community and instrument identities;
- which activities and transaction directions are permitted;
- credit/exposure limits per peer and in aggregate;
- fees, quote source/policy, quote validity, and rate-construction method if conversion is offered;
- clearing cadence, settlement method, finality definition, and idempotency rules;
- reserve/guarantee terms, liability allocation, default limits, and loss waterfall if any;
- dispute resolution, emergency suspension, agreement expiry, and exit/termination handling;
- privacy boundaries and what evidence counterparties can demand.

Use separate explicit modes:

1. **Recognition only:** another community recognizes evidence or credentials; this confers no spending rights or liability.
2. **Trade/commitment only:** parties exchange goods/services, with each party's local accounting and acceptance rules retained.
3. **Mutual-credit clearing:** named peers track capped reciprocal obligations and settle according to an agreed clearing policy. Credit limits bound exposure.
4. **Explicit conversion:** a quote identifies both exact instruments, amounts, rate, fees, validity window, source evidence, and execution ID. Quotes are not settlement, and settlement must not execute twice.
5. **Shared federation-issued instrument:** the federation is the clearly named issuer with its own profile, governance and liability policy. Member currencies do not become equivalent to the shared instrument, and membership alone does not imply that each community guarantees all federation liabilities.

No mode may assume a 1:1 rate, common backing, or unlimited settlement guarantee. If a community exits, the agreement must specify how existing obligations are reconciled without granting new authority to the federation over unrelated local balances.

## 7. Failure isolation and repair

A failure in one currency or peering edge must not silently alter unrelated local ledgers. The default response should be to bound or pause the affected edge, preserve evidence, and initiate the previously agreed dispute/recovery path.

Required behavior includes:

- stale or missing oracle/FX evidence causes a fail-closed rejection or an explicitly declared degraded path with stricter limits; never silently fall back to an unverified claimant value;
- duplicate, reordered, and retried messages cannot create double credit or settlement;
- missing amount fields, negative amounts, stale profile revisions, and changed payloads under a reused idempotency key fail closed;
- a community's aggregate gross commitment across all peering edges remains within its configured cap, not merely each edge in isolation;
- a network partition cannot increase a peer's exposure above its already qualified offline limit;
- a currency can be suspended without silently confiscating balances or erasing claims;
- correction and reversal events append a traceable reason and evidence; they do not rewrite history;
- a federation-level guarantee or common risk pool is explicit, capped, funded under its own rules, and independently auditable;
- members can export their account and obligation records when leaving, subject to privacy and lawful retention constraints.

Cryptographic signatures prove who signed a statement under a key, not that the statement is factually true. Physical-service claims, reserve claims, prices, governance approvals, and redemption promises require appropriate external evidence and explicit limitations.

## 8. Standards and state-of-the-art comparison

This is a comparison of design responsibilities, not a claim that one system universally outperforms another.

| Reference system | Established strength | Mycelix opportunity |
|---|---|---|
| [Community Exchange System (CES)](https://www.community-exchange.org/home/what-is-the-ces/) and LETS/time banks | Community exchange models that operate across communities and do not require conventional money for every exchange | Strong typed instrument identity, explicit federation authority, verifiable provenance, and adversarially tested cross-community exposure limits |
| [Interledger Protocol v4](https://interledger.org/developers/rfcs/interledger-protocol/) and [Settlement Engines](https://interledger.org/developers/rfcs/settlement-engines/) | Ledger-neutral routing; peer accounts and limits; clear distinction between clearing obligations and settlement; idempotent settlement APIs | Reuse the boundary discipline while making community policy, evidence, local authority, and participant remedies first-class |
| [W3C Verifiable Credentials 2.0](https://www.w3.org/TR/vc-data-model/) | Portable, tamper-evident claims and separation of issuer, holder, and verifier; trust is not automatically transitive | Bind credentials and evidence to narrow economic actions and scopes without treating reputation as universal authority |
| [BIS Annual Economic Report 2025](https://www.bis.org/publ/arpdf/ar2025e.pdf), unified-ledger architecture | Explicit trade-offs between unified, subnet/layered, and separate-ledger architectures, including autonomy versus composability | Test when local autonomy plus a thin clearing layer beats a single universal ledger on resilience, switching cost, privacy, and risk isolation |

Mycelix should not try to beat Interledger by claiming superior raw payment throughput, or beat community-exchange systems solely by adding more token features. The differentiator worth proving is **federation-safe economic composition**: autonomy at the edges, explicit and bounded contracts at the boundaries, and evidence that can be challenged.

## 9. Adversarial test matrix

Make these negative-first tests part of the qualification contract before enabling production-value paths.

### Authority and identity
- two instruments with the same symbol but different issuers;
- creator authenticates as a valid member but not as an authorized agent of the declared issuer;
- small-community path attempts to bypass issuer authorization;
- child attempts to mutate a parent profile;
- federation attempts to amend a local currency without a scoped capability;
- capability expired, revoked, replayed, or bound to a prior policy revision;
- proposal approves parameters A but the write attempts parameters B.

### Accounting and interoperability
- mutual-credit exchange replayed, reordered, raced, or partially recovered;
- duplicate settlement ID or idempotency key used with a different payload;
- a child belongs to two federations and one edge's negative balance is incorrectly netted against the other;
- a community stays below each bilateral limit but exceeds its aggregate exposure cap by opening more federation edges;
- a rejected over-cap obligation partially changes state or consumes an event ID before failing;
- an amount-limited capability omits the amount, uses a negative amount, or attempts new issuance with no ceiling;
- same numeric amount across two currencies is added, netted, or treated as parity;
- missing, stale, manipulated, or contradictory FX quote;
- unverified “collective backing” promoted into a spendable/reserve claim;
- currency retirement attempted while conservation or unsettled-obligation predicates fail.

### Topology and resilience
- nested federation with at least 10 authority levels and branching membership graphs;
- federation split, merge, or exit with unsettled obligations;
- partition while balances near peer exposure cap;
- compromised federation node attempts cross-branch mint/freeze;
- Sybil community creates many nominal child issuers to evade aggregate exposure caps;
- one currency's default, capture, or malformed history must not contaminate unrelated currencies.

### Property-level invariants
- Every accepted mutation has a verifiable, action-bound authority proof.
- Cross-currency arithmetic is rejected absent an explicit, versioned conversion/clearing policy.
- Mutual-credit net positions conserve value within the right instrument and transaction domain.
- An accepted operation is idempotent under retry; rejected or stale operations create no hidden balance change.
- Currency/profile/policy identity survives all adapters and exported receipts.

## 10. Benchmarks and success criteria

Use published systems as architectural baselines, then measure real behavior under matched scenarios. There is no single accepted benchmark suite for federated community currencies, so report reproducible workload definitions and raw results instead of declaring an unsupported “SOTA win.”

### Security gates

- **Zero** unauthorized issuance or policy mutation accepted in the adversarial corpus.
- **Zero** same-symbol identity collisions or implicit cross-currency conversions accepted.
- **Zero** net-position drift in mutual-credit property tests over at least 1,000,000 generated operations, including replay, reordering, concurrency, disputes, and recovery.
- **Zero** double settlements in at least 100,000 duplicate/retry/reordering cases.
- Every fail-closed path has a negative control proving the test would catch the defect.
- Thresholds apply to a frozen source revision; no weakening of a gate to obtain a pass.

These are proposed qualification targets, not achieved results.

### Performance and resilience

Benchmark 100, 1,000, and 10,000 participants across 1, 10, and 100 interconnected communities, varying transaction rate, credit exposure, federation depth, network partition duration, and clearing cadence. Report:
- throughput and p50/p95/p99 latency for local exchange, clearing, and conversion separately;
- write amplification and bytes per transaction;
- recovery time and reconciliation work after crash/partition;
- maximum exposure when a peer disappears;
- behavior as the currency/federation graph grows and as cached evidence becomes stale.

Record hardware/runtime, seeds, workload generator version, exact commit, profile digest, and output digests. Compare against the current local mutual-credit implementation first; compare against Interledger/other implementations only with equivalent trust, settlement, and finality assumptions.

### Economic and social outcomes

Never collapse outcomes into one score. Report at least:
- credit-limit utilization and liquidity availability;
- arrears/default rate, loss severity, and recovery time;
- time-to-clear and cost to resolve disputes;
- newcomers' successful participation and access to essential goods/services;
- account/issuer/validator/authority concentration;
- community autonomy and cost to exit/switch;
- exposure leaked across federation boundaries;
- privacy/observability burden and ability to challenge corrections.

Use matched baseline regimes, fixed and held-out shock sets, multiple random seeds, ablations, and independent validation. Include local mutual credit, CES/LETS-like exchange, bilateral clearing, a shared federation instrument, and no-local-currency controls. A result that improves throughput but worsens default, newcomer access, privacy, or power concentration must expose those trade-offs.

## 11. Implementation sequence

1. **Authority and value prerequisites:** close the already-tracked author-binding and typed-amount/settlement work. Treat the currency creator/activation/amendment/retirement lifecycle as untrusted until exact-source qualifications establish otherwise.
2. **Profile contract:** reuse the existing Finance types and define canonical identity, family-specific amount semantics, policy digest, and governance capability scope.
3. **Reference model:** implement a small pure Rust model and property-based negative controls for nested authority, per-currency conservation, peering limits, and failure isolation. Keep it separate from live value mutation initially.
4. **Federation agreement and clearing:** model signed/versioned peering agreements, clearing obligations, idempotent settlement receipts, default, and exit. Add multi-agent integration tests.
5. **Comparative simulation:** compare regimes under the same demand, resource, shock, and adversary traces using the vector metrics above.
6. **Narrow pilot:** only after independent review, test one voluntary producer/cooperative or mutual-aid network with capped non-convertible mutual credit and transparent exit. Do not start by promising external redemption or universal parity.

## 12. Claim boundary

This design does not establish currency stability, legal-tender status, regulatory compliance, FX parity, reserve adequacy, insolvency protection, external settlement finality, economic optimality, or production readiness. Any such claim needs its own jurisdiction-specific or source-specific evidence and test contract.
