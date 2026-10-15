# hAppRadar Second-Wave Research: Governance, Shared Knowledge, and Evidence Integrity

**Observed:** 2026-10-15  
**Status:** design research only; no integration, security audit, runtime compatibility, or production-readiness claim.

This follow-up expands the first nine-profile comparison without changing the existing machine-checked matrix or its fixed profile count. It evaluates four additional hApps for *patterns worth testing*, not dependencies to import. Source pins are exact commits; hAppRadar listings are discovery evidence, not compatibility proof.

## Findings at a glance

| Project | Pattern worth learning from | Candidate Mycelix use | Main blocker / caution | Recommendation |
|---|---|---|---|---|
| [liquid-democracy-holochain](https://happradar.com/projects/liquid-democracy-holochain) | Topic-scoped delegation and direct voting | Governance proposal and delegation UX | hAppRadar lists Holochain 0.6.0, HDK 0.6.0, HDI 0.7.0, client 0.20.2; no published release was recorded in the listing | Study semantics; implement no delegation import until revocation, cycles, scope, and tally rules are explicit |
| [ShareFeed](https://happradar.com/projects/sharefeed) | Small-group link sharing with source context | Pulse / knowledge commons evidence discovery | Listing records Holochain 0.6.0, HDK 0.6.0, HDI 0.7.0, client 0.20.0; no release recorded | UX and provenance benchmark, not a trust or moderation oracle |
| [talking-stickies](https://happradar.com/projects/talking-stickies-holochain-apps) | Shared collaborative board and concurrent updates | Cooperative planning, dispute timelines, community workshops | Listing records Holochain 0.4.0, HDK 0.4.0, HDI 0.5.0, client 0.18.0 | Reuse workflow lessons only; runtime/DNA composition is out of scope |
| [prometheus-happ](https://happradar.com/projects/prometheus-happ) | Evidence ingestion separated from canonical meaning/governance | Commons / regenerative supply-chain observations | Listing records Holochain 0.6.1; upstream describes a pilot runtime and a blocking real-data integrity gate | Highest-value assurance pattern to study; keep real data disabled until persistence-bound controls are independently demonstrated |

## Source revision pins

These pins make the inspection reproducible. A pin proves which source revision was inspected, not that the source is safe, compatible, deployed, or endorsed.

- **liquid-democracy-holochain** — `Niki-Godless/liquid-democracy-holochain@206a57bb685f6c6a377145825daea6f8491ecf2e` ([commit](https://github.com/Niki-Godless/liquid-democracy-holochain/commit/206a57bb685f6c6a377145825daea6f8491ecf2e)). The commit message explicitly describes the final UI state with Holochain 0.6.0 before a downgrade; treat the repository's present state and this pinned snapshot carefully rather than assuming a stable release.
- **ShareFeed** — `lightningrodlabs/sharefeed@7e10718f9f2c8e6f1547da7d223e61e3e8f5e085` ([commit](https://github.com/lightningrodlabs/sharefeed/commit/7e10718f9f2c8e6f1547da7d223e61e3e8f5e085)).
- **talking-stickies** — `holochain-apps/talking-stickies@8f69baca95c0b588339c39d880dd7cd192862af4` ([commit](https://github.com/holochain-apps/talking-stickies/commit/8f69baca95c0b588339c39d880dd7cd192862af4)). This pinned commit explicitly targets Holochain 0.4 and an older Moss/Weave line.
- **Prometheus hApp** — `Permaculture-DAO/prometheus-happ@a406ae841856dcfa4550187e8b441536789f7fbf` ([commit](https://github.com/Permaculture-DAO/prometheus-happ/commit/a406ae841856dcfa4550187e8b441536789f7fbf)). This revision adds a real-data evidence integrity gate, including a requirement that the real-data flag remain disabled until the gate is implemented and verified.

## What Mycelix should learn — and what it should not copy

### 1. Governance: delegation is an authority graph, not a convenient vote count

A liquid-democracy UI can make delegation understandable, but the protocol must define the hard cases:

- Delegation is scoped by proposal/domain and has explicit effective-time semantics.
- Revocation and redelegation have a deterministic cutoff; no stale delegation may silently remain active.
- Cycles, self-delegation, duplicate paths, and transitive delegation are rejected or resolved by a documented deterministic rule.
- Tally snapshots bind to an immutable proposal revision, eligibility snapshot, and cutoff; a later edit cannot rewrite an already closed vote.
- A delegate cannot gain broader authority than the delegator possessed.
- Abstention, missing peers, conflicting actions, and late-arriving actions remain visible; no UI-derived quorum is authoritative.

**Suggested Mycelix contract test:** generate a small delegation graph and apply actions in different arrival orders. For every valid order, the same canonical cutoff snapshot must produce the same result; a revoked or out-of-scope delegation must never count. Include a cycle and a proposal-edit-after-close negative control.

### 2. Knowledge commons: a shared feed is not a truth or safety system

ShareFeed's group-oriented sharing is a useful interaction pattern for Pulse and knowledge commons. Mycelix should preserve source URL, author, capture time, content digest, quote/highlight boundaries, and any edit/retraction lineage. A share means only that a participant shared a reference; it does not establish that the page is accurate, safe, licensed for republication, or still available.

**Suggested contract tests:** altered URL with unchanged displayed title; deleted/redirected source; stale cached content; quote detached from its source context; repeated post replay; revocation of future access after another peer already copied the content. Retraction must be a new attributable event, not a claim that all copies disappeared.

### 3. Collaborative boards: concurrency and provenance before a unified workspace

Talking-stickies is a benchmark for small-group collaborative artifacts, not a compatible dependency. If Mycelix later standardizes shared task or dispute records, specify operation identity, actor, causal predecessor/version, idempotency key, and conflict policy before selecting a common data model.

**Suggested contract tests:** same operation delivered twice; two peers edit the same object from the same predecessor; one peer reconnects with stale state; delete/archive races with an edit; replayed operation under a different author identity. The result must be deterministic or explicitly multi-valued—not silently last-write-wins where that could change an obligation or governance record.

### 4. Evidence integrity: Prometheus offers the strongest assurance pattern in this batch

The pinned Prometheus commit's real-data gate makes several strong distinctions that map well to Mycelix commons and supply-chain work:

- Enforce real-data authorization at the durable persistence/integrity boundary, not only at an MQTT gateway or UI.
- Bind a calibration artifact to the specific sensor and collection interval; a valid hash alone is insufficient.
- Authenticate the whole routing/security envelope, not just payload bytes.
- Separate reviewer attestation from the sensor/operator's own payload.
- Include subject and sensor identity in deduplication/idempotency keys.
- Keep synthetic observations distinguishable from real observations.
- Passing an ingestion gate does not itself prove scientific validity, ecological outcomes, certification, entitlement, or financial value.

This is a pattern to study, not a statement that Prometheus has already passed its own gate. Its pinned document explicitly labels the gate blocking. Mycelix should likewise keep a real-data switch fail-closed until direct-call bypasses, wrong-sensor calibration, rerouting, cross-subject deduplication, spoofed reviewer metadata, and retry cases are tested against the actual durable write path.

## Cross-hApp implementation opportunity: a provenance envelope

Rather than adopting four separate runtimes, define one small, versioned **observation envelope** that can represent a claim without promoting it to authority:

- `schema_id` and `schema_version`
- `source_app`, pinned source/profile identifier, and source record ID
- subject, actor/issuer, and community/resource scope
- event identity, sequence/revision, and causal predecessor
- observed-at and recorded-at timestamps with explicit clock assumptions
- payload digest plus canonicalization algorithm/version
- evidence class: synthetic, reported, observed, reviewed, or independently corroborated
- correction/retraction reference
- license/privacy classification and allowed disclosure policy
- explicit claim ceiling: what this record does **not** establish

The envelope must not have fields such as `trusted=true`, `settled=true`, or `approved=true` that a source can self-assert. Those are derived, domain-specific decisions backed by separate authorization and review records. Preserve unknown fields as versioned extension data only when the schema explicitly allows them; otherwise reject or quarantine.

### Acceptance criteria for a first executable fixture

1. Canonical encoding is deterministic and digest-bound; changing any identity, scope, unit, timestamp, or payload field changes the digest.
2. Exact retry is idempotent, while same payload from a different subject or sensor remains a distinct observation.
3. A signature authenticates origin/integrity only; it does not upgrade the evidence class or grant authority.
4. Revocation prevents future authorized access where supported but never claims retroactive deletion of already copied plaintext.
5. A correction appends a linked event; it does not rewrite the original.
6. Unknown, stale, conflicting, unsupported, or unreviewed observations cannot mutate SAP, TEND, MYCEL, governance outcomes, or settlement.
7. Synthetic fixtures cover every negative control; no personal, clinical, or otherwise sensitive real data is needed.
8. The fixture remains a research contract until a verifier outside the candidate PR's control checks the exact commit.

## Executable provenance contract — first increment

The first executable reference fixture and its Rust-side contract checks now live beside the ValueFlows corpus:

- [Machine-readable v1 fixture](../../mycelix-finance/tests/reference/valueflows-interop/provenance-envelope-contract-v1.json)
- [Rust contract checker and adversarial tests](../../mycelix-finance/tools/valueflows-interop-checker/src/main.rs)
- The existing [ValueFlows Interoperability Qualification workflow](../../.github/workflows/valueflows-interop-qualification.yml) already watches the shared fixture/checker paths, so this increment uses the existing qualification lane rather than adding a new workflow or dependency.

The fixture is intentionally synthetic and self-contained. The checker pins the schema version, rejects unreviewed fields, keeps all external Finance/governance/settlement authority false, restricts signatures to origin/integrity, requires append-only corrections, and fixes ten adversarial-control conditions, dispositions, and ordered guard sets. Six additional Rust test functions exercise those boundaries.

**Important limit:** the checker currently validates the SHA-256 field's syntax only. It does **not** recompute the payload digest, implement RFC 8785/JCS canonicalization, verify signatures, prove source-file correspondence, or establish the truth of an observation. Those remain explicit implementation gates; no digest-verification or Rust test/build pass is claimed for this change until an exact-head run completes.

RFC 8785/JCS is the nominated canonicalization target because the RFC defines deterministic JSON serialization for cryptographic operations; it is not yet implemented by this fixture. In particular, JCS inputs must not have duplicate object member names, strings must preserve their code points, and number formatting follows its specified serialization rules. See [RFC 8785](https://www.rfc-editor.org/rfc/rfc8785.html).

## Recommended order

1. **Do not import old DNAs.** All four profiles are useful as design references; their listed Holochain tuples are older than the stable 0.7 qualification baseline documented in the main matrix.
2. **Prototype the provenance envelope** beside the existing ValueFlows export-only corpus, not inside authoritative Finance mutation paths.
3. **Add the observation identity/idempotency controls** first, because they support governance, collaborative work, and sensor/economic evidence.
4. **Add a deterministic delegation graph fixture** as a separate domain contract with explicit proposal/cutoff semantics.
5. **Only then** select one narrow two-agent runtime experiment, with pinned artifacts, explicit data continuity, and a protected independent verifier.

## Evidence and limits

Discovery pages inspected on 2026-10-15:

- https://happradar.com/projects/liquid-democracy-holochain
- https://happradar.com/projects/sharefeed
- https://happradar.com/projects/talking-stickies-holochain-apps
- https://happradar.com/projects/prometheus-happ

The Holochain version, release, activity, and maturity fields above reflect those listings and are time-sensitive. The exact Git commit pins are listed separately. This document adds no runtime dependency and makes no claim that any listed project is production-ready or interoperable with Mycelix.
