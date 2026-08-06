# Refoundation Tranche 6: Threshold Credential Governance

**Date:** 2026-08-04
**Protocol version:** 0.7.0
**Status:** implemented statically; Rust build verification pending

## Purpose

Tranche 5 made actor/key/role authority append-only, but every credential transition still depended on one active administrator signature. That left administrator compromise, hurried key rotation, and service-key replacement as single-point authority failures.

Tranche 6 adds a second append-only protocol above the credential registry. Credential mutations are now proposed, independently approved, delayed, and executed only while the bound registry revision and approving administrators remain valid.

## Security invariants

1. Governance initialization requires at least two active registry administrators.
2. Every governance event is signed by a registered active `registry_admin`.
3. A distinct acceptance-service key signs the server-observed governance receipt time.
4. Proposals bind the exact credential-registry head and the canonical action hash.
5. Approvals count unique actor identities, not keys or event volume.
6. Approvers must still be active administrators at execution time.
7. Activation delay and expiry are enforced from server receipt time.
8. A changed credential head makes a proposal stale.
9. Direct non-genesis registry append is disabled after governance initialization and remains disabled across restart through a durable cutover marker.
10. Governance must retain a non-expiring acceptance-service recovery key.
11. Acceptance-service keys may never be actor keys.
12. Execution is serialized with every other governance mutation.
13. A fully signed execution acceptance record is fsynced before any registry side effect.
14. Recovery finalizes the exact pre-signed record and never requires an old private key.
15. Transparency checkpoints verify historical prefixes and remain valid as either log grows.
16. Governance-policy initialization is offline-only; the network API cannot create the root policy.
17. Readiness verifies that the configured private acceptance key is itself currently governed and active.

## Protocol objects

The new canonical protocol defines:

- `CredentialGovernancePolicy`
- `CredentialGovernanceAction`
- `CredentialGovernanceEnvelope`
- `SignedCredentialGovernanceEvent`
- `RecordedCredentialGovernanceEvent`
- `CredentialGovernanceProposal`
- `CredentialTransparencyCheckpoint`

Governed action classes are:

- append one already-signed credential event;
- authorize an acceptance-service key;
- revoke an acceptance-service key;
- update the governance policy.

Governance payloads are:

- `policy_initialized`
- `proposal_opened`
- `proposal_approved`
- `proposal_cancelled`
- `proposal_executed`
- `transparency_checkpoint_published`

The codec uses explicit numeric tags and domain-separated canonical binary encodings rather than serializer-dependent bytes.

## Proposal state

A proposal records its proposer, bound registry head, action, action hash, receipt-time activation and expiry, reason, unique approvals, and terminal state.

Persisted states are `pending`, `cancelled`, and `executed`. `expired` and `stale` are effective query states derived from current time and registry head without rewriting history.

A stale proposal cannot receive another approval. Parallel proposals may coexist, but execution of one credential mutation makes every competing proposal bound to the old head harmless.

## Two-phase execution

Governance execution has an external side effect when the action appends a credential event. The implementation therefore serializes all governance mutations and uses a durable pending record:

1. validate proposal, threshold, delay, expiry, active approvers, and registry head;
2. create the final authority-service acceptance signature;
3. fsync the complete recorded governance execution and expected sequence;
4. append the credential event through the governance-only registry method;
5. append the exact pre-signed governance record;
6. remove and fsync the pending record.

On startup:

- if neither side committed, the pending record is removed;
- if the credential event committed, the exact pre-signed governance record is finalized;
- if the governance record committed, its hash must match the pending record;
- any conflicting event or record fails closed.

Service-key and policy actions have no separate registry side effect, so their prepared governance record is finalized directly during recovery.

## Authority-service key governance

Service-key authorization and revocation use the same threshold path as actor credentials. The projection rejects key reuse, invalid Ed25519 public keys, actor/service key overlap, and removal of the final non-expiring recovery key. New validity and revocation times are clamped to no earlier than the server-observed execution time, so a delayed proposal cannot rewrite earlier acceptance history.

The current private key signs new acceptance records. Historical records remain verifiable from the governed public-key history; recovery of an interrupted execution uses the pre-signed pending record rather than the current private key.

## Transparency checkpoints

A checkpoint commits to event counts, heads, and Merkle roots for both:

- the credential-registry prefix;
- the governance-journal prefix.

Replay verifies the historical prefix present when the checkpoint was accepted. Later registry or governance growth does not invalidate an old checkpoint.

This tranche generates and signs checkpoints but does not claim independent external publication. Operators must export checkpoints to a separately retained transparency channel.

## API cutover

New canonical endpoints expose governance state, events, individual records, proposals, checkpoint candidates, signed mutation append, and atomic execution.

Once threshold governance is initialized, `POST /api/v1/scientific/authority/events` returns HTTP 410. Network clients must submit signed governance objects, and the JWT subject must exactly match the signed administrator actor. `policy_initialized` is rejected by the network API and must be committed offline before the service starts.

Readiness now requires initialized durable governance, configured acceptance signing whose current private key is governed and active, at least one active and one continuing acceptance-service key, a policy threshold no lower than the configured minimum administrator count, and no pending execution recovery.

## Offline operator tooling

The CLI adds commands to:

- initialize threshold governance;
- open a proposal from a strict action JSON file;
- approve a proposal;
- cancel a proposal;
- execute a mature proposal;
- publish and optionally export a transparency checkpoint using the exact signed candidate timestamp.

All private-key readers reject symbolic links and oversized files and enforce mode `0600` or stricter on Unix through the existing credential tooling.

## Known limitations

- A single threshold applies to every action class.
- Emergency cancellation is a single active-administrator safety action; formal recovery councils are not yet modeled.
- Checkpoint publication is local; no independently operated witness or gossip protocol is included.
- The durable implementation remains a single-process file reference backend.
- Canonical cross-language golden vectors are still required.
- The standalone archive still lacks the sibling `mycelix-zkp-core` dependency.

## Validation boundary

The implementation was reviewed with static structural scans, duplicate-definition checks, JSON/YAML parsing, whitespace checking, ordered patch replay, and tree-equivalence checks. A complete Rust toolchain was unavailable in the execution environment, so `cargo fmt`, `cargo check`, `cargo clippy`, and `cargo test` remain mandatory before merge.
