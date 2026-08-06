# Threshold Scientific Credential Governance

**Protocol:** `mycelix-desci-credential-governance` v1
**Status:** experimental, append-only authority layer

The scientific credential registry can authorize identities, keys, roles, and organizations. Those mutations are too powerful to depend on one administrator signature. Threshold governance therefore separates a proposed credential change from its activation.

## Security model

Every governance record has two independent signatures:

1. a registered `registry_admin` signs the governance event;
2. the authority acceptance service signs the server-observed `received_at`.

The governance record hash commits to both signatures and becomes the next governance event's predecessor. Actor keys and acceptance-service keys must be distinct.

A proposal binds:

- the exact credential-registry head;
- the complete proposed action and action hash;
- an activation time;
- an expiry time;
- the proposing administrator;
- a human-readable reason.

Approvals are counted by unique actor, not key or event count. Risk-tiered policy can additionally require approvals from distinct administrator organizations. At execution, approvers must still be active registry administrators with at least one active key. Revoking an administrator during the delay therefore removes that approval from the executable threshold.

## Lifecycle

```text
proposal_opened
      ↓
unique administrator approvals
      ↓
activation delay
      ↓
head/authority revalidation
      ↓
proposal_executed
```

A proposal becomes stale whenever another credential mutation changes the bound registry head. A stale or expired proposal cannot receive further approvals or execute. Cancellation remains append-only and never deletes the original proposal or approvals. Client-supplied activation and expiry timestamps are signed intent: the accepted projection clamps them to at least the server receipt time plus the policy delay and at most the server receipt time plus the policy TTL.

## Governed actions

Threshold governance currently covers:

- signed credential-registry events;
- authorization of a new governance acceptance-service key;
- scheduled revocation of a governance acceptance-service key;
- governance-policy updates;
- risk-tiered quorum-policy updates;
- transparency-witness enrollment and revocation.

Registry genesis and governance-policy initialization remain offline bootstrap operations and are never available through the network API.

## Initialization

Register at least two administrators before initializing governance. The default operator policy uses a two-administrator threshold, a 24-hour activation delay, and a seven-day proposal lifetime.

```bash
mycelix-desci credential-governance-init \
  --registry ./data/scientific-credentials.json \
  --bootstrap-trust-file ./config/credential-bootstrap-trust.json \
  --acceptance-signing-key-file ./config/authority-receipt-signing.key \
  --governance ./data/scientific-credential-governance.json \
  --administrator did:key:initial-registry-admin \
  --administrator-signing-key-file ./config/registry-admin.seed \
  --approval-threshold 2 \
  --activation-delay-seconds 86400 \
  --proposal-ttl-seconds 604800
```

Successful initialization permanently disables direct non-genesis credential mutation in memory and writes a durable sidecar marker beside the credential registry. Restarting or using an older CLI does not restore the single-administrator path.

The service key used above signs credential/governance acceptance time. A deployment may deliberately share it with the domain-separated scientific-authority receipt protocol, but it must never be registered as an actor key. The signed publication outbox uses a distinct delivery-service key and rejects overlap with receipt or actor keys. Historical public service keys must remain in their configured trust sets after rotation. New service-key validity begins no earlier than its execution receipt time, and a requested revocation is effective no earlier than execution, preventing retroactive alteration of accepted governance history.

## Offline operations

`credential-governance-propose` accepts a strict JSON `CredentialGovernanceAction`. This makes the CLI protocol-complete without inventing separate flags for every future credential transition.

```bash
mycelix-desci credential-governance-propose \
  <common authority arguments> \
  --action-file ./proposal-action.json \
  --reason "Register the new independent replication coordinator"

mycelix-desci credential-governance-approve \
  <common authority arguments> \
  --proposal-id <uuid>

mycelix-desci credential-governance-execute \
  <common authority arguments> \
  --proposal-id <uuid>
```

The action file for a registry change contains an already signed `SignedScientificCredentialEvent`. Execution verifies that exact event, appends it through the governance-only registry path, and records the credential append receipt in the governance response.

## Crash recovery

Execution is serialized with every other governance mutation. Before any credential side effect, the service creates and fsyncs the fully signed governance acceptance record. It then:

1. persists a pending execution record;
2. commits the credential event when applicable;
3. appends the pre-signed governance execution record;
4. removes the pending record.

Startup reconciles the pending record before serving requests. Recovery never requires the former acceptance private key: the exact acceptance signature was persisted before the side effect. A conflicting committed event or governance record fails closed.

## Risk-tiered quorums

After bootstrap, readiness requires a governed risk policy. Routine, sensitive, and critical actions snapshot progressively stronger approval, organization-diversity, and delay rules. See [Risk-Tiered Scientific Credential Governance](RISK_TIERED_CREDENTIAL_GOVERNANCE.md).

## Transparency checkpoints

A checkpoint commits to both append-only histories:

- credential-registry event count, head, and Merkle root;
- governance event count, head, and Merkle root;
- signed issue time.

The API exposes a checkpoint candidate, and the CLI can publish and export it. Governed external witnesses may independently verify and sign the checkpoint hash. Deployments can require organization-diverse witnesses on the latest checkpoint before readiness passes. See [External Transparency Checkpoint Witnesses](EXTERNAL_TRANSPARENCY_WITNESSES.md).

## API

| Method | Path | Purpose |
|---|---|---|
| `GET` | `/api/v1/scientific/authority/governance` | Policy, proposals, service keys, and summary |
| `GET` | `/api/v1/scientific/authority/governance/events` | Paged recorded governance history |
| `GET` | `/api/v1/scientific/authority/governance/events/{id}` | One event with event and record hashes |
| `GET` | `/api/v1/scientific/authority/governance/proposals/{id}` | Proposal and effective status |
| `GET` | `/api/v1/scientific/authority/governance/checkpoint-candidate` | Current checkpoint candidate |
| `POST` | `/api/v1/scientific/authority/governance/events` | Open, approve, cancel, or publish checkpoint |
| `POST` | `/api/v1/scientific/authority/governance/execute` | Execute a mature proposal atomically |

For every mutation, the JWT subject must exactly equal the signed governance actor.

## Remaining limitations

- Risk classification is currently code-defined rather than a separately versioned policy artifact.
- Witness compromise notices do not yet invalidate a historical interval.
- File storage now rejects stale multi-process writers, but cross-journal execution is crash-recovered rather than one database transaction.
- Canonical cross-language golden vectors remain to be published.
