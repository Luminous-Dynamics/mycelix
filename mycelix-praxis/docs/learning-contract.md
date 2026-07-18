# Learning client contract

Praxis clients and the learning coordinator share their public wire types and
names from `praxis-core::contracts`. Integrity entries and raw Holochain
`Record` values are not browser response types.

## Version 1 surface

| Function | Input | Output | Purpose |
| --- | --- | --- | --- |
| `get_learning_contract` | `()` | `LearningContractInfo` | Discover the contract version and implemented capabilities. |
| `list_course_summaries` | `()` | `Vec<CourseSummary>` | List stable course projections without exposing integrity records. |
| `sync_progress` | `ProgressSyncInput` | `ProgressSyncReceipt` | Publish progress for the calling agent. |
| `get_my_progress` | `()` | `Vec<ProgressSnapshot>` | Read the deterministic latest value for each course or curriculum unit. |

The hApp role is `praxis` and the zome is `learning_coordinator`. Clients must
import the shared constants instead of duplicating these strings.

## Dashboard projection version 1

The browser reads Live learner overview data through one shared endpoint:

| Zome | Function | Input | Output |
| --- | --- | --- | --- |
| `integration_coordinator` | `get_dashboard_snapshot` | `()` | `DashboardSnapshot` |

The integration coordinator composes gamification, due-review, mastery,
existing recommendation, and recent activity projections. Every subsection is
optional: `None` means its coordinator was unavailable, while `Some([])` or
`Some(0)` is a successful empty result. The browser checks
`DASHBOARD_CONTRACT_VERSION` before rendering the response.

Dashboard reads are side-effect free. In particular,
`get_active_recommendations` returns only existing, unexpired recommendations
owned by the current learner; it does not call the generation path or create
DHT entries. Skill and recommendation targets remain authenticated references
until a shared catalogue lookup can provide trustworthy display names.

Illustrative civic, device, financial, legal, logistics, housing, and emergency
scenarios are visible only in Demo mode. They carry an explicit fiction notice
and their controls are disabled; Local and Live modes never present them as
operational state.

## Credential projection version 1

The credential browser calls
`credential_coordinator::list_my_credential_summaries` and receives a
versioned `CredentialList`. The coordinator follows the current learner's
`LearnerToCredentials` links, deduplicates targets, checks each entry's
`subject_id` against the authenticated agent, and returns shared
`CredentialSummary` values instead of raw `Record` values.

Issuance now creates the learner index in addition to issuer and course links.
Credentials issued before this coordinator change are not discoverable through
the new list until a deliberate subject-link backfill is run.

Proof presence is not proof verification. The current coordinator does not yet
implement signature or revocation checking, so `verify_credential` fails
closed and the browser says that verification has not run. A release must not
restore a positive UI state until the coordinator verifies the canonical
payload, issuer signature, expiry, and published revocation status.

The privacy switchboard is explicitly a local selection draft. CLR publishing,
portable identity export, and public-portfolio listing are not connected in
this build, so their controls are disabled and no example portfolio is
presented as a DHT record.

## Authority, ordering, and privacy

`sync_progress` does not accept a learner identity or timestamp. The
coordinator derives both from the authenticated zome call and conductor time;
integrity validation also binds the progress entry and learner index to the
committing agent. When two records have the same timestamp, the action hash is
the deterministic tie-breaker.

`LearnerProgress` is currently a **public entry**. The Leptos UI therefore uses
an explicit “Publish & verify progress” action and discloses the visibility
before writing. Do not turn this into automatic background synchronization
without a privacy migration and fresh learner consent.

## Deployment and migration

This contract adds `LearnerToProgress` to the learning integrity zome. Rebuild
the WASM/DNA/hApp and reinstall it; the new integrity definition is a new DNA,
not an in-place coordinator-only upgrade. Historical progress written before
this index exists is not returned by `get_my_progress` until a deliberate
backfill is designed.

The supplied `mycelix-leptos-client` now fails closed unless a Rust-side signer
or the standard `window.__HC_ZOME_CALL_SIGNER__` host callback returns encoded
call bytes and a nonzero 64-byte Ed25519 signature. Praxis tracks authenticated
transport connectivity and signing readiness separately; the progress action
stays disabled and the Live banner remains explicit when only the transport is
connected. Release qualification still requires an authorized real-conductor
lane that exercises this transport, not only `@holochain/client`.

Run the dependency-free conformance gate with:

```sh
python3 scripts/validate_learning_contract.py
```
