# Identity Vault Private Entry Boundary v0

**Status:** first implementation slice under review; not end-to-end qualified.  
**Tracks:** [IDENTITY-PRIVACY-001 / #5344](https://github.com/Luminous-Dynamics/mycelix/issues/5344).

## Paradigm shift

**Privacy is a protocol property, not a code comment.** A source-chain record is not private merely because a module describes it as private. Holochain entry payload visibility must be declared on the entry type. Holochain's entry guide describes explicit private entry visibility, and its source-chain query model allows the author to query their own private entries while public action/link metadata remains a distinct concern:
- https://developer.holochain.org/build/entries/
- https://developer.holochain.org/build/working-with-data/
- https://developer.holochain.org/build/querying-source-chains/

This slice explicitly classifies all six existing Identity Vault payload types as owner-private, moves the profile/key lookup path off public link indexes, and rejects new public index links for these private types. It does **not** retroactively privatize older public data.

## Entry-by-entry visibility policy

| Entry type | v0 decision | Reason | Disclosure path |
|---|---|---|---|
| `Profile` | Private | Bio, avatar and metadata may be identifying or sensitive | Owner-local profile read; the existing field-selective `disclose_profile` remains a separate API whose remote access still depends on Holochain capability policy |
| `MasterKey` | Private | Generic key inventory and usage labels are not a public directory contract | Owner-local lookup only. A peer-discoverable X25519 key needs a separate typed, author-bound, proof-of-possession binding |
| `TierMembershipProof` | Private | Raw proof bytes, tier and commitment metadata can be linkable; proving a claim must be a deliberate disclosure | This slice removes the public index link. A separately reviewed verifier presentation endpoint is still needed; this is a deliberate compatibility break for clients that previously relied on the public `AgentToProof` index |
| `DigitalWill` | Private | Successor, resource distribution and pre-signed proof are sensitive | No public index/disclosure path in this slice |
| `AiMineralization` | Private | Agent linkage, archive shards and life-cycle metadata are sensitive | No public index/disclosure path in this slice |
| `SymbioteSnapshot` | Private | Snapshot CID, timestamp and encrypted-key shard can reveal relationships or support correlation | No public index/disclosure path in this slice |

The enum variant order is preserved to avoid unnecessary entry-type index churn. All six existing `LinkTypes` variants are retained for source compatibility, but integrity validation rejects creation of new links for them. New privacy work should create a dedicated public key-binding or disclosure schema rather than reclassifying this heterogeneous store as public.

## Code path changes in this slice

- Added explicit `#[entry_type(visibility = "private")]` annotations to all six variants in `mycelix-personal/zomes/identity-vault/integrity/src/lib.rs` and the workspace mirror.
- Integrity validation rejects newly created `AgentToProfile`, `AgentToKeys`, `AgentToProof`, `AgentToWill`, `AgentToSnapshot` and `AgentToMineralization` links. The enum variants remain defined.
- Removed coordinator writes for `AgentToProfile`, `AgentToKeys` and `AgentToProof`. The coordinator no longer calls `create_link` or `get_links` for this private store.
- `get_my_profile` queries private Profile create actions newest-first from the current agent's source chain.
- `get_my_keys` queries MasterKey create actions from the current agent's source chain.
- Added an ignored three-agent Sweettest scenario: Alice can read her profile and generic key records; Bob's owner-local queries return no Alice profile/keys.
- Updated comments to distinguish explicitly private entry payloads from source-chain placement alone.
- Kept root/workspace integrity, coordinator and Sweettest files byte-identical after editing.

## Test evidence and limits

**Authored, not yet executed:** the Identity Vault Sweettest uses synthetic values and is marked ignored because it requires the built Personal DNA and a conductor. The scenario checks owner-local API behavior only. It is not proof that a second agent cannot directly request a known public action hash through the DHT. A separate packaged-DNA multi-agent test must attempt direct retrieval of each new private payload while confirming that owner-local reads remain functional.

The integrity unit test exercises the six-link classification helper. Native tests, formatting, WASM build, strict Clippy, packaged-DNA build and multi-agent conductor scenarios still need an exact-head run. A queued, pending or skipped job is not a pass.

## Migration and observable metadata

Changing entry visibility changes the integrity/DNA definition and therefore the resulting DNA hash. Existing entries created under the previous public definitions, and previously published link actions, cannot be made confidential retroactively by this source change. Before deploying:
1. classify legacy entries and links as potentially public;
2. warn users instead of silently relabelling them private;
3. plan a user-controlled export/re-encryption into the new DNA/network;
4. test old/new DNA migration and preserve enum ordering where possible;
5. do not promise that deleting a link or deploying a new DNA erases old DHT copies.

Private payloads also do not make all action metadata anonymous. Public action headers, author/timing, entry-type indexes and hashes may remain observable. This is content-privacy hardening, not metadata anonymity.

## Benchmarks and release gates (targets, not results)

- 6/6 sensitive Identity Vault entry variants have explicit visibility.
- Zero newly public DHT payloads for the six variants under a fresh, newly built DNA.
- Zero new public index links for the six owner-private categories.
- 100% owner-local read parity for profile and generic key data across root/workspace projections.
- 100% rejection of all six forbidden new index-link types in integrity tests.
- Three-agent runtime: authorized owner-local reads succeed; another agent's local profile/key reads remain empty; direct DHT retrieval of known private action hashes returns no private payload.
- 100% correct classification of migration fixtures as legacy-public versus new-private.
- Measure p50/p95/p99 source-chain query latency and packaged-DNA startup cost after the privacy gates pass. Do not trade confidentiality for a faster lookup.
- Bind native/WASM/Clippy/Sweettest receipts to one exact source SHA, DNA bundle hash, Holochain version and fixture digest.

## Industry comparison / claim ceiling

This uses Holochain's explicit entry-visibility and owner source-chain query model as the correctness baseline. There is no SOTA or production-security superiority result yet. A comparative claim requires reproducible tests of both direct DHT access and the authorized disclosure path, measured latency and the legacy migration behavior—not just source annotation checks.

This slice does not establish POPIA or clinical compliance, key enrollment, authenticated public X25519 discovery, verifier presentation for tier proofs, or retroactive erasure.
