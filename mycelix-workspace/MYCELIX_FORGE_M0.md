# Mycelix Forge M0 — Sovereign Verified Repository

**Status:** FORGE-001 design contract  
**Maturity:** Pre-implementation / review required  
**Scope:** Protocol boundary, threat model, invariants, and FORGE-002 → FORGE-008 qualification plan

## 1. Purpose

Mycelix Forge is not a replacement version-control system and is not a GitHub clone on Holochain.

M0 defines a **Sovereign Verified Repository**: an ordinary Git repository whose project identity, authority, review decisions, and source provenance can be verified without trusting the forge that happens to host it.

The M0 target is intentionally narrower than a complete decentralized forge. It proves that Mycelix can separate:

- project identity from repository location,
- authority from forge-admin status,
- review from mutable pull-request state,
- provenance from the CI provider,
- verification from network availability.

The first dogfood target is this Mycelix repository while it remains usable through normal GitHub workflows.

## 2. Non-goals for M0

M0 does **not** attempt to implement:

- a new source-control format,
- Git-object storage in the Holochain DHT,
- a blockchain or global consensus ledger,
- native P2P repository replication,
- ForgeFed federation,
- a GitHub-class web interface,
- decentralized CI markets,
- contributor reputation,
- project funding or bounties,
- automatic merge/release authority for Symthaea or any AI agent.

Those may be layered on after the trust substrate is qualified.

## 3. Architectural boundary

The protocol core MUST remain transport-, forge-, identity-provider-, and execution-provider-neutral.

```text
                         Mycelix Forge
                              │
                    portable core objects
                              │
          ┌───────────────────┼────────────────────┐
          │                   │                    │
       adapters            adapters             adapters
          │                   │                    │
       gittuf             Xenia/DID              SLSA
       GitHub               COSE                 in-toto
       Radicle            Holochain              SCITT
       ForgeFed              ...                 OCI/SPDX
                              │
                              ▼
                         Nix / Spore
```

The foundational Forge types MUST NOT require Holochain, Radicle, GitHub, gittuf, Xenia, Spore, or SLSA to deserialize and verify their intrinsic structure.

## 4. Core protocol objects

M0 reserves the following conceptual objects. FORGE-002 onward will freeze exact Rust/API representations.

### `ProjectIdentity`

Stable identity of a project independent of Git remote URLs, forge accounts, or mirrors.

Required properties:

- content-derived or root-authority-bound identifier,
- cryptographic algorithm agility,
- deterministic canonical representation,
- no dependency on a DNS name or GitHub repository ID,
- explicit protocol/version domain separation.

### `AuthorityEpoch`

A versioned statement defining which principals hold which project capabilities.

It MUST support:

- scoped capabilities,
- threshold authorities where policy requires them,
- rotation,
- revocation,
- explicit predecessor linkage,
- deterministic epoch identity,
- expiry where appropriate,
- multi-device identities without equating a person to one device key.

### `ProjectPolicy`

Versioned policy governing acceptance, review, build, release, recovery, and witness requirements.

Historical evidence is evaluated against the exact policy digest it names. A later policy change MUST NOT reinterpret an earlier authorization.

### `ChangeProposal`

An exact proposed source transition.

At minimum it binds:

- project identity,
- base revision,
- proposed revision,
- resulting tree/object identity,
- authority epoch,
- policy digest,
- protocol version.

### `ReviewAttestation`

A signed decision over an exact `ChangeProposal` subject.

Approval MUST NOT float across subsequent patch revisions. Changing any subject-defining byte creates a new subject and requires policy reevaluation.

### `QualificationReceipt`

Machine-produced evidence about an exact subject, such as source validation, tests, builds, security checks, or reproducibility observations.

A qualification receipt is evidence, not self-declared truth. Acceptance is determined by `ProjectPolicy`.

### `ReleaseManifest`

Exact statement of the accepted release source, artifacts, policy, authority epoch, and supporting evidence roots.

### `WitnessReceipt`

Portable external evidence that a statement was registered/observed by a transparency or witness service. SCITT-compatible receipts are the preferred standards direction; Mycelix MUST NOT require a proprietary global ledger.

## 5. Normative invariants

The following are M0 invariants. A later design MAY strengthen them but MUST NOT silently weaken them.

### F-I01 — Git remains valid Git

A Mycelix Forge repository MUST remain an ordinary usable Git repository. Source history is not re-encoded into a Mycelix-only format.

### F-I02 — Hosting is not identity

Changing a GitHub/GitLab/Forgejo/Radicle remote MUST NOT change `ProjectIdentity`.

### F-I03 — Exact-subject authorization

Every approval, qualification, and release authorization MUST cryptographically bind the exact subject it applies to.

### F-I04 — Final-revision review

A review approval MUST apply to the final reviewed source transition. A source/tree mutation after approval invalidates applicability of that approval unless policy explicitly defines a separately verifiable transformation.

### F-I05 — Explicit authority context

Signed project actions MUST identify the authority epoch and policy context under which the signer claims authority.

### F-I06 — Capability-scoped authority

Authority MUST be expressed as scoped capabilities rather than a single universal `admin` bit. Possessing one capability MUST NOT imply unrelated authority.

### F-I07 — Epoch-bounded revocation and rotation

Authority history MUST be append-only/version-linked. Revocation or rotation creates a new authority state and MUST NOT rewrite valid historical evidence.

### F-I08 — Crypto agility

Digests, keys, and signatures MUST carry algorithm identifiers. Protocol objects MUST NOT encode an assumption that one hash or signature algorithm is permanent.

### F-I09 — Deterministic signing representation

Anything signed or hashed as a protocol subject MUST have one deterministic canonical representation. Human-editable configuration MAY have a separate representation.

### F-I10 — Offline verification

A release verification bundle MUST be verifiable without contacting GitHub, Mycelix, a transparency service, or an identity server once the required evidence is locally present.

### F-I11 — Open evidence export

Where mature standards exist, Mycelix SHOULD export/consume them rather than invent incompatible equivalents. Initial interoperability targets are gittuf/in-toto, SLSA, COSE, SCITT, SPDX, and OCI artifacts.

### F-I12 — No routine global consensus

Routine collaboration MUST NOT require blockchain/global consensus. Consensus, threshold decisions, or external witnesses are used only where project policy requires shared finality or equivocation detection.

### F-I13 — Holochain is not the Git blob store

Holochain MAY provide discovery, organization state, relationships, reputation evidence, governance, builder discovery, and replicated coordination objects. Git source objects remain in Git-compatible storage/replication systems.

### F-I14 — AI has no implicit authority

Symthaea or another AI MAY analyze, recommend, detect, or produce explicitly authorized machine evidence. It MUST NOT obtain merge, root, recovery, or release authority merely by being integrated with Forge.

### F-I15 — Verification does not trust Mycelix

A verifier MUST be able to validate a completed M0 evidence bundle with an independent implementation. `mycelix.org` or a running Mycelix network is not a root of trust.

## 6. Trust boundaries and required failure behavior

| Threat / failure | Required M0 behavior |
|---|---|
| Malicious or compromised Git mirror | Cannot substitute a different `ProjectIdentity` or silently satisfy exact-subject policy |
| Forge administrator compromise | Forge admin status alone conveys no project protocol authority |
| Maintainer key compromise | Blast radius limited to capabilities and epoch accepted by policy |
| Revoked/stale key | New actions under an invalid epoch are rejected |
| Source changed after review | Prior approval no longer applies |
| Force-pushed protected history | Detected or rejected by repository policy adapter |
| CI lies about a subject | Receipt subject mismatch is rejected; later build quorum can detect artifact mismatch |
| Transparency service unavailable | Previously issued local receipts remain verifiable |
| One witness equivocates | Multi-witness policy can expose inconsistent observations without global consensus |
| GitHub/Mycelix network outage | Local repository and evidence remain inspectable/verifiable |
| Algorithm deprecation | New authority epoch/policy can migrate algorithms without redefining project identity semantics |
| Malicious AI recommendation | Has no effect unless a human-authored deterministic policy explicitly consumes authorized machine evidence |

## 7. Canonicalization direction

FORGE-002 must select and freeze canonical test vectors before production use.

Preferred direction:

- deterministic CBOR for signed/hashed wire objects, aligned with COSE/SCITT ecosystems,
- explicit domain-separation strings per object type and protocol version,
- algorithm-qualified digest wrapper rather than raw fixed-size SHA-specific fields,
- human-facing TOML/JSON/YAML only as configuration/authoring forms, never as ambiguous signing representations unless canonicalization is formally specified.

No canonical format is considered frozen by FORGE-001 alone.

## 8. M0 data flow

```text
ordinary Git repository
        │
        ▼
ProjectIdentity
        │
        ▼
AuthorityEpoch + ProjectPolicy
        │
        ▼
exact ChangeProposal
        │
        ▼
ReviewAttestation(s)
        │
        ▼
source-policy verification
        │
        ▼
SLSA source provenance export
        │
        ▼
local verification bundle
```

FORGE-009+ will extend this into independent Nix/Spore build qualification, artifact reproduction, release thresholds, and transparency receipts.

## 9. M0 adversarial acceptance tests

The M0 tranche is not qualified until automated tests demonstrate at least:

1. **Remote relocation:** changing repository remote URLs leaves project identity unchanged.
2. **One-byte mutation:** changing one subject-defining byte changes the proposal digest and invalidates prior review applicability.
3. **Wrong project:** an attestation from project A cannot validate project B.
4. **Wrong epoch:** a review claiming an authority epoch not accepted by policy fails.
5. **Revocation:** a principal valid in epoch N cannot create newly accepted actions after policy advances to an epoch that revokes it.
6. **Wrong policy:** evidence bound to policy P cannot silently satisfy P'.
7. **History rewrite:** the repository policy adapter detects a disallowed protected-reference rewrite.
8. **Offline verify:** a fixture repository plus evidence bundle verifies with networking disabled.
9. **Unknown algorithms:** unsupported algorithms fail closed with a typed error rather than being treated as equivalent.
10. **Round-trip determinism:** independent serialization/deserialization produces the same canonical digest test vectors.

## 10. FORGE-001 → FORGE-008 tranche

### FORGE-001 — Threat model and invariants

**This document.**

Exit gate:

- protocol scope/non-goals explicit,
- trust boundaries explicit,
- normative invariants explicit,
- M0 adversarial gates explicit,
- adapter boundary explicit.

### FORGE-002 — Portable project identity core

Create a small HDK-free Rust crate (proposed: `mycelix-forge-core`) containing only foundational identifiers/canonicalization needed for `ProjectIdentity`.

Deliverables:

- `ProtocolVersion`,
- algorithm-qualified `Digest`,
- `ProjectIdentity`,
- deterministic canonical encoding,
- domain separation,
- frozen test vectors,
- mutation/round-trip/property tests.

No Holochain, GitHub, Radicle, Xenia, gittuf, or network dependency.

### FORGE-003 — Authority epochs and capabilities

Add:

- `PrincipalId`,
- typed capabilities,
- `AuthorityEpoch`,
- predecessor linkage,
- thresholds,
- expiry/revocation semantics,
- monotonic transition validation.

Required adversarial tests include replay of stale authority, capability escalation, threshold underflow, malformed predecessor, and algorithm substitution.

### FORGE-004 — gittuf repository-policy adapter

Integrate forge-independent Git policy outside the core crate.

Goals:

- map Forge project principals/capabilities into repository-policy principals/rules,
- verify protected source history without trusting GitHub branch settings,
- retain native gittuf/in-toto interoperability where possible,
- keep gittuf details out of the portable core model.

### FORGE-005 — Mycelix/Xenia identity bridge

Bind protocol principals to Mycelix/Xenia identities without making those identities mandatory for independent verification.

Goals:

- multi-device key binding,
- rotation and recovery mapping,
- crypto-agile signatures,
- COSE-oriented external representation where suitable,
- no frontend/browser key-material assumptions in the core crate.

### FORGE-006 — Exact-subject change proposals

Implement `ChangeProposal` with explicit base/source/tree/policy/authority binding.

Exit gate: every mutation test that changes a subject-defining field causes a distinct canonical digest and prior review no longer applies.

### FORGE-007 — Signed review attestations

Implement review decisions as portable evidence rather than mutable forge state.

Required behavior:

- exact proposal binding,
- reviewer principal binding,
- authority/policy context binding,
- offline verification,
- stale/revoked authority rejection according to policy,
- deterministic evidence fixtures.

### FORGE-008 — SLSA Source provenance adapter

Export/verify source provenance for the protected source transition.

Exit gate:

- provenance identifies the exact source revision,
- protected-branch/source-policy evidence can be evaluated independently of GitHub,
- fixture can be verified offline,
- Forge-native evidence and SLSA representation are cross-checked by tests.

## 11. Dependency rule for the first tranche

The dependency direction MUST stay inward toward the portable core:

```text
mycelix-forge-gittuf  ─┐
mycelix-forge-xenia   ─┼──> mycelix-forge-core
mycelix-forge-slsa    ─┘
```

`mycelix-forge-core` MUST NOT depend back on adapters.

Holochain-specific coordination, Radicle replication, ForgeFed, GitHub bridges, Spore builders, SCITT witnesses, OCI publication, and Symthaea analysis are post-M0 adapters/services unless a later reviewed design demonstrates a core requirement.

## 12. Dogfood target

The first end-to-end M0 demonstration SHOULD use `Luminous-Dynamics/mycelix` itself while preserving GitHub as a normal hosting/collaboration endpoint.

Success means a verifier can establish, from local Git data plus the evidence bundle:

- which project this is,
- which authority epoch governed the action,
- which policy applied,
- exactly which source transition was reviewed,
- which authorized reviewer(s) approved it,
- what source provenance describes the accepted revision,

without treating GitHub's database, UI badges, or account permissions as the root of truth.

## 13. Deferred decisions

FORGE-001 intentionally does not freeze:

- exact URI syntax for project IDs,
- exact canonical CBOR profile,
- exact hash/signature suites,
- whether Radicle COBs or Mycelix-native replicated objects become the primary collaboration transport,
- exact SCITT deployment/witness topology,
- private-repository epoch-encryption format,
- Holochain entry/zome layout,
- release/build quorum syntax,
- reputation or economic mechanisms.

Each must be resolved by evidence and interoperability testing rather than by prematurely expanding the protocol core.

## 14. Design principle

> Do not decentralize Git again. Decentralize the institutional trust, authority, provenance, collaboration, and release machinery that accumulated around Git — while ensuring the resulting evidence remains independently verifiable even if Mycelix itself disappears.
