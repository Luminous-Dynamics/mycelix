# MYC-CAP-002G2A — Governance decision currentness v1

Status: candidate executable profile. The previous hosted subject completed **20/20 tests and deterministic receipt reconstruction** before failing only on a stale duplicated verifier-file hash; this replacement keeps theorem/test bytes unchanged and makes the exact Git subject/scope the implementation-byte authority.

## Purpose

Derive whether one historical G1 stewardship decision remains current relative to one exact designated constitution profile and explicit revocation set.

Core separation:

```text
historical authorization != current authority
current constitution designation != constitution amendment authority
CURRENT != execution authority
```

## Currentness states

```text
CURRENT
HISTORICAL
REVOKED
PENDING
NOT_ASSESSED
```

No wall-clock time is an authority source in v1.

## Protected constitution boundary

Before a designated G1 profile can be treated as current it must validate structurally, bind the exact project, retain the protected chamber topology, contain no `CAPITAL` chamber, and keep `ASSET_LOCK_REMOVAL` and `STEWARD_SEAT_SALE` prohibited/constitutional/enforcer-protected.

A registry pointer therefore cannot bless a constitution that silently weakens the commons boundary.

## Currentness theorem

```text
designation PENDING -> PENDING
designation REVOKED -> NOT_ASSESSED
G1 authorization not AUTHORIZED -> NOT_ASSESSED
exact decision receipt explicitly revoked -> REVOKED
G1 receipt profile != designated active profile -> HISTORICAL
otherwise -> CURRENT
```

`HISTORICAL` remains audit evidence but is not current authority for a new execution.

Revocations are exact project/registry/epoch/authority/decision-receipt bound. Re-authorization requires a new governance decision; v1 has no resurrection flag.

## Output boundary

Every receipt fixes:

```text
execution_authority_established = false
legal_validity_established = false
democratic_legitimacy_established = false
```

`CURRENT` is relative only to the exact supplied designated constitution and revocation set.

## Semantic commitments

Designated G1 constitution semantic SHA-256:

`945b558049d7570cd47b52d7d822b2dff142a46b3d5fa117ed24dadbcd1059cb`

G1 decision receipt semantic SHA-256:

`5a330393381cdce9bd3d509a3aeab50fe66bb9c688cfc479c1bcb5efc0cbdccc`

Currentness profile semantic SHA-256:

`750981209aaab465dc4e54195b711a9e59f3a29f2d3e0af71bb63f65fa5e543f`

Frozen currentness receipt semantic SHA-256:

`929f5cdd8eef25e56d557a17a4f4a5ab59694579555d60ad75dc90a937973a1e`

Implementation/test/fixture/receipt bytes are committed by the exact Git subject and exact six-file diff scope. The hosted workflow reconstructs the receipt twice and requires byte identity. Manually duplicated source-file SHA literals are not a second qualification authority.

## Hosted evidence carried forward

Superseded run `35085978695` established that exact-parent checkout, Python compile, **20/20 fail-closed regressions**, and deterministic receipt reconstruction all executed successfully. It did not qualify the subject because the subsequent duplicated verifier-file checksum was stale.

The replacement subject must re-execute the same semantics under the corrected qualification contract before PASS can be claimed.

## Nonclaims

Even hosted PASS would establish only currentness of one supplied G1 decision relative to one exact designated constitution and revocation set. It would not establish execution authority, amendment authority, legal validity, democratic legitimacy, identity authenticity, social consensus, designation legitimacy, or wisdom.
