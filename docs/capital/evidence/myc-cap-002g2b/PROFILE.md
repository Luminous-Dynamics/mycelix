# MYC-CAP-002G2B — Constitution amendment transition v1

Status: candidate executable profile. Superseded run `35086060687` completed **30/30 tests and deterministic receipt reconstruction** before failing only on a stale duplicated fixture-file checksum. This replacement preserves theorem/test/fixture/receipt bytes and makes the exact Git subject/scope the implementation-byte authority.

## Purpose

Qualify one append-only constitution transition from epoch N to N+1 without allowing current designation, emergency power, refinancing, custody change, or a generic governance vote to become a universal constitution-edit key.

Core separation:

```text
current constitution != amendment authority
amendment authorization != successor designation
accepted amendment != legal validity
```

## Conservative v1 amendment surface

The frozen classes are:

```text
POLICY_PARAMETER
REPRESENTATION
CONSTITUTIONAL_STRUCTURE
PROTECTED_INVARIANT
```

Only `POLICY_PARAMETER` is permitted in v1. The verifier computes the semantic constitutional diff itself; caller-supplied change summaries are not authority.

Set-like G1 fields are normalized before comparison so harmless ordering changes do not become fake amendments.

The allowed v1 change paths are limited to OPERATOR_RENEWAL PUBLIC/USERS quorum and approval thresholds. Hidden semantic changes outside the frozen path set block the transition.

## Protected commons invariants

Both prior and successor G1 profiles must preserve the protected chamber topology, forbid a `CAPITAL` chamber, keep `ASSET_LOCK_REMOVAL` and `STEWARD_SEAT_SALE` prohibited/constitutional, and retain protected enforcer requirements.

The successor epoch must equal exactly `prior_epoch + 1`. Refinancing, operator replacement, custody change, insolvency, migration, or SPV replacement cannot reset the constitutional epoch.

## No self-amendment

The exact G2A receipt establishes only prior-constitution currentness. Amendment authorization is reconstructed separately.

Even `transition_state = ACCEPTED` fixes:

```text
successor_designation_established = false
legal_validity_established = false
democratic_legitimacy_established = false
```

A separate G2C designation transition must make the successor current.

## Semantic commitments

Amendment profile semantic SHA-256:

`454e3c00a47f1e970ebf9a1bfa688515e70ccca0aeb824dc50352476c1322398`

Prior G1 profile semantic SHA-256:

`945b558049d7570cd47b52d7d822b2dff142a46b3d5fa117ed24dadbcd1059cb`

Successor G1 profile semantic SHA-256:

`84cbcaff53366ab611402fc5114e9f4eafa066ef753f8e722ae9caff13b79284`

Prior G2A currentness receipt semantic SHA-256:

`929f5cdd8eef25e56d557a17a4f4a5ab59694579555d60ad75dc90a937973a1e`

Frozen amendment receipt semantic SHA-256:

`d26ee8750ef11b23809321501cdd2dc8e7d491b9e12a5baa5ace9afea85b339b`

Implementation/test/fixture/receipt file identity is committed by the exact Git subject and exact six-file diff scope. The hosted workflow reconstructs the receipt twice and requires byte identity; manually duplicated file SHA literals are not a separate authority plane.

## Hosted evidence carried forward

Superseded run `35086060687` proved that exact parent checkout, Python compile, all **30** fail-closed regressions, and deterministic receipt reconstruction executed successfully. The run remained a FAILURE because a later manually copied fixture checksum was stale, so it does not qualify the subject.

The replacement subject must re-execute the theorem under the corrected qualification contract before PASS can be claimed.

## Nonclaims

Even hosted PASS would establish only that one supplied constitution transition satisfies this frozen amendment profile over supplied evidence. It would not establish successor designation/currentness, legal or constitutional-law validity, democratic legitimacy, ballot/identity authenticity, social consensus, jurisdictional enforceability, or wisdom.
