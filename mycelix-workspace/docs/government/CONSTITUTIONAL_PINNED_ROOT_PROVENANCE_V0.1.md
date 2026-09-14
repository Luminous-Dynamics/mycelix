# Constitutional Pinned-Root Provenance v0.1

Status: **normative bootstrap-provenance profile**

GOVSYS-003B qualifies one exact generation-zero GOVSYS-003A Root-A commitment against one externally provisioned exact pin.

## Governing theorem

```text
exact generation-zero Root-A candidate
+ one externally provisioned exact Root-A pin
        -> QualifiedPinnedRootProvenance
```

and explicitly:

```text
pinned-root provenance
!= root currentness
!= source coverage
!= transition authorization
!= legal legitimacy
!= ordinary policy currentness
!= actor authority
!= execution authority
!= external-effect authority
```

The pin is an explicit deployment trust assumption. GOVSYS-003B proves exact agreement with that assumption; it does not manufacture the assumption from Root-A itself.

The network remains infrastructure for institutions. It is not the sovereign.

## Exact qualified parent

GOVSYS-003B is a direct child of qualified GOVSYS-003A exact subject:

`235834fc726c1a79467e6f910a2cf39fb9abd2e3`

Root-A profile:

`mycelix-constitutional-trust-root-v1-sha256-framed-semantic`

Normative generation-zero Root-A fixture digest:

`c3f9ba9b323f20c2d2ebd597e424857e8f3459d31393886fd6a7f835a6a93d6d`

The parent exact-head hosted qualification is run `34836891820` and passed its Root-A oracle, both derived trust-role vectors, exact-surface/ancestry checks and immutable-checkout postflight.

## Registered identifiers

Protocol:

`mycelix-constitutional-pinned-root-provenance-v0.1`

Provisioned-pin profile:

`mycelix-provisioned-constitutional-root-pin-v1`

Provenance identity profile:

`mycelix-constitutional-pinned-root-provenance-v1-sha256-framed-semantic`

Unframed provenance domain separator:

`mycelix/public-institution/constitutional-pinned-root-provenance/v1`

Digest algorithm: SHA-256.

## Provisioned pin

v0.1 accepts exactly one explicit pin with exactly:

```text
pin_profile
root_identity_profile
root_digest
bootstrap_profile
provisioning_ref
```

The pin is not discovered by searching a trust store. There is no "first match", "latest", wildcard, quorum, preference, fallback or caller-selected list in v0.1.

`provisioning_ref` is bounded audit metadata identifying the external provisioning ceremony/configuration/source. A non-empty reference is not proof by itself.

## Candidate requirements

The independently recomputed Root-A candidate MUST satisfy all of the following:

- Root-A validation succeeds under the exact qualified GOVSYS-003A profile;
- `generation == 0`;
- `predecessor_root_digest == None`;
- `bootstrap_mode == pinned-constitutional-commitment`;
- candidate `bootstrap_profile` exact-matches the pin bootstrap profile;
- candidate Root-A identity profile exact-matches the pin root-identity profile; and
- independently recomputed candidate Root-A digest exact-matches the pin digest.

A root declaring pinned bootstrap mode does not create a provisioned pin. The pin remains a separate input.

## Generation-zero only

A new external pin MUST NOT bless an arbitrary successor generation.

Even when an external pin exactly names a recomputed generation-1 Root-A digest, GOVSYS-003B rejects it.

```text
fresh external pin for generation > 0
!= predecessor-authorized constitutional rotation
```

Normal successor authorization belongs to GOVSYS-003C/#839. Recovery or reprovisioning requires a separately named theorem with explicit trust semantics.

## Complete Root-A trust commitment

Because the pin matches the **complete** Root-A digest, it indirectly anchors every semantic field committed by Root-A, including two independent trust roles:

```text
root source / coverage / verification profile / source anchor
        !=
rotation profile / rotation-authority anchor
```

GOVSYS-003B does not separately accept caller-selected source-verifier or transition-verifier configuration. Changing either trust role changes Root-A identity and invalidates the old pin.

The pin does not prove either verifier has executed successfully.

## No currentness clock

GOVSYS-003B deliberately has no `now`, wall-clock or trusted-time input.

Root-A validity timestamps are part of semantic identity, but bootstrap provenance is not a live-currentness theorem. A candidate may exact-match its provisioned pin even if its `valid_from_ms` is later than the machine executing this qualification.

Root-D owns live/effective currentness after rooted lineage and independently authenticated source coverage exist.

## No ordinary policy recursion

The GOVSYS-003B input surface contains no ordinary policy-provider receipt, policy-currentness result, policy namespace search, authority grant, administrative decision, reputation/Phi score or runtime effect permission.

```text
ordinary policy currentness
cannot certify
its own constitutional Root-A bootstrap
```

## Provenance identity

Successful qualification has a deterministic audit identity over the exact external assumption:

```text
DOMAIN_UNFRAMED
|| frame(PROVENANCE_IDENTITY_PROFILE)
|| frame(PROTOCOL_VERSION)
|| frame(PIN_PROFILE)
|| frame(ROOT_A_IDENTITY_PROFILE)
|| frame(raw_32_byte_root_digest)
|| frame(bootstrap_profile)
|| frame(provisioning_ref)
```

where:

```text
frame(x) = u64_le(len(x)) || x
```

The normative fixture provenance digest is:

`dc28fbf6ba1396dd34908c24fca6faa51bc98b0b78dcab25091e07be425313db`

This identity records which exact external pin assumption was matched. It does not upgrade that assumption into legal/current/effect authority.

Changing only `provisioning_ref` changes provenance identity while leaving Root-A identity unchanged. Changing Root-A semantics changes the pinned Root-A digest and therefore changes provenance identity as well.

## Deterministic text and digest validity

Text is exact UTF-8, non-empty, bounded, contains no ASCII control byte and has no leading/trailing ASCII space. No trimming, Unicode normalization, alias expansion or case folding occurs.

Digest values are raw non-zero 32-byte semantic values. JSON uses exactly 64 hexadecimal characters as transport; hexadecimal letter case is representation-only.

## Adversarial corpus

The executable oracle freezes at least:

- exact generation-zero Root-A + exact one pin -> qualify provenance;
- wrong pin root digest -> deny;
- wrong Root-A identity profile -> deny;
- wrong bootstrap profile -> deny;
- zero pin digest -> deny;
- malformed provisioning reference -> deny;
- candidate bootstrap mode other than pinned -> deny;
- generation 1 with a freshly recomputed exact matching pin -> deny;
- generation 1 with predecessor + exact matching pin -> deny;
- source-anchor mutation under old pin -> deny;
- rotation-authority-anchor mutation under old pin -> deny;
- pin hex case changes only representation, not meaning;
- provisioning-ref change changes provenance identity;
- no ambient clock participates; and
- positive provenance does not imply currentness or consequential authority.

## Intended composition

```text
GOVSYS-003A exact Root-A identity
+ external exact pin
        ↓
GOVSYS-003B QualifiedPinnedRootProvenance
        ↓
#839 predecessor-owned transition verification
        ↓
GOVSYS-003C / #837 rooted lineage
        ↓
GOVSYS-003D / #837 covered current head
```

## Explicit nonclaims

GOVSYS-003B does not establish source authenticity, closed-world source coverage, current root, predecessor transition authorization, legal/democratic legitimacy, policy adoption, administrative competence, judicial competence, execution authority or external-effect permission.
