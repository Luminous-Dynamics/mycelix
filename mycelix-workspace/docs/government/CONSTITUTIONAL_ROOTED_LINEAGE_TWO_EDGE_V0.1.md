# GOVSYS-003C-B — Fresh Two-Edge Constitutional Rooted Lineage v0.1

## Theorem

This tranche qualifies one fresh, cryptographically independent, two-edge observed constitutional history:

```text
external Root-B pin(G0)
        ↓
G0 --K0--> G1 --K1--> G2
                    immutable
        ↓
actual CORE-LINEAGE projection
        ↓
stable observed constitutional history
```

It is a child of the already-qualified GOVSYS-003C-A one-edge composition theorem. It does not rewrite that evidence lineage.

The core claim is:

```text
exact Root-B bootstrap
+ two independently reverified predecessor-authorized Ed25519 transitions
+ exact G0→G1→G2 endpoint binding
+ one derived constitutional lineage domain
+ replay-conflict closure over the supplied edge set
+ actual CORE-LINEAGE structural projection
=
one stable supplied two-edge constitutional history

!= globally complete constitutional history
!= globally fork-free constitutional history
!= current constitutional root
!= authority to perform external effects
```

Root-D remains the owner of closed-world coverage and current-head promotion.

## Fresh cryptographic corpus

The fixture is intentionally unrelated to the original #839 signing material.

- G0 generation: `0`
- G1 generation: `1`
- G2 generation: `2`
- G0→G1 is authorized by independent fixture key K0.
- G1 embeds a different rotation-authority anchor for K1.
- G1→G2 is authorized by K1.
- G2 is `immutable` and carries no future rotation authority.

Neither private fixture key is checked in. The repository contains only canonical Ed25519 SPKI public material and the signatures over the exact frozen transition transcripts.

This makes the second edge an actual predecessor-authority inheritance test. Reusing edge0's valid signature for edge1 is rejected. Reusing K0's public material for edge1 is also rejected because it does not match G1's committed rotation authority.

## Stable identities

```text
G0
2684cc6c8b8f13e2e564b0bb7682dde0d333124d4a5db4ed04eaae1ad99023c9

G1
8f868c8c33698e6734ead90df9383f81141b6ff52d8ab765aa2aa5d6a028810c

G2
49be9b61e1c3f2b8f016e3b4e9c416dc83c689760a83cfb28ec9f6ff1f7d8825

edge 0
19e565417b53d57e0e73250dae14d33665d9461839083ce6260dc296d03cf2e1

edge 1
4d41a29c0c3ad8a51bde874d2fb25a709b1a9b7099fa82da855eabb4374111f7

lineage domain
d95e2f1af546480ebfcc1218f7ad671ac38ac3898cdce89a91328407dc3bc005

CORE-LINEAGE stable commitment
579c8810162e88958117b085fad6d01b599d8222a37fcd6e540f787c4fcdd5b1

constitutional history
d814323282d6f9ec9ca5c6ab32fe3cd5cd529f73286a9075588b89e7afd8c18f

Root-B provenance
c0055cf1a57e7ee013b81425ba78bf5f04d1d2d0d0a8f7ac61bd8ec3db0c9003

qualification provenance
c6e554cad1cf2610bae81f64d9760c60913e9184690f07297a10e7e84775de3f
```

## Structural semantics

CORE-LINEAGE treats supplied transitions as a semantic set, not a caller-ordered log. Therefore:

- reversed input order must yield the same stable commitment;
- an exact duplicate edge is harmless and normalizes away;
- a same-predecessor/same-successor edge with another transition identity is a parallel-transition conflict;
- a same-predecessor/different-successor edge is a fork conflict;
- a generation gap is rejected;
- a middle predecessor mismatch is rejected;
- effective-time regression is rejected; and
- lineage-domain substitution is rejected.

The Python qualification oracle independently reconstructs the exact two-edge CORE-LINEAGE transcript. A separate Rust harness then calls the actual inherited `mycelix-core-lineage` implementation and must obtain the exact same commitment.

## Terminal semantics

G2 is deliberately immutable. The inherited cryptographic transition verifier must reject any attempt to use G2 as the predecessor of a third ordinary rotation.

This does **not** mean G2 is globally current. It only means this supplied chain terminates in a root that cannot itself authorize another ordinary transition under this profile.

## Authority boundary

This tranche does not mint authority. Successful output carries:

```text
grants_currentness = false
grants_effect_authority = false
```

No local “latest generation” rule is permitted to convert the endpoint into current constitutional authority.

The network remains infrastructure for institutions. It is not the sovereign.
