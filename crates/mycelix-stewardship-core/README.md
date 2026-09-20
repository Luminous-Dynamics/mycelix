# mycelix-stewardship-core

`mycelix-stewardship-core` is the dependency-free theorem layer for Mycelix stewardship work.

STEW-001 freezes only structural artifact identity:

```text
logical subject
!= revision
!= representation
!= exact bytes
```

The first profile contains typed IDs for the logical subject, revision, and representation plus an exact 256-bit content commitment and a media-neutral representation kind.

## Why the layers remain distinct

A composition is not one edition. An edition is not one encoding. An encoding is not its logical work. Identical bytes can also appear in distinct stewardship contexts, so:

```text
same digest != same logical subject
```

Likewise:

```text
same logical subject != same revision != same representation != same bytes
```

`IdentityRelationV1` makes these states explicit, including the important conflict case where one representation ID is associated with different byte commitments.

## Scope boundary

This crate establishes no:

- authorship or creator identity;
- copyright or legal title;
- cultural/community legitimacy;
- access or disclosure permission;
- epistemic truth;
- preservation durability;
- provenance relation;
- reciprocity obligation;
- signature verification;
- hash computation.

Those belong to later STEW layers. A content digest carried by this crate is evidence input, not proof that a hashing process was performed correctly.

## Identifier policy

Protocol IDs are bounded opaque ASCII strings. Human names, titles, cultural descriptions and translations must remain separate Unicode-bearing metadata rather than being forced into protocol identifiers.

## Qualification

The dedicated workflow runs exact-head checks, formatting, host tests, wasm32 compilation, strict Clippy, rustdoc, and a dependency-closure assertion.
