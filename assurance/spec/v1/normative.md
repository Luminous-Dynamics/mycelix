# MYCELIX-ASSURE/V1 Normative Kernel Specification

Status: ASSURE-002A protocol-kernel freeze candidate.

This specification defines the minimal normative surface for the ASSURE-V1 kernel.
ASSURE-002A does not decode assurance capsules, evaluate claims, establish authority,
or emit positive qualification results.

Normative terms MUST, MUST NOT, SHALL, SHALL NOT, and MAY are used as requirements.

## Scope

SPEC-001. The ASSURE-V1 specification package is normative. Rust code implements it;
Rust code does not redefine it.

SPEC-002. Normative identifiers and numeric registry assignments are append-only within
a major protocol version. Published assignments MUST NOT be renumbered or reused.

SPEC-003. Unknown security-relevant semantic values MUST NOT be interpreted using a
permissive default.

SPEC-004. Normative protocol hashing uses SHA-256 in V1. Hash-algorithm negotiation is
not part of ASSURE-V1.

SPEC-005. All normative hashes MUST use the registered domain separator appropriate to
the hashed object class.

SPEC-006. Security-relevant absence MUST NOT acquire meaning through a language-level
default. Unbounded, unknown, unavailable, and absent semantics MUST be explicit when
introduced by later tranches.

SPEC-007. ASSURE-002A MUST NOT expose APIs that decode capsules, evaluate claims,
establish effective authority, approve actions, or produce positive qualification.

SPEC-008. Normative mutations invalidate any prior validation or qualification witness.

## Time

TIME-001. Normative V1 time values are signed 64-bit Unix microseconds.

TIME-002. Finite validity intervals use half-open semantics [start, end).

TIME-003. A finite interval with end <= start is invalid.

## IDs and Digests

ID-001. NodeId is an intra-capsule logical identifier and is distinct from a content
digest.

ID-002. NodeId value zero is reserved and MUST NOT identify a normative node.

DIGEST-001. Digest values are exactly 32 bytes.

DIGEST-002. Content digests commit to canonical semantic content under an object-class
domain separator.

## Engineering boundary

ENG-001. The normative kernel MUST forbid unsafe Rust code.

ENG-002. The normative kernel MUST NOT contain build scripts in ASSURE-002A.

ENG-003. The default Cargo feature set MUST be empty.

ENG-004. Holochain, networking, async runtimes, databases, simulation engines, AI
systems, and application-domain crates MUST NOT be dependencies of assurance-core in
ASSURE-002A.

ENG-005. CI for assurance-core MUST be merge-gating rather than informational.

## Positive-assurance prohibition

POS-001. ASSURE-002A through ASSURE-002E MUST NOT expose a public result representing
Established, EffectiveAuthority, constitutional legitimacy, or political legitimacy.

POS-002. The ability to emit bounded positive epistemic results is reserved for a later
qualified tranche.
