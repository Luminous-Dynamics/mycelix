# Mycelix Public Election Adversarial Corpus v0.1

Status: **ADV-001 structural executable corpus; not cryptographic protocol qualification**

Profile: `mycelix-public-election-adversarial-corpus-v1`

## Purpose

ELECT-001 through ELECT-012 define public-election authority, privacy boundaries, transparency/witness invariants, offline verification, and physical evidence contracts. ADV-001 changes the mode of work from adding architecture to **trying to break the architecture we already have**.

The governing rule is:

> A negative test passes only when the system produces the exact expected fail-closed classification at the expected assurance boundary.

"It errored somewhere" is not sufficient adversarial evidence.

## Two evidence planes

ADV-001 deliberately separates:

1. **structurally executable cases** — attacks that can be run today against the pure Rust ELECT-001..012 contracts; and
2. **protocol-dependent gaps** — properties that cannot honestly be qualified until a concrete credential, ballot, receipt-freeness, E2E verification, and tally protocol has been selected.

A green structural corpus therefore does **not** mean Mycelix has proved ballot secrecy, coercion resistance, receipt freeness, cast-as-intended, recorded-as-cast, or tallied-as-recorded.

Those gaps remain explicit certification blockers for later protocol-security claims.

## Stable attack registry

The v1 corpus contains 26 executable attack cases. Every case has a stable typed ID, target layer, primary election theorem, exact expected disposition, exact normalized finding, certification-blocking flag, and single-fault/composition classification.

### Anonymous authority

- conflicting use of one scope-local nullifier with a changed ballot;
- unresolved conflicting nullifier census;
- census count mismatch.

Exact replay remains a positive-control behavior rather than being treated as a second vote.

### Transparency lineage

- checkpoint sequence gap;
- predecessor substitution;
- same-sequence split-view/equivocation.

### Witness independence

- three witness keys under one control domain;
- duplicate witness key under different claimed domains.

The expected failure must be the independence/key theorem itself, not a generic parse error.

### Evidence package and offline verifier

- archive/path traversal (`..`);
- non-public/secret material smuggled into the public package;
- missing challenge ledger;
- verifier network dependency enabled;
- missing required certification verification stage;
- failed stage dominating indeterminate stage;
- multiple verifier releases sharing one implementation lineage;
- multiple independent lineages controlled by one builder domain.

### Physical election evidence

- one physical ballot silently missing from conservation accounting;
- integer overflow in ballot accounting;
- custody sequence gap;
- custody container-state discontinuity;
- sibling custody fork;
- missing/duplicate CVR reconciliation;
- invalid RLA risk limit;
- audit sample larger than the frozen population;
- governed adjudication without adjudication evidence.

### Cross-layer composition

The first composition case makes a `PhysicalAudit` stage failure dominate an otherwise passing offline verifier run. This encodes the intended theorem that valid cryptographic/electronic evidence cannot erase a failed physical assurance channel.

Later corpus tranches should add more composed attacks as concrete tally and challenge verifiers become executable.

## Protocol-dependent gap ledger

ADV-001 keeps seven explicit unresolved protocol-security gaps:

1. cryptographic eligibility-proof soundness;
2. ballot secrecy against the actual selected transcript/proof construction;
3. receipt freeness;
4. cast-as-intended verification;
5. recorded-as-cast verification;
6. tallied-as-recorded verification; and
7. coercion resistance.

Every gap is marked as blocking a protocol-security claim.

This is intentional. Structural tests around anonymous statement shapes are not evidence that the eventual anonymous credential construction is unlinkable. A type named `ReceiptFree` would not constitute receipt-freeness evidence. The corpus must refuse those category errors.

## Positive controls

The corpus also proves that corresponding valid controls remain admissible:

- clean nullifier census;
- valid checkpoint successor;
- independent witness quorum;
- complete public evidence package;
- fully offline verifier policy;
- all-required-stage passing verifier receipt; and
- conserved physical ballot accounting.

A validator that rejects everything is not secure. Negative cases and positive controls are both required.

## Determinism

The corpus is pure and offline. It uses fixed synthetic digests and no:

- network access;
- wall-clock time;
- randomness;
- Holochain runtime;
- Symthaea runtime;
- filesystem state; or
- external process authority.

The only external process in qualification is the build/test toolchain itself; case semantics are deterministic Rust values and validators.

## Qualification semantics

The dedicated Rust 1.96 lane must prove:

- exact one-commit lineage above the exact #775 subject;
- production-source formatting;
- isolated lock generation and metadata capture;
- all corpus tests passing;
- warning-fatal library Clippy;
- registry size and gap-ledger presence;
- exact execution of all registered cases;
- presence of cross-layer `PhysicalAudit` failure composition; and
- no Holochain/network/random/time authority introduced into the corpus.

The evidence artifact preserves lock, metadata, test output, Clippy output, and immutable-checkout status.

## Deliberate non-claims

ADV-001 does not yet test:

- a concrete anonymous credential backend;
- issuer blindness;
- real nullifier cryptography;
- cryptographic ballot encoding;
- voter-device malware resistance;
- actual E2E challenge/confirmation UX;
- mixnet or homomorphic tally proofs;
- trustee DKG/decryption;
- concrete RLA mathematics or sample randomness;
- proof-system side channels;
- parser fuzzing of a real evidence archive format;
- denial-of-service/resource exhaustion of a real verifier;
- legal certification; or
- coercion resistance.

Those become executable only when their concrete subjects exist.

## Next phase after ADV-001

Do not immediately add Holochain integration. The stronger sequence is:

1. qualify this structural adversarial corpus;
2. select 2–3 concrete anonymous-authority candidates and run them against a common unlinkability/nullifier test-vector contract;
3. select 2–3 concrete E2E ballot/tally candidates and map each against the twelve election theorems;
4. build at least one second, independently implemented verifier for the portable evidence record;
5. add mutation/fuzz/resource-bound corpora around the canonical archive format; and
6. only then integrate a qualified protocol into Mycelix runtime/storage surfaces.

The objective is not to make attacks impossible to imagine. It is to make outcome-changing failures **detectable, classifiable, reproducible, and incapable of silently acquiring certification authority**.
