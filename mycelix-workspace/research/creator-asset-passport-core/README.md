# Creator Asset Passport Core (experimental)

**State:** research prototype, not production software. Related tracker: #5181. Parent design: [Creator Sovereignty Control Plane](https://github.com/Luminous-Dynamics/mycelix/pull/5166).

## Paradigm shift

Treat an artist's catalogue as a set of typed, portable assets and explicit claims—not a pile of files owned by one platform. The prototype separates three questions that are often collapsed:

1. **Identity:** which kind of thing is this, and which external identifiers have been recorded?
2. **Claims:** what contribution/split is proposed, confirmed, disputed, or still unknown for one asset and one rights pool?
3. **Permission:** has this exact party been explicitly granted this exact action, purpose, asset, and time window?

These are intentionally independent. An identifier is not an ownership certificate. A credit is not a split agreement. A confirmed split claim is not a use grant. Permission to analyze metadata does not permit distribution, model training, derivative generation, or voice/likeness synthesis.

## Implemented surface

- Standard-library-only Rust crate isolated with its own Cargo workspace.
- Typed asset kinds and separate ISWC / ISRC variants. The current checks validate syntax and asset-kind compatibility only; they do not validate check digits, registration status, or authentic assignment.
- Duplicate external identifiers are surfaced, not auto-merged. Replaying the same identifier assignment to the same asset is idempotent.
- Rights-pool-scoped split assessment with exact basis points (10,000 = 100%), duplicate-party rejection, explicit partial status, and fail-closed complete-split arithmetic.
- Explicit, asset-bound grants for metadata analysis, publication, distribution, commercialization, model training, derivative generation, and voice/likeness synthesis.
- Exact grantee, purpose, action, asset, start time and expiry checks; revocation is evaluated by time so historical uses before revocation can still be audited.
- Grant-history resolution by immutable grant ID: a revocation dominates an earlier grant independent of event ordering, while conflicting reuse of a grant ID fails closed.
- No implicit rights inheritance between related assets.

## Important non-goals and limitations

This crate does **not** authenticate identity, verify digital signatures, prove that an evidence reference is truthful, establish legal ownership, resolve conflicting claims, validate contracts, calculate royalties, make payments, or determine whether a grantor has legal authority. A status named Confirmed is a workflow assertion provided by the caller, not an adjudication by the software. A syntactically valid ISWC/ISRC is not proof the identifier was issued to this asset.

The registry is in-memory and has no serialization, signed event history, external catalogue sync, rights-claim reconciliation, territory-specific rules, or import/export adapters. It is an invariant kernel to test before building any of those. DDEX compatibility should be addressed through explicit, versioned adapters rather than by presenting this internal model as a replacement for an industry standard. The ISWC / ISRC distinctions and DDEX exchange boundaries are described in the public [ISWC guidance](https://www.iswc.org/iswc) and [DDEX Recording Data and Rights standards](https://ddex.net/standards/recording-data-and-rights/).

## Benchmark contract — targets, not results

The following are release gates to measure; they have **not** been demonstrated by this PR.

| Benchmark | Target | Why it matters |
|---|---:|---|
| Wrong-kind ISWC / ISRC fixtures | 100% rejected | Prevent work / recording identity confusion |
| Duplicate external identifier assigned to different assets | 100% surfaced, zero auto-merges | Avoid silent catalogue corruption |
| Complete split arithmetic | 100% exact; never normalized | Shares are claims, not guesses |
| Partial / unknown split handling | 100% stays explicitly partial | Unknown shares must not become invented shares |
| Unauthorized action, purpose, party, asset, or time | 0 false allows across golden adversarial corpus | A permission checker fails on false grants, not only false denials |
| Independent expected-outcome fixtures | 100% agreement on fixed corpus | Deterministic, reviewable behavior |
| Repeated request evaluation | Same decision for same inputs | Reproducibility |
| Performance target | p95 under 1 ms for 10,000 grants on a named reference machine | Keep policy checks cheap; benchmark only after correctness |
| Mutation/property testing | No invariant-breaking mutation survives the harness | Catch more than hand-picked cases |

Do not market these targets as achieved. Before a release gate can be claimed, pin the commit, Rust toolchain, machine, corpus SHA-256, command, raw output and test-run exit status. Queued or missing CI evidence is not a pass.

## Run when a Rust toolchain is available

From the repository root, run:

    cargo test --manifest-path mycelix-workspace/research/creator-asset-passport-core/Cargo.toml

No runtime performance or compilation result is claimed in this source-only review. The current environment did not expose cargo, rustc, or rustfmt.

## Next high-value steps

1. Run and repair the test suite on a pinned stable Rust toolchain.
2. Add property-based or exhaustive boundary tests for split sums and time windows.
3. Add append-only, replayable grant/revocation events and independent reconstruction.
4. Create small fixture adapters for externally supplied identifiers and DDEX metadata; preserve raw source bytes/hash and parser/version metadata. Do not silently map missing values.
5. Add a differential oracle and fuzz corpus before relying on the policy check in a publishing or AI-workflow gate.
