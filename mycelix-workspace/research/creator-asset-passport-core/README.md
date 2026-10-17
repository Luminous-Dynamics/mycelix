# Creator Asset Passport Core (experimental)

**State:** research prototype, not production software. Related tracker: #5181. Parent design: [Creator Sovereignty Control Plane](https://github.com/Luminous-Dynamics/mycelix/pull/5166).

## Paradigm shift

Treat an artist's catalogue as a set of typed, portable assets and explicit claims—not a pile of files owned by one platform. The prototype separates three questions that are often collapsed:

1. **Identity:** which kind of thing is this, and which external identifiers have been recorded?
2. **Claims:** what contribution/split is proposed, confirmed, disputed, or still unknown for one asset and one rights pool?
3. **Permission:** has this exact party been explicitly granted this exact action, purpose, asset, and time window?

These are intentionally independent. An identifier is not an ownership certificate. A credit is not a split agreement. A confirmed split claim is not a use grant. Permission to analyze metadata does not permit distribution, model training, derivative generation, or voice/likeness synthesis.

## Implemented surface

- Rust crate isolated with its own Cargo workspace; BLAKE3 is pinned for content-digest integrity.
- Typed asset kinds and separate ISWC / ISRC variants. The current checks validate syntax and asset-kind compatibility only; they do not validate check digits, registration status, or authentic assignment.
- Duplicate external identifiers are surfaced, not auto-merged. Replaying the same identifier assignment to the same asset is idempotent.
- Rights-pool-scoped split assessment with exact basis points (10,000 = 100%), duplicate-party rejection, explicit partial status, fail-closed complete-split arithmetic, and bounded/control-character-free evidence references.
- Explicit, asset-bound grants for metadata analysis, publication, distribution, commercialization, model training, derivative generation, and voice/likeness synthesis.
- Exact grantee, purpose, action, asset, start time and expiry checks; revocation is evaluated by time so historical uses before revocation can still be audited. Evidence references have the same field-size/control-character constraints across split claims and grants.
- Order-independent usage evaluation by immutable grant ID: a revocation dominates an original grant regardless of history ordering; conflicting reuse of a grant ID blocks only the exact scopes implicated, not unrelated catalogue permissions.
- An append-only in-memory issue/revocation event log with event-ID idempotency, monotonic event-time checks, deterministic replay, and atomic rejection of invalid transitions.
- A separately implemented event-stream reconstruction path for differential comparison against append-derived effective grant state; it does not call the append method.
- A versioned binary portable envelope for grant/revocation history, plus a separate full passport snapshot envelope covering asset records, external identifiers, split claims, and the embedded grant-history envelope. Both have deterministic encoding, strict decoding and domain-separated BLAKE3-256 content digests.
- No implicit rights inheritance between related assets.

## Important non-goals and limitations

This crate does **not** authenticate identity, verify digital signatures, prove that an evidence reference is truthful, establish legal ownership, resolve conflicting claims, validate contracts, calculate royalties, make payments, or determine whether a grantor has legal authority. A status named Confirmed is a workflow assertion provided by the caller, not an adjudication by the software. A syntactically valid ISWC/ISRC is not proof the identifier was issued to this asset.

The registry and event history are in-memory only. Events are not signed or durably persisted. The grant-history envelope covers only issue/revocation events; the separate full passport snapshot envelope adds current asset records, external identifiers, split claims, and the grant-history envelope. Neither format carries original external source payloads, source-message provenance hashes, rights-claim reconciliation results, territory-specific terms, or legal authority proofs. The crate also lacks external catalogue sync, territory-specific rules, and standards adapters. Its BLAKE3 digest detects changed canonical bytes but does not authenticate authorship or prove legal rights. It is an invariant kernel to test before building any of those. DDEX compatibility should be addressed through explicit, versioned adapters rather than by presenting this internal model as a replacement for an industry standard. The ISWC / ISRC distinctions and DDEX exchange boundaries are described in the public [ISWC guidance](https://www.iswc.org/iswc) and [DDEX Recording Data and Rights standards](https://ddex.net/standards/recording-data-and-rights/).

## Portable grant-history envelope v1

The public Rust API is portable::GrantHistoryEnvelope. It encodes a single ordered grant-event history; it does not yet export the complete asset registry or split-claim graph.

The binary format is deliberately specified rather than relying on a platform's default serializer:

1. Magic bytes MYCELIX-CSCP-GRANT-HISTORY followed by one NUL byte.
2. Schema version as unsigned 16-bit big-endian integer (1).
3. Event count as unsigned 64-bit big-endian integer.
4. Events in source order. Each begins with event ID (u32 big-endian UTF-8 byte length + bytes), occurred-at timestamp (u64 big-endian), actor ID (length-prefixed string), evidence reference (length-prefixed string), and event-kind tag.
5. Event-kind 1 means grant issued: grant ID, asset ID, grantee ID, action tag, purpose, issuer ID, grant evidence reference, valid-from timestamp, expiry-presence tag (0 absent; 1 followed by u64 expiry).
6. Event-kind 2 means revocation: grant ID only.
7. Action tags are stable explicit values: 1 analyze metadata, 2 publish, 3 distribute, 4 commercialize, 5 train model, 6 generate derivative, 7 synthesize voice/likeness.

Every string is UTF-8 with a u32 big-endian byte length and a 256-byte maximum. Integers are big-endian. Unsupported versions/tags, malformed UTF-8, oversized fields/counts, truncated bytes, invalid replay order, and trailing bytes are rejected. The canonical content digest uses BLAKE3-256 over the domain prefix MYCELIX-CSCP-GRANT-HISTORY-DIGEST-V1 plus a NUL byte, followed by the complete canonical envelope bytes. The digest is not embedded in the envelope and should travel as a separate integrity field.

The on-disk binary format is experimental and not a ratified cross-language standard. Two frozen byte vectors are checked by the Rust encoder and a standard-library-only Python reference encoder in the test suite: an issue/revocation history and a UTF-8/absent-expiry case using the voice/likeness action tag. The Python script is test tooling only, not application code. This is not yet a broad adversarial corpus or a cross-language BLAKE3 digest vector. Freeze the format only after multiple edge-case vectors, independent digest agreement, fuzzing, and review. A valid digest proves byte agreement, not the identity of an author or legal consent.

## Full passport snapshot envelope v1

The public Rust API is passport_envelope::CreativeAssetPassportEnvelope. This is a separate outer format; it embeds the canonical grant-history envelope described above rather than changing that format.

The canonical binary layout is:

1. Magic bytes MYCELIX-CSCP-ASSET-PASSPORT followed by one NUL byte.
2. Schema version as unsigned 16-bit big-endian integer (currently 1).
3. Asset count as unsigned 64-bit big-endian integer, then assets in ascending AssetId order.
4. Each asset contains a length-prefixed AssetId, one explicit asset-kind tag (1 work, 2 sound recording, 3 music video, 4 release, 5 visual asset, 6 live performance), an optional-title tag plus title when present, and an identifier count.
5. Each external identifier has an explicit type tag: 1 ISWC, 2 ISRC, 3 other identifier followed by its scheme and value. Identifier records are sorted by typed scheme/value key; values themselves are preserved.
6. Split-claim count, followed by claims sorted by asset ID, rights-pool tag, party ID, claim-state tag, optional basis points, and evidence reference. Rights-pool tags 1–5 are composition, master recording, performer rights, synchronization, and visual asset; state tags 1–3 are proposed, confirmed, disputed.
7. The embedded grant-history envelope's byte length as unsigned 64-bit integer, followed by its complete canonical bytes.

String fields use UTF-8 plus a u32 big-endian byte length, capped at 256 bytes. The outer decoder rejects unsupported versions/tags, malformed UTF-8, missing asset references, grant events referring to unregistered assets, identifier collisions, non-canonical ordering, malformed embedded grant history, truncation and trailing bytes. It preserves disputed/proposed claims rather than converting them into an ownership decision.

The outer BLAKE3-256 digest is computed over domain prefix MYCELIX-CSCP-ASSET-PASSPORT-DIGEST-V1 plus a NUL byte, followed by the entire canonical outer envelope. The inner grant-history envelope has its own digest domain. Neither digest proves the issuer's identity, consent authenticity, legal title, or truth of the evidence. The full snapshot still omits original external source payloads and provenance hashes; those must be included in a future evidence-carrying exchange manifest, not inferred from metadata.

The full snapshot format now has two frozen cross-language byte vectors checked by the Rust encoder and a test-only standard-library Python reference encoder: a simple work/ISWC/complete-split fixture and a Unicode recording with ISRC plus other identifiers, unknown split, and disputed claim. This is still a small corpus, not a broad separate-language implementation or cross-language BLAKE3 digest vector. The format remains experimental and must not be advertised as interoperable or standards-compliant until an independent implementation reproduces a diverse outer-envelope and digest corpus.

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
| Grant + revocation both present in history | Zero allows at/after revocation in either event order; valid pre-revocation history stays allowed | Prevent stale-event consent resurrection |
| Event-log replay | Rebuilt effective grant state equals state derived during append; duplicate exact event is idempotent | 100% fixture agreement |
| Failed append | Invalid event leaves event count and effective-grant state unchanged | Zero partial mutations |
| Event identity collision | Same event ID with different content is rejected | 100% conflict detection |
| Event time order | Decreasing timestamps are rejected; same-time distinct events are deterministic | 100% fixture agreement |
| Portable envelope round-trip | Decode then encode yields byte-identical versioned canonical bytes; effective grant state is preserved | 100% fixture agreement |
| Unsupported version, malformed tag, truncation or trailing bytes | Import rejects rather than silently ignoring unknown/corrupt data | 100% rejected |
| Content digest | Same canonical bytes yield same 256-bit BLAKE3 digest; changed content yields a different digest in regression fixtures | 100% fixture agreement; no authenticity claim |
| Cross-language canonical bytes/digest | A second implementation yields byte-identical envelope bytes and digest for frozen fixtures; current source includes two golden byte vectors, while the independent digest vector is still outstanding | 100% of the frozen corpus before format freeze |
| Full snapshot round-trip | Assets, external IDs, split claims, grant history and revocation state survive encode/decode exactly | 100% fixed-fixture agreement |
| Snapshot reference integrity | Every split claim and every grant/revocation history target resolves to a registered asset | Zero unresolved references accepted |
| Snapshot canonical ordering | Asset/identifier/claim input ordering does not change canonical bytes; noncanonical wire order is rejected | 100% fixture agreement |
| Full snapshot cross-language vectors | Independent implementation reproduces outer envelope bytes and digest for normal, disputed, Unicode, unknown-ID, and boundary cases | 100% golden-vector agreement before format freeze |
| Conflicting immutable terms or revocation times under one grant ID | 100% conflict detection for implicated exact scopes; unrelated valid permissions remain evaluable | Integrity without catalogue-wide denial of service |
| Performance target | p95 under 1 ms for 10,000 grants on a named reference machine | Keep policy checks cheap; benchmark only after correctness |
| Mutation/property testing | No invariant-breaking mutation survives the harness | Catch more than hand-picked cases |

Do not market these targets as achieved. Before a release gate can be claimed, pin the commit, Rust toolchain, machine, corpus SHA-256, command, raw output and test-run exit status. Queued or missing CI evidence is not a pass.

## Test inventory and current maturity

The current source has 53 in-module unit-test functions and 27 integration-test functions across asset/identifier checks, split arithmetic, exact-scope grants, grant-history replay, grant-history binary vectors, full passport round trips, a frozen full-snapshot byte vector, identifier/claim canonical ordering, unresolved asset references, and digest mutation fixtures. These are **authored test cases, not passed tests**.

The grant-history envelope has two frozen Rust/Python reference byte vectors. The full passport snapshot envelope has two frozen Rust/Python byte vectors: a work with an ISWC and complete composition claim, plus a Unicode recording with ISRC/other identifiers and a disputed unknown-share claim. Neither format yet has a cross-language BLAKE3 digest golden vector, broad fuzz campaign, or independent external implementation report.

## Dependency reproducibility boundary

The crate pins its direct BLAKE3 dependency, but a Cargo.lock is not yet committed. The draft CI job now resolves a lock per exact commit, runs formatting and tests with that generated lock, prints its SHA-256, and uploads the lockfile artifact even if the test step fails. This makes the specific run's resolved dependency graph inspectable; it does **not** guarantee that separate runs resolve identical transitive versions. Before a non-research release, commit a reviewed lockfile and require --locked without regeneration.

## Run when a Rust toolchain is available

From the repository root, run:

    cargo test --manifest-path mycelix-workspace/research/creator-asset-passport-core/Cargo.toml

No runtime performance or compilation result is claimed in this source-only review. The current environment did not expose cargo, rustc, or rustfmt.

## Next high-value steps

1. Run and repair the test suite on a pinned stable Rust toolchain.
2. Extend differential verification with generated event streams, mutation testing, and externally supplied expected snapshots.
3. Extend the envelope to include typed asset registry entries and split claims without weakening the current versioned schema or silently backfilling missing fields.
4. Create fixture adapters for external identifiers and DDEX metadata/revenue exchange; preserve raw source digests and parser/version metadata and never silently map missing values.
5. Add fuzzing before relying on the policy check in a publishing or AI workflow gate.
