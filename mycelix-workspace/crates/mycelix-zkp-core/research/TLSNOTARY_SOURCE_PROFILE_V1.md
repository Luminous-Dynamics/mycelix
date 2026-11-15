# TLSNotary → PPAC Source Profile V1

**Status:** Research-only qualification profile. Not an implementation, conformance statement, security certification or production admission.

**Owners:** Mycelix #6245 / #2821 / #2640 / #2641 / #2813 / #6230; Symthaea #12301 / #12269 / PR #12295.

## 1. Objective and claim ceiling

Evaluate whether an exact private value can be traced from an authenticated TLS session through a precisely selected HTTP response field into a private predicate proof, without disclosing the entire response.

The required relation is:

~~~text
TLSNotary session / Session Header
  -> authenticated transcript commitment
  -> exact HTTP object or range opening
  -> parser + claim-schema interpretation
  -> private witness
  -> exact predicate
  -> request-bound proof receipt
~~~

The proof must establish that the **same value** appears in the committed transcript and in the predicate witness. A host-supplied value, a source-reference digest or a valid proof over an unrelated value is insufficient.

A successful profile can establish only a narrow source/session proposition under a selected TLSNotary protocol and trust model. It cannot establish provider truth, account ownership, legal entitlement, issuer authority for an arbitrary purpose, freshness without LISE evidence, human consent, or action authority.

This profile composes existing Mycelix EPI / source / finance / PEC owners. It does not introduce a new evidence ontology, issuer registry, identity system, ZKP backend, generic source authority, currentness service or consent authority.

## 2. Mandatory role separation

Do not use the word “verifier” as though it denotes one role.

| Role | When | What it does | Trust implication |
|---|---|---|---|
| In-session verifier | During TLS | Participates in the live TLS proof protocol | Direct verifier can trust its own participation; the evidence is designated-verifier |
| Delegated notary | During TLS | Performs the in-session verifier role for parties not online | Offline relying parties inherit trust in the notary's signed attestation |
| Attestation verifier | After TLS | Checks the notary's signature, Session Header, commitment openings and server identity | Signature validity does not remove trust in the notary that witnessed the session |
| Application verifier | After source qualification | Checks parser, claim schema, predicate, audience, purpose, policy and domain rules | Must not promote transport authenticity to business truth or action authority |

The zero-knowledge component proves that disclosed/opened data is consistent with the committed session data and can hide undisclosed fields. It does not create an honest in-session witness after the fact.

## 3. Three non-interchangeable trust profiles

### TLSN-MPC-DIRECT-V1

**Use when the relying institution can participate online.**

- The prover connects to the server.
- The prover and verifier jointly perform the TLS cryptographic operations using MPC.
- The verifier sees encrypted application traffic during the session.
- The prover cannot unilaterally construct requests or forge server responses under the protocol's assumptions.
- The relying verifier itself witnesses the session.
- The result is designated-verifier evidence, not a portable attestation for arbitrary third parties.
- MPC-TLS server-identity privacy is available by default under its specific protocol assumptions.

Do not add a general-purpose notary merely for terminology. Direct-verifier and notary modes have different commitment constructions and artefact formats.

### TLSN-MPC-NOTARY-V1

**Use when later/offline parties need a portable presentation.**

- A delegated notary performs the in-session verifier role.
- The notary signs a Session Header containing commitments to transcript plaintext and TLS-specific server data.
- The notary is designed not to learn plaintext or the server identity during the protocol.
- Later attestation verifiers validate the notary signature, commitment openings, TLS-specific data and server identity under an explicit trust-root profile.
- The notary is nevertheless a delegated witness. Offline parties trust the notary's correct participation, key custody, scope and signing policy.
- Observable metadata can include session time, request/response lengths, round trips and cipher suite.

Portable verification is not trustless verification. A notary signature is a trust-bearing witness receipt.

### TLSN-PROXY-V1

**Separate performance/security profile. Never silently substitute for MPC-TLS.**

- The verifier proxies encrypted TLS packets and records traffic.
- The verifier knows the server name and connects to the server on the prover's behalf.
- The server sees the verifier's network endpoint rather than the prover's.
- The verification checks a ZK proof against the observed TLS 1.2 handshake / traffic.
- The verifier-to-server network path becomes a security assumption.
- Server identity is not hidden from the proxy verifier.
- It must not inherit MPC-TLS's direct-connection or server-identity privacy claims.

If the application requires the strongest source/session assurance, begin with MPC-TLS. Proxy mode is a separately qualified lower-overhead choice only where its network-path assumptions are accepted.

## 4. TLS version and upstream release gate

The current official TLSNotary FAQ describes TLS 1.2 support and TLS 1.3 as roadmap work. A Beta.1 announcement lists TLS 1.3 as a next-release priority; a planned feature is not evidence of support.

Before selecting a provider:
1. pin the exact TLSNotary repository, release and commit;
2. reproduce the negotiated TLS version and cipher profile;
3. prove that the selected build supports the exact suite;
4. reject TLS 1.3-only providers until a released implementation and independent test evidence establish support.

No mode may infer TLS 1.3 support from a product roadmap or an upstream issue.

## 4a. Frozen upstream baseline: TLSNotary v0.1.0-beta.1

The latest public GitHub release checked for this research tranche is **v0.1.0-beta.1**, released 14 November 2026, at release commit `40a0695`:
https://github.com/tlsnotary/tlsn/releases/tag/v0.1.0-beta.1

Treat this exact tag/commit as the first reproducible candidate. It is a prerelease, not a production security certification.

Relevant release changes that must become admission regression tests:

- `tlsn` consolidates the former prover and verifier crates and re-exports core types; attestation-related functionality is moved into a separate `attestation` crate. Pin the exact API and crate graph rather than copying APIs from older alpha tutorials.
- Core fix #1179 preserves transcript commitment order. Test exact order binding, duplicate/reordered commitments and compatibility behavior.
- Fix #1196 bounds prover-supplied transcript commitment ranges. Test reversed, overlapping, duplicated, empty, out-of-range and overflow ranges against the exact release API.
- Fix #1211 updates `mpz` to beta.1 and corrects GHASH byte order. Include known-answer tests and wrong-order negative controls.
- Session teardown fixes #1207/#1202/#1172/#1167/#1175 make sessions complete or fail with a transport error rather than hang, busy-loop or silently drop output. Test peer close, EOF, partial writes, cancellation and abrupt disconnect.
- Mux stream limit is configurable and must match on both peers; transcript use is bounded/logged against configured maxima; WASM config rejects unknown fields.

Do not treat the short release note as a full cryptographic audit. The exact `tlsn`, `attestation`, `tlsn-core`, `mpz` and proof-transcript dependency graph and feature flags must be recorded in the reproducibility receipt.

### Full-reveal fast path is a separate privacy profile

TLSNotary documents a full-reveal optimization: if the prover reveals an entire traffic direction, the verifier can receive the corresponding write key and decrypt that direction locally. The ZK proof of TLS key derivation still runs, but the ciphertext-to-plaintext proof for that direction is skipped.

This is a legitimate performance profile only when revealing the entire direction is acceptable. It is not selective disclosure.

- Full response reveal may be appropriate for public or non-sensitive synthetic API fixtures.
- It is generally inappropriate where the response contains unrelated balances, identifiers, transaction history, health information or account metadata.
- The fast path is all-or-nothing per direction; one private byte returns that direction to the full ZK path.
- The request direction commonly contains credentials, cookies or bearer tokens and should not be fully revealed by default.
- A verifier receiving the full response must be counted as a disclosure recipient in the privacy model.

Benchmark at least three privacy profiles separately:
1. full response reveal (fast path);
2. selective response disclosure with all unrelated fields redacted;
3. mixed response with one sensitive byte/field, forcing the ZK path.

Never compare the full-reveal timing against selective disclosure and claim a protocol speedup without reporting the disclosure difference. The full-reveal fast path has its own measured upstream study: https://tlsnotary.org/blog/2026/05/19/fast-reveal/

### Version pin and source admission

Before a runtime adapter is written:
- pin release tag `v0.1.0-beta.1` and full source commit `40a069565a2eaecbb5e691dbc634be20e8498daf` (resolve and record the full commit SHA `40a069565a2eaecbb5e691dbc634be20e8498daf`);
- pin the Rust toolchain and every TLSNotary/mpz transitive source revision;
- pin feature flags, target triples and commitment strategy;
- verify Beta.1's actual selected protocol mode and negotiated TLS 1.2 profile;
- retain upstream source, build metadata, SBOM and generated bindings as evidence artifacts;
- do not track a floating branch or semver range as a cryptographic verifier identity;
- treat every release/feature change as a new verifier profile requiring differential regression and LISE impact analysis.

## 5. Transcript commitments are mode-specific

The upstream documentation states that the commitment construction used with a general-purpose notary differs from the direct Prover-to-Verifier case. In the notary path, the commitment is produced over the active plaintext encoding, and a Merkle-tree commitment hides the number of commitments.

Consequently, this profile requires exact versioned references for:

- transcript encoding and byte-order conventions;
- Session Header schema and signature algorithm;
- plaintext transcript commitment profile;
- TLS-specific server-identity commitment profile;
- commitment leaf/range encoding;
- Merkle tree construction, root and opening format;
- hash algorithm and domain separation;
- transcript direction (request vs response);
- HTTP message/object boundaries;
- commitment strategy and Secret artefact format;
- verifier implementation and release;
- notary key epoch and trust-root snapshot, where applicable.

Do not assume the generic BLAKE3 transcript digest is the same object as the notary-mode authenticated commitment. Do not assume direct-verifier and notary-mode commitment bytes are interchangeable.

### Commitment strategy trade-offs

| Strategy | Flexibility | Cost / restriction |
|---|---|---|
| Custom ranges | Only preselected ranges can later be disclosed | Smallest artefact; cannot reveal uncommitted subranges |
| HTTP objects | Selective disclosure of headers and JSON HTTP objects | Useful default for many API claims; larger artefact |
| Per-byte | Arbitrary later byte-level disclosure | Maximum flexibility; largest artefact |

TLSNotary's documented default BLAKE3 profile estimates approximately 250 bytes of additional Secret artefact per additional committed range. The Session Header attestation is documented as constant-sized with range count while the Secret artefact grows linearly. These figures must be reproduced under the exact pinned release and commit strategy.

## 6. ZK composition boundary

TLSNotary documents QuickSilver-based zero-knowledge proofs for transcript hash commitments and describes passing a commitment into a general-purpose ZK system to open it and prove arbitrary properties over the committed data.

That is a promising composition route, not an existing Symthaea capability.

Current Symthaea PR #12295 implements request-context binding only. It does not:
- verify a TLSNotary Session Header or notary signature;
- verify a TLSNotary Merkle/range opening;
- bind a private balance to a selected transcript field;
- prove the application parser produced the same value;
- establish provider or account authority.

Symthaea #12301 owns this missing join.

Candidate composition:

~~~text
authenticated session/header and commitment root
+ exact range/HTTP-object opening
+ parser/schema/predicate profile
+ private field witness
+ audience/request challenge
-> proof over the same committed value
~~~

If any verification is performed outside the guest, the application must consume an independently authenticated verifier receipt bound to the exact root, opening, field, parser profile, mode and request. A host-supplied boolean such as “source_valid” is not evidence.

Whether signature and root verification occur inside RISC Zero or behind an independently authenticated verifier receipt remains an explicit design question. Resolve it using a stated adversary model. Do not choose the easier path without identifying the new trust boundary.

## 7. HTTP and application semantics

The proof must bind the exact request and response context, not only a hostname or a JSON value.

At minimum bind:
- scheme, hostname/SNI and server certificate validation profile;
- HTTP method, path, query and API deployment/product;
- redirect chain and redirect policy;
- response status and media type;
- request/response transcript commitment identities;
- content encoding, decompression and charset derivation;
- parser implementation, version and duplicate-key rules;
- exact JSON Pointer or byte-range selector;
- claim schema, currency/unit/decimal scale, sign and rounding semantics;
- temporal field interpretation and freshness predicate;
- exact predicate digest and public request digest;
- session mode, release, commitment strategy and verifier/notary identity;
- expected Symthaea guest ImageID.

Raw response bytes, decoded content, parsed JSON, selected field and normalized value are different evidence artifacts with derivation links. A parser output cannot be treated as a field opening unless the proof shows its relation to the authenticated bytes.

Example synthetic predicate:

~~~text
currency == ZAR
AND available_minor_units >= required_minimum_minor_units
AND as_of is inside the request's permitted freshness interval
~~~

“1250.00” is not a complete financial statement without currency, decimal scale, sign, account scope, timestamp and parser semantics.

## 8. Integration ownership

| Concern | Existing owner |
|---|---|
| Exact artifact/snapshot/selector and derivation | EPI #2616 / #2624 |
| Evidence dependency and scoped independence | EPI #2621 |
| Live web capture fidelity | WEB-CAPTURE #2640 |
| SSRF, redirect, DNS rebinding and acquisition isolation | WEB-CAPTURE-SEC #2641 |
| Provider-specific origin/request/response receipt | FIN-ALPACA-SOURCE #2821 |
| Exact bounded response and parser handoff | FIN-ALPACA-WIRE #2813 |
| Privacy objective/adversary/leakage semantics | PEC #2109 / #2110 |
| Request-bound private predicate and transcript opening | Symthaea #12301; #12269 |
| Consent, operator/session and exact action | Xenia #449 / #450 / #455 and action-specific owner |
| Currentness, dependency impact and targeted requalification | LISE #6219 |
| Institutional and financial claim semantics | Owning Mycelix domain; not the transport adapter |

A TLSNotary adapter may contribute source-session evidence to the existing source receipt. It may not create a second source-authority system or turn a valid TLS response into account ownership.

## 9. Privacy and leakage model

Evaluate separately what is disclosed to:
- prover;
- in-session verifier;
- delegated notary;
- later attestation verifier;
- application verifier;
- downstream institution;
- public observer.

Include:
- plaintext fields intentionally revealed;
- commitment and Merkle metadata;
- request/response lengths;
- timing, round count and cipher suite;
- server identity and certificate metadata;
- stable account/subject identifiers;
- repeated commitment-root correlation;
- predicate result and adaptive-query inference;
- credential/status lookup side channels;
- retained Secret artefact and transcript data.

Selective disclosure is not anonymity. A valid presentation can remain linkable through stable identifiers, repeated roots, timing and metadata. Query-family controls belong to PEC / #2109 / #2110; this profile does not create a second privacy budget engine.

## 9a. Attested verifier, prover-side plaintext: a fourth TLS witness profile

Mycelix [WEB-CAPTURE-TEE-VERIFIER-001 / #6254](https://github.com/Luminous-Dynamics/mycelix/issues/6254) tracks an additional composition suggested by TLSNotary's June 23, 2026 TEE analysis:

**Run the delegated MPC-TLS verifier inside a measured TEE, while keeping TLS decryption and application plaintext on the prover device.**

The TEE is the witness/verifier, not a plaintext processor.

This aims to combine:
- stronger integrity against a malicious host/operator because the verifier receipt key is bound to measured code and attestation;
- prover-side cryptographic confidentiality because the enclave sees protocol/ciphertext material but never the TLS write/decryption key or plaintext;
- user-controlled selective disclosure through TLSNotary's existing transcript commitment/opening path.

It is a different profile from the common TEE relay in which a user encrypts credentials/session token to an enclave, the enclave makes the provider request, and the enclave sees the full response in plaintext.

The four distinct profiles are:
1. ordinary delegated MPC-TLS notary (integrity anchored in notary/operator key);
2. **attested MPC-TLS verifier with prover-side plaintext** (integrity anchored in measured TEE code/TCB, privacy remains prover-side only if dataflow proves no plaintext/key ingress);
3. TEE plaintext processor (plaintext and often credentials enter enclave; confidentiality is attestation-based);
4. direct relying-party verifier (institution participates in its own session).

### Mandatory boundary for the hybrid

- The exact measured verifier image, TCB version, vendor root, source/build mapping and protocol release are qualified.
- The enclave participates as the verifier but does not receive TLS write keys or application plaintext.
- The attested receipt binds image identity, TCB, TLSNotary release/mode, session/commitment root, request, audience and policy.
- If a key or plaintext enters the enclave, the run is reclassified as a TEE plaintext processor; it cannot be claimed as the hybrid privacy profile.
- A valid hardware quote identifies measured bytes; it does not prove those bytes implement the reviewed source semantics. Reproducible build/source mapping is a separate requirement.

This profile is not trustless: it relies on hardware/vendor attestation, the TCB and measurement verification. Its proposed advantage is preserving a cryptographic plaintext boundary while strengthening the integrity of the delegated verifier.

Primary analysis: https://tlsnotary.org/blog/2026/06/23/where-trust-lives/

## 10. Industry baseline and matched benchmark

TLSNotary Beta.1, announced 14 November 2026, reports this 1 KiB request / 2 KiB response workload:

| Profile | Cable | Fiber | Mobile 5G |
|---|---:|---:|---:|
| MPC native | 13.35 s | 3.43 s | 9.63 s |
| MPC browser/WASM | 14.15 s | 4.17 s | 10.45 s |
| Proxy native | 1.30 s | 0.83 s | 1.29 s |
| Proxy browser/WASM | 1.57 s | 1.12 s | 1.55 s |

Vendor methodology: 10 measured samples plus 3 warmups; Azure Standard_F8als_v6; AMD EPYC 9004; 8 vCPU / 16 GB RAM; SMT disabled; named network profiles. These are external reference measurements, not Mycelix results and not a matched comparison against Symthaea.

A fair comparison must freeze:
- identical request and response bytes;
- exact TLS mode/security assumptions;
- exact selected fields and parser/predicate;
- same native/browser target and hardware;
- exact network bandwidth/latency profile;
- proof/commitment/reveal requirements;
- exact source/authority/freshness criteria.

Measure separately:
1. preprocessing;
2. online TLS/MPC session;
3. Session Header / live-verifier outcome;
4. commitment/opening generation;
5. selective disclosure;
6. parser and predicate;
7. RISC Zero prove;
8. independent receipt verification;
9. LISE qualification and requalification.

Workload sweep: 2 KiB / 10 KiB / 100 KiB / 1 MiB response; 1 / 4 / 16 / 64 selected fields; custom-range, HTTP-object and per-byte commitments; native and WASM. Compare MPC-TLS and proxy only as distinct trust profiles.

Report p50/p95/p99, raw sample distributions, CPU, peak memory, bytes both directions, energy, communication rounds, proof/header/Secret size, verifier cost, failure rate and leakage. Use at least 1,000 samples per primary cell for stable tail estimates, or state the confidence limits of smaller runs.

Primary metric: **time-to-qualified-private-decision**, measured from a valid request to an independently checkable, source-qualified private predicate result. Bare protocol time is not the institutional outcome.

## 11. Prospective qualification gates

- Exact TLSNotary release, commit and trust profile frozen.
- Positive direct-MPC-TLS and delegated-notary controls distinguish their different claim ceilings.
- Proxy-mode results never satisfy the MPC-TLS profile.
- Exact Session Header, signature, server identity and transcript commitment independently verified.
- Wrong root, wrong opening, wrong range, wrong parser, wrong predicate, wrong witness, wrong ImageID and replay all reject.
- Credential, token, cookie and unrelated response fields do not leak into portable artefacts or logs.
- Notary key/trust-root changes create new evidence lineage and trigger LISE requalification.
- Stale source data does not pass freshness merely because its TLS transcript remains cryptographically valid.
- External provider truth, account ownership, institutional authorization and effect execution remain separate claims.
- Native proof execution, independent reproduction and runtime qualification are separately evidenced.

## 12. Comparative zkTLS / web-data provenance landscape

The market is not one homogeneous protocol family. Each design moves trust and computation among the prover, verifier, notary/attestor, proxy, TEEs and server. The comparison below is qualitative and based on published/official descriptions; it is not a security audit or matched benchmark.

The machine-readable protocol and benchmark contract is maintained in `TLS_SOURCE_PROTOCOL_MATRIX_V1.json`. It freezes six comparator families, eight profile IDs, role/trust assumptions, reported-but-unmatched baselines, common fixture sizes, measurement stages, fairness rules and prospective acceptance targets. Its status is explicitly `protocol_and_targets_only_not_executed`; no row is a qualification result.

| System / research | Main mechanism | Where trust/computation sits | Important qualification boundary |
|---|---|---|---|
| DECO (2019/2020) | Three-party TLS handshake; selective opening and context integrity over TLS data | Prover + verifier jointly participate with server; verifier sees/validates server provenance | Original DECO construction targets TLS 1.2 and requires verifier in-session participation; context integrity and extraction semantics must be bound |
| DIDO+ (2025) | Three-party TLS / MPC and selective disclosure for restricted TLS 1.3 websites | Prover/verifier MPC with TLS 1.3 session, under its specific paper construction | Separate research implementation; do not infer that DECO, TLSNotary or zkPass inherit its TLS 1.3 support |
| TLSNotary | MPC-TLS, optional delegated notary, transcript commitment openings; separate proxy mode | Direct verifier witnesses live session; delegated notary signs portable Session Header; proxy mode adds network-path assumptions | Distinguish direct verifier, notary and proxy commitment/trust profiles; no generic TLSNotary proof abstraction |
| zkPass | Three-party TLS + VOLE-based ZK; documented hybrid proxy/MPC modes | Proxy mode is presented as typical; MPC mode is a fallback when source restrictions prevent proxy use | Its own technical overview describes significant MPC-mode bandwidth overhead and a distinct proxy network assumption; independently qualify deployed version and nodes |
| Reclaim | Split TEE services with MPC/TLS attestation and proof generation; public Trust Package describes hardware attestation | Hardware attestation and exact published image measurements are trust roots, with split service duties | Verify exact CPU/TEE vendor chain, cloud configuration, image measurement, side-channel assumptions, rollback and operator controls; source availability does not replace runtime attestation |
| Scribe (2026) | TLS provenance -> attestor-signed salted value commitment -> transparent STARK -> app spend tag -> Merkle spent set / Falcon root | Attestation node is an explicit witness/trust root; aggregator owns off-chain spent state | Salt changes can create new tags for the same account; signed root does not prove append-only correctness; PQ root signing is not whole-stack PQ |

Primary references:
- DECO: https://arxiv.org/abs/1909.00938
- DECO paper / context-integrity detail: https://www.cs.cornell.edu/~shmat/shmat_ccs20.pdf
- DIDO+: https://doi.org/10.1109/TDSC.2025.3595711
- zkPass technical overview: https://docs.zkpass.org/overview/technical-overview
- zkPass FAQ: https://docs.zkpass.org/supports/faq
- Reclaim Trust Package: https://trust.reclaimprotocol.org/section/security
- Reclaim TEE implementation overview: https://github.com/reclaimprotocol/reclaim-tee
- Scribe: https://doi.org/10.1007/s12083-026-02246-2

### What Mycelix should benchmark

Use the same deterministic HTTPS fixture, request/response bytes, parser, claim schema, predicate, privacy goal and adversary assumptions across systems. Record where a system needs a verifier, delegated notary, attestor committee, proxy, TEE, or online aggregator. Do not compare proof time when one system includes a witnessed TLS session and another excludes it.

For each system, separate:
1. source-session establishment / handshake;
2. online MPC or proxy work;
3. attestor/notary/TEE evidence;
4. transcript or value commitment generation;
5. selective disclosure and extraction;
6. predicate proof generation;
7. verification;
8. uniqueness/spent-set update;
9. LISE qualification/requalification;
10. complete time-to-qualified-private-decision.

Benchmark security modes separately. A faster proxy profile cannot beat an MPC-TLS profile by silently changing trust assumptions. A TEE profile must include hardware-attestation verification and image measurement in the end-to-end boundary.

## 13. Scribe as a peer-reviewed industry/research comparator

**Scribe: Private Web2-to-Web3 eligibility proofs for decentralized applications with transparent STARKs**, published 2 September 2026 in *Peer-to-Peer Networking and Applications* (volume 19, article 133), is a particularly relevant comparator:
https://doi.org/10.1007/s12083-026-02246-2

Scribe is not just a TLSNotary wrapper. It composes:
- client-to-Web2 TLS provenance using a DECO/TLSNotary-style workflow;
- an attestation node that extracts a field and signs a salted commitment binding application ID, server ID, extraction rule, data and salt;
- a Winterfell transparent STARK that verifies the circuit-friendly attestation, recomputes the commitment and proves a policy predicate without revealing the value;
- an application-scoped spend tag/nullifier;
- an off-chain aggregator maintaining a Merkle-indexed set of spent tags;
- a Falcon-signed root publication to an on-chain anchor.

### Reported results and how to compare them

The paper reports Apple M4 Pro / 24 GB measurements. Its STARK policy benchmark repeats one threshold check from 1 to 256 times:

| Measurement | Reported range/result |
|---|---:|
| STARK proving | 6 ms to 346 ms |
| STARK verification | 0.4 ms to 9.7 ms |
| STARK proof size | 32.1 KB to 77.8 KB |
| TLSNotary workflow proving | 856 ms average |
| TLSNotary workflow verification | 749 ms average |
| Aggregator amortized processing | 1.10 ms/user at batch 1; about 0.31 ms/user at batches 128–1024 |
| Root-only publication | about 1.50M gas/batch; paper reports 2,929 gas/user at batch 512 |

The STARK benchmark used 10 runs per configuration; aggregator batches 10 runs; gas five runs. These are useful research baselines, not high-confidence tail-latency estimates, production service levels or direct comparisons to the TLSNotary Beta.1 measurements above. The paper's TLSNotary workflow timings are from a separate experiment and should not be mixed with its STARK timings or TLSNotary Beta.1's network-profile benchmark.

The policy circuit is a relatively narrow threshold-style workload. It does not have the same cost as proving TLS transcript openings, JSON parsing, exact field selection, issuer authority, LISE currentness, Xenia consent and a complete institutional decision. Benchmark each component and the full chain separately.

### What Scribe does not solve for us

**1. Attestation-node honesty remains a trust assumption.** Scribe states that an attestation node could sign without executing a valid provenance session and that this misbehavior is not cryptographically detectable by the base construction. Threshold signing, TEE, staking/reputation and collateral are suggested mitigations, but none is part of the basic trustless theorem. A threshold committee only helps if node independence, key distribution, quorum corruption and common-mode risks are measured.

**2. Salted commitment does not automatically give account-level uniqueness.** Scribe derives the application spend tag from the salted commitment. The paper explicitly notes that the same Web2 account can request attestations with different salts and obtain different spend tags. Therefore the spent-tag set prevents reuse of the same tag, not necessarily multiple claims by the same account. The desired uniqueness subject must be defined by the domain: account, legal person, credential, benefit entitlement, election, institution or campaign are not interchangeable. Do not solve this by publishing a stable global identifier. Instead investigate a privacy-preserving, issuer-authorized, purpose-scoped subject nullifier with explicit recovery, revocation and sybil assumptions.

**3. Attestation signature needs its own cryptographic qualification.** The paper's prototype describes a Rescue-based circuit-friendly signature and an elliptic-curve alternative. Its security theorem assumes an EUF-CMA-secure signature primitive; the prototype construction must not be treated as a standard signature scheme merely because it is circuit-friendly. Pin the exact construction, encoding, key lifecycle, domain separation, multi-message security and chosen-message security. Require independent cryptographic review before admission.

**4. Falcon only protects the aggregator's root authorization.** A Falcon signature on a new root authenticates the signing key; it does not prove that the root correctly includes every previously accepted spend tag, that the aggregator did not equivocate, or that the update is append-only. Require a verifiable state transition/consistency proof, externally auditable inclusion/receipt log, equivocation detection and explicit censorship/fallback semantics. A malicious aggregator holding its signing key can otherwise authorize an incorrect root.

**5. Post-quantum qualification is layer-specific.** Falcon at root publication does not make TLS, the attestation signature, hash/commitment construction, notary trust, STARK parameters, or the entire protocol post-quantum secure. Record every cryptographic layer and its security profile separately.

**6. Proof zero-knowledge is not network anonymity.** Scribe's proof-privacy analysis does not eliminate timing, traffic volume, network metadata, server identity or repeated-query leakage. Keep those in the end-to-end privacy model.

### Two architectures to benchmark against each other

**A. Transcript-proof-native (higher-assurance research target)**

~~~text
TLSNotary session / delegated notary attests exact transcript commitment
 -> Symthaea verifies selected HTTP-object/range opening
 -> deterministic parser + claim schema are bound to exact bytes
 -> private predicate is evaluated over the same parsed value
~~~

This reduces reliance on an external attestation node's field-extraction code, but increases the cost and complexity of commitment opening and parsing inside the proof.

**B. Attestor-extracted value (Scribe-style baseline)**

~~~text
TLS provenance verifier + extraction profile
 -> attestation node signs commitment to (app, server, rule, value, salt)
 -> STARK/RISC Zero verifies signature + commitment + predicate
 -> application-scoped nullifier is checked against an authenticated spent set
~~~

This can be faster and easier to integrate, but explicitly trusts the attestation node's extraction procedure. A threshold/committee variant is a separate research profile, not an automatic upgrade.

Benchmark both over the same frozen response, extracted field, predicate, privacy target, hardware and adversary assumptions. The end-to-end result must include source session, attestation/commitment, proof, verification, replay/uniqueness check and currentness qualification.

### Additional research questions

1. Can the subject-nullifier be derived from a credential-bound secret or issuer-scoped pseudonym without creating a global cross-application correlation handle?
2. Can a verifier prove entitlement uniqueness without learning the underlying stable account ID?
3. What happens when a credential rotates, the issuer changes key, an account is recovered, two accounts merge, or the participant contests an account-linkage assertion?
4. Can multiple independent provenance witnesses sign the same canonical transcript commitment, and what precisely constitutes independence?
5. Can the aggregator expose a consistency proof between old and new spent-tag roots, not merely a signed new root?
6. Can Mycelix's existing append-only/effect/reconciliation owners provide the state-transition evidence without creating a second ledger?
7. What is the privacy cost of online uniqueness checking, and can the system avoid revealing which application or account was queried?
8. Does in-circuit verification of a standard PQ signature materially change proving cost relative to a circuit-friendly classical signature? Measure rather than assume.

## 14. Claim ceiling

A pass establishes only the exact proposition qualified for the selected TLSNotary mode: a value in a specified authenticated transcript commitment was opened and interpreted under a frozen parser/schema/predicate profile, and the private predicate proof used that same value.

It does not establish provider truth, legal account ownership, beneficial ownership, issuer authority for other purposes, completeness of the provider's data, absence of coercion, universal anonymity, currentness beyond the LISE evidence frontier, or authorization for a consequential action.

## 15. Primary references

- TLSNotary MPC-TLS: https://tlsnotary.org/docs/protocol/mpc-tls/
- Notarization: https://tlsnotary.org/docs/protocol/notarization/
- Verification: https://tlsnotary.org/docs/protocol/verification/
- Commitments: https://tlsnotary.org/docs/mpc/commitments/
- Commit strategies: https://tlsnotary.org/docs/protocol/commit_strategy/
- Server identity privacy: https://tlsnotary.org/docs/protocol/server_identity_privacy/
- Proxy mode: https://tlsnotary.org/docs/protocol/proxy-mode/
- FAQ / TLS version and QuickSilver composition: https://tlsnotary.org/docs/faq/
- Public verifiability and notary trust model: https://tlsnotary.org/blog/2026/06/17/public-verifiability/
- Beta.1 performance (14 November 2026): https://tlsnotary.org/blog/beta1-performance-next-steps/
- Scribe (2026): https://doi.org/10.1007/s12083-026-02246-2
