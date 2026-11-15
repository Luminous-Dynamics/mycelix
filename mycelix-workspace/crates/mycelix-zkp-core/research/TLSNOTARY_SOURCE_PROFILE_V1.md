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

## 12. Claim ceiling

A pass establishes only the exact proposition qualified for the selected TLSNotary mode: a value in a specified authenticated transcript commitment was opened and interpreted under a frozen parser/schema/predicate profile, and the private predicate proof used that same value.

It does not establish provider truth, legal account ownership, beneficial ownership, issuer authority for other purposes, completeness of the provider's data, absence of coercion, universal anonymity, currentness beyond the LISE evidence frontier, or authorization for a consequential action.

## 13. Primary references

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
