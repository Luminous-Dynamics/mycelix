# WEB-CAPTURE-001 — Acquisition and Capture Fidelity v0.1

Status: architecture contract only. This document changes no runtime, crawler, Holochain schema, network capability, Symthaea authority, archive implementation, or qualification result.

Tracking issue: `#2640`.

Related program surfaces:

- EPI-000..011 / EPI-OSINT-000;
- EPI-BRIDGE-001;
- WEB-CAPTURE-SEC-001;
- WEB-CAPTURE-TEST-001;
- WEB-ARCHIVE-001;
- Symthaea RES-SEC / RES-BRIDGE;
- Xenia EPI-ATTEST;
- Sol-Atlas ATLAS-EPI.

## 1. Ownership

Mycelix owns the canonical semantic distinction between captured artifacts, observations, derivations, claims, assessments, and admissions.

A web acquisition adapter owns only bounded acquisition/capture facts under its declared profile.

Symthaea may reason over captured material but does not gain Mycelix write authority, persistent grammar/curriculum authority, tool authority, or scientific qualification from capture alone.

Xenia may attest exact commitments under a separate cryptographic profile but does not make captured content true.

Sol-Atlas may render capture/evidence lineage but must not strengthen it.

## 2. Core theorem

```text
URL requested
!= endpoint reached
!= HTTP response observed
!= exact HTTP message content captured
!= selected representation established
!= content-decoded derivative produced
!= DOM/browser state observed
!= rendered pixels observed
!= archive replay fidelity established
!= publisher authenticated
!= source truthful
!= claim supported
!= action authorized
```

A later layer may compose some of these facts, but no layer may silently upgrade one into another.

## 3. Acquisition is an evidence-producing activity

Every request attempt is an EPI-004-style derivation/activity subject rather than a side effect hidden behind an artifact.

A future `AcquisitionAttemptV1` should bind, under a versioned profile, the material request context needed to interpret the observation:

```text
attempt identity
acquisition profile
requested locator / target URI
method
request-header profile
redirect policy
credential/context class without raw secrets by default
user-agent/content-negotiation profile
resolver/proxy/network profile where material
capture-time assertions
resource and timeout limits
tool / implementation / version / environment
purpose/policy reference where required
outcome state
```

The same URL under different cookies, language, user-agent, time, resolver/proxy profile, or authentication context is not presumed to be the same acquisition observation.

## 4. Acquisition outcomes are typed

Do not expose one generic `success: bool`.

The vocabulary should be closed and versioned, with states conceptually including:

```text
ResponseObserved
RedirectLimitReached
BodyLimitReached
Timeout
DnsFailure
TlsFailure
ConnectionFailure
ProtocolFailure
PolicyBlocked
PrivateNetworkBlocked
DecodeFailure
Cancelled
UnknownFailure
```

A failed attempt is evidence about an attempt. It is not evidence that the target resource does not exist.

## 5. HTTP exchange observation

A future `HttpExchangeObservationV1` represents protocol-semantic HTTP facts, not an invented universal raw-wire format.

It may bind:

```text
acquisition attempt
HTTP version / transport profile
request-target semantics
response status
selected response fields
redirect relation
remote endpoint observation
TLS observation references
message-content length
message-content artifact reference
server-provided digest observations
HTTP-message-signature observations
capture truncation state
```

HTTP/1.1 serialization, HTTP/2 frames, and HTTP/3 frames are different protocol surfaces. An implementation must not reconstruct HTTP/1.1-looking bytes from HTTP/2/3 and claim those were observed on the wire.

## 6. Exact message content is not decoded content

RFC 9530 distinguishes message content (`Content-Digest`) from selected representation (`Repr-Digest`). Preserve that distinction rather than storing one overloaded `body_hash`.

At minimum, the model should be able to distinguish:

```text
HttpMessageContentArtifact
SelectedRepresentationDigestObservation
DecodedContentArtifact
```

For example, when `Content-Encoding: gzip` or `br` is used, decoded text/image bytes are a derived artifact. They do not replace the encoded message content that was captured.

```text
captured encoded bytes
-> content-decoding activity
-> decoded derivative bytes
```

Both identities and the derivation remain inspectable.

## 7. Digest-field boundary

RFC 9530 digest fields are integrity observations.

When present, record:

- exact field value;
- digest algorithm/profile;
- which semantic byte sequence the field covers;
- independent recomputation result;
- whether the field itself is covered by a separately verified authentication mechanism.

Never infer:

```text
Content-Digest matches
-> publisher authenticated
```

An unauthenticated digest can be modified together with content by an attacker. A digest match proves only the bounded fixity relation described by its profile.

## 8. HTTP Message Signature boundary

RFC 9421 verifies only the declared covered HTTP message components under the configured key/signature profile.

Preserve:

```text
covered component identifiers
signature parameters
signature suite/profile
key binding
verification result
uncovered material / coverage limitations
```

Do not collapse to `signed=true`.

```text
signature verifies
!= whole exchange authenticated
```

unless an explicit application profile requires and verifies sufficient component coverage.

Signer identity and signer authority remain independent evidence questions.

## 9. TLS / endpoint observation

TLS hostname validation is a channel/endpoint fact under the configured PKI and transport profile.

```text
TLS valid for hostname
!= publisher-person identity
!= origin-server directness
!= source truth
```

CDNs, reverse proxies, delegated infrastructure, and shared hosting make that distinction load-bearing.

Retain only the TLS evidence required by the declared profile and privacy policy, e.g. verification disposition and certificate/SPKI commitments. Do not retain sensitive session secrets merely for provenance.

## 10. Redirects are first-class

Every observed redirect hop is a distinct acquisition/HTTP observation.

```text
A -> 301 B
B -> 302 C
C -> 200 X
```

must not become:

```text
A -> X
```

without preserving the exact intermediate lineage.

Every followed redirect also passes WEB-CAPTURE-SEC-001 destination authorization independently.

## 11. Browser state is a derivative

A browser-rendered page is produced by executing untrusted content plus subresources under a specific environment.

A future browser-state artifact/profile should bind material environment state such as:

```text
browser/runtime/version
viewport/device profile
locale/timezone where material
script execution profile
cache/service-worker profile
navigation/readiness condition
resource dependency references
DOM/state commitment
capture-time assertion
blocked resources / execution failures
```

But:

```text
DOM snapshot
!= original HTTP response bytes
```

because script execution, browser parsing, third-party resources, service workers, personalization, and time can change it.

## 12. Screenshot/render output is a derivative

A screenshot proves only a bounded rendered-pixel observation for its exact viewport/state/profile.

```text
screenshot
!= complete DOM
!= hidden text absent
!= off-screen content absent
!= archive/resource graph complete
```

Screenshot identity retains derivation to browser state and should not replace the underlying content/archive evidence.

## 13. Replay fidelity is an assessment

Tools such as Browsertrix can compare archive replay against crawl observations using screenshot, text, and resource comparisons. Treat such results as a profile-specific `ReplayFidelityAssessment`, not as archive truth or completeness.

```text
high replay similarity
!= complete original capture
```

Any numeric coordinates retain metric/profile identity and must not become one universal archive-validity score.

## 14. Transform lineage

Every material transformation is explicit EPI-004 derivation.

Typical chain:

```text
network response content
 -> content decoding/decompression
 -> charset decoding
 -> parser/browser DOM
 -> extraction
 -> normalization
 -> selector target
```

A later selector must resolve against the exact artifact/profile it claims to target. Derived text must never masquerade as the bytes received from the network.

## 15. Truncation and partial capture

Local body caps, timeouts, interrupted streams, missing ranges, blocked subresources, browser failures, or archive-write failures remain explicit.

```text
partial capture
!= complete snapshot
```

A profile may content-address a partial artifact, but it must remain typed as partial/truncated rather than promoted to a complete source snapshot.

## 16. Dynamic/personalized content

Where observable and permitted, acquisition context may retain material classes for:

- cookie/authentication context;
- locale/language;
- viewport/device;
- explicitly used location profile;
- session/cache/service-worker state;
- A/B/personalization uncertainty;
- blocked third-party resources.

Do not claim another observer will receive the same content merely because the URL matches.

## 17. Privacy / minimization

Default public-web capture avoids retaining reusable secrets.

Sensitive classes include:

```text
Cookie / Set-Cookie
Authorization / Proxy-Authorization
bearer/session tokens
private form data
authenticated account identifiers
private query parameters
TLS/session secret material
```

Use explicit disclosure states such as:

```text
Included
Redacted
CommitmentOnly
Omitted
```

under a separate policy profile.

```text
field omitted for privacy
!= field absent from source exchange
```

Redaction produces a derived object; it does not mutate the original artifact identity.

## 18. WARC boundary

WARC 1.1 is an archive interchange format.

Preserve its native record context while keeping EPI identity separate.

Relevant WARC facts include:

```text
WARC-Record-ID
WARC-Date
WARC-Target-URI
WARC-Block-Digest
WARC-Payload-Digest
record type
revisit refs
segmentation/truncation
```

Important invariants:

```text
WARC-Record-ID != ArtifactId
WARC-Block-Digest != WARC-Payload-Digest
WARC-Date != publication time != event time != trusted time
```

WARC digest fields are optional and algorithm-agile; importing them does not replace an EPI-approved artifact commitment.

WEB-ARCHIVE-001 owns the detailed mapping.

## 19. WACZ boundary

WACZ 1.2.0 packages WARC content, indexes, page/context metadata, and a manifest with file fixity. It also permits signing extensions while explicitly not mandating one trust model.

Use it as a portable archive capsule/interchange profile, not the semantic root.

```text
WACZ manifest validates
!= capture complete
!= source authentic
!= claim true
```

Pin the exact WACZ version/profile because 1.2.0 remains a Webrecorder Draft.

## 20. Security dependency

WEB-CAPTURE-SEC-001 is independently load-bearing before public-web acquisition.

A fidelity-correct crawler that can reach localhost, cloud metadata services, private networks, user secrets, or privileged Symthaea/Mycelix APIs is not safe.

Likewise, a secure fetcher that collapses decoded/rendered derivatives into source bytes is not epistemically correct.

Both theorems are required.

## 21. First executable gate

Do not start with Browsertrix or public internet.

WEB-CAPTURE-TEST-001 should first use a deterministic local fixture server and independently frozen vectors for:

```text
fixed response
valid digest
invalid digest
compressed response
allowed redirect
blocked redirect
large/truncated body
timeout
instruction-like text
```

Expected first product boundary:

```text
AcquisitionAttempt
+ HttpExchangeObservation
+ exact HttpMessageContentArtifact
+ one decoded derivative
+ exact selector
```

with no browser, WARC, WACZ, crawler, or public-network requirement.

## 22. Qualification nonclaims

Even an eventual successful first qualification does not establish:

- factual truth;
- source honesty;
- publisher identity;
- global capture completeness;
- legal permission to crawl;
- browser-engine security;
- WARC/WACZ conformance;
- independent corroboration;
- scientific validity;
- Symthaea persistence/tool authority;
- action authorization;
- production readiness.

It establishes only the exact acquisition/capture/derivation distinctions proved under the qualified profile.
