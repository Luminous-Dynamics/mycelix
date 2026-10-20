# WEB-LOCATOR-TEST-001 — Offline locator and public-target admission corpus v0.1

Status: architecture/test-vector subject only. No runtime PASS claim.

Tracks: #2651, #2649, #2644, #2641, #2640.

## 1. Purpose

Freeze a deterministic zero-network contract for the first web-locator parsing and public-target admission boundary before any HTTP client is allowed to dereference a target.

The tranche proves only bounded parser/policy semantics for exact fixtures:

```text
supplied locator
 -> parsed web URL
 -> HTTP target projection
 -> synthetic resolution observation
 -> target-policy decision
```

It does not perform DNS, TCP, TLS, HTTP, browser, archive, or public-network I/O.

## 2. Governing non-equivalences

```text
URL parses
!= target safe to fetch

host string looks public
!= parsed semantic address public

synthetic DNS answer exists
!= endpoint authorized

endpoint authorized
!= connection established

connection established
!= response observed

response observed
!= artifact captured

locator admitted
!= source authentic
!= content true
!= EPI evidence admitted
```

## 3. Standards/profile anchors

Design inputs at authoring time:

- WHATWG URL Living Standard, observed current 2026-09-10;
- RFC 9110 HTTP Semantics: target URI excludes fragment;
- RFC 3986: parse components before percent-decoding reserved data; never double-decode;
- Unicode UTS #46 revision 36 / Unicode 18.0.0, dated 2026-08-31;
- IANA IPv4 and IPv6 Special-Purpose Address registries;
- Mycelix workspace dependency family `url = "2.5"`.

These references constrain design but do not establish conformance.

The executable subject must bind the exact resolved Rust `url` and `idna` crate versions, exact Rust toolchain, and exact frozen external-registry/test-corpus inputs before PASS.

## 4. Parser implementation rule

Use the existing workspace `url` dependency family for the Rust product implementation. Do not add a second competing web URL parser merely for this feature.

The V1 parser profile must expose at least:

```text
parser family / implementation
implementation version
IDNA implementation/profile
scheme
serialized URL
userinfo-present state
semantic host kind/value
parser-normalized port
effective destination port
path
query
fragment
HTTP target projection
```

The original bounded locator remains separately retained because parser serialization can normalize it.

## 5. Independent expected-value rule

The Rust product parser must not be its own expected-value oracle.

The seed fixture in this subject was authoring-cross-checked against the WHATWG `URL` implementation in Node.js v22.16.0. That cross-check is intentionally non-authoritative: it does not qualify Rust parity, policy behavior, or any network theorem.

The executable qualifier should additionally bind a small frozen subset of upstream web-platform-tests URL cases or another independently committed WHATWG expected-value source.

```text
WPT/parser parity
!= Mycelix target admission
```

## 6. Typed pipeline

The intended API boundary is conceptually:

```text
SuppliedLocatorV1
   -> ParsedWebUrlV1
   -> PublicTargetCandidateV1
   -> SyntheticResolutionObservationV1
   -> TargetAdmissionDecisionV1
   -> AdmittedWebTargetV1 | RefusedWebTargetV1
```

After this boundary exists, the public-capture HTTP client should accept an admitted target binding rather than an arbitrary raw URL string.

## 7. Fragment boundary

RFC 9110 target-resource routing excludes the fragment because fragments are client-side secondary-resource identifiers.

Therefore:

```text
https://example.org/a#one
https://example.org/a#two
```

retain distinct supplied/parsed locator states while projecting the same HTTP target URI:

```text
https://example.org/a
```

No capture receipt may claim the origin server received the fragment.

## 8. Percent-encoding boundary

Never percent-decode the whole locator before parsing components.

The seed corpus freezes:

```text
/a%2Fb
/a%252Fb
```

as distinct parser serializations. `%252F` must not become `/` through repeated decoding.

Reserved-delimiter interpretation is owned by the selected parser/profile; application code must not apply an additional blanket decoding pass.

## 9. Query boundary

Query ordering and duplicate names remain observable:

```text
?x=1&x=2
!= assumed equivalent to
?x=2&x=1
```

Do not sort, deduplicate, or decode/re-encode query pairs for canonical acquisition identity without a separate application-specific theorem.

## 10. Host and unusual IPv4 boundary

Security classification consumes the parser's semantic host result, not the raw host substring.

WHATWG-compatible parsing can normalize historical IPv4 spellings such as:

```text
127.1
0177.0.0.1
0x7f.0.0.1
2130706433
```

into `127.0.0.1`.

Every such normalized loopback form must be refused under the public-web policy.

## 11. IPv4-mapped IPv6

IPv4-mapped IPv6 must inherit the embedded IPv4 restriction.

```text
[::ffff:7f00:1]
```

cannot bypass loopback policy merely because its outer syntax is IPv6.

The classifier should retain the parsed IPv6 value and a mapped-IPv4 classification when applicable.

## 12. IDNA boundary

The parser owns domain-to-ASCII behavior under its exact `idna`/UTS-46 profile.

Seed vectors include:

```text
https://faß.de/
 -> https://xn--fa-hia.de/

https://例え.テスト/
 -> https://xn--r8jz45g.xn--zckzah/
```

and an invalid percent-encoded host delimiter case.

Visual similarity is never DNS identity.

## 13. Userinfo boundary

The V1 public-web policy should be stricter than bare parser acceptance and refuse any userinfo-bearing locator by default.

This is an intentional strengthening over the minimum #2651 text because public OSINT does not need ambient URL credentials and userinfo is a common source of host-display confusion and secret leakage.

Example:

```text
https://user@example.org@evil.test/
```

has semantic host `evil.test`; policy must never use naive substring/display interpretation.

Password/userinfo values must not be copied into normal security receipts.

## 14. Port boundary

Retain the supplied locator separately from parser serialization because a default explicit port can disappear during serialization:

```text
https://example.org:443/a
 -> https://example.org/a
```

Policy evaluates the effective destination port. V1 should permit only a frozen port set and refuse other ports before resolution/connect.

## 15. Address-policy source

Do not implement public-target admission as only:

```text
not RFC1918 && not loopback
```

The public profile should freeze exact IANA IPv4/IPv6 special-purpose registry snapshots and conservatively refuse addresses contained in special-purpose blocks, plus multicast/broadcast/reserved classes handled by the profile.

V1 intentionally prefers false refusal over accidental SSRF authority.

This conservative rule also avoids treating globally-reachable-but-special translation ranges as automatically safe without a separate translation theorem.

## 16. Required special-use families

The fixture corpus must cover at least:

IPv4:

```text
0.0.0.0/8
10.0.0.0/8
100.64.0.0/10
127.0.0.0/8
169.254.0.0/16
172.16.0.0/12
192.0.0.0/24 special-use members
192.0.2.0/24
192.168.0.0/16
198.18.0.0/15
198.51.100.0/24
203.0.113.0/24
multicast
240.0.0.0/4
255.255.255.255
```

IPv6:

```text
::/128
::1/128
::ffff:0:0/96
fc00::/7
fe80::/10
2001:db8::/32
ff00::/8
other exact special-purpose prefixes in the frozen IANA snapshot
```

The lists above are minimum test families, not the canonical registry itself.

## 17. Hostname resolution model

V1 uses only deterministic synthetic resolution observations:

```text
hostname -> ordered/set address observations
```

No resolver or public network call is permitted.

The conservative public policy refuses when:

- any candidate address is prohibited;
- the answer set is empty/unknown where an endpoint is required;
- the address classification is unsupported;
- address state changes invalidate the admitted binding.

A later measured profile may safely loosen the mixed-answer rule, but V1 does not.

## 18. Rebinding/connect-time model

The offline corpus models the eventual TOCTOU theorem:

```text
authorized endpoint A
+ observed connected peer B
+ A != B
-> connection invalid/refused
```

Future acquisition code must not authorize hostname H against address A and then hand H back to an uncontrolled client that independently re-resolves it.

## 19. Redirect boundary

A redirect is a new locator observation and a new authorization decision:

```text
Location observed
 -> resolve relative reference under exact base/profile
 -> ParsedWebUrlV1
 -> TargetAdmissionDecisionV1
```

Prior admission never transfers through a redirect.

## 20. Seed vector manifest

`WEB_LOCATOR_TEST_001_SEED_V0_1.json` contains authoring vectors for:

- fragments;
- historical IPv4 spellings;
- IPv4-mapped IPv6;
- IDNA;
- invalid host encoding;
- percent encoding / no double decode;
- query ordering;
- explicit default port;
- misleading userinfo;
- disallowed userinfo;
- path dot-segment normalization;
- special-purpose IP families;
- disallowed ports;
- mixed synthetic resolution;
- connect-time mismatch.

The seed file is a preregistered test input, not executed qualification evidence.

## 21. External WPT subset

The final executable corpus should add a bounded vendored projection of exact upstream WPT URL vectors.

Retain:

```text
upstream repository
exact commit
source path/blob identity
selected cases/content commitment
local projection derivation
```

Do not vendor the entire mutable upstream corpus into the product theorem.

## 22. Fixture authority ceiling

Every seed case is machine-readably compatible with:

```text
network_io_performed = false
DNS_authenticity_established = false
endpoint_reachability_established = false
source_authenticity_established = false
artifact_captured = false
claim_support_established = false
truth_established = false
action_authority = false
```

## 23. Qualification requirements

An eventual executable subject must at minimum prove:

1. exact parser/toolchain/dependency profile bound;
2. seed manifest bytes/content commitment frozen;
3. independent expected parser values do not call product parser;
4. historical IPv4 spellings cannot bypass loopback refusal;
5. mapped IPv6 cannot bypass embedded IPv4 policy;
6. all frozen special-purpose address vectors refuse;
7. fragment never enters target URI;
8. reserved percent-encoding is not blanket-decoded;
9. no double decode;
10. query ordering/duplicates preserved;
11. userinfo refused by public profile;
12. semantic host, not display substring, drives policy;
13. disallowed port refuses before resolution;
14. mixed safe/unsafe synthetic resolution refuses;
15. connect-peer mismatch invalidates binding;
16. redirect gets fresh authorization;
17. parser compatibility cannot substitute for policy admission;
18. refusal cannot become resource-absence evidence;
19. no network syscall/API is required by the product/test corpus;
20. no admitted locator mints EPI artifact/evidence authority.

## 24. Implementation order

```text
A. freeze docs + seed vector manifest
B. add dependency-light locator/policy types
C. implement parser projection using exact workspace `url` dependency
D. implement registry-backed address classifier
E. execute offline vector corpus
F. independently qualify exact product subject
G. only then let WEB-CAPTURE-TEST-001 HTTP client consume AdmittedWebTargetV1
```

## 25. Nonclaims

This document and seed corpus do not establish parser conformance, public-web safety, DNS authenticity, SSRF completeness, endpoint reachability, domain ownership, source identity, content truth, legal crawl permission, EPI evidence validity, or production readiness.
