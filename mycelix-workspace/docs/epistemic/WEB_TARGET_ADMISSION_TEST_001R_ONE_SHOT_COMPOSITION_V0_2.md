# WEB-TARGET-ADMISSION-TEST-001R — One-Shot Origin/Endpoint Composition Corpus v0.2

## Status

Architecture/test-vector repair subject only. No DNS, sockets, TLS, HTTP, proxy, browser, EPI admission, or runtime network authority.

Tracks WEB-LOCATOR-001B / #2660 and repair issue #2690.

This subject **supersedes the V0.1 composition corpus for future implementation/qualification** while preserving V0.1 as historical evidence. It does not rewrite history and does not claim any dependency has merged or qualified.

## Why V0.2 exists

V0.1 predated several load-bearing boundaries:

- WEB-NET-ALLOC-001 / #2678: IPv6 ordinary-public eligibility is bounded to the current IANA `2000::/3` allocation envelope after stronger refusal checks;
- WEB-DNS-001A semantics: only an exact, complete bounded resolution observation may feed V1/V2 admission;
- every observed CNAME target/final canonical name must be re-evaluated under the domain-policy profile;
- resolver, endpoint-policy, and allocation-envelope identities must be bound into the decision;
- HTTPS/SVCB discovery must not silently widen the admitted target;
- ambient proxy, HSTS upgrade, Alt-Svc, connection coalescing, or hidden re-resolution must not create authority.

Therefore:

```text
old corpus frozen correctly
!= old corpus sufficient for current admission theorem
```

## Candidate dependency identities

```text
WEB-LOCATOR-TEST-001         PR #2652  113c72bd2bbe5f6c3c7b76f80605682081ac8033
WEB-DOMAIN-POLICY-TEST-001   PR #2663  d118567508c5f3cc6dba9c544756438c6a0a63d4
WEB-NET-REGISTRY-TEST-001    PR #2664  18d3d1f81de8f29dccb96e24fef787b4f90588e2
WEB-DNS-TEST-001             PR #2666  417e5fd3f3f9c442083e0a029dd89eb049ffa717
WEB-NET-ALLOC-001            PR #2678  2c25ad533256a4523788d9bf0df8d5db28c2dc31
```

These remain candidate semantic inputs only.

## Profile identity must change

The positive/negative theorem changed materially, so the profile identity also changes.

```text
mycelix:ordinary-public-target-admission:v1
!=
mycelix:ordinary-public-target-admission:v2
```

V2 is not a silent widening/narrowing of V1. Historical decisions remain bound to the profile that produced them.

## Core theorem

```text
parser observation
!= domain eligibility

domain eligibility
!= DNS completeness

DNS completeness
!= endpoint eligibility

endpoint eligibility
!= target admission

target admission
!= socket connection

socket-peer membership
!= TLS authentication

TLS authentication
!= HTTP response capture

HTTP response capture
!= source authenticity
!= claim truth
```

## OrdinaryPublicTargetAdmissionV2

V2 is deliberately narrow:

```text
scheme                         = https only
HTTP -> HTTPS upgrade          = disabled
allowed effective port         = 443 only
userinfo                       = refuse
proxy                           = direct only / ambient proxy disabled
resolver profile               = exact bound profile only
resolution completeness        = Complete only
search domains                 = disabled upstream
hosts-file inheritance         = disabled upstream
CNAME target domain policy     = every observed name must be eligible
mixed endpoint set             = refuse
endpoint profile               = exact bound profile only
IPv6 allocation envelope       = exact bound profile only
HTTPS/SVCB discovery            = disabled
HTTPS/SVCB TargetName           = no authority
HTTPS/SVCB port                 = no authority
HTTPS/SVCB ipv4hint/ipv6hint    = no authority
Alt-Svc                         = disabled
connection coalescing           = disabled
post-admission DNS lookup       = forbidden
redirect                        = full fresh admission
admission reuse                 = one-shot
connected peer                  = exact admitted-set member
```

No item above opens a connection.

## Input provenance bindings

A domain-host decision must bind, at minimum:

```text
exact parsed-locator identity/profile
origin authority projection
initial domain-policy profile + decision
exact resolution observation identity
exact resolver profile identity
resolution outcome = Complete
complete CNAME ancestry
policy decision for every observed CNAME name
complete A + AAAA endpoint set
endpoint-policy profile identity
endpoint-policy assessment for every endpoint
IPv6 allocation-envelope profile identity
admission profile identity
acquisition-attempt identity
```

Copying equivalent-looking values into a new object does not establish those references.

## CNAME firewall

The initial hostname passing domain policy does not authorize aliases.

```text
alias.public-synthetic.invalidtld.
 -> foo.localhost.
```

must refuse under V2 even if the alias itself was eligible.

Every observed CNAME target/final canonical name is re-evaluated under the exact domain-policy profile.

```text
starting name eligible
!= all CNAME names eligible
```

## DNS completeness firewall

Only an exact `Complete` resolution observation may feed positive domain-host admission in V2.

Typed failures or incomplete states including:

```text
NameError / NXDOMAIN
NoData
Timeout
TruncatedWithoutQualifiedRetry
PartialFamilyResult
Cancelled
CnameLoop
CnameDepthExceeded
AnswerLimitExceeded
```

cannot be converted to a positive endpoint set.

```text
A success + AAAA timeout
!= complete resolution
```

## Full endpoint-set rule

Every exact A/AAAA endpoint from the complete observation is classified.

```text
[eligible A, prohibited B]
-> RefusedMixedEndpointSet
```

V2 does not filter a prohibited answer and continue with a clean subset.

## IPv6 allocation envelope

After stronger special-purpose/multicast matching, V2 requires the endpoint-policy assessment to be bound to the exact current allocation-envelope profile.

```text
4000::1
-> outside-envelope refusal

2001:4860:4860::8888
-> value-only classifier-eligible candidate
```

The latter remains only an offline value fixture; the corpus performs no network I/O.

## Direct-IP path

Direct IP literals still pass through the endpoint-policy profile.

```text
Parsed IPv4/IPv6 literal
 -> DirectIpLiteral provenance
 -> endpoint policy
 -> allocation-envelope rule where applicable
 -> target admission
```

No DNS observation is fabricated for direct literals.

## HTTPS/SVCB firewall

RFC 9460-style HTTPS/SVCB discovery is explicitly out of V2's admitted input space.

An HTTP/TLS library must not silently widen authority by consulting or consuming:

```text
HTTPS/SVCB AliasMode TargetName
SvcParam port
alpn / no-default-alpn
ech
ipv4hint
ipv6hint
```

V2 binds only the A/AAAA observation already admitted by its resolver profile.

A future SVCB-aware profile must separately qualify TargetName domain policy, endpoint hints, port changes, ALPN/ECH semantics, and fresh admission.

## HSTS / upgrade firewall

V2 accepts only an already-parsed HTTPS locator.

```text
http://...
+ HSTS knowledge
!= admitted HTTPS target
```

HTTP-to-HTTPS rewriting requires a separate transformation/provenance theorem and fresh parsing/admission.

## Alt-Svc / coalescing firewall

An HTTP response or client cache must not widen the target to an alternative host/port/connection.

```text
Alt-Svc
!= new target admitted

HTTP/2 or HTTP/3 connection coalescing eligibility
!= target admission
```

Both are disabled until separately qualified.

## Origin authority vs transport endpoint

For a domain target, preserve both:

```text
OriginAuthorityV2
  scheme=https
  host=www.public-synthetic.invalidtld
  effective_port=443
  target=/report
  TLS server-name expectation=www.public-synthetic.invalidtld
  HTTP authority expectation=www.public-synthetic.invalidtld

AdmittedTransportEndpoint
  address=<one member of exact admitted set>
  port=443
```

The connector may connect only to an admitted endpoint while TLS/HTTP identity remains bound to the origin name.

```text
anti-rebinding
!= replace origin hostname with IP everywhere
```

## Resolver/profile mismatch

Equivalent-looking answers from a different resolver profile are not substitutable.

```text
same hostname + same IPs
+ different resolver profile
!= same admissible observation
```

The same applies to endpoint-policy and allocation-envelope profile identities.

## Rebinding / lineage

Resolution R1 and R2 are different observations even for the same hostname.

```text
R1 -> 8.8.8.8
R2 -> 127.0.0.1

admission(R1)
!= authority(endpoint from R2)
```

## One-shot capability

A positive decision binds exactly one acquisition-attempt identity.

```text
first consume -> handoff candidate
second consume -> replay refusal
```

No TTL or cache lifetime converts the decision into a reusable bearer capability.

## Redirects

Every redirect target gets a full fresh pipeline:

```text
parse
-> domain policy
-> resolution
-> CNAME policy
-> endpoint policy
-> fresh admission
```

No same-origin shortcut exists in V2.

## Proxy firewall

Ambient `HTTP_PROXY`, `HTTPS_PROXY`, `ALL_PROXY`, browser/system proxy settings, PAC files, or environment-specific proxy discovery cannot change a V2 direct target decision.

Proxying changes endpoint, DNS locus, privacy, and SSRF boundaries, so it requires a new profile.

## Peer observation

A later connector produces a distinct observation:

```text
selected endpoint
observed socket peer
membership in admitted set
```

Peer membership success establishes only endpoint-set consistency.

```text
peer member
!= TLS authenticated
```

## Machine-readable corpus

`WEB_TARGET_ADMISSION_TEST_001_V0_2.json` freezes V2 profile identity, exact dependency heads, and vectors for:

- safe single/dual-stack complete observations;
- mixed endpoint refusal;
- special-use initial host refusal;
- CNAME special-use refusal and all-eligible CNAME chain;
- unambiguous userinfo refusal;
- port refusal;
- direct IPv4 and IPv6 policy;
- IPv6 outside-envelope refusal;
- incomplete/negative DNS states;
- resolver/profile mismatches;
- R1/R2 lineage substitution;
- one-shot replay;
- redirects;
- SVCB/HTTPS disablement;
- HSTS/upgrade disablement;
- Alt-Svc/coalescing disablement;
- ambient proxy isolation;
- origin/endpoint split;
- peer match/mismatch.

No vector performs network I/O.

## Qualification requirements

A future V2 composition implementation must prove at least:

1. exact profile/dependency identities are bound;
2. userinfo/scheme/port refusal occurs before DNS composition;
3. special-use initial domains refuse;
4. every CNAME name is re-evaluated under the exact domain-policy profile;
5. only Complete DNS observations can reach positive endpoint composition;
6. every A/AAAA endpoint is classified;
7. mixed endpoint sets refuse without filtering;
8. IPv6 allocation-envelope mismatch/refusal is preserved;
9. direct IPs cannot bypass endpoint policy;
10. resolver-profile substitution refuses;
11. endpoint/allocation-profile substitution refuses;
12. R1/R2 observation substitution refuses;
13. HTTPS/SVCB cannot widen target/port/address/ALPN authority;
14. HSTS/Alt-Svc/coalescing cannot widen authority;
15. redirect requires fresh admission;
16. one-shot replay refuses;
17. origin hostname and transport IP remain distinct;
18. ambient proxy state has no authority;
19. peer mismatch invalidates connector continuation;
20. no test performs network I/O or creates TLS/HTTP/EPI authority.

## Authority ceiling

A future PASS establishes only deterministic offline composition into a bounded one-shot target-admission decision under V2.

It does not establish reachability, DNS authenticity, BGP/RPKI validity, TLS authenticity, HTTP correctness, source identity, source truth, capture completeness, EPI admission, crawl legality, or consequential action authority.
