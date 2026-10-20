# WEB-DNS-TEST-001 — Deterministic Synthetic Resolution Corpus v0.1

## Status

Architecture/test-vector subject only. The corpus performs no public DNS or network I/O.

Tracks WEB-DNS-001 / #2659, WEB-DOMAIN-POLICY-001 / #2657, WEB-NET-REGISTRY-001 / #2654, WEB-LOCATOR-001B / #2660, and WEB-CAPTURE-TEST-001 / #2644.

## Governing theorem

```text
domain eligible for ordinary DNS
!= resolution observed

resolution observed
!= DNS data authentic

address returned
!= endpoint admitted

resolution R1
!= resolution R2

TTL observed
!= capability lifetime
```

## SyntheticResolverV1

The first qualification profile is intentionally non-networked:

```text
network_io        = false
absolute names    = required
search domains    = disabled
hosts file        = not consulted
mDNS / LLMNR      = disabled
A + AAAA          = bounded
CNAME depth       = bounded
answer count      = bounded
```

This profile exists to qualify resolution *semantics* before selecting a live resolver library.

## Why synthetic first

A live resolver introduces several variables at once:

```text
resolver operator
transport
cache state
search domains
hosts/NSS behavior
timing
packet loss
DNSSEC state
network policy
```

The synthetic corpus lets Mycelix first prove the model and failure vocabulary without confusing those variables with implementation correctness.

## Absolute-name firewall

The public profile never expands:

```text
service
```

using an ambient suffix such as:

```text
corp.example
```

Single-label public targets are already refused by WEB-DOMAIN-POLICY-001. The DNS layer therefore consumes an exact normalized absolute name.

```text
search-domain expansion
!= public DNS evidence
```

## Hosts-file firewall

The public resolver profile does not silently consult `/etc/hosts` or platform equivalents.

A fixture explicitly models:

```text
ambient hosts mapping -> 127.0.0.1
synthetic DNS answer  -> 8.8.8.8
```

and requires the resolver observation to preserve only the synthetic DNS profile result.

A later local/enterprise profile may deliberately bind hosts-file state, but that is a different resolver identity.

## Complete answer sets

Resolution observations preserve the full bounded address set.

```text
[8.8.8.8, 127.0.0.1]
```

does not become:

```text
8.8.8.8
```

merely because an implementation chooses the first answer.

WEB-NET-REGISTRY-001 and WEB-LOCATOR-001B decide whether the *complete observed set* can be admitted.

## A / AAAA partiality

A and AAAA outcomes are retained separately.

```text
A = success
AAAA = timeout
```

is a partial-family observation, not silently upgraded to complete resolution.

V1 composition may refuse or remain indeterminate rather than pretending the unobserved family is empty.

## CNAME ancestry

The corpus retains alias ancestry:

```text
alias.public-synthetic.invalidtld.
 -> origin.public-synthetic.invalidtld.
 -> address
```

and separately tests:

```text
public-looking alias
 -> foo.localhost.
```

A CNAME target that enters special-use domain policy cannot inherit the starting name's ordinary-public eligibility.

```text
starting name eligible
!= alias target eligible
```

## CNAME loops and bounds

V1 freezes explicit failures for:

```text
CNAME loop
CNAME depth exceeded
answer limit exceeded
```

No infinite recursion or unbounded response growth belongs in the resolver path.

## NXDOMAIN vs NODATA

These remain distinct:

```text
NXDOMAIN
!= NODATA
```

and neither should be promoted into a broader claim that a resource can never exist.

## Rebinding / repeated lookup

The corpus contains two observations for the same name:

```text
R1 -> 8.8.8.8
R2 -> 127.0.0.1
```

They are distinct lineages.

```text
R1 admission
!= authority to consume R2
```

The eventual connector must connect only to the exact endpoint set bound to the chosen admission, without handing the hostname to another library for re-resolution.

## Truncation and retry

A truncated response without a separately qualified retry/fallback path is `Indeterminate`, not complete.

Later UDP->TCP, DoT, or DoH fallback behavior must be profile-visible rather than hidden library convenience.

## Failure vocabulary

The fixture exercises typed states including:

```text
NameError
NoData
Timeout
PartialFamilyResult
CnameLoop
CnameDepthExceeded
AnswerLimitExceeded
TruncatedWithoutQualifiedRetry
Cancelled
```

Exact wire names may be frozen in the executable child.

Avoid one generic `dns_failed`.

## Machine-readable fixture

`WEB_DNS_TEST_001_SYNTHETIC_V0_1.json` freezes twenty semantic cases:

```text
safe A
safe AAAA
safe dual-stack
mixed public + loopback
mixed public + ULA
safe CNAME
CNAME -> special-use target
CNAME loop
CNAME depth overflow
NXDOMAIN
NODATA
timeout
truncation
answer limit
partial address family
search-domain disabled
hosts-file ignored
rebind R1
rebind R2
cancelled
```

Positive public-looking addresses are value-only fixtures.

```text
fixture contains address
!= fixture contacts address
```

## Qualification requirements

A future implementation must prove at least:

1. no test opens a socket or performs DNS;
2. exact absolute query name is preserved;
3. search-domain expansion is impossible in the synthetic profile;
4. ambient hosts-file data is not consulted;
5. A and AAAA states remain separate;
6. full answer sets are retained;
7. mixed endpoint sets are not filtered into a falsely clean resolution;
8. CNAME ancestry is retained;
9. CNAME target domain policy is re-evaluated;
10. loops/depth/answer limits fail closed;
11. NXDOMAIN and NODATA remain distinct;
12. timeout/cancelled/truncated remain distinct;
13. partial-family success is not complete resolution;
14. R1 and R2 are different observation identities;
15. R1 admission cannot authorize an R2 endpoint;
16. TTL cannot manufacture capability lifetime;
17. resolver success creates no target-admission authority;
18. DNSSEC fields, if absent, remain explicitly not established;
19. no resolution outcome creates source authenticity/truth;
20. no result creates EPI/action authority.

## Implementation direction

The first executable child may use a pure trait/in-memory fixture implementation:

```text
ResolverProfileV1
+ absolute domain
+ deterministic fixture
        ↓
ResolutionObservationV1
```

Only after this semantics layer qualifies should a live resolver adapter be introduced.

A live adapter remains required to disable or explicitly bind ambient resolver behavior.

## Authority ceiling

A future PASS establishes only deterministic resolution-observation semantics under `SyntheticResolverV1`.

It does not establish:

```text
public DNS correctness
DNSSEC validity
endpoint safety
reachability
TLS authenticity
source authenticity
content truth
EPI admission
action authority
```

## Non-scope

No recursive resolver, DNSSEC validator, DoH/DoT deployment, censorship bypass, mDNS, Tor naming, enterprise split DNS, BGP/RPKI, socket connection, TLS, HTTP, or browser execution belongs in this tranche.
