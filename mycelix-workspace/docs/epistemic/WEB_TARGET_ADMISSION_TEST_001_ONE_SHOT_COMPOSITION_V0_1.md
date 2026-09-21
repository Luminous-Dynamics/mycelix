# WEB-TARGET-ADMISSION-TEST-001 — One-Shot Origin/Endpoint Composition Corpus v0.1

## Status

Architecture/test-vector subject only. No sockets, TLS, HTTP, proxy, browser, EPI admission, or runtime network authority.

Tracks WEB-LOCATOR-001B / #2660 and composes the candidate semantics frozen by:

```text
WEB-LOCATOR-TEST-001        PR #2652  113c72bd...
WEB-DOMAIN-POLICY-TEST-001 PR #2663  d1185675...
WEB-NET-REGISTRY-TEST-001  PR #2664  18d3d1f8...
WEB-DNS-TEST-001           PR #2666  417e5fd3...
```

Those references identify candidate corpora only. This subject does not claim that any dependency is merged or qualified.

## Core theorem

```text
origin authority
!= transport endpoint

target admitted
!= socket connected

socket peer matched
!= TLS authenticated

TLS authenticated
!= HTTP response captured

HTTP response captured
!= source truthful
!= content true
```

## OrdinaryPublicTargetAdmissionV1

The initial composition profile is deliberately strict:

```text
scheme                 = https only
effective port         = 443 only
URL userinfo           = refuse
special-use domain     = refuse
single-label domain    = refuse
mixed endpoint set     = refuse
proxy                   = disabled
admission reuse         = one-shot
redirect                = fresh admission
connected peer          = exact admitted-set member
DNS re-resolution       = forbidden after admission
```

The profile is a composition theorem over already-separated observations. It does not open a connection.

## Origin authority vs transport endpoint

For a domain URL:

```text
https://www.public-synthetic.invalidtld/report
```

the system retains:

```text
OriginAuthorityV1
  scheme = https
  host = www.public-synthetic.invalidtld
  port = 443
  target = /report

TransportEndpointV1
  IP = admitted concrete address
  port = 443
```

These MUST remain distinct.

The future connector uses the transport endpoint for the socket, while preserving the origin host for:

```text
TLS SNI/server-name
certificate hostname validation
HTTP Host / :authority
origin semantics
```

Replacing the origin hostname with an IP literal everywhere is not an acceptable anti-rebinding strategy because it changes HTTPS semantics.

## Domain-host admission

For a domain host:

```text
ParsedWebUrlV1
 -> PublicDomainPolicyDecisionV1
 -> exact ResolutionObservationV1
 -> EndpointPolicyDecisionV1 for every candidate
 -> TargetAdmissionDecisionV1
```

No stage can be skipped by copying data into a later type.

## Direct-IP admission

For an IP-literal host:

```text
ParsedWebUrlV1
 -> domain policy = NotApplicable
 -> resolution = DirectIpLiteral
 -> endpoint policy
 -> TargetAdmissionDecisionV1
```

Direct IP syntax does not bypass special-purpose classification.

## Complete endpoint-set rule

V1 admits only when every candidate endpoint in the exact resolution observation is eligible.

```text
[8.8.8.8, 127.0.0.1]
-> refuse mixed endpoint set
```

The admission layer does not filter out the prohibited address and continue with the clean one.

A future weaker/filtered-set theorem would require a separately qualified profile.

## One-shot capability

A positive decision is scoped to one exact acquisition attempt.

Conceptually:

```text
AdmittedWebTargetV1 {
  profile,
  locator/origin identity,
  resolution identity,
  admitted endpoint set,
  acquisition_attempt_id,
  consumed = false
}
```

Consumption is irreversible under V1.

```text
first use
-> may hand off to connector

second use
-> refuse replay
```

This prevents an old DNS result from becoming a long-lived bearer permission.

## DNS rebinding firewall

After admission:

```text
hostname
MUST NOT be handed to a connector that resolves it again
```

Instead:

```text
admitted endpoint set
 -> choose/race only members
 -> connect exact endpoint
 -> inspect actual peer
 -> require peer ∈ admitted set
```

A fresh lookup is a fresh resolution observation and therefore needs a fresh admission decision.

## Connected-peer observation

Connection result is separate:

```text
ConnectedPeerObservationV1 {
  admission_ref,
  selected_endpoint,
  observed_peer,
  membership_result
}
```

A peer match establishes only that the socket peer belongs to the admitted set.

```text
peer match
!= TLS certificate valid
```

Peer mismatch fails before TLS/HTTP continuation.

## Redirects

A redirect response cannot reuse the prior target's authority.

```text
A admitted
A returns Location: B
!= B admitted
```

The redirect target enters the full parser/domain/resolution/endpoint-policy pipeline again.

Even where same-origin optimization is later possible, V1 prioritizes semantic clarity over reuse.

## Userinfo

All URL userinfo refuses under the public target profile.

```text
parser observed userinfo
!= network authority
```

This is stricter than merely refusing passwords and removes an unnecessary credential/display-confusion surface.

## Port boundary

V1 admits only HTTPS effective port 443.

A non-default port such as 8443 refuses before DNS.

Later port profiles can be separately named and qualified.

## Proxy firewall

The direct public profile ignores/refuses ambient:

```text
HTTP_PROXY
HTTPS_PROXY
ALL_PROXY
browser/system proxy state
```

Proxy use changes the socket endpoint, resolver location, origin visibility, SSRF boundary, and privacy semantics.

Therefore proxy support must be a separate explicit profile.

## Corpus

`WEB_TARGET_ADMISSION_TEST_001_V0_1.json` freezes cases for:

```text
single safe endpoint
safe dual-stack set
mixed safe+unsafe set
special-use domain
userinfo
disallowed port
direct special IP
direct ordinary IP value-only
NXDOMAIN
partial-family result
one-shot replay
redirect
peer match
peer mismatch
origin/endpoint split
ambient proxy state
```

All addresses and domains are value-only fixture inputs. No test performs network I/O.

## Qualification requirements

A future implementation must prove at least:

1. userinfo refusal happens before DNS/connection;
2. disallowed port refusal happens before DNS;
3. special-use domain refusal happens before DNS;
4. direct IPs cannot bypass endpoint policy;
5. every resolved candidate is classified;
6. mixed eligible/prohibited sets refuse;
7. safe dual-stack set preserves both endpoints;
8. admission binds the exact resolution observation;
9. R1 admission cannot consume R2 endpoints;
10. admission is one-shot and nonce/attempt replay fails;
11. redirect requires fresh admission;
12. connector cannot re-resolve hostname;
13. connection racing can use only admitted-set members;
14. actual peer outside the set invalidates connection;
15. peer membership success grants no TLS theorem;
16. origin hostname survives separately for SNI/certificate/HTTP authority;
17. ambient proxy configuration cannot alter a direct-profile decision;
18. admission creates no HTTP/capture observation;
19. admission creates no EPI truth/evidence/action authority;
20. no test performs network I/O.

## Product implementation direction

The eventual code should expose types that make the transition visible:

```text
ParsedWebUrlV1
DomainPolicyDecisionV1
ResolutionObservationV1
EndpointPolicyDecisionV1[]
        ↓
TargetAdmissionDecisionV1
        ↓
AdmittedWebTargetV1
```

Only `AdmittedWebTargetV1` can be accepted by the future connector.

A raw `Url`, raw hostname, or arbitrary `SocketAddr` must not be accepted by the public capture connector API.

## Authority ceiling

A future PASS establishes only correct offline composition of exact parser/domain/resolution/endpoint-policy inputs into a one-shot target decision.

It does not establish:

```text
network reachability
TLS authenticity
HTTP correctness
source identity
source truth
artifact capture
crawl legality
EPI admission
action authority
```

## Non-scope

No live resolver, socket connector, TLS stack, HTTP client, proxy/Tor implementation, HTTP/2 or HTTP/3 connection coalescing, browser navigation, source authentication, content analysis, or EPI mutation belongs in this tranche.
