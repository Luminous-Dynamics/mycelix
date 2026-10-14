# WEB-NET-REGISTRY-TEST-001 — Offline Public-Endpoint Policy Corpus v0.1

## Status

Architecture/test-vector subject only. No DNS, sockets, TLS, HTTP, browser, EPI admission, or runtime authority.

Tracks WEB-NET-REGISTRY-001 / #2654, WEB-LOCATOR-001 / #2649, WEB-DOMAIN-POLICY-001 / #2657, WEB-DNS-001 / #2659, WEB-LOCATOR-001B / #2660, and WEB-CAPTURE-SEC-001 / #2641.

## Governing theorem

```text
IP address parses
!= ordinary public endpoint

IANA Globally Reachable = true
!= Mycelix OrdinaryPublicEndpointV1 admitted

not matched by one handwritten private-range list
!= safe public target
```

IANA registry facts and Mycelix acquisition policy remain separate layers.

## Exact registry provenance

The corpus is derived from the IANA IPv4 and IPv6 Special-Purpose Address registries as observed through an rsync-derived Git mirror.

```text
registry authority: IANA
registry updated:   2025-10-09

authoring mirror:
  repository: larseggert/iana-assignments
  commit:     ea3e987980dbdb643d60445cd971c1baa78c8c80

IPv4:
  XML:  iana-ipv4-special-registry/iana-ipv4-special-registry.xml
  blob: eda3cd1fe1948450e5fac0f06847d42b8d49e38e
  TXT:  5cea73713e5f2efb05db2b45462e19bf2a7156e2

IPv6:
  XML:  iana-ipv6-special-registry/iana-ipv6-special-registry.xml
  blob: 08da69b6d84a1786031b328f5359fb4d4ed8c767
  TXT:  30be24053c993850a0456bb271ed078fbb5e4ddf
```

The mirror is transport/provenance evidence; IANA remains the registry authority.

## Registry facts are not crawler policy

The IANA registries distinguish fields including:

```text
Source
Destination
Forwardable
Globally Reachable
Reserved-by-Protocol
```

These coordinates remain distinct.

Mycelix must not collapse them into:

```text
public: bool
```

or infer:

```text
Globally Reachable == true
-> ordinary OSINT endpoint admitted
```

The V1 public-capture policy is intentionally stricter.

## OrdinaryPublicEndpointV1

V1 defines *ordinary direct public-web endpoint eligibility*, not general Internet reachability.

Policy:

```text
any current special-purpose registry prefix
-> refuse

IPv4 multicast 224.0.0.0/4
-> refuse

IPv6 multicast ff00::/8
-> refuse

unsupported / ambiguous classification
-> refuse
```

A later specialized profile may authorize a narrow special-purpose mechanism, but that is a new policy identity.

## Why globally reachable special-purpose entries still refuse

Examples in the frozen registries include globally reachable special-purpose entries such as:

```text
192.0.0.9/32
64:ff9b::/96
2001:1::1/128
```

Their IANA reachability property is retained as evidence, but V1 still refuses them because special-purpose routing/translation/anycast semantics are outside the ordinary direct-public-web theorem.

## Longest-prefix semantics

Special-purpose registries contain broad and more-specific overlapping prefixes.

Example:

```text
192.0.0.0/24
  contains
192.0.0.9/32
```

Classification must retain all matching rows where useful for audit and apply deterministic longest-prefix/specific-row semantics for the active disposition.

A broad row cannot erase a more-specific registry fact.

## Terminated/deprecated rows

Historical/terminated rows remain explicit corpus data rather than being silently dropped by wall-clock logic.

Examples include:

```text
192.88.99.0/24
2001:10::/28
```

The executable implementation must define whether historical rows participate in the current policy table under the frozen registry profile; V1's conservative fixture retains them as refused.

Refreshing registry/current-row semantics creates a new policy subject.

## IPv4-mapped IPv6

IPv4-mapped IPv6 requires two retained facts:

```text
outer IPv6 classification
+
embedded IPv4 classification
```

For example:

```text
::ffff:127.0.0.1
```

must never bypass loopback refusal.

Under V1 the outer mapped-address prefix is already special-purpose/refused, but the embedded classification is still retained so later profiles cannot accidentally widen it.

## Translation prefixes

Prefixes such as:

```text
64:ff9b::/96
64:ff9b:1::/48
```

are refused in V1 rather than attempting to infer safety of an embedded/translated IPv4 target without a separately qualified translation theorem.

## Multicast

The IANA special-purpose registry is not the only input needed for ordinary public endpoint policy.

V1 explicitly adds:

```text
IPv4 224.0.0.0/4
IPv6 ff00::/8
```

as refused multicast classes.

These extra policy prefixes are separately tagged as Mycelix policy data, not falsely attributed as rows in the special-purpose registry.

## Machine-readable fixture

`WEB_NET_REGISTRY_TEST_001_SEED_V0_1.json` freezes:

```text
exact registry provenance
26 normalized IPv4 prefixes
25 normalized IPv6 prefixes
selected IANA Globally Reachable / Reserved-by-Protocol facts
V1 refusal disposition
extra multicast policy prefixes
adversarial vectors
```

The fixture intentionally includes positive value-only classifier cases such as `8.8.8.8` solely to prove a non-special address can be classified as eligible by the *address-policy layer*.

```text
eligible value in offline fixture
!= permission to contact that address
!= network I/O performed
```

No test should contact any fixture address.

## Qualification vectors

A future executable classifier must prove at least:

1. every frozen IPv4 special-purpose prefix refuses;
2. every frozen IPv6 special-purpose prefix refuses;
3. IPv4 multicast refuses;
4. IPv6 multicast refuses;
5. loopback/private/link-local/shared/documentation/benchmark/reserved cases refuse;
6. globally reachable special-purpose entries still refuse;
7. longest-prefix overlap is deterministic;
8. terminated/deprecated rows follow the exact frozen profile;
9. `::ffff:127.0.0.1` cannot bypass embedded IPv4 loopback refusal;
10. translation prefixes refuse absent a translation-specific theorem;
11. an address outside all denied classes may become `OrdinaryPublicEndpointEligible`;
12. eligibility creates no socket/network capability;
13. classifier does not consult a live registry at runtime;
14. registry/profile substitution changes policy identity;
15. unknown/ambiguous data fails closed;
16. standard-library convenience classification can only be diagnostic, not canonical policy authority;
17. no target decision creates source trust or truth;
18. no result creates EPI evidence/action authority;
19. no public network I/O occurs;
20. historical decisions remain bound to the registry/policy version originally used.

## Implementation direction

The first code child should be a pure value classifier.

Conceptually:

```text
IpAddr
+ PublicEndpointPolicyV1
+ FrozenRegistryTableV1
        ↓
EndpointPolicyDecisionV1
```

No resolver, socket, HTTP, TLS, browser, proxy, or process execution belongs in this crate.

For domain-host captures, WEB-DNS-001 supplies the complete observed candidate address set and this classifier evaluates every candidate.

## Complete-set composition

WEB-LOCATOR-001B should initially require:

```text
every candidate endpoint passes OrdinaryPublicEndpointV1
```

before creating an admitted endpoint set.

```text
[ordinary-public A, prohibited B]
-> refuse mixed endpoint set
```

This classifier itself does not select/race/connect endpoints.

## Authority ceiling

A future PASS can establish only deterministic address-policy classification for the exact frozen registry and Mycelix policy profile.

It cannot establish:

```text
reachability
BGP/RPKI validity
service identity
TLS authenticity
DNS authenticity
source authenticity
content truth
crawl legality
EPI admission
action authority
```

## Non-scope

No geolocation, ASN reputation, BGP/RPKI, firewall enforcement, DNS, Tor/proxy policy, TLS, HTTP, browser execution, source authentication, content analysis, or truth scoring belongs in this tranche.
