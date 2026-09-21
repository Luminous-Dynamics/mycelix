# WEB-NET-ALLOC-001 — IPv6 Allocation Envelope v0.1

## Status

Architecture/test-vector subject only. No DNS, sockets, TLS, HTTP, EPI admission, or network authority.

Tracks #2677 and strengthens WEB-NET-REGISTRY-001 / #2654 / PR #2664 before endpoint-classifier qualification.

## Defect being closed

The IPv4/IPv6 Special-Purpose Address registries are not a complete statement of which IPv6 addresses are currently ordinary public unicast allocation space.

Therefore:

```text
no special-purpose match
!= ordinary-public IPv6 endpoint
```

A conservative public-web policy must separately bind the top-level IANA IPv6 Address Space allocation state.

## Exact source evidence

Registry authority: IANA IPv6 Address Space.

Observed through the same rsync-derived mirror used by #2664:

```text
repository: larseggert/iana-assignments
commit:     ea3e987980dbdb643d60445cd971c1baa78c8c80

path:       ipv6-address-space/ipv6-address-space.xml
XML blob:   c4d137e88ef40a6b03dcff7e88c08d14b4e94977
TXT blob:   1061c404100a734dd180f601336fc47d226f9029

registry Last Updated: 2025-10-23
```

IANA remains the registry authority. The mirror is transport/provenance evidence only.

## Current allocation theorem

The frozen registry states that IANA should currently limit IPv6 unicast allocation to addresses beginning with binary `001`, represented by:

```text
2000::/3
```

The surrounding top-level ranges remain reserved, special, local, deprecated, or multicast under the frozen registry.

For `OrdinaryPublicEndpointV1`:

```text
IPv6 address
+ no stronger special-purpose/multicast refusal
+ inside 2000::/3
-> OrdinaryPublicEndpointEligible candidate

IPv6 address
+ no stronger special-purpose/multicast refusal
+ outside 2000::/3
-> RefusedOutsideCurrentIanaGlobalUnicastEnvelope
```

Eligibility remains classification only.

## Precedence

The allocation envelope is a fallback after the more-specific #2664 table.

```text
special-purpose / multicast match
-> preserve that specific refusal reason

otherwise IPv6 inside 2000::/3
-> eligible candidate

otherwise
-> outside-envelope refusal
```

Examples:

```text
2001:db8::1
-> Documentation special-purpose refusal

ff02::1
-> multicast refusal

fc00::1
-> ULA special-purpose refusal

64:ff9b::808:808
-> translation special-purpose refusal

4000::1
-> outside-current-IANA-global-unicast-envelope refusal
```

## Deprecated IPv6 forms

The top-level registry also makes visible cases that a special-purpose-only deny table can miss.

Examples include:

```text
::/96
```

formerly used for IPv4-compatible IPv6 addresses and deprecated by RFC 4291, and:

```text
fec0::/10
```

formerly Site-Local and now reserved/deprecated.

The V1 envelope refuses both because they are outside current `2000::/3` IANA allocation space unless a more-specific existing refusal already applies.

## Frozen registry projection

`WEB_NET_ALLOC_TEST_001_IPV6_ENVELOPE_V0_1.json` records all 20 top-level rows from the frozen registry and labels only:

```text
2000::/3 -> current_iana_global_unicast_envelope
```

All other top-level rows are outside the V1 ordinary-public allocation envelope.

This projection does not replace the underlying IANA registry.

## Versioning law

A future IANA allocation expansion MUST produce a new registry/corpus/policy identity.

```text
allocation snapshot A1
-> policy P1

allocation snapshot A2
-> policy P2
```

Historical admission decisions remain bound to the policy actually used at the time. Never silently reinterpret an old target as if a later allocation profile had governed it.

## Product repair

The unreviewed construction commit:

```text
5a8125214359581e6df158db589f42fcf9660b6d
```

was built directly from #2664 and therefore has an over-broad IPv6 positive fallback.

It MUST NOT become the product review subject.

Rebuild/refreeze WEB-NET-REGISTRY-001A as a fresh direct child of this exact allocation-envelope subject, preserving the original #2664 registry facts plus this additional IPv6 eligibility theorem.

## Required qualification vectors

The repaired product must prove at least:

1. `2606:4700:4700::1111` is value-only eligible under the allocation envelope and no stronger refusal;
2. `4000::1` refuses outside the current IANA global-unicast envelope;
3. `fec0::1` refuses outside the envelope;
4. `::808:808` refuses outside the envelope;
5. `2001:db8::1` retains special-purpose Documentation refusal rather than generic eligibility;
6. `ff02::1` retains multicast refusal;
7. `fc00::1` retains ULA special-purpose refusal;
8. `fe80::1` retains link-local special-purpose refusal;
9. `64:ff9b::808:808` retains translation special-purpose refusal;
10. IPv4 policy behavior is unchanged;
11. no classifier result grants socket/network authority;
12. no live IANA network lookup occurs at runtime.

## Nonclaims

Membership in `2000::/3` does not establish route reachability, BGP/RPKI validity, ownership, DNS correctness, TLS authenticity, service identity, source authenticity, content truth, EPI validity, or permission to connect.
