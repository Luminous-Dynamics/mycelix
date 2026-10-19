# WEB-DOMAIN-POLICY-TEST-001 — Offline Special-Use Domain Admission Corpus v0.1

## Status

Architecture/test-vector subject only. No DNS, sockets, HTTP, browser, EPI admission, or runtime authority.

Tracks WEB-DOMAIN-POLICY-001 / #2657, WEB-LOCATOR-001 / #2649, WEB-LOCATOR-001A / #2655/#2656, WEB-DNS-001 / #2659, WEB-NET-REGISTRY-001 / #2654, and WEB-LOCATOR-001B / #2660.

## Governing theorem

```text
domain parsed successfully
!= ordinary public-DNS name
!= DNS resolution authorized
!= endpoint admitted
!= network connection authorized
```

and:

```text
special-use registry membership
!= malicious
!= nonexistent
!= unreachable
```

The V1 public-capture policy is intentionally narrower than general-purpose networking.

## Exact registry evidence

The corpus is derived from the IANA Special-Use Domain Names registry as observed through an rsync-derived Git mirror.

```text
registry authority: IANA Special-Use Domain Names
registry updated:   2026-05-22

authoring mirror:
  repository: larseggert/iana-assignments
  commit:     ea3e987980dbdb643d60445cd971c1baa78c8c80

exact mirrored registry:
  XML path: special-use-domain-names/special-use-domain-names.xml
  XML blob: 57f28f08b60876b1bba0a922aeea6a3fee1a836c
  TXT blob: 2fee65d67307efb80ce9b16d2813c57222c95898
```

The mirror is transport/provenance evidence, not a replacement registry authority. A future qualifier should independently compare the vendored normalized set with IANA's authoritative publication.

## Registry semantic rule

The source registry states that special-use designation applies to listed names **and their subdomains**.

Therefore the policy matcher is DNS-label aware.

```text
foo.localhost
-> matches localhost

evil-localhost.example
-> does not match localhost merely by substring
```

Raw suffix/string containment is not sufficient.

## Comparison profile

Policy comparison occurs only after the WHATWG/IDNA parser has produced a semantic `Domain` host.

V1 comparison:

```text
parser-normalized ASCII domain
 -> ASCII case fold
 -> remove one presentation trailing dot for comparison
 -> split into DNS labels
 -> label-boundary suffix match
```

The comparison key is not locator identity and does not rewrite the preserved supplied locator.

```text
PolicyDomainKey
!= SuppliedLocatorV1
!= ParsedWebUrlV1 identity
```

## Public-domain decision vocabulary

Conceptual V1 outcomes:

```text
OrdinaryPublicDnsEligible
RefusedSpecialUse { matched registry suffix }
RefusedSingleLabel
UnsupportedName
```

Optional non-authoritative resolver-class hints may explain why a name needs another future profile:

```text
.local      -> mDNS
.onion      -> Tor
localhost   -> localhost synthetic semantics
home.arpa   -> home-network semantics
alt         -> alternate namespace
```

A hint is not permission to invoke that resolver.

## All registry names fail closed under OrdinaryPublicDnsV1

Every normalized registry entry in the machine-readable fixture is refused by the ordinary-public-DNS profile.

This includes names which can exist or resolve under particular circumstances.

Especially important:

```text
example.
example.com.
example.net.
example.org.
```

are special-use registry entries.

Therefore these names may remain useful in parser examples and deterministic/synthetic fixtures, but **must not be treated as positive ordinary-public-DNS targets**.

## Single-label firewall

V1 refuses domain hosts with only one DNS label even if they are not in the IANA registry.

Examples:

```text
printer
metadata
intranet
service
```

Reason:

```text
single-label input
+ ambient resolver/search configuration
-> target semantics can change
```

Ordinary public capture must not depend on search suffixes, NSS plugins, VPN resolver state, mDNS/LLMNR fallbacks, or local aliases.

A later internal-network profile can explicitly own those semantics.

## `.onion`

`.onion` remains parseable as a domain name, but ordinary-DNS admission refuses it.

```text
WHATWG parser accepts .onion
!= public DNS may query it
```

A future Tor acquisition adapter requires a separately qualified Tor transport/resolver profile.

## `.local`

`.local` and subdomains refuse before ordinary DNS.

No fallback path may silently hand them to the OS resolver after policy refusal.

## `localhost`

`localhost` and every subdomain refuse ordinary public resolution before DNS.

The policy does not need to prove which loopback address an operating system would choose; it prevents the public profile from asking.

## `home.arpa`

Home-network naming semantics are outside the ordinary-public profile and refuse before DNS.

## Reverse/special infrastructure names

The registry includes reverse-tree and infrastructure names such as `in-addr.arpa`, `ip6.arpa`, `ipv4only.arpa`, `resolver.arpa`, and `service.arpa` entries.

Registry membership remains the rule; do not maintain a separate hand-coded folklore list.

## Fixture structure

`WEB_DOMAIN_POLICY_TEST_001_SEED_V0_1.json` freezes:

```text
profile identity
registry authority/provenance
exact authoring mirror commit/blob
normalization theorem
all 42 normalized registry entries
RFC references
deprecated state where observed
ordinary-public decision
resolver-class hints where useful
adversarial policy vectors
```

The machine-readable fixture is frozen in this subject as Git blob:

```text
966c29ad6f157f804d2ae4ac98cb2ef6952cee26
```

The executable qualifier must additionally compute and bind an independent SHA-256 (or later approved digest profile) over the exact fixture bytes. This architecture subject does not claim that independent digest has been executed.

## Qualification vectors

The future executable implementation must prove at least:

1. every one of the 42 normalized registry suffixes refuses;
2. a subdomain of every suffix refuses;
3. matching is ASCII case-insensitive after parser/IDNA normalization;
4. a trailing-dot presentation does not bypass comparison;
5. raw substring matches do not count;
6. `foo.localhost` refuses while `evil-localhost.<ordinary-domain>` does not match `localhost`;
7. `printer.local` refuses before ordinary DNS;
8. `.onion` refuses ordinary DNS and can expose only a non-authoritative Tor resolver-class hint;
9. `router.home.arpa` refuses;
10. `www.example.org` refuses under the public profile;
11. parser examples using `example.org` remain valid parser tests but are not positive network-policy tests;
12. single-label names refuse without search-domain expansion;
13. an ordinary synthetic multi-label name not in the registry can become `OrdinaryPublicDnsEligible`;
14. eligibility does not create a DNS observation;
15. policy refusal is not evidence that a resource is absent;
16. policy code exposes no socket/resolver/HTTP capability;
17. registry/profile substitution changes policy identity;
18. unknown future registry data cannot silently inherit admission;
19. no result creates EPI evidence/truth/action authority;
20. no test performs public network I/O.

## Update discipline

Registry refresh creates a new corpus/profile subject.

```text
registry snapshot R1
 -> normalized corpus C1
 -> policy P1

registry snapshot R2
 -> normalized corpus C2
 -> policy P2
```

Historical acquisition decisions remain bound to the policy actually used. Never silently re-evaluate old evidence under a newer registry and call it the original decision.

## Implementation direction

The first code child should be pure and dependency-light.

Conceptually:

```text
ParsedHostV1::Domain
        ↓
PublicDomainPolicyV1
        ↓
DomainPolicyDecisionV1
```

No DNS function belongs in the domain-policy crate.

The implementation may consume a generated static suffix table derived from this exact fixture. Generation itself must be deterministic and auditable.

## Authority ceiling

A future PASS can establish only deterministic domain-policy classification under the exact frozen registry/profile.

It cannot establish:

```text
DNS existence
DNS authenticity
endpoint safety
reachability
domain ownership
source authenticity
content truth
crawl legality
EPI admission
action authority
```

## Non-scope

No public-suffix-list ownership theorem, phishing/confusable detector, Tor client, mDNS client, DNSSEC, recursive resolver, enterprise split DNS, reputation scoring, IP endpoint policy, TCP/TLS/HTTP, browser execution, source authentication, or content analysis belongs in this tranche.
