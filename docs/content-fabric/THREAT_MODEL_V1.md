# Mycelix Content Fabric v1 Threat Model

Status: CF-00 design contract.

## Assets to protect

- immutable content integrity;
- confidentiality of private content;
- correctness of placement-policy enforcement;
- availability of replicas and caches;
- separation between delivery evidence, commercial settlement and software admission;
- operator control over privileged actions;
- recoverability after node, transport, Holochain, marketplace or finance outages.

## Trust assumptions

The fabric does not trust storage providers for content integrity. Clients verify immutable content cryptographically. Provider identity/reputation informs availability and service-quality decisions, not byte correctness.

A provider's assertion that replicas are independent is insufficient. Failure-domain independence must distinguish at least provider, operator, machine, site, ASN/network, region, jurisdiction and power domain when relevant. Stronger attestation/evidence can be added later.

Symthaea is advisory. A planner output is evidence/proposal, not authorization.

## Threats and required behavior

| Threat / failure | Required result |
| --- | --- |
| Provider returns wrong bytes | digest verification rejects the content |
| Provider truncates or corrupts ingest | partial/corrupt object never becomes addressable |
| Provider disappears mid-fetch | transfer may resume or fail over to another provider |
| Provider advertises missing content | availability/reputation evidence degrades; integrity is not compromised |
| Advertisement replay | bounded expiry/version/sequence checks reject stale state where feasible |
| Planner scores forbidden jurisdiction highly | candidate is excluded before scoring |
| Sybil providers claim independent replicas | claims are not automatically counted as distinct failure domains |
| Malicious edge cache modifies NAR | Nix/content verification rejects it |
| Symthaea emits a bad plan | authorized executor may reject it; no direct side effect occurs |
| Holochain unavailable | already cached immutable content remains readable |
| Iroh unavailable | HTTP/local transport can remain usable |
| Marketplace unavailable | already-authorized technical delivery follows explicit lease policy |
| Finance unavailable | usage may be receipted while settlement remains pending |
| Edge/home cache disappears | only replicas whose policy permits loss are affected |
| Crash during ingest | temporary bytes are not promoted into the CAS |

## Confidentiality

Private content MUST be encrypted before placement on untrusted storage. Content addressing must not be interpreted as permission to disclose plaintext. Key management is outside the storage provider's trust boundary.

## Policy bypass

Hard requirements such as jurisdiction, minimum replicas, minimum failure domains, encryption and retention are constraints, not weighted preferences. They must be applied before cost, latency, energy, locality or other optimizer scores.

## Authority escalation

The planner MUST NOT:

- create or renew leases;
- pay providers;
- bypass jurisdiction or encryption requirements;
- change firewall/covenant policy;
- delete durable data;
- mint privileged capabilities.

Privileged execution remains with explicit authorized components/operators.

## Supply-chain boundary

Receiving bytes from a provider does not authorize installation. Content delivery and software admission are separate checks. Existing Nix admission/signature policy remains authoritative for whether a delivered NAR is acceptable.

## Commercial boundary

A technical ServiceReceipt proves/records delivery facts. Marketplace policy decides whether those facts are billable. Finance records settlement/finality. These meanings must not collapse into one record.

## v0.1 exclusions

Proof-of-storage, Byzantine erasure-coding protocols, anonymous payment, global anti-Sybil guarantees and universal failure-domain attestation are not required for Edge Seed. They may be layered on after deterministic replication and failure recovery are demonstrated.
