# Mycelix Content Fabric v1 Authority Boundaries

Status: normative CF-00 boundary document.

## Principle

Evidence, recommendation, authorization, execution, billing and settlement are distinct capabilities. Possessing one does not imply the others.

## Symthaea Content Planner

May:

- rank eligible providers;
- propose replicate, repair, prefetch, evict or rebalance actions;
- attach rationale and an evidence digest;
- consume bounded observations such as latency, capacity, health and demand summaries.

May not:

- create/renew/cancel placement leases;
- transfer funds or settle invoices;
- override jurisdiction, encryption, retention or failure-domain requirements;
- alter firewall/covenant/network enforcement policy;
- delete durable content;
- mint privileged capabilities;
- convert an observation directly into an enforcement action.

## Authorized executor

An executor may translate a valid proposal into an action only when it independently verifies:

1. proposal schema/version;
2. referenced current policy/intent;
3. hard constraints;
4. relevant lease/capability/offering state;
5. caller/operator authorization;
6. freshness/replay bounds.

The executor is allowed to reject any planner proposal.

## Holochain coordination

Integrity/coordinator zomes may validate and publish coordination records. They do not store bulk bytes and do not become a byte-integrity oracle. DHT availability is not required to read already cached immutable content.

## Content node

The node may ingest, verify, store, serve and evict data subject to local policy. It must not decide software admission, create commercial settlement, or infer authority from possession of bytes.

## Marketplace and Finance

Content Fabric records technical delivery facts. Marketplace interprets agreements and billability. Finance records payment/settlement/finality. A Delivery/ServiceReceipt is not itself payment authority.

## Nix admission

The Nix edge cache is a delivery path, not an admission authority. Existing artifact/signature/admission policy decides whether a NAR is trusted for installation.

## Xenia / privileged administration

Privileged administration remains outside planner authority. Remote administrative capabilities should be explicit, narrowly scoped and auditable rather than inferred from provider status or planner recommendations.

## Failure behavior

When a dependency is unavailable, the system should degrade along authority boundaries rather than silently widening authority. Examples:

- no planner: deterministic/manual placement can continue;
- no Holochain: cached bytes remain readable;
- no Marketplace: existing lease policy governs technical delivery;
- no Finance: delivery may be receipted but settlement remains pending;
- no Iroh: fallback transport may be used;
- no admission service: delivered software is not silently trusted.
