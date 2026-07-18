# Multi-Conductor Network Evidence v1

## Purpose

The disposable promotion profile proves signed multi-agent behavior inside one conductor. It intentionally does not prove DHT propagation across conductor processes, behavior during a network partition, or post-reconnection conflict visibility.

The `network` profile closes that narrower evidence gap with two independent Holochain conductor processes connected through controlled local bootstrap and signaling services.

## Required topology

The evidence run uses exactly two conductors:

- seller conductor with its own data root, admin interface, app interface, installed app, agent key, token, and signing credentials;
- buyer conductor with a distinct copy of every boundary above.

Both conductors install the exact hApp bound by `artifact-manifest.json`. The profile does not reuse a single conductor with multiple agents and does not treat multiple app interfaces as multi-conductor evidence.

## Controlled network services

The runner must supply two executable hooks:

- `MARKETPLACE_NETWORK_SERVICES` starts and stops local bootstrap and signaling services and writes a receipt;
- `MARKETPLACE_NETWORK_CONTROL` reports network state and performs partition and healing operations.

The service receipt must bind the exact hook digest and expose only localhost or literal private/loopback endpoints. Public development bootstrap or signaling services are rejected for this profile.

The repository deliberately does not prescribe one privileged firewall, namespace, proxy, or service implementation. The controlled runner may select an implementation appropriate to its host, but its digest, method receipts, and observed behavior enter the signed evidence.

## Behavioral experiment

The profile runs two partition experiments.

### Safe automatic projection

1. Both conductors observe one pending transaction.
2. Seller confirms while isolated and buyer cancels while isolated.
3. Each conductor observes only its local authored branch during the partition.
4. After healing, both conductors expose the same `confirmed` and `cancelled` heads.
5. Both project the existing cancellation as `auto_resolved` and retain confirmation as superseded evidence.

### Unsafe conflict followed by bilateral authority

1. Both conductors observe one confirmed transaction.
2. Seller records shipment while isolated and buyer cancels while isolated.
3. After healing, both conductors must expose the same unresolved `shipped` and `cancelled` heads with no canonical winner.
4. Seller and buyer independently approve the same existing shipped head against that exact two-head set.
5. Both approvals must propagate and remain independently addressable.
6. One party publishes the bilateral resolution referencing both approval actions.
7. Both conductors must expose `authorized_resolved`, reason `bilateral_agreement`, and the same shipped canonical head.
8. The cancelled head must remain visible as superseded evidence, and the authority record must remain attached to the projection.

A hook returning “partitioned” or an API call returning success is insufficient. The run must observe local divergence, post-heal conflict, independent approval propagation, and the final identical authorized projection.

## Promotion assertions

The verifier requires:

- topology `two_conductor_isolated_network`;
- two distinct conductor processes and admin endpoints;
- locally controlled network services;
- SHA-256 bindings for the service implementation, control hook, and topology receipt;
- listing and transaction propagation within 120 seconds each;
- divergent valid writes during partition;
- post-heal safe projection and unsafe-conflict visibility on both peers within 120 seconds;
- exactly two distinct heads for each experiment;
- `arbitrary_winner_selected: false` before authority exists;
- independent buyer and seller approval actions bound to the exact unsafe head set;
- identical `authorized_resolved` shipped projection on both peers;
- preservation of cancelled as superseded evidence;
- non-fixture evidence bound to the exact source revision, hApp, DNA, and client version.

## Run

Build the network artifact profile:

```sh
scripts/build-promotion-artifacts.sh network /tmp/marketplace-network-promotion
```

Provide controlled-runner hooks and execute:

```sh
export MARKETPLACE_NETWORK_SERVICES=/absolute/path/to/network-services-hook
export MARKETPLACE_NETWORK_CONTROL=/absolute/path/to/network-control-hook
scripts/run-network-promotion.sh /tmp/marketplace-network-promotion
```

Seal the result with the same Ed25519 release envelope used by other profiles:

```sh
export MARKETPLACE_RELEASE_SIGNING_KEY=/secure/marketplace-release-key.pem
scripts/seal-promotion-bundle.sh network /tmp/marketplace-network-promotion \
  /tmp/mycelix-marketplace-network.tar.gz
scripts/verify-promotion-bundle.sh /tmp/mycelix-marketplace-network.tar.gz
```

## Evidence contents

A network evidence directory includes:

- `network.json` and `promotion.json`;
- separate seller and buyer conductor logs;
- separate conductor configuration digests;
- network service and topology receipts;
- service and control hook digests;
- exact hApp/DNA/source/client bindings;
- aggregate SHA-256 ledger.

Tokens, private keys, actor environment files, and mutable conductor databases are forbidden from the signed release bundle.

## Claims this does not establish

A passing network profile does not establish:

- internet-scale or geographically distributed convergence;
- availability through arbitrary or prolonged partitions;
- general automatic conflict resolution;
- arbitration-authorized network convergence;
- Byzantine tolerance at any percentage;
- correctness or security of bootstrap/signaling implementations beyond the recorded run;
- security of the controlled runner or partition hook;
- production readiness.

It establishes only that this exact build, under the recorded two-conductor controlled topology, propagated shared state, preserved an unsafe conflict without arbitrary canonicalization, propagated two independent party approvals, and exposed the same authority-bound projection on both peers while retaining the original branches.

This is transaction conflict policy v2 and network evidence contract version 3. It does not establish general semantic merge, rollback of external effects, arbitration-authorized network convergence, fault attribution, or legal enforceability.
