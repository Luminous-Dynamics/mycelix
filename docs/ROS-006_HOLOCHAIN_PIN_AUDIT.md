# ROS-006 Holochain dependency pin audit

Audit date: 2026-10-01  
Scope: repository manifests and the committed mycelix-civic/Cargo.lock inspected on branch feat/ros-006-qualification-boundary-v2.  
Purpose: avoid assuming that a repository-wide Holochain upgrade is a local ROS-006 change.

## Findings

| Location | Declared Holochain versions | Observation |
| --- | --- | --- |
| crates/mycelix-core-types/Cargo.toml | optional HDI =0.7.1, holo_hash =0.6.1, holochain_integrity_types =0.6.1, hdk_derive =0.6.1 | Explicit optional host integration; hdk is not declared in this manifest. |
| mycelix-workspace/Cargo.toml | HDK/HDI/Holochain family pinned to 0.6.1 / 0.7.1 | Main workspace dependency set has an explicit coordinated 0.6.1-era pin set. |
| mycelix-civic/Cargo.toml | HDK =0.6.1, HDI =0.7.1, related types =0.6.1 | Explicit exact pins. |
| mycelix-civic/Cargo.lock | HDK 0.6.1, HDI 0.7.1, hdk_derive / holo_hash / holochain_integrity_types / holochain_zome_types 0.6.1 | The inspected civic lockfile resolves the declared versions consistently. |
| mycelix-commons/Cargo.toml | HDK 0.6.0, HDI 0.7.0, integrity types 0.6.0; holo_hash uses compatible 0.6 range | Commons has a distinct, older workspace pin set. |

## Compatibility interpretation

The official Holochain 0.6 compatibility table currently lists Holochain/HDK 0.6.3 and HDI 0.7.3 as the latest compatible versions, and describes 0.6 as maintenance-mode. That is a statement about the latest compatible set, not proof that every earlier 0.6.x set is inherently invalid.

The inspected civic lockfile confirms that the civic workspace resolves its exact 0.6.1 / 0.7.1 family consistently. The important repository-level issue is therefore workspace divergence and maintenance intent, not a demonstrated compile failure in the 0.6.1 pins.

The commons workspace declares 0.6.0-era dependencies, while the main workspace and civic workspace declare 0.6.1-era dependencies. Since these are separate workspaces, their manifests and lockfiles must be evaluated independently. A successful resolution in one workspace does not establish compatibility for another.

## ROS-006 implications

1. Keep mycelix-core-types independent of HDK/HDI by default. Its optional integration dependencies should not leak into the default qualification core.
2. Before implementing a Holochain adapter, choose its actual host workspace and inherit that workspace's pinned dependency set rather than inventing a new mixed set.
3. Do not blanket-bump all Holochain dependencies solely to match the latest compatibility table. Validate each workspace independently and account for DNA compatibility: Holochain's tooling guidance notes that integrity-zome dependency updates change the DNA hash, potentially creating a separate network/DHT identity.
4. Record the selected conductor, HDK, HDI, zome types, serialized-bytes, and relevant derive/hash versions together in an adapter-specific compatibility manifest.
5. Treat successful cargo check as compile evidence only. Add host-level/sweettest validation for actual must_get_* behavior and the ROS mapping of Valid, Invalid, and Unresolved.

## Recommended decision

For ROS-006, do not change dependency pins in this PR without a runnable, workspace-specific verification path and an explicit compatibility decision. The next implementation should target the eventual Relationship 360 integrity/coordinator workspace and inherit its chosen version matrix. If Relationship 360 is intentionally to use a newer Holochain release, treat that as a separate, deliberate DNA/version migration with its own test and compatibility evidence.

## Source

- Holochain 0.6 compatibility table: https://developer.holochain.org/resources/compatibility/holochain-0.6/
- Holochain tooling compatibility guidance, including DNA hash implications: https://developer.holochain.org/resources/compatibility/
- Holochain 0.6 upgrade guidance: https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.6/
