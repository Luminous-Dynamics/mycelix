# MYC-INT-007K — Node Seed Package and Independent Bootstrap Contract

Status: design/fixture only. Child of MYC-INT-007I.

## Purpose

Turn the node-maturation story into an exact bootstrap subject that can be replayed and challenged.

The contract separates:

```text
seed package
!= receiving-node identity
!= local governance
!= local certification
!= federation membership
```

A mature node may export reusable capability, knowledge, software and selected physical support. The receiving node remains a new institutional subject.

## Two exact subjects

007K freezes:

1. `NodeSeedPackageV1` — what Node A offers to Node B;
2. `NodeBootstrapRunV1` — what Node B accepts, rejects, adapts, regenerates and proves during bootstrap.

The same contract must later support B → C without special-case logic.

## Seed package classes

The package may contain the 007I classes:

- `DesignPack`;
- `DeploymentPack`;
- `ConformancePack`;
- `OperationsPack`;
- `TrainingPack`;
- `PhysicalStarterPack`;
- `FederationIntroductionPack`.

Every component carries source generation/provenance and `authority = None`.

## Identity boundary

Fresh bootstrap requires new receiving-node identities and secrets.

At minimum distinguish:

```text
SeedPackageId
DeploymentPackageId
NodeIdentity
HostIdentity
ApplicationAgentIdentity
FederationIdentity
```

No private identity is derived from the source node or copied from the package.

`FreshNode` and `SameNodeRecovery` remain separate modes. 007K covers `FreshNode` only.

## Selective admission

Node B evaluates each offered component independently.

Allowed decisions:

- `AcceptAsIs`;
- `AcceptWithLocalAdaptation`;
- `Reject`;
- `Defer`;
- `MoreEvidenceRequired`.

A rejected optional component must remain absent.

Local adaptation creates a new local generation and declares translation/loss where applicable.

## Bootstrap independence theorem

The run must be able to establish, under the declared package/environment profile:

```text
bootstrap completed
AND seeder A unavailable
→ B retains declared local capabilities
```

This does not mean all capabilities are locally available. The run records which functions remain externally dependent.

## Federation boundary

Federation is a post-bootstrap local choice.

```text
bootstrap complete
!= federated

peer introduction
!= trust grant
!= authority grant
```

B must be able to operate its declared local profile before joining a federation.

## Certification / standing / balances

Imported artifacts can retain foreign evidence and certification metadata, but:

```text
foreign certification
!= local certification

foreign reputation
!= local standing

source-node credits/balances
!= receiving-node balances
```

007K forbids seed components that silently populate these domains.

## Evidence required from a bootstrap run

A run should retain evidence for:

- exact seed-package commitment/generation;
- exact deployment package/build inputs;
- component admission decisions;
- new identity/key-generation events without secret disclosure;
- local governance initialization/ownership;
- local adaptations and translation losses;
- rejected/deferred components;
- external dependencies after bootstrap;
- seeder availability transitions;
- local-operation interval while seeder is unavailable;
- federation enrollment, if later chosen;
- export of a second-generation seed package when testing N7.

## Recursive seeding

N7 is stronger than successful A → B bootstrap.

The eventual recursion test is:

```text
A seeds B
A becomes unavailable
B operates independently under its declared profile
B produces package K2 from B-owned state/artifacts
B seeds C
C generates fresh identity/governance
A is not required
```

Lineage/provenance may refer back to A-origin public artifacts. Authority does not.

## Nonclaims

007K does not establish physical/economic self-sufficiency, governance legitimacy, legal compliance, ecological sustainability, production deployment safety, or successful real-world community replication.