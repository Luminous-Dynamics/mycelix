# mycelix-content-policy

Pure fail-closed hard-policy qualification for Mycelix Content Fabric placement candidates.

The crate sits between the deterministic CF-05A state projection and any CF-06 optimizer. It deliberately contains no optimizer, Symthaea, Holochain, Iroh, filesystem, marketplace, billing, or execution logic.

Its job is narrow:

1. validate the storage intent, placement target, projection quality, and evaluation time;
2. reject candidates that cannot satisfy hard jurisdiction, encryption, retention, or required provider-policy evidence;
3. count only sufficiently assured failure-domain facts;
4. refuse to construct a `PolicyQualifiedPoolV1` unless the surviving pool can satisfy minimum replicas and required failure-domain diversity;
5. validate any later planner-selected subset before it can be considered policy-valid.

`HardPolicyGateConfigV1::strict()` requires a locally complete queried-index snapshot, a clean projection, and `IndependentlyAttested` provider facts.

## Trust boundary

This crate **does not verify attestation signatures or mint assurance levels**. An upstream verification adapter is responsible for mapping evidence into `PolicyAssuranceV1`. Raw CF-05 provider self-claims must remain `SelfClaimed`; relabeling them as independently attested would defeat the boundary.

## Non-goals

- scoring or ranking providers;
- deciding cost/latency/energy tradeoffs;
- creating leases;
- authorizing Iroh reads;
- writing CAS data;
- proving global Holochain finality;
- asserting that provider identity alone establishes failure-domain independence.
