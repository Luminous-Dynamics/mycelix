# PRISM × Holochain Evidence Fabric

**Status:** Research proposal
**Date:** 2026-10-06
**Scope:** PRISM qualification/evidence federation and future browser/runtime integration.

## Executive decision

Use Holochain as a distributed evidence and policy-lineage plane around PRISM, never as the online kernel-enforcement authority.

Critical rule:

    Holochain unavailable
        !=
    PRISM unable to install an already locally-qualified policy

A browser or renderer must be able to enforce a locally verified PRISM policy without network access or DHT availability.

Holochain can make surrounding evidence durable, multi-agent, auditable, challengeable, and federated.

## Why this composition fits Holochain

Holochain source chains are signed local histories and its DHT validates and replicates public operations. Its validation model is deterministic, while DHT state is not global consensus and validation receipts are only evidence of recent validation/availability rather than permanent finality. See the Holochain validation, DHT, and validation-receipt documentation.

Therefore:

    local policy inputs
          |
          v
    PRISM compiler
          |
          v
    local structural + semantic validator
          |
          v
    kernel installation
          |
          +----> local evidence receipt
                          |
                          v
                   optional Holochain
                          |
                   distributed evidence

Never make a DHT observation equivalent to local kernel authorization.

## Comparison with Ladybird

Ladybird currently has the more mature browser stack and a proven layered security architecture. In 2026 it has continued moving browser subsystems such as style/layout/painting toward Rust, while keeping JavaScript, multiple processes, broker validation, seccomp, Landlock, and compositor/process isolation. Its threat model explicitly assumes a renderer can be compromised and requires other processes to treat renderer messages as untrusted.

Our target is not to beat Ladybird at browser completeness.

Ladybird:

    mature web compatibility
    Rust migration
    multi-process browser
    JS runtime
    seccomp + Landlock
    browser-centric security model

Luminous:

    capability-native application model
    no-JavaScript high-assurance path
    WIT/component interfaces
    explicit authority supervisor
    provenance-bound PRISM policy
    distributed qualification evidence
    Mycelix/Holochain evidence and governance plane

The strongest combined result is therefore complementary rather than competitive: use Ladybird/Servo as implementation and compatibility references, while making capability semantics and evidence unusually strong.

## Novel Holochain use 1: distributed PRISM qualification attestations

A PRISM policy can be qualified independently on different architectures, kernels, and operator environments.

Represent each attestation as a signed statement containing:

- policy input digest;
- policy digest;
- emitted bytecode digest;
- structural-validator domain;
- semantic-validator domain;
- compiler build identity;
- dependency lock identity;
- architecture and kernel profile;
- qualification profile;
- test inventory digest;
- evidence artifact digest;
- result;
- attestor;
- issued-at and optional expiry.

Holochain can validate the record shape, authorship, references, and status transitions while preserving independent attestations.

Example:

    policy P
      |
      +-- x86_64 native attestation
      +-- AArch64 native attestation
      +-- RISC-V compile attestation
      +-- independent operator reproduction

This is stronger than a scalar trust score because each evidence class remains explicit.

## Novel Holochain use 2: policy lineage and supersession

Store a graph of policy generations, supersession, revocation, and replacement:

    P0 -> P1 -> P2
               |
               +-> revoked

The distributed layer can answer who authored and attested a policy generation and which later artifact superseded it.

PRISM still answers the local question: what exact filter is installed?

This separation prevents historical evidence from being mistaken for current enforcement.

## Novel Holochain use 3: distributed incident and quarantine intelligence

A renderer, broker, runtime, or policy incident can be expressed as a signed report bound to exact artifact identities.

Example fields:

- bad runtime digest;
- bad policy digest;
- observed escape attempt class;
- renderer assignment identity;
- qualification context;
- incident evidence digest;
- reporting agent.

Holochain's validation and warrant model is interesting here because invalid network behavior can generate signed evidence that other peers use in local defensive decisions.

The safe PRISM interpretation is:

    distributed incident signal
          |
          v
    local verifier
          |
          +--> mark artifact untrusted
          +--> quarantine future launches
          +--> require fresh qualification

Do not weaken an already-installed local sandbox because a remote DHT observation disappeared or claimed a contradictory state.

## Novel Holochain use 4: privacy-preserving execution provenance

Do not publish browsing content, URLs, cookies, raw renderer traces, or raw syscall transcripts to the public DHT.

Publish commitments instead:

- content commitment;
- component commitment;
- capability-manifest commitment;
- runtime commitment;
- browser-build commitment;
- PRISM policy commitment;
- qualification commitment.

Sensitive evidence remains local or in explicitly authorized private storage.

This lets an operator prove that an execution used an identified qualified artifact set without publishing the user's browsing history.

## Novel Holochain use 5: distributed governance of qualification authorities

Use the Mycelix DKG/trust-governance plane to define which agents may issue which classes of PRISM attestation.

Example:

    PRISM-LINUX-RENDERER-V1
      requires:
        native x86_64
        native AArch64
        component qualification
        cross-architecture qualification
      optional:
        independent operator reproduction

The DKG governs the verifier set or threshold for a named profile.

It must never become a universal numeric verdict such as 9/10 trust = safe.

## Novel Holochain use 6: countersigned high-risk policy bundles

Holochain countersigning is useful when several parties must jointly authorize one exact artifact.

Possible applications:

- enterprise-managed privileged browser mode;
- device-control capabilities;
- cross-organization compute access;
- administrator workflows;
- shared machine policy.

The resulting countersigned artifact can be an input into local PRISM policy construction.

Holochain is not the installer and does not become part of the kernel path.

## Novel Holochain use 7: capability declaration registry

A future application can publish a signed declaration of the capabilities it requests:

- component identity;
- requested WIT world/interfaces;
- requested capability classes;
- expected resource budgets;
- publisher identity.

The browser treats this as a request, never an authorization.

Local policy and explicit user/administrator controls remain authoritative.

## What must remain local

Never require Holochain for:

- seccomp attachment;
- Landlock installation;
- process creation;
- privilege reduction;
- renderer termination;
- local capability revocation;
- origin authorization;
- browser startup;
- emergency containment.

The machine must remain secure while offline.

## PRISM improvement 1: proof-carrying policy atoms

Every emitted allow path should be traceable to a canonical policy atom:

    capability
      + syscall
      + argument predicate
      + clause identity
      + semantic justification
      = allow path

Reject any ALLOW that has no declared semantic owner.

This is stronger than only validating cBPF control-flow correctness.

## PRISM improvement 2: authority-delta reporting

For every policy revision produce a deterministic delta:

- added syscalls;
- removed syscalls;
- changed predicates;
- changed architectures;
- changed policy-domain versions;
- changed evidence requirements.

This makes security review bounded and comparable across revisions.

## PRISM improvement 3: independent reference semantics

Maintain a small reference evaluator independent from the emitter.

Qualification becomes:

    policy
      -> reference semantics
      -> emitted-bytecode semantics
      -> native kernel probes

The three layers must agree where they are expected to agree.

## PRISM improvement 4: negative-space coverage

Do not only test named forbidden syscalls.

Systematically test nearby authority:

- same syscall with forbidden arguments;
- neighboring syscalls in the same privilege family;
- alternate fd types;
- new kernel interfaces;
- malformed but structurally valid bytecode;
- legal-looking jumps into later ALLOW paths.

The goal is to find unintended authority, not simply confirm the deny list.

## PRISM improvement 5: kernel-interface freshness

Bind qualification to a kernel interface profile, including:

- architecture ABI;
- syscall-number mapping;
- seccomp assumptions;
- Landlock ABI assumptions;
- required kernel feature probes.

A policy qualified against one interface profile must not silently inherit another profile's qualification.

## PRISM improvement 6: qualification freshness

Every qualification artifact should identify:

- exact policy identity;
- exact compiler identity;
- exact dependency identity;
- exact runtime identity;
- kernel profile;
- issued-at;
- expiry or freshness class.

Historical qualification remains useful historical evidence, but it should not masquerade as current qualification.

## PRISM improvement 7: explicit syscall justification closure

Every allowed syscall should be attributable to a browser subsystem or runtime service.

Illustrative model:

    read    <- renderer IPC input
    write   <- renderer IPC output
    mmap    <- runtime memory
    futex   <- runtime synchronization

An ALLOW with no justification is a qualification failure.

This provides a concrete path toward policy minimization without automatically broadening the sandbox from observed behavior.

## PRISM improvement 8: monotonic composition theorem

Adding a sandbox layer should never increase effective authority.

For layers A and B:

    effective(A + B) <= effective(A)
    effective(A + B) <= effective(B)

The exact formalization depends on the authority lattice, but the implementation should test monotonicity explicitly.

## PRISM improvement 9: policy mutation harness

Generate mutations over:

- syscall numbers;
- architecture tags;
- argument offsets;
- masks;
- predicate operator;
- jump targets;
- clause boundaries;
- final actions;
- policy metadata;
- evidence-domain identifiers.

For every mutation the verifier should either reject it or produce a different committed identity.

This extends the mutation discipline already proving valuable in the V2 seccomp work.

## PRISM improvement 10: no silent compatibility widening

Backward compatibility is itself a security decision.

When a new policy format or browser API requires a compatibility exception, the exception should:

1. have an explicit versioned identifier;
2. appear in the capability/policy identity;
3. have dedicated positive and negative tests;
4. have an explicit expiry/removal plan;
5. never silently alter the meaning of an existing policy digest.

## Recommended architecture

    Browser
       |
       v
    capability semantics
       |
       v
    Authority Supervisor
       |
       +---- WIT/component runtime
       |
       +---- broker authorization
       |
       +---- information-flow policy
       |
       v
    PRISM
       |
       +---- seccomp
       +---- Linux enforcement
       |
       v
    kernel

    Mycelix/Holochain
       ^
       |
    evidence / lineage / incidents / attestors / governance

The Holochain plane is adjacent to the security boundary, not inside it.

## Architecture target relative to Ladybird

Ladybird is a compelling reference implementation for:

- browser process topology;
- Rust migration strategy;
- memory-safe layout/style/painting;
- compositor isolation;
- seccomp/Landlock layering;
- hostile-renderer threat modeling.

The Luminous architecture should add:

- no-JavaScript high-assurance execution;
- WIT-native application capabilities;
- explicit capability attenuation;
- capability-bound information flow;
- provenance-bound policy identity;
- distributed qualification evidence;
- decentralized policy/incident lineage.

That is a materially different target from 'Ladybird but with more Rust.'

## Hard non-goals

Holochain must not:

- decide whether a seccomp filter may be installed;
- replace kernel enforcement;
- become a global permission oracle;
- receive private browsing telemetry by default;
- turn attestation counts into a universal safety score;
- resolve contradictory runtime authority by majority vote.

PRISM must not:

- become dependent on DHT availability;
- interpret distributed evidence as kernel truth;
- silently broaden policies from observed syscalls;
- conflate component, kernel, and canonical full-workspace evidence.

## Immediate implementation sequence

1. Finish PRISM V2 exact-head qualification without perturbing its current evidence topology.
2. Add a standalone policy-atom / syscall-justification model and mutation harness.
3. Define PrismQualificationAttestationV1 using the existing Mycelix canonical signing machinery.
4. Add a Holochain metadata-only evidence zome that stores attestations, supersession, revocation, and incident claims.
5. Add a deterministic projection layer that produces a local evidence snapshot without treating it as global finality.
6. Build the browser Phase-0 capability runtime against the same canonical identity/evidence principles.
7. Only then connect the capability runtime to real WIT resources and a minimal Wasm interpreter.

## Final thesis

PRISM should be the local proof-oriented OS authority reducer.

Holochain should become the decentralized memory of the security system:

- who asserted what;
- which exact artifact was qualified;
- which environment produced the evidence;
- which evidence was independently reproduced;
- what was later revoked or superseded;
- which incidents challenged an artifact.

That gives us something Ladybird, Chromium, and conventional browser sandboxes do not attempt to provide as their central abstraction:

**local enforcement with distributed, cryptographically bound security history — without making the distributed network a prerequisite for local safety.**
