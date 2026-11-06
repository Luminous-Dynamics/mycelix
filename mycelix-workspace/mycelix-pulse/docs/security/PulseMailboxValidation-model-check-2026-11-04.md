# Pulse SharedMailbox integrity — bounded model-check receipt

**Result:** PASS for the stated finite abstract transition model and exact model-check run  
**Workflow:** [Pulse Authority Lifecycle Model, run #32](https://github.com/Luminous-Dynamics/mycelix/actions/runs/41852865450)  
**Artifact:** [TLC logs and counterexamples](https://github.com/Luminous-Dynamics/mycelix/actions/runs/41852865450/artifacts/13335762572), ID 13335762572  
**PR head checked:** 256a47a5f62e63e4796024c02075ab82a1024a08  
**PR base:** 9e12dd46bdd8371e8ed9626a17bc06f21c141a52  
**Merge checkout:** 3c7b802b37f052675e29f044f4c01798959b9489  
**TLC:** 1.8.0 / build 2026.08.11.125311  
**TLC JAR SHA-256:** ab323b79802aedc3203b3f9af37c6aca3ed43f4e0225b36f2aa77b26de46c05f

## Results

| Configuration | Result | States generated | Distinct states | Outcome |
|---|---|---:|---:|---|
| PulseMailboxValidation.cfg | PASS | 37 | 20 | Safe state queue exhausted; no invariant error |
| PulseMailboxValidation-proposed-state-mutant.cfg | Expected counterexample | 10 | 10 | OnlyOriginalOwnerMayUpdate violated |
| PulseMailboxValidation-field-mutation-mutant.cfg | Expected counterexample | 13 | 13 | ProtectedFieldsRemainImmutable violated |

The workflow only passes the two mutant steps if TLC exits non-zero and the expected invariant name appears in the log. A generic parse failure does not satisfy those checks.

## Repeat run

The same mailbox model configurations were rerun in model workflow [run #36](https://github.com/Luminous-Dynamics/mycelix/actions/runs/41853504791), against PR head `d31079d4606e05268e33d4ea2bb310bc6bb99a93` (merge checkout `dec4aeef`). The safe model again completed 37 generated / 20 distinct states; the proposed-state mutant again violated `OnlyOriginalOwnerMayUpdate`; and the field-smuggling mutant again violated `ProtectedFieldsRemainImmutable`.

Artifact ID `13336008629`, ZIP SHA-256 `421090d8fee0115e78b3ce436ebfa5399bd0c6f6a6d24071dd899381af09de41`. This reproduces the same finite scope, not additional scenario coverage.

## Repeat run

The safe and mutant mailbox configurations reproduced their prior outcomes in [model workflow run #60](https://github.com/Luminous-Dynamics/mycelix/actions/runs/41857220290), on PR head `34b1d695b8ce7d5bc878ee9d3025fa738514f1d2` (merge checkout `8e4943ff`). Safe: 37 generated / 20 distinct states, empty queue; proposed-state mutant: `OnlyOriginalOwnerMayUpdate`; protected-field mutant: `ProtectedFieldsRemainImmutable`.

Artifact ID `13337303730`, ZIP SHA-256 `17da4b4450375fd6c262aa59f80d680954f297a2a8cd4a55b8a2b542995ca36a`. Same bounds and model logic; this does not expand scenario coverage.

## Modeled security contract

The model starts with an Owner and an existing Admin. A deliberately permissive coordinator may author any proposed operation. The receiving validator is modeled independently and enforces:

- direct SharedMailbox transitions require the original owner;
- the action is exactly one member append from the current base version;
- the added member's added_by matches the action author;
- duplicate members and assigning a new Owner role are rejected;
- protected mailbox fields are unchanged;
- remote observer membership remains a subset of validator-integrated canonical membership.

The proposal-authoring step is separate from validation and integration. The safe model explores attacker self-promotion, existing Admin direct addition, a legitimate owner append, and a legitimate-looking append with a protected-field mutation.

## Counterexamples

### Proposed-state self-authorization

The mutant grants authority when the author appears as an Admin in the proposed state. TLC found the attacker's self-append-as-Admin schedule, then violated OnlyOriginalOwnerMayUpdate when the forged update was integrated.

### Protected-field smuggling

The mutant ignores the protected-field-mutation bit. TLC found an otherwise valid owner member append bundled with a protected field mutation, then violated ProtectedFieldsRemainImmutable.

## Boundaries

This is a finite abstract model with four agents, a small role set, one pending proposed update at a time, and a single mailbox. It does not simulate Holochain's actual Op validation pipeline, source-chain cryptographic signatures, validation receipts, DHT gossip, forks from competing signed source-chain updates, network partitions, or client behavior. It does not prove the production integrity WASM is correctly wired to this model.

The runtime proof remains [issue #5781](https://github.com/Luminous-Dynamics/mycelix/issues/5781): use two conductors, a deliberately permissive attacker coordinator, unchanged production integrity WASM, source-chain action evidence, remote validator disposition, and an independent observer's canonical mailbox state.

**Claim ceiling:** bounded model-check evidence for this exact TLA+ specification/configuration and TLC build only; not a Sweettest, conductor run, general Byzantine or linearizability proof, or production qualification.
