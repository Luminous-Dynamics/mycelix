# Pulse public source-chain restore — bounded model-check receipt

**Result:** PASS for the stated finite abstract restore contract and exact run  
**Workflow:** [Pulse Authority Lifecycle Model, run #346](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42367109650)  
**Artifact:** [TLC logs, counterexamples and receipt](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42367109650/artifacts/13571254803), ID 13571254803  
**Artifact ZIP SHA-256:** `1f508e68a4740634745a017544151100158a7dccbef343f1b58e1b5d47e5ed06`  
**Exact PR head:** `acd9ed7e6d4779e17b936ab5589d58ebfb7bdc39`  
**PR base:** `9e12dd46bdd8371e8ed9626a17bc06f21c141a52`  
**TLC:** 1.8.0 / build `2026.08.11.125311`  
**TLC JAR SHA-256:** `ab323b79802aedc3203b3f9af37c6aca3ed43f4e0225b36f2aa77b26de46c05f`  
**Java:** Temurin OpenJDK 21.0.13 LTS

## Restore-model results

| Configuration | Result | Generated / distinct states | Invariant |
|---|---|---:|---|
| `PulseChainRestore.cfg` | PASS; state queue exhausted | 889 / 440 | No invariant error |
| `PulseChainRestore-early-enable-mutant.cfg` | Expected counterexample | 6 / 6 | `EnableRequiresAllCellsRestored` |
| `PulseChainRestore-no-peer-empty-mutant.cfg` | Expected counterexample | 6 / 6 | `EnableRequiresAllCellsRestored` |
| `PulseChainRestore-missing-key-mutant.cfg` | Expected counterexample | 2 / 2 | `RestoreRequiresOriginalKey` |
| `PulseChainRestore-legacy-grant-mutant.cfg` | Expected counterexample | 54 / 35 | `HostCapabilityMustBeFreshlyReissued` |

The workflow only accepts mutant checks when TLC exits non-zero and the expected invariant name appears in the log. A parser failure or unrelated invariant is not accepted as a successful negative control.

## Modeled contract

The model distinguishes:
- possession of the original AgentPubKey private key;
- restore request and `AwaitingRestore` state;
- peer availability;
- fetched actions versus validated/restored public actions;
- completion of each cell's required public action set;
- explicit app enable;
- non-restoration of private entries;
- non-restoration of host grants;
- fresh capability issuance after the app has been enabled.

The model includes two cells and two valid public actions plus one invalid action. Public action validation is modeled separately from network retrieval. The app can only be enabled after every required action for every cell has been restored. Private entries remain absent. A host grant can become active only after explicit fresh issuance.

## New negative controls

1. **Premature enable:** the mutant enables while one or more cell action requirements remain incomplete. TLC catches the violation of `EnableRequiresAllCellsRestored`.
2. **No-peer empty-success:** the mutant treats an unavailable peer and incomplete restore as a successful empty chain. The same enable invariant catches it.
3. **Missing original key:** the mutant starts restore without the original AgentPubKey's private key. `RestoreRequiresOriginalKey` catches it.
4. **Legacy grant resurrection:** the mutant treats restored application metadata as a live system CapGrant without issuing a fresh host grant. `HostCapabilityMustBeFreshlyReissued` catches it.

## Full-suite regression

The same run also re-executed the earlier authorization, mailbox, checkpoint, signal-trust, lineage, causal-frontier, trust-root and host-call-linearization model families. The workflow job completed successfully; all safe configurations exhausted their finite queues and all intended mutants were caught.

This confirms that adding chain restoration did not silently remove the existing model gates. Detailed state counts for those models remain in `pulse-authority-model-check-2026-11-06.md` and their prior exact receipts.

## Boundaries

This is a finite abstract state-space result, not a Holochain runtime test. It does not exercise actual `AdminRequest::InstallApp`, `restore_from_dht`, Lair key custody, DHT peer discovery, signed source-chain actions, entry-hash verification, validation receipts, real per-cell chain restoration, conductor restart, `AwaitingRestore` status transitions or `enable_app`.

It does not prove that private data, CapSecrets or CapClaims are recoverable; the model explicitly keeps private entries absent and requires host grants to be issued anew. It is not a 0.6 database migration and does not establish 0.7.0→0.7.1 capability compatibility.

**Claim ceiling:** bounded TLC validation of this exact model/configuration set. Real 0.7.1 multi-conductor restoration and capability re-issuance remain open under [#5905](https://github.com/Luminous-Dynamics/mycelix/issues/5905).
