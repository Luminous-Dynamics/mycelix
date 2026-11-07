# Pulse 0.7.1 source-chain restoration — bounded model-check receipt

**Result:** PASS for the finite abstract restore model and exact model-check run  
**Workflow:** [Pulse Authority Lifecycle Model, run #346](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42367109650)  
**Job:** `128387371232` — Safe model and negative-control counterexamples  
**Artifact:** [TLC logs, counterexamples and run receipt](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42367109650/artifacts/13571254803)  
**Artifact ID:** `13571254803` (52,029 bytes)  
**Artifact ZIP SHA-256:** `1f508e68a4740634745a017544151100158a7dccbef343f1b58e1b5d47e5ed06`  
**Exact PR head:** `acd9ed7e6d4779e17b936ab5589d58ebfb7bdc39`  
**TLC:** 1.8.0 / build `2026.08.11.125311`  
**TLC JAR SHA-256:** `ab323b79802aedc3203b3f9af37c6aca3ed43f4e0225b36f2aa77b26de46c05f`

## Restore-model result

| Configuration | Result | States generated | Distinct states | Detected invariant |
|---|---|---:|---:|---|
| `PulseChainRestore.cfg` | PASS | 368 | 136 | Queue exhausted; no invariant error |
| `PulseChainRestore-early-enable-mutant.cfg` | Expected counterexample | 6 | 6 | `EnableRequiresAllCellsRestored` |
| `PulseChainRestore-no-peer-empty-mutant.cfg` | Expected counterexample | 6 | 6 | `EnableRequiresAllCellsRestored` |
| `PulseChainRestore-missing-key-mutant.cfg` | Expected counterexample | 2 | 2 | `RestoreRequiresOriginalKey` |
| `PulseChainRestore-legacy-grant-mutant.cfg` | Expected counterexample | 54 | 35 | `HostCapabilityMustBeFreshlyReissued` |

All five workflow steps completed successfully. Each negative-control step only succeeds when TLC exits non-zero and the expected invariant name is present in its log. The workflow job also reran the lifecycle, mailbox, checkpoint, signal-trust, lineage, causal-frontier, trust-root and host-call linearization models; the full 26-phase table is recorded below.

## Encoded recovery contract

The model represents a fresh 0.7.1 install restoring public source-chain actions from DHT peers. It keeps separate:

- original AgentPubKey private-key availability;
- restore request and `AwaitingRestore` status;
- peer availability;
- fetched actions versus validator-accepted public actions;
- completion of every cell chain;
- explicit application enable;
- private-entry state;
- host capability state and fresh grant re-issuance.

The safe model requires the original agent key, keeps the app awaiting restoration while any required cell-chain action is missing, and requires each restored public action to pass validation. It never restores private entries or host CapGrants. A host grant must be freshly issued after restore.

The negative controls demonstrate four distinct failures:

1. enabling the app before every required cell chain is restored;
2. treating “no peers available yet” as successful empty restoration;
3. starting restoration without the original AgentPubKey key;
4. resurrecting legacy application capability metadata as though it were a live host CapGrant.

## Full model campaign on the same exact head

The same job executed nine safe models and seventeen negative controls. Every safe model exhausted its finite state queue without an invariant error, and every negative control was caught by its named invariant.

| Model family | Safe generated / distinct | Negative controls caught |
|---|---:|---|
| Authorization grant/revoke lifecycle | 83,941 / 10,249 | Stale projection: `NoProjectionAuthority`; duplicate grant retry: `GrantIdentityInjective` |
| SharedMailbox transition validation | 37 / 20 | Proposed-state self-promotion: `OnlyOriginalOwnerMayUpdate`; field smuggling: `ProtectedFieldsRemainImmutable` |
| AuthorityCheckpoint freshness | 27 / 17 | Checkpoint bypass: `NoFalseFreshAuthority` |
| Remote signal trust/replay | 21 / 12 | Forged signal: `NoUnverifiedSignalMutatesTrustedState`; replay: `AtMostOneEffectPerLogicalEvent` |
| Lineage-bound checkpoint | 31 / 14 | Foreign lineage: `NoForeignLineageCanBeTrusted` |
| Multi-chain causal frontier | 2,637 / 544 | Scalar-height shortcut: `NoFalseFreshCausalFrontier` |
| Trust-root and key lifecycle | 8 / 4 | Self-root: `OnlyPinnedRootCanAuthorizePolicy`; rollback: `PolicyEpochNeverRollsBack`; revoked key: `NoRevokedKeyAcceptedUnderCurrentPolicy` |
| Host-call/revocation linearization | 889 / 440 | Projection-based race: `CallDecisionIsLinearized` |
| Public source-chain restore | 368 / 136 | Premature enable / no-peer empty success: `EnableRequiresAllCellsRestored`; missing key: `RestoreRequiresOriginalKey`; legacy grant resurrection: `HostCapabilityMustBeFreshlyReissued` |

The job's successful conclusion means the safe configurations returned zero and the mutant checks each verified the intended non-zero TLC exit plus the expected invariant name.

## Boundaries and claim ceiling

This establishes that the exact finite TLA+ models preserve their encoded invariants and detect the selected mutants. It does **not** execute Holochain's real `AdminRequest::InstallApp { restore_from_dht: true }`, Lair key custody, DHT peer discovery, actual validation-receipt signatures, interrupted conductor recovery, or `enable_app`.

It does not prove that any Mycelix application has restored successfully, that private data is recoverable from DHT, or that 0.7.0 capability grants survive 0.7.1. Those runtime requirements remain open under [#5905](https://github.com/Luminous-Dynamics/mycelix/issues/5905) and the broader 0.7.1 qualification contract [#5719](https://github.com/Luminous-Dynamics/mycelix/issues/5719).

**Claim ceiling:** bounded model-check evidence for the exact TLA+ source/configuration set at PR head `acd9ed7e6d4779e17b936ab5589d58ebfb7bdc39` using the pinned TLC build. It is not a conductor test, Sweettest, DHT interoperability test, cryptographic audit, or production qualification.
