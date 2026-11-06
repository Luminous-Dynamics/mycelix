# Pulse authorization assurance models — exact-head TLC receipt

**Workflow:** [Pulse Authority Lifecycle Model, run #283](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42215970543)  
**Artifact:** [TLC logs, counterexamples and receipt](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42215970543/artifacts/13501137151), ID `13501137151`  
**PR:** [#5771](https://github.com/Luminous-Dynamics/mycelix/pull/5771)  
**Exact PR head:** `8b9a2de05c1c562e6f6135fc2645679e756a6cff`  
**Merge checkout:** `3c2725ca` (workflow checkout; merge of PR head into the base recorded by GitHub Actions)  
**TLC:** 1.8.0, build `2026.08.11.125311`  
**TLC JAR SHA-256:** `ab323b79802aedc3203b3f9af37c6aca3ed43f4e0225b36f2aa77b26de46c05f`

## Exact model results

| Model | Safe generated / distinct states | Safe result | Mutant result |
|---|---:|---|---|
| Authorization grant/revoke lifecycle | 83,941 / 10,249 | Queue exhausted; no invariant error | Projection authority caught by `NoProjectionAuthority`; duplicate grant retry caught by `GrantIdentityInjective` |
| SharedMailbox validation | 37 / 20 | Queue exhausted; no invariant error | Proposed-state self-promotion caught by `OnlyOriginalOwnerMayUpdate`; protected-field smuggling caught by `ProtectedFieldsRemainImmutable` |
| AuthorityCheckpoint freshness | 27 / 17 | Queue exhausted; no invariant error | Stale projection treated as fresh caught by `NoFalseFreshAuthority` |
| Remote-signal trust/replay | 21 / 12 | Queue exhausted; no invariant error | Forged signal caught by `NoUnverifiedSignalMutatesTrustedState`; replayed logical effect caught by `AtMostOneEffectPerLogicalEvent` |
| Lineage-bound checkpoint | 31 / 14 | Queue exhausted; no invariant error | Equal epoch from foreign lineage caught by `NoForeignLineageCanBeTrusted` |
| Two-chain causal frontier | 2,637 / 544 | Queue exhausted; no invariant error | Scalar-height / cross-chain sum shortcut caught by `NoFalseFreshCausalFrontier` |

All six safe model configurations completed with zero states left on queue and no invariant error. All eight intentional mutants produced the named invariant counterexample; the workflow's negative-control steps passed only when the mutant process failed and the expected invariant name was present in its log.

## Counterexample coverage

The run exercises these distinct bad abstractions:

1. stale application/DHT projection is not host authority;
2. retry after lost grant acknowledgement must not mint a second live host grant for one logical issuance request;
3. proposed membership cannot authorize its own Admin promotion;
4. a valid mailbox member append cannot smuggle unrelated protected-field mutation;
5. a freshness checkpoint that has not been observed must produce `UNKNOWN`, not trusted active state;
6. remote hints cannot mutate trusted state without verified backing evidence;
7. replay cannot create a second logical effect for one event;
8. matching numeric generation/epoch is not sufficient when the DNA/cell/source-chain lineage is foreign;
9. independent source-chain frontiers must be compared componentwise and with ancestry; scalar summation can manufacture freshness.

## Claim ceiling

These are finite abstract TLA+ models. They do not execute the Holochain conductor, Sweettest, HDK/HDI validators, DHT gossip, network partitions, production grant storage, cryptography or browser/client code. The run establishes that the *encoded finite models* preserve their specified invariants and that the selected mutants are detected. It does not prove unbounded linearizability, Byzantine consensus, general partition safety, production revocation latency, or runtime correctness.

The production-runtime tranche remains open under [#5772](https://github.com/Luminous-Dynamics/mycelix/issues/5772); malicious coordinator and receiving validator runtime proof remains open under [#5781](https://github.com/Luminous-Dynamics/mycelix/issues/5781).
