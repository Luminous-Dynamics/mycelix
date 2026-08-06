# Real-conductor scenario adapter protocol v1

The scenario runner deliberately separates **model evidence** from **distributed
conductor evidence**. A release may not substitute `model_adapter.py` for a real
adapter.

A real adapter is an executable accepting one of:

```text
adapter --capabilities
adapter --scenario <scenario-id>
```

Every invocation prints exactly one JSON object to stdout and exits nonzero on
harness, conductor, gossip, or assertion failure.

## Capabilities response

```json
{
  "adapter_protocol": "mycelix-conductor-scenario-adapter-v1",
  "adapter_kind": "real-conductor",
  "conductor_count": 3,
  "dna_hash": "<installed DNA hash>",
  "agents": ["verifier", "prover_a", "prover_b"],
  "evidence_bundle_hash": "<canonical SHA-256 of retained evidence>"
}
```

The conductors must use distinct databases and networking identities. Three
cells inside one conductor do not satisfy this contract.

## Scenario response

```json
{
  "adapter_protocol": "mycelix-conductor-scenario-adapter-v1",
  "adapter_kind": "real-conductor",
  "scenario_id": "partitioned-anchor-refresh-fork",
  "observed_state": "CredentialAnchorRefreshFork",
  "evidence": {
    "level": "multi-conductor",
    "bundle_hash": "<scenario evidence bundle hash>",
    "observed_by": ["verifier", "prover_a", "prover_b"]
  }
}
```

The evidence bundle should retain conductor logs, action hashes, partition and
heal timestamps, network state snapshots, zome-call inputs and outputs, DNA and
hApp hashes, and the final state observed by each conductor.

`run_scenarios.py --require-real` rejects adapters that identify themselves as a
model, expose fewer than three conductors, omit required agents, or fail to
provide evidence hashes.
