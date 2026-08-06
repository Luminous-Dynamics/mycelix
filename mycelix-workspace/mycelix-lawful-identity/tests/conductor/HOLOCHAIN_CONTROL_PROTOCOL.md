# Holochain control protocol v1

`holochain_real_driver.py` is the reviewed scenario orchestrator. It owns the
scenario contract, operation ordering, runtime dependency pinning, timeouts,
evidence retention, and final-state agreement. The control executable owns only
Holochain-specific mechanics:

- starting or connecting to three isolated conductors;
- admin/app websocket calls;
- partition and heal operations;
- time advancement in the test environment;
- returning raw action identities and final state observations.

The control executable is hash-pinned in the driver configuration. Its
capability response, executable bytes, scenario contract, conductor identities,
DNA identity, hApp identity, and Holochain version are jointly committed by the
driver `runtime_lock_hash`, which is then covered by the reviewed driver release.

## Commands

```text
control --capabilities
control --step --scenario <id> --step-name <step> --sequence <n>
control --observe --scenario <id> --conductor <id>
```

Each invocation must emit exactly one UTF-8 JSON object and exit nonzero on any
failed conductor operation. Unknown fields are rejected.

## Capabilities

```json
{
  "control_protocol": "mycelix-holochain-control-v1",
  "holochain_version": "0.6.1",
  "dna_hash": "uhC0k...",
  "happ_hash": "uhH0k...",
  "agents": ["verifier", "prover_a", "prover_b"],
  "conductors": [
    {
      "id": "verifier",
      "agent_pub_key": "uhCAk...",
      "database_identity_hash": "<canonical base64 SHA-256>",
      "network_identity_hash": "<canonical base64 SHA-256>"
    }
  ]
}
```

The driver compares this response byte-for-byte at the semantic JSON level with
its reviewed configuration before and after each scenario.

## Step response

```json
{
  "control_protocol": "mycelix-holochain-control-v1",
  "scenario_id": "partitioned-competing-proofs",
  "step": "submit-a",
  "sequence": 1,
  "at_unix_micros": 1700000000000000,
  "kind": "zome-call",
  "actor": "prover_a",
  "action_hashes": ["uhCkk..."],
  "artifacts": [
    {
      "path": "zome-call-response.json",
      "content_b64": "eyJvayI6dHJ1ZX0K"
    }
  ]
}
```

Artifacts are data, not filesystem paths supplied by the control process. The
driver canonicalizes, confines, size-bounds, writes, and hashes them itself.
A step may return no artifacts, but the complete scenario still retains request
and response JSON for every operation.

## Observation response

```json
{
  "control_protocol": "mycelix-holochain-control-v1",
  "scenario_id": "partitioned-competing-proofs",
  "conductor_id": "verifier",
  "state": "ChallengeConsumptionConflict",
  "observed_at_unix_micros": 1700000001000000,
  "action_hashes": ["uhCkk..."],
  "chain_head_action_hash": "uhCkk...",
  "source_chain_sequence": 42,
  "integrated_op_count": 120
}
```

Every configured conductor must independently return the contract's expected
state. Divergence fails the scenario rather than selecting a majority result.

## Driver configuration

The driver reads `MYCELIX_HOLOCHAIN_DRIVER_CONFIG`. The file is strict JSON:

```json
{
  "schema": "mycelix-holochain-real-driver-config-v1",
  "control_protocol": "mycelix-holochain-control-v1",
  "control_executable": "/absolute/path/to/reviewed-control",
  "control_executable_sha256": "<canonical base64 SHA-256>",
  "scenario_contract": "/absolute/path/to/settlement_scenarios_v2.json",
  "scenario_contract_sha256": "<canonical base64 SHA-256>",
  "holochain_version": "0.6.1",
  "dna_hash": "uhC0k...",
  "happ_hash": "uhH0k...",
  "agents": ["verifier", "prover_a", "prover_b"],
  "conductors": [],
  "step_timeout_seconds": 900
}
```

Configuration and executable paths may not traverse symlinks. Any executable,
configuration, contract, capability, or conductor-identity drift fails closed.
