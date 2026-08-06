# Real conductor driver protocol v1

`real_conductor_adapter.py` owns evidence retention, hashing, scenario-contract
binding, and adapter output. A Holochain-specific driver owns only conductor
lifecycle, partition control, zome calls, gossip healing, and raw observation
capture.

The driver path is supplied through `MYCELIX_CONDUCTOR_DRIVER`. It must be one
executable file and support:

```text
driver --capabilities
driver --scenario <scenario-id> --evidence-dir <absolute-directory>
```

Every invocation prints exactly one JSON object and exits nonzero on any failed
conductor operation or assertion.

## Capabilities

```json
{
  "driver_protocol": "mycelix-real-conductor-driver-v1",
  "driver_release_hash": "<canonical standard-base64 SHA-256>",
  "conductor_count": 3,
  "dna_hash": "<installed DNA hash>",
  "happ_hash": "<installed hApp hash>",
  "holochain_version": "0.6.1",
  "agents": ["verifier", "prover_a", "prover_b"],
  "conductors": [
    {
      "id": "verifier",
      "agent_pub_key": "<agent key>",
      "database_identity_hash": "<hash of isolated database identity>",
      "network_identity_hash": "<hash of isolated network identity>"
    }
  ]
}
```

The adapter verifies that conductor, database, and network identities are
unique and that the required roles are present.

## Scenario result

The driver writes raw logs, zome-call request/response JSON, action hashes,
network snapshots, and partition/heal evidence under `--evidence-dir`. It then
returns:

```json
{
  "driver_protocol": "mycelix-real-conductor-driver-v1",
  "scenario_id": "partitioned-competing-proofs",
  "observed_state": "ChallengeConsumptionConflict",
  "started_at_unix_micros": 1700000000000000,
  "completed_at_unix_micros": 1700000001000000,
  "events": [
    {
      "sequence": 0,
      "kind": "partition-created",
      "actor": "harness",
      "at_unix_micros": 1700000000000000
    }
  ],
  "observations": [
    {
      "conductor_id": "verifier",
      "state": "ChallengeConsumptionConflict",
      "observed_at_unix_micros": 1700000001000000,
      "response_path": "observations/verifier.json"
    }
  ]
}
```

Paths are relative to the scenario evidence directory. The adapter—not the
driver—enumerates every retained file, computes sizes and SHA-256 hashes,
constructs the canonical manifest, and verifies it before returning success.

This protocol makes evidence tamper-evident and reproducible. It does not by
itself prove that an operator's claimed process was a genuine conductor; CI
must run the reviewed driver in a controlled environment and retain the full
bundle.
