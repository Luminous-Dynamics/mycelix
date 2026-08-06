# Real conductor driver build recipe v1

The reviewed driver is the executable source file
`tests/conductor/holochain_real_driver.py`. It is intentionally standard-library
only, so the release artifact does not depend on an unpinned Python package
environment.

A reviewed release must bind:

1. The exact driver executable bytes.
2. `tests/conductor/real_driver.lock`.
3. This build recipe.
4. The exact Git commit and tree.
5. The exact scenario contract.
6. A generated, immutable runtime configuration.
7. The exact control executable and its capability transcript.

The driver release manifest uses a two-pass identity handshake. During the first
capability call the manifest tool injects a placeholder release hash and hashes
the remaining capabilities. It computes the release identity from the driver
bytes, reviewed capabilities, source identity, recipe, lock, and Holochain
version. During the second call it injects that computed identity and rejects
any other capability drift.

The runtime configuration is created with:

```bash
python3 tests/conductor/holochain_driver_config.py \
  --create \
  --config /secure/review/driver-config.json \
  --control /secure/review/holochain-control \
  --scenario-contract tests/conductor/settlement_scenarios_v2.json
```

The output is mode `0444`. It contains no private keys or tokens. Conductor
secrets remain in the control process's protected runtime environment.

The driver is then reviewed and signed with
`scripts/create_reviewed_driver_release.sh`. The resulting manifest identifies
the driver release; the runtime configuration's `runtime_lock_hash` is included
in reviewed capabilities and cannot be changed without a new driver release.
