# QCAP-001A3A — bounded output and execution-limit admission

Tracked by #1702 under QCAP-001A3 / #1695.

Direct predecessor: QCAP-001A2 `d4e150fbdcb15b86ea12d27b3d42f85a25c272ff`.

A3A is additive: the A2 runner, v2 schemas, and v2 historical vectors are inherited unchanged. A3A adds a new `qcap3` reference adapter and v3 execution/receipt surface.

## Established product behavior

- content-addressed execution-limits profile v1;
- execution-context v3 binds the exact limits-profile reference;
- bounded streaming capture with O(`max_gate_output_bytes`) captured-output memory;
- exact-limit output is admitted;
- first observed byte above the limit becomes `RunnerInfrastructureFailure(OutputLimitExceeded)`;
- gates after runner failure become `GateNotRun`;
- retained stdout after process exit has an independent short drain ceiling;
- receipt v3 binds captured-output digest/byte count, truncation state, typed runner-failure reason, and an **effective** exit code;
- manifest admission bounds gate count, individual/total script bytes, args/aggregate arg bytes, canonical manifest bytes, and claim/nonclaim UTF-8 bytes;
- accounting uses checked u64 addition and fails closed on overflow;
- independent v3 vectors reproduce PASS, theorem-FAIL, and output-limit receipts without importing production qcap3 modules.

## Why `effective_exit_code`

The receipt does not call this field `process_exit_code`. Runner-detected subject-integrity failure can convert a process that exited 0 into theorem `GateFail`; claiming the resulting 10 was the raw OS process exit would be false provenance. `effective_exit_code` records the QCAP gate-contract outcome after runner integrity checks.

## Claim boundary

A3A establishes deterministic operational admission for the registered resource dimensions and bounded gate-output capture for the reference adapter.

It does **not** yet establish verified executable snapshots, closed Python import/bytecode closure, fully registered executable PATH semantics, adversarial process-escape containment, network isolation, QCAP framework qualification, or any FIN-ECO theorem.

Local tests and vectors are product evidence only, not qualification.
