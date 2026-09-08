# SSF Canonical Replay Actuator Execution v0.1

Canonical external-effect bridge for the single authorized replay attempt.

The only replay execution input is `FinalCanonicalReplayInvocationReadyV1`. Reservation remains non-effecting, trusted time is freshly qualified immediately before and after invocation, and raw actuator receipts are preserved unchanged.

Pure reservation/freshness/post-effect classification is delegated to `mycelix-ssf-canonical-execution-semantics` so replay does not maintain a second independent definition of canonical effect semantics.

The bridge maps the provider-neutral kernel result into the existing canonical outcome vocabulary used by downstream evidence/history layers. Once `invoke_reserved_effect` begins, no ordinary retryable error path remains; post-effect uncertainty becomes canonical `OutcomeUnknown`.

The resulting outcome contains no replay authority and structurally permits no third attempt.
