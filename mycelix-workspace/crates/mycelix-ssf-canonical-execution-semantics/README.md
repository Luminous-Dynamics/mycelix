# SSF Canonical Execution Semantics v0.1

Defines pure, provider-neutral reservation, pre-invocation, post-invocation, and canonical effect-classification semantics for SSF actuator adapters.

The crate contains no authority typestate constructor, trusted-time qualifier call, journal/store write, actuator reservation call, or actuator invocation. It only evaluates already supplied exact evidence.

Its semantic enums are independent of the current #284 adapter types, avoiding a dependency cycle and allowing both initial and replay execution adapters to converge on the same kernel in a future parent restack.

Post-invocation ambiguity is never represented as a retryable error. Canonical effect classification never rewrites raw actuator evidence and creates neither effect authority nor replay authority.
