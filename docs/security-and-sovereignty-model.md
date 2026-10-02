Recovery and reconciliation must not erase security-relevant history merely because a newer local state is selected.

### I-7: AI remains subordinate to enforceable policy

AI output is advisory unless an independently authorized policy explicitly grants a capability to an automated actor. Even then, the granted capability is bounded and revocable.

### I-8: Fail closed on authority ambiguity

When a security-critical authorization cannot be established deterministically, the operation must not be treated as authorized merely because an AI system considers it plausible.

## Security event envelope

Successful enforcement events produced from `EnforcementRequest` now carry the kernel-derived capability commitment in addition to any external capability reference. They also carry the opaque authority-freshness commitment supplied by the authority adapter. The event constructor additionally requires the recorded actor identity to equal the enforcement subject, preventing an authorized action from being attributed to a different principal. This makes the audit record able to distinguish the exact capability semantics, authority-generation evidence, and authorized actor that crossed the enforcement boundary from caller-supplied labels. `SecurityEvent::new()` cannot mint an `Allow` record directly; successful Allow records must originate from an enforcement request that already crossed permit revalidation. Deny and Indeterminate records remain directly recordable, while legacy serialized Allow records remain historical evidence rather than authorization primitives.

Security-sensitive events should converge toward a common conceptual envelope:

- event identifier
- actor identity
- capability used
- action
- target/resource