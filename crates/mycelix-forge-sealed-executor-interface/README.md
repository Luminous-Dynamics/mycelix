# mycelix-forge-sealed-executor-interface

FORGE-009J closes the protocol gap between an exact FORGE-009I executor-confinement binding and the concrete execution surface that a later runtime is allowed to expose.

The positive SealedExecutorInterfaceBindingV1 requires:

1. the exact FORGE-009I transaction-plan evidence;
2. one exact authorized principal from the closed-world FORGE-009I binding set;
3. the exact executor identity bound to that principal;
4. a deterministic interface commitment derived from the exact plan;
5. an independent verifier accepting that interface commitment.

The fixed interface profile deliberately excludes caller-supplied:

- shell commands;
- executable selection;
- command arguments;
- transaction stdin;
- refs;
- Git object IDs;
- environment overrides.

The interface commitment is a protocol subject, not proof that a host, process, or operating system enforces it. The independent verifier is the evidence source for that enforcement claim.

## Invocation firewall

SealedExecutorInvocationV1 can be materialized only from the positive sealed-interface binding plus the exact transaction plan. It is deliberately non-deserializable and contains only:

- the bound executor identity;
- the bound repository identity;
- the sealed interface commitment;
- the exact Git argv;
- the exact NUL-framed Git stdin.

It contains no executable path, repository path, environment map, shell text, or caller-supplied arguments. It is an invocation description, not an execution authorization or process capability.

This tranche does not:

- authorize a merge;
- qualify M0;
- execute Git;
- mutate a repository;
- prove hostile-host resistance;
- provide a generic process-spawn API.

A later concrete runtime must accept only the positive sealed interface subject, and a later authority layer must still mint any execution authorization.
