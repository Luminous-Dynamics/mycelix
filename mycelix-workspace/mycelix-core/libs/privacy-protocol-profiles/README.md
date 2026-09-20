# privacy-protocol-profiles

Backend-neutral structural profiles for MPC, FHE, PSI, and PIR above `privacy-computation-core`.

This crate implements **no cryptography** and grants no privacy theorem, backend qualification, production admission, or application authority. Its job is to make protocol-specific semantic incompatibilities fail closed before a concrete backend is selected.

The crate is split into independently reviewable protocol modules:

- `mpc` — computation class, corruption bound, participant topology, output recipients and output-disclosure consistency;
- `fhe` — exact numeric semantics, key model, bootstrapping/target, topology consistency, no access-pattern overclaim;
- `psi` — exact set operation, collection/output semantics, equality/session domains, cardinality/elements/aggregate-value leakage and aggregate function/value-domain binding;
- `pir` — exact server/collusion topology, query shape, immutable database-snapshot identity, query-index privacy without ORAM overclaim.

Common fail-closed rules require non-empty backend/version/profile identities and reject `Interactive { rounds: 0 }`.

Important boundaries:

- threshold/DKG machinery is not relabeled as general MPC;
- output-producing MPC profiles must explicitly declare output-value leakage; `NoOutput` cannot claim output may reveal;
- FHE `ApproximateReal` does not satisfy `ExactInteger`;
- threshold/multi-key FHE key-party counts must match declared participant topology;
- plain FHE does not claim metadata/access-pattern privacy;
- PSI v1 is TwoParty-only because its recipient vocabulary is client/server/both;
- PSI equality/session domains must be non-empty and versionable;
- full intersection disclosure cannot claim hidden cardinality when revealed elements determine that cardinality;
- PSI cardinality/aggregate profiles must hide matching elements and explicitly bind their own result disclosure;
- PSI aggregate profiles additionally bind an aggregate function and associated-value domain;
- PSI aggregate-value leakage is distinct from intersection cardinality leakage;
- OPRF/VOPRF is a construction building block, not equivalent to PSI;
- PIR database snapshots must be explicitly identified;
- PIR server topology must match participant topology with checked arithmetic (no saturating overflow);
- plain PIR must declare query-index hiding but cannot claim access-pattern privacy;
- PIR remains distinct from ORAM, anonymity, response integrity/currentness, and authorization.

Validated profiles convert only into the common `PrimitiveCapability` structural vocabulary. `Compatible` remains `StructuralOnly`.
