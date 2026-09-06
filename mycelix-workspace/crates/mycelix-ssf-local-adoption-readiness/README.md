# SSF Local Adoption Readiness v0.1

This crate is a pure, non-authoritative boundary between exact remote-path authorization coverage and a later local adoption policy decision.

A readiness token can exist only when the supplied remote history is an exact `RemoteStrictExtension`, its full path is authorization-covered, the exact claimed common ancestor is independently qualified, the exact local tip is independently established as the current local security context, and all six required local freshness prerequisites are known and bounded.

Every required local freshness prerequisite also carries an explicit evidence commitment: membership, identity key, revocation snapshot, active policy, machine health, and scientific qualification. The common-ancestor qualification carries its own exact evidence root. These commitments identify evidence lineages; they do not make the evidence true by themselves.

`NotApplicable` and `Unknown` are both rejected for the six mandatory local freshness prerequisites.

Readiness is **not adoption approval** and contains **no effect authority**. A later approved local policy evaluator must independently decide whether the ready remote state may be adopted, and any resulting authority must be separately bounded, registered, and reconciled with outcome evidence.
