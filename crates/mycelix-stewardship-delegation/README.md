# mycelix-stewardship-delegation

STEW-013A constructs a structural, evidence-bearing scoped delegation chain without claiming that the root delegator possessed valid authority, that any principal is authenticated, or that delegation currentness is true.

```text
well-formed delegation records
+ exact parent topology
+ non-amplifying scope
-> ScopedDelegationChainCandidateV1

candidate
!= root authority verified
!= delegate authenticated
!= current delegation verified
!= runtime authorization
!= capability
```

## Conservative v1 scope algebra

Each hop retains:

- one exact `StewardedSubjectIdentityV1` target;
- explicit domain allow-list;
- explicit action allow-list;
- explicit purpose/context allow-list;
- root authority-source reference;
- explicit subdelegation mode;
- independent delegation-currentness assertion;
- separately typed mandate/currentness/binding evidence.

For every child:

```text
child.target == parent.target
child.domains  subset_of parent.domains
child.actions  subset_of parent.actions
child.purposes subset_of parent.purposes
```

No prefix, role-name, timestamp, list-position, reputation, stake, or enum-ordinal hierarchy is inferred.

## Topology

Input record order is irrelevant. Parent IDs define the chain. V1 rejects duplicate IDs, multiple roots, missing parents, branching/multiple children, cycles/disconnected records, principal continuity breaks, authority-source changes, and depth beyond the frozen bound.

## Subdelegation

`Forbidden`, `ExactScopeOnly`, and `NarrowerScopeOnly` have explicit relations. `ProfileGoverned` can be retained on a terminal hop, but it cannot authorize another hop in v1 because its external profile has not been evaluated here.

```text
subdelegation mode recorded
!= permission profile evaluated
```

## Currentness

Revoked, superseded, expired, and indeterminate hops remain representable. STEW-013A does not erase them or turn `AssertedCurrent` into verified currentness. STEW-013B owns profile-relative applicability/currentness admission.
