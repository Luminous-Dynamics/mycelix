# Directory Fabric v0.1 — Request-scoped causal cuts and decision/effect receipts

**Status:** executable pure-Rust reference contracts authored; not compiler-, cryptographic-, runtime- or protocol-qualified.  
**Programme:** [MYC-DIR-FABRIC-001 / #6428](https://github.com/Luminous-Dynamics/mycelix/issues/6428)  
**Parent design/reference:** [#6430](https://github.com/Luminous-Dynamics/mycelix/pull/6430) and [#6434](https://github.com/Luminous-Dynamics/mycelix/pull/6434)  
**Implementation:** `tools/myc-dir-fabric-reference/src/causal_cut.rs` and `src/receipts.rs`.  
**Adversarial contract:** [MYC_DIR_CAUSAL_CUT_RECEIPTS_V0_1.json](../benchmarks/MYC_DIR_CAUSAL_CUT_RECEIPTS_V0_1.json) plus its Draft 2020-12 schema.  
**Claim ceiling:** finite synthetic reference semantics only. No signature verification, complete dependency-closure proof, independent verifier deployment, SCITT integration, Holochain runtime, real effect-owner integration, performance result or production enforcement is established.

## 1. Architectural decision: one request, one complete causal cut

A decentralized authorization request should not require every peer, realm, issuer or unrelated authority in the network to be globally current. It must, however, have sufficient evidence for every authority dependency that could alter the outcome of this exact request.

The proposed **Request-Scoped Causal Authority Cut (RSCAC)** is the exact, versioned set of authority positions needed to evaluate one request under one policy and one dependency-graph snapshot.

For request:

`Q = (realm, subject, subject-key-epoch, workload, resource, action, audience, purpose, context, nonce, evaluation-profile)`

the closure resolver identifies the relevant authority dependencies, including where applicable:

- realm policy and realm-root epoch;
- subject membership and membership lifecycle;
- issuer admission, key status and credential status;
- relevant group and role relationships;
- delegation ancestors, attenuation constraints and revocation;
- federation contract and foreign-to-local mapping;
- device posture and workload trust-domain/audience evidence;
- resource policy, explicit denies, obligations and revocation state.

The cut contains exact identifiers, dependency kinds and minimum authority positions. The authority positions are per authority (or per source-chain lineage), not one summed height. A higher sequence does not prove ancestry by itself.

**Completeness and minimality are different properties.**

- Completeness is a safety requirement: every dependency capable of changing the result must be represented.
- Minimality is an efficiency and availability objective: avoid requiring unrelated authorities to be current.

A conservative superset can be safe but may increase latency and false denial during partitions. An incomplete cut can be unsafe and must never produce Allow. The v0.1 reference models exact binding and completeness-attestation consumption; it does not prove a resolver returns the mathematically minimal cut.

## 2. Two-stage closure verification

A resolver-produced list is not self-authenticating. The requester and resolver must not be the only parties deciding that the dependency list is complete.

The target protocol has two separate artifacts:

### 2.1 Dependency-graph snapshot

The resolver operates on an authenticated, versioned dependency graph snapshot, identified by an exact digest. It must include the policy/entity relationships from which dependencies are derived, including deny-relevant and revocation-relevant edges.

If the graph snapshot is unavailable, stale under the required realm profile, or cannot be authenticated, closure cannot be claimed complete.

### 2.2 Dependency closure receipt

An independently pinned verifier evaluates the exact tuple:

`(request_digest, realm, policy_digest, dependency_graph_digest, cut_digest, resolver_profile_digest, proof_digest, verifier_id, verifier_build_digest)`

The receipt states one of:

- `VerifiedComplete`
- `Incomplete`
- `Rejected`
- `Unavailable`
- `Unverified`

Only `VerifiedComplete` is eligible to continue. The verifier must be independent of the resolver's ability to omit or alter inputs; verifier identity/build and trust roots must be supplied out of band under the relying realm's policy. The closure proof format, verifier signature, anti-rollback and key-rotation protocol remain future work.

The reference crate accepts the receipt as an external verifier result and checks that its request, realm, policy, graph, cut and resolver-profile bindings match. It does not itself verify the proof digest, signature, authority of the verifier, or graph authenticity. In particular, a caller must never set `VerifiedComplete` merely because it constructed a non-empty dependency array.

### 2.3 Local frontier comparison

After closure verification, the PEP checks every required authority position against its authenticated local snapshot:

- exact authority position: covered;
- older local position: behind;
- higher local sequence with a verified exact ancestry witness: covered;
- higher sequence without that witness: incomparable;
- same sequence with a different digest: incomparable;
- missing non-genesis authority position: behind/incomplete.

An unrelated authority's progress cannot compensate for one required dependency being behind. If any component is behind or incomparable, no Allow is possible. A known explicit forbid may still produce Deny when its exact witness is independently valid and bound to the full request.

This is deliberately narrower than global consistency. The requirement is a complete, authenticated causal lower bound for the dependencies that matter to this request.

## 3. Compose with existing authorization instead of duplicating it

Directory Fabric must not create another policy language, capability algebra or host-call lifecycle model.

The intended composition is:

`Complete dependency closure`
`+`
`Sufficient local authority frontier`
`+`
`Existing full-context policy result`
`+`
`Exact decision receipt`

Existing work remains authoritative for its domain:

| Existing work | Ownership retained |
|---|---|
| [#4743 / PR #4752](https://github.com/Luminous-Dynamics/mycelix/pull/4752) | One exact effective capability atom must match the full request context (resource/action/audience/time) with exact contributor provenance; contextual laundering is denied |
| [#4854](https://github.com/Luminous-Dynamics/mycelix/issues/4854) | Bounded constraint satisfiability/subsumption, normalization and deny preservation |
| [Pulse #5771](https://github.com/Luminous-Dynamics/mycelix/pull/5771) | Causal frontier, grant identity/lifecycle, host-call admission, revocation, retry identity and bounded formal models |
| [Pulse #4903](https://github.com/Luminous-Dynamics/mycelix/issues/4903) | Incomplete DHT visibility is unknown; stale application projection is never host permission |
| [Pulse #5924](https://github.com/Luminous-Dynamics/mycelix/issues/5924) | Revocation linearization at actual host admission; pre-admitted in-flight execution is distinguished from new post-revocation admission |
| [LISE #6236](https://github.com/Luminous-Dynamics/mycelix/issues/6236) | Trajectory-wide revocation propagation, route/session inventory and stale-capability exposure across PEPs, tokens, workers and connectors |
| [LISE #6235](https://github.com/Luminous-Dynamics/mycelix/pull/6235) | Candidate-blind trajectory scorecard and separate safety metrics |
| [#6050](https://github.com/Luminous-Dynamics/mycelix/issues/6050) | Machine subject, key epoch, instance, artifact digest and workload identity remain distinct |
| [#889](https://github.com/Luminous-Dynamics/mycelix/issues/889) | Non-authoritative enterprise protocol TCK; protocol evidence does not become Mycelix authority |
| [#1149](https://github.com/Luminous-Dynamics/mycelix/issues/1149) and [#1191](https://github.com/Luminous-Dynamics/mycelix/issues/1191) | Bounded local capability leases and runtime branch-enforcement boundaries |

The Directory Fabric reference module supplies the composition boundary and evidence binding; it does not replace these owners.

## 4. Policy evaluation: completeness is not the same as policy syntax

A critical interoperability seam exists around policy evaluation errors.

Cedar's authorization algorithm has explicit semantics for `permit`, `forbid` and evaluation errors. A matching `permit` can produce an Allow even when another policy evaluation returns an error, because error policies are skipped; a matching `forbid` takes precedence. That behaviour is a property of the Cedar policy algorithm, not proof that an external identity/issuer/realm dependency set is complete.

Directory Fabric must maintain two separate domains:

1. **Authority/dependency completeness:** did the verifier establish all mandatory identity, issuer, membership, revocation, delegation and policy-graph inputs for this request?
2. **Policy evaluation:** given the complete, authenticated inputs, what does the selected policy engine decide?

A missing required authority dependency is not a harmless policy error to skip. The PEP must stop before treating a Cedar permit as sufficient. An adapter may retain Cedar's documented error behaviour for policy statements that its profile explicitly declares non-mandatory, but it cannot classify missing mandatory authority evidence as an ignorable statement error.

v0.1 decision-receipt rules:

| Cut assessment | Policy result | Reference decision |
|---|---|---|
| Sufficient | exact full-context Permit witness | Allow candidate |
| Not sufficient / unknown | Permit witness | Indeterminate |
| any cut state | exact, request-bound explicit Forbid witness | Deny |
| Sufficient | complete-policy-set DefaultDeny | Deny |
| Not sufficient / unknown | DefaultDeny | Indeterminate |
| any state | mandatory policy evaluation unavailable | Indeterminate |

The word “candidate” matters: production still needs signature verification, subject/issuer admission, runtime enforcement, and exact PEP binding. A receipt is evidence of a decision claim, not the decision's truth by itself.

## 5. Two receipts, not one: authorization versus effect

A positive authorization is not evidence that a protected operation happened. Conversely, a successful downstream operation does not prove it was properly authorized.

### 5.1 Authorization decision receipt

The v0.1 canonical decision record binds:

- decision ID and realm;
- exact request digest;
- policy digest and epoch (epoch is represented through the bound policy/snapshot profile);
- exact dependency cut digest;
- dependency-closure receipt digest;
- policy snapshot digest;
- evaluator build digest;
- request nonce/challenge digest;
- named clock profile;
- issued-at and expiry;
- cut assessment;
- outcome (Allow/Deny/Indeterminate);
- outcome basis kind and exact permit/forbid/default-deny/cause digest.

These bytes are versioned and domain-separated. The relying party must hash and sign them using a pinned signature/hash profile. The current Rust reference emits canonical bytes but does not compute the digest, sign, verify signer identity, provide durable nonce consumption, or prove trusted time.

### 5.2 Effect observation receipt

The effect owner produces a distinct record binding:

- same request digest;
- exact authorization decision receipt digest;
- stable logical operation ID;
- unique physical attempt ID and consecutive attempt number;
- stable idempotency-key commitment;
- effect-owner identity;
- outcome: Committed, RejectedNoEffect, Unknown, ReconciledCommitted, ReconciledNoEffect or NotObserved;
- exact observation digest.

The verifier must check every physical attempt for the same logical operation preserves request, idempotency commitment and effect owner. There may be at most one commit for the logical operation. Unknown does not mean no effect and therefore blocks a new mutation until the effect owner reconciles the exact logical/idempotency key. A no-effect reconciliation may permit a retry of that same logical operation; if a subsequent retry becomes Unknown, unresolved status returns.

The local reference history checker enforces these structural rules. It does not authenticate effect-owner identity or prove that the owner enumerated every route/session/worker capable of producing an effect. That stronger inventory and observation obligation belongs with LISE #6236.

### 5.3 Transparency is a third, separate plane

SCITT (RFC 9943) can make signed decision statements transparently registrable and auditable. A transparency-service receipt proves registration/inclusion under that service's profile; it does not prove that the statement is true, that the evaluator was authorized, that the permission remains current, or that a downstream action happened.

The correct chain is therefore:

`Authority and closure evidence`
`→ Authorization decision receipt`
`→ Actual effect-owner observation receipt`
`→ Optional transparency registration and receipt`

SCITT receipt != authorization grant.  
SCITT receipt != effect receipt.  
Hash commitment != issuer identity.  
Signature validity != organizational authority.

## 6. Paradigm shifts and research questions

### A. From global freshness to request-scoped causal sufficiency

The main research question is whether a verifier can independently prove that a request's dependency cut is complete and that the PEP's snapshot dominates the minimum authority positions for each dependency.

This extends, rather than replaces, the per-request consistency work in SpiceDB and OpenFGA. SpiceDB's ZedToken `at_least_as_fresh` mode gives a useful read-after-write comparator; OpenFGA exposes higher-consistency versus latency-oriented query modes. Mycelix's proposed difference is a cut across independently governed authority histories, not just another consistency token.

### B. From roles to an evidence graph with explicit deny edges

The dependency resolver must include not only positive relationships but also deny-relevant constraints, revoked delegations, issuer/key status, exclusions and policy obligations. Omitting a deny dependency must be a specific negative control. Deny preservation continues to be owned by #4854.

### C. From “one Allow” to an accountable action lifecycle

The decision/effect receipt separation allows measurement of what happened after an authorization decision: whether an effect committed, was rejected, remained unknown or required reconciliation. This allows LISE to measure stale capability exposure and false claims of successful execution without creating a competing revocation engine.

### D. From protocol adapters to evidence-preserving translation

SCIM, OIDC/OAuth, SPIFFE/SPIRE and LDAP adapters should translate external identity evidence into bounded, source-attributed inputs. They must preserve unknown/unsupported fields and never promote protocol success into authorization.

- SCIM is provisioning and lifecycle exchange, not an authorization decision.
- OIDC tokens must be audience/purpose bound and validated under the configured issuer profile; sender-constrained proof-of-possession should be evaluated for sensitive APIs.
- SPIFFE identity and SVIDs are workload identity evidence in an explicit trust domain, not human membership or unlimited service authority.
- LDAP read-only projections do not implement Kerberos, Windows domain join, DNS, PKI, Group Policy or endpoint management.

The v1 migration path remains read-only import → shadow decisions → SCIM lifecycle → OIDC application → SPIFFE workload bridge → one local Linux PEP → narrow production pilot. Kerberos/Windows remains a separate programme.

## 7. Benchmark protocol additions

The machine-readable benchmark preregistration now includes 29 metrics, six dependency-cut-size profiles (1/8/32/64/128/256 dependencies), 19 fault schedules and comparators for Cedar and SCITT. The causal-cut corpus has 24 scenarios (8 Allow, 4 Deny, 12 Indeterminate) and its own schema.

New non-compensable gates:

- Zero false complete-closure claims when one required dependency is omitted.
- Zero Allow decisions with incomplete, unverified, unavailable or non-dominating mandatory dependencies.
- 100% detection of seeded omissions against a separately held-out dependency oracle.
- Zero accepted decision-receipt substitutions.
- Zero effect commits without an exact matching Allow receipt.
- Zero duplicate commits per logical operation/idempotency key.
- Zero new mutation attempts while a prior attempt remains Unknown without exact no-effect reconciliation.
- Zero SCITT inclusion receipts treated as identity, permission or effect authority.
- Zero permits surviving omission of a policy/authority dependency declared mandatory for the request.

Provisional pure-model targets (not measured):

| Metric | Target |
|---|---:|
| Local cut/frontier assessment p95 at 64 dependencies | ≤500 μs |
| Decision receipt composition p95 (excluding signatures) | ≤100 μs |
| False complete dependency closures | 0 |
| Allow with incomplete required cut | 0 |
| Omitted mandatory dependency detection | 100% |
| Effect commit without exact Allow receipt | 0 |
| Duplicate logical effect | 0 |
| Retry while unknown/unreconciled | 0 |

Report dependency-cut size relative to the reachable policy/entity graph so the project can measure whether request-scoped resolution actually reduces the currentness burden. Report worst-case and distribution, not just averages. Do not measure only the pure local function: also measure independent closure resolver latency, signature verification, cache hits/misses, proof bytes, graph snapshot storage and effect reconciliation.

Use at least 10 independent process-level repetitions, open-loop offered load and exact source/build/runtime/graph/corpus digests. Keep actual Holochain conductor tests separate from pure-reference tests and synthetic peer simulations. Benchmark all comparators on matched policy/request semantics and freshness requirements; do not collapse security and sovereignty into a single weighted score.

## 8. Adversarial matrix

The companion 24-case corpus includes positive controls and attacks across:

- complete and incomplete dependency closure;
- unverified resolver self-assertion;
- omitted revocation and issuer dependencies;
- independent progress masking a stale required authority;
- higher sequence without proven ancestry;
- same authority supplying multiple roles at distinct positions;
- unrelated authority staleness that must not block a genuinely complete minimal cut;
- exact default-deny and explicit-forbid cases;
- skip-on-error permit trap when a mandatory authority input is missing;
- request/audience, realm, policy, graph and cut substitution;
- SCITT receipt presented as authority;
- Allow with no effect observation;
- effect-owner commit/reject/unknown/no-effect reconciliation;
- changed logical operation or idempotency identity;
- duplicate committed effects;
- commit evidence attached to a Deny decision.

The corpus is finite and synthetic. It does not establish completeness of any real policy graph.

## 9. Implementation and qualification gates

1. Review the RSCAC dependency vocabulary, resolver/verifier independence and policy-graph snapshot semantics.
2. Independently implement the closure resolver and hidden completeness oracle; do not derive both expected and observed dependencies from the same code.
3. Define the canonical dependency-graph snapshot and proof format, pinned verifier identity, signature profile, hash agility, revocation and anti-rollback behaviour.
4. Add property-based and differential tests for omitted deny/revocation edges, stale required dependencies, unrelated stale authorities, graph changes between resolve/verify and policy error boundaries.
5. Compose full-context policy evaluation from #4752/#4743 and #4854; retain their exact witness and deny-preservation rules.
6. Connect Pulse #5771/#4903/#5924 so actual host admission and DHT currentness are not replaced by a local application projection.
7. Connect LISE #6236/#6235 for effect-owner inventory, bitemporal propagation and candidate-blind scoring.
8. Add signed decision receipts, nonce consumption, key lifecycle, trusted-clock and independently signed effect-owner observations.
9. Add optional SCITT statement registration only after the relying party's trust policy authenticates the appropriate transparency-service roots.
10. Re-run matched local, relationship, directory, workload and distributed-runtime benchmarks with exact reproducibility receipts.

The selected Holochain version matrix and runtime regression gate remain in [MYC-DIR-HC-QUAL-001 / #6435](https://github.com/Luminous-Dynamics/mycelix/issues/6435). This reference composition does not bypass that gate.

## 10. External standards and comparator references

- **Cedar authorization algorithm:** [Cedar Authorization](https://docs.cedarpolicy.com/auth/authorization.html). Keep policy evaluation errors distinct from missing, mandatory Mycelix authority dependencies. A Cedar `permit` result cannot repair an incomplete request-scoped authority cut.
- **SCITT transparency:** [RFC 9943](https://www.rfc-editor.org/rfc/rfc9943.html). Use transparency registration for signed decision statements only after a relying party pins the transparency-service profile and roots. Inclusion is not truth, permission or effect.
- **SpiceDB consistency:** [Authzed consistency](https://authzed.com/docs/spicedb/concepts/consistency). Compare using `at_least_as_fresh` ZedTokens and, separately, fully consistent reads. Mycelix must demonstrate the additional value of a complete cut across independent authority histories; freshness tokens alone are not a differentiator.
- **OpenFGA consistency:** [OpenFGA consistency](https://openfga.dev/docs/interacting/consistency). Compare `HIGHER_CONSISTENCY` and `MINIMIZE_LATENCY` on equivalent relationship graphs and request sets.
- **OAuth security:** [RFC 9700](https://www.rfc-editor.org/rfc/rfc9700.html) and [DPoP / RFC 9449](https://www.rfc-editor.org/rfc/rfc9449.html). Audience and sender binding are adapter security controls; neither token validity nor proof-of-possession establishes local organizational authority.
- **SPIFFE federation:** [SPIFFE Federation Specification](https://spiffe.io/docs/latest/spiffe-specs/spiffe_federation/). Bundle trust is trust-domain scoped; do not treat an arbitrary valid SVID or bundle as a Mycelix membership or resource permission.

## 11. Explicit claim ceiling

This work is a pure reference contract and finite synthetic corpus. It does not establish:

- that any resolver found every real policy/authority dependency;
- that a dependency graph snapshot is complete, authentic or non-equivocating;
- that closure/effect receipts are signed by trustworthy independent parties;
- that a cryptographic ancestry/inclusion proof was verified;
- that a trusted clock exists or that time/lease data is correct;
- that a Holochain host grant was revoked or a conductor rejected a call;
- that an external service/resource effect occurred or did not occur;
- that the effect-owner route/session inventory is complete;
- that SCITT, OIDC, SCIM, LDAP, SPIFFE or Cedar integration is protocol-conformant;
- that the system is production-ready, faster than SOTA or a drop-in Active Directory replacement.

Source-present, structurally inspected, compiled, test-executed, model-checked, runtime-qualified, protocol-conformant and physically measured remain distinct evidence states.
