# Space-engineering evidence profile for Mycelix-DeSci

**Status:** proposed profile for research workflows; not a physics certification scheme.  
**Purpose:** make space-infrastructure claims traceable and independently reviewable without confusing signed provenance, decentralized governance, and physical truth.

## Governing rule

A receipt proves that a defined artifact was submitted, received, and handled under a specified authority and policy. A signature, DHT record, consensus result, or trusted author does **not** by itself prove that a physical claim is correct.

The existing scientific authority-receipt journal is the natural place to anchor event identity, authority, receive time and receipt-chain continuity. This profile specializes the evidence carried by a space-engineering research event; it does not replace the journal's authorization or cryptographic checks.

## Evidence classes

Every claim must carry exactly one primary evidence class. Supporting references may include other classes.

- **SourceFact:** explicitly quoted or transcribed from a named primary source, including page/table/figure and architecture boundary.
- **Measurement:** instrument or system observation with calibration, timestamp, uncertainty and collection method.
- **Derived:** result reproducible from identified inputs and equations.
- **ModelResult:** output from a versioned simulation or solver with exact configuration, domain and limitations.
- **EngineeringAssumption:** an explicit chosen value or range with rationale, uncertainty and sensitivity plan.
- **IndependentCheck:** a result produced by a genuinely separate implementation, derivation or reviewer.
- **TestFixture:** synthetic input used only to test software contracts; never a real engineering value.

A source URL or artifact ID establishes traceability, not correctness. A completed JSON schema establishes shape, not physical validity.

## Minimum run record

A model result intended to support a decision must bind the following into one immutable run manifest:

| Field | Required meaning |
|---|---|
| schema and version | Exact interpretation of the record |
| case ID and parent | Stable case identity and lineage |
| source revision | Repository, full commit SHA, file path, Git blob and source SHA-256 |
| inputs | Canonical SI-valued input artifact and digest; units and frame IDs |
| method | Equations, solver/library version, numerical method, tolerances and relevant build features |
| environment | Exact invocation, runtime/toolchain versions and material environment identifiers |
| output | Full result artifact digest, deterministic report and machine-readable status |
| uncertainty | Input uncertainty, numerical error estimate, sensitivity and convergence status |
| exclusions | Forces, bodies, boundary fluxes, faults and effects not modelled |
| reviewer | Independent review disposition and the exact artifacts reviewed |
| claim ceiling | The strongest statement that these artifacts justify—and explicit unsupported statements |

Never cite a newer source revision using an older run's output. If a source, input, solver, configuration or test changes, the evidence status becomes stale until rerun or an explicit impact analysis shows why the change is irrelevant.

## Type-appropriate promotion requirements

The record state is interpreted together with its evidence class. Do not make a source fact pretend to be a solver run, and do not let a model result omit its execution identity.

- **Derived / ModelResult:** `PassedAtSpecifiedFidelity` requires an immutable run-manifest digest, analysis-tool fingerprint and at least one content-addressed result artifact.
- **SourceFact / Measurement / EngineeringAssumption:** a passed record requires the content-addressed source or measurement/assumption artifact, but does not require a solver run or analysis-tool fingerprint merely because it is being reviewed. Measurement records should bind calibration and collection-method material in their source/evidence artifacts.
- **IndependentCheck:** a passed record requires a separate independent-check record that binds its method, tool fingerprint and result artifact. It must not reuse the primary implementation as an “independent” check.
- **TestFixture:** may exercise validators and state transitions, but can never be promoted as passed engineering evidence.

The typed envelope validates these requirements and digest shape. The artifact verifier must still fetch the cited bytes and recompute digests; the receipt verifier must still authenticate signatures and authority. A structurally valid record is not proof that these external bytes, signatures or scientific claims are correct.

## Evidence-state vocabulary

Use machine-readable states that never imply more than they establish:

- **Blocked:** required inputs, provenance, authority, or model definition is missing or malformed.
- **NotRun:** a planned calculation was not executed.
- **Running:** execution is in progress; no result claim is available.
- **PassedAtSpecifiedFidelity:** only the bounded checks named in the manifest passed.
- **Inconclusive:** valid evidence was produced but cannot decide the question within its uncertainty.
- **Failed:** a declared acceptance condition was violated.
- **Stale:** the evidence does not match the current source/input/configuration identity.
- **Unavailable:** the artifact or dependency could not be retrieved or verified.
- **Disputed:** a credible conflicting artifact or review exists and has not been reconciled.

A passing provenance or contract check is a prerequisite for review, not a pass of the physical model. “Queued”, “workflow created”, “artifact uploaded” and “review requested” must not map to PassedAtSpecifiedFidelity.

## Authority receipt binding

For each accepted canonical engineering event, the receipt should refer to:

- exact claim/event ID, event hash, stream and sequence;
- immutable run-manifest digest and output artifact digests;
- source, input and executable/tool digests;
- the evidence class, status and claim ceiling;
- the signing actor, authorization snapshot, policy version and receipt time;
- prior receipt hash and, where applicable, explicit supersession/conflict links.

Use the existing journal's distinct actor and receipt-service keys and its crash-reconciliation rules. A receipt must not overwrite the earlier record when a result is corrected: create a new event that supersedes or disputes the old one while preserving history.

## Independent review and promotion

1. **Contract review:** schema, required fields, units, provenance and exact source fingerprints are valid.
2. **Reproduction review:** a clean invocation reproduces deterministic artifacts where determinism is expected.
3. **Independent-method review:** at least one critical result is checked through a method that does not share the same implementation path or golden output.
4. **Domain review:** the reviewer checks the declared physical assumptions, control-volume boundary, uncertainty and applicable domain of validity.
5. **Claim review:** every public summary is no stronger than the specific evidence record.

Reviewers must be independent of the author for claims promoted beyond internal exploration. Governance can record who approved a decision; governance cannot waive a failed physical acceptance condition or turn missing data into a positive finding.

## Orbital-ring example: deliberately bounded

A source-hashed circular two-body reference can support a statement such as “the ideal point-mass circular-orbit fixture matches its analytic reference within tolerance.” It cannot support “the proposed orbital ring is in equilibrium”, “the ring is stable”, or “the ring survives support loss”.

For a skyhook load-step study, the event must point to a fully specified state/frame, support and rider model, force inventory, energy and momentum boundaries, transition profile, actuator limits, tolerances, convergence report and independent reference case. If any required field is missing, the scenario remains Blocked / NotRun. Test fixtures may exercise this gate but cannot fill the engineering manifest.

## Integration boundary

- Symthaea owns scenario semantics, executable physical-model checks and source-bound evidence outputs.
- Symtropy owns reusable geometry/dynamics primitives, with frame and physical-model boundaries explicit.
- Mycelix-DeSci owns signed research-event provenance, authority, receipt history and review decisions.
- Sol Atlas owns a read-only presentation of source evidence, freshness and status; it cannot make a physical acceptance decision.

These responsibilities may exchange versioned artifacts and hashes. They must not create a circular authority loop in which the visualization or an agent's score is accepted as its own independent verifier.

## Acceptance checklist for implementing this profile

- [ ] A schema test rejects missing required fields, unknown keys, duplicate JSON keys and non-standard numbers.
- [ ] The receipt binds exact source/input/output digests and retains supersession history.
- [ ] Stale, disputed, unavailable and blocked evidence have distinct machine-readable states.
- [ ] A synthetic test fixture cannot be accepted as an engineering input.
- [ ] A deterministic repeat is compared byte-for-byte where deterministic output is claimed.
- [ ] An independent-method result is identified separately from a repeated run of the same checker.
- [ ] Public summaries cite the exact accepted evidence record and preserve its claim ceiling.
