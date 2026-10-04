# Mycelix Comparative Security Benchmark v0.1

**Status:** executable reference-model benchmark  
**Claim ceiling:** `ReferenceModelOnly`

This benchmark is designed to make future comparative-security claims falsifiable.

It does **not** model SIPRNet, NIPRNet, a named commercial product, or any real deployment unless an exact topology and threat model are supplied and independently evidenced.

## Same experiment rule

Every architecture profile is evaluated against the same:

- attacker identity;
- protected resources;
- compromise event types;
- scenario list;
- measurement functions.

Only the declared security topology and authorization requirements vary.

This prevents an architecture from “winning” by receiving an easier adversarial workload.

## Metrics

Boundary-depth reporting is a maximum declared path depth, not discovery of undocumented real network paths.

The benchmark reports separate vectors:

- reachable resources after compromise;
- maximum declared trust-boundary path depth among reached resources;
- standing privilege count;
- revocation latency;
- cross-domain exposure.

There is deliberately **no composite security score**.

A system can be better on containment while worse on another dimension. Collapsing those outcomes into one scalar would hide the tradeoff.

## Synthetic reference architectures

### M-E1-reference

The Mycelix reference model requires six dimensions for protected-resource access:

```
subject identity
device posture
workload identity
security domain
policy version
purpose
```

A stolen credential therefore supplies only one required dimension. An endpoint or workload compromise supplies only one other dimension.

The synthetic model makes A/B reachable with subject identity plus segment access, while C/D additionally require the admin-plane dimension. A full administrator compromise therefore reaches all four resources, while a stolen credential reaches only two.

### B1-segmented-enterprise-reference

This is a deliberately generic segmented-enterprise model in which protected resources require:

```
subject identity
network segment
```

It is not a claim about any particular enterprise architecture.

### B2-perimeter-reference

This is a deliberately simplified network-perimeter reference in which protected resources require subject identity and compromised network reachability can expose the resource set.

It is a reference topology, not an empirical assertion about a named product or government network.

## Reachability theorem

For each compromise event:

```
reachable(resource, event)
iff
required resource dimensions
⊆
dimensions compromised by the event
```

The benchmark therefore makes the security assumption itself visible.

A different topology must produce a different explicit artifact.

## Falsification

A comparative result must be rejected as invalid when:

- the threat model differs;
- resources differ between runs;
- the harness silently satisfies missing authorization dimensions;
- the benchmark assumes a real-world baseline property not contained in the baseline artifact;
- a design feature exists only in prose;
- a composite score hides per-metric regressions;
- the benchmark harness has stronger authority than the architecture it measures.

## Execution

Run:

```bash
python3 scripts/security/run_mycelix_comparative_benchmark_v0_1.py
```

The current v0.1 artifact is a deterministic reference-model benchmark. It does not require network access.

## Important next step

Replace synthetic baseline profiles with independently documented exact deployments.

For every measured deployment, bind:

```
threat-model ID/version
implementation profile
artifact/build digest
hardware/VM profile
network/topology profile
administrator model
authorization policy
attack schedule
measurement environment
observed evidence
```

Only then can a result be described as an empirical comparative measurement.

## Relation to the enclave program

The benchmark composes naturally with:

```
#3976 enclave profile
  -> #3977 RATS Evidence/appraisal
  -> #3982 trusted-time
  -> #3983 TPM Evidence
  -> #3994 measured-component representation
  -> #3973 protected release
  -> #3974 comparative benchmark
```

The resulting program can test a bounded security theorem instead of asserting that an entire network category is “secure.”
