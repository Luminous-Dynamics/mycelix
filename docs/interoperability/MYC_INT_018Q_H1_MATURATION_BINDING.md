# MYC-INT-018Q — H1 Evidence → Node Maturation Binding

Status: design/binding fixture only. Tracks #3297. Child of the H1 package/validator line and cross-references the 007O–007R maturation evidence line.

## Purpose

Bind the current H1 wet-bench evidence surface to technical node-maturation requirements without allowing a successful instrumented subsystem to become a whole-node maturity claim.

```text
H1 subsystem evidence
!= whole-node maturation evidence

successful sensing/circulation
!= productive food loop

subsystem outage/recovery evidence
!= whole-node resilience
```

Canonical machine-readable binding:

`docs/interoperability/fixtures/MYC_INT_018Q_H1_MATURATION_BINDING.json`

## Relation classes

- `CanSatisfyUnderExactProfile` — the named H1 evidence can satisfy that requirement **at the explicitly declared subsystem scope** when real evidence is later produced under the exact profile.
- `ContributesButInsufficient` — H1 can contribute evidence but cannot establish the maturation requirement or transition by itself.
- `CannotSatisfy` — current H1 semantics do not contain the required evidence class.
- `NotApplicable` — the requirement is outside the H1 subject.

None means the requirement is currently satisfied. 018Q is a mapping fixture, not run evidence.

## N0→N1

H1 can support `local-acquisition-or-evidence-path` because the frozen wet-bench profile defines local acquisition channels, explicit missing/stale/conflict semantics, observation export expectations, and runtime optionality for Mycelix/Holochain/Symthaea/Fleet/network.

H1 can only **contribute** to `dependency-map`. It can enumerate the bench's hardware, process, energy, water, calibration, operator and software dependencies, but it cannot describe the full node's food, health, legal/finance, skills, critical-import and other dimensions.

```text
H1 dependency inventory
!= node dependency map
```

## N1→N2

Current H1a/H1b/H1c are instrumentation/process stages, not a productive-output profile.

```text
clean-water circulation
!= food production

nutrient-solution sensing
!= crop growth
!= harvest
!= useful output delivered
```

Therefore H1 cannot satisfy the current frozen N1→N2 requirements:

- `at-least-one-real-productive-loop`;
- `work-material-observations`;
- `outcome-feedback`.

A later productive-loop profile must add exact input/work/output/loss/outcome evidence. Do not silently widen H1 to do this.

## N2→N3

H1 is useful here, but only as a subsystem contributor.

The frozen H1 stack already contains candidate evidence hooks for:

- FI-12 restart/power cycle;
- FI-13 network partition;
- FI-16 manual override;
- FI-17 unauthorized actuation attempt;
- local manual stop/abort;
- local acquisition without Mycelix/Holochain/Symthaea/Fleet/network;
- retained fault/manual-intervention/abort evidence.

This can contribute to `bounded-outage-continuity-evidence` and `failure-recovery-evidence`.

The H1 install/run profile can satisfy a **local H1 safe-stop requirement under exact profile** if the required physical evidence is later produced.

But:

```text
H1 safe stop established
!= whole-node N3 established
```

Other node capabilities remain separately evidenced or missing.

## Domain boundaries

### Water

H1 may observe flow, level, conditional volume, solution temperature and leak state.

```text
reservoir recirculation
!= node water independence
```

A node-level water-dependency claim requires declared total-water boundaries/denominators and evidence outside H1 where relevant.

### Energy

H1 may observe subsystem power and conditional energy.

```text
H1 energy measured
!= node energy independence
```

### Food

Current H1 has no crop-production/harvest semantics.

### Repair/recovery

H1 can preserve faults, restart behavior, manual interventions and recovery evidence, but does not currently model a complete repair-work loop.

### Compute/network

H1 can contribute evidence that local acquisition continues during bounded network/federation outages.

### Governance / standing / federation

H1 grants none.

## Source classes

Future H1-to-maturation evidence should retain 007Q source-class distinctions. Physical sensor observations can become `DirectObservation` under the exact measurement profile; run/package facts may be `SourceOwnedOperationalFact`; reconstructed values and Symthaea findings remain their own source classes.

```text
Symthaea analysis
!= H1 physical observation
```

## Whole-node extrapolation prohibition

Every binding carries a scope ceiling. No H1 result may automatically change another maturation dimension.

Examples:

```text
H1 water loop success
!= food locally available
!= health support locally available
!= critical imports reduced
!= legal/finance dependency reduced

H1 partition survival
!= whole-node resilience
```

## Productive-loop gap

The largest intentional gap after 018Q is a reusable **ProductiveLoop** evidence profile. Hydroponics can be the first concrete conformer, but the semantic contract should also allow later productive loops such as fabrication, repair, water treatment or energy service.

A ProductiveLoop profile should bind:

- exact useful-output subject;
- bounded production window;
- material/input observations;
- work observations;
- process observations;
- useful-output observations;
- loss/waste/failure observations;
- outcome feedback;
- external dependencies/imports;
- corrections/supersession;
- no whole-node extrapolation.

## Nonclaims

018Q does not establish N1, N2 or N3 for any real node; food production; crop performance; agronomic qualification; whole-node resilience; economic independence; ecological sustainability; legal compliance; governance legitimacy; or federation authority.
