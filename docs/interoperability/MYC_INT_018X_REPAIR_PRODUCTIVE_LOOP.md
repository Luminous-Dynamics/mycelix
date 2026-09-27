# MYC-INT-018X — RepairProductiveLoopV1

Status: design/synthetic conformer only. Tracks #3313. Sibling of the hydroponic H2 ProductiveLoop line.

## Purpose

Demonstrate that ProductiveLoopV1 can represent a non-food useful-output loop by composing existing Mycelix work-order/material/work capabilities with before/after capability evidence.

The first synthetic reference case uses restoration of the H1 circulation capability, but the conformer is intended to generalize to tools, machines, appliances, network equipment and other repairable assets.

Canonical fixture:

`docs/interoperability/fixtures/MYC_INT_018X_REPAIR_PRODUCTIVE_LOOP.json`

## Existing owners

Mycelix Manufacturing already owns work orders and their lifecycle, including product/quantity/due date/status/priority plus BOM/routing references and immutable status-update history. RepairProductiveLoop does not replace that owner.

Craft/Identity can later supply worker/skill/work-history references. Domain systems continue to own the actual before/after observations.

## Core separations

```text
repair demand
!= diagnosis

work order
!= work performed

planned part
!= consumed/replaced part

work order Completed
!= capability restored

one successful functional test
!= recurrence-free repair
```

## Useful output

For this conformer the ProductiveLoop useful output is:

`RestoredCapabilityUnderExactVerificationProfile`

not a newly manufactured object.

A restoration record must bind the exact asset/capability subject and a verification profile rather than merely copying a work-order status.

## First synthetic H1 case

The initial case begins with evidence compatible with H1's existing failure vocabulary:

```text
pump command = on
measured flow = absent/degraded
```

That condition is a failure observation, not an automatic diagnosis.

Possible interventions are recorded as action classes only when actually performed. The fixture must not infer that a blockage, failed pump, connector fault or another cause exists without evidence.

Post-repair verification should include the relevant combination of:

- requested/commanded state;
- electrical state where observed;
- measured flow under an exact test profile;
- leak/containment check;
- manual stop verification;
- residual limitation/deviation record.

A separate recurrence window is required after the immediate functional test.

## ProductiveLoop mapping

Inputs/materials:

- planned BOM/parts remain separate from parts actually consumed/replaced.

Work:

- work events remain separate from compensation, ITC, reputation or governance standing.

Process:

- inspections/interventions stay domain-owned evidence.

Useful output:

- restored capability under exact verification profile.

Loss/failure:

- failed attempts, damaged/rejected parts, unresolved defects, downtime and recurrence remain explicit.

Outcome feedback:

- restoration result, operator/user verification where applicable, recurrence during the observation window and remaining unknowns.

## N1→N2 boundary

A future physical evidence-complete repair loop can contribute a real useful-output loop plus work/material observations and outcome feedback.

It still does not automatically establish N2 for the whole node; the maturation transition record remains the separate evaluator.

## Nonclaims

018X does not establish a physical repair, worker qualification, restored capability, compensation entitlement, safety qualification, N2 maturity, whole-node resilience or economic value.
