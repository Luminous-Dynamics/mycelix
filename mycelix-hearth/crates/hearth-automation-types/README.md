# hearth-automation-types

Pure-Rust contracts for Mycelix Hearth household automation.

This crate is intentionally **not** an automation engine and **not** a device integration layer. It defines the stable semantic boundary shared by the Hearth DHT/zome layer, a future local `hearth-edge` executor, and user interfaces.

## Design rules

1. **Household intent, not device syntax.** Plans name semantic capabilities and entities. Matter clusters, Home Assistant entity IDs, MQTT topics, vendor APIs, and driver details belong behind edge adapters.
2. **No floating-point durable state.** Automation values use integers, basis points, or explicit fixed-point values.
3. **Authority is explicit and fail-closed.** A3+ execution requires authority. A5 critical actions require explicit approval, a household decision, or explicit prohibition.
4. **Command acceptance is not outcome verification.** `CommandStatus` and `VerificationStatus` are separate types. A completed automation receipt must carry a verified final outcome.
5. **High-consequence actions need observable outcomes.** A3+ intents and plan steps require at least one verification expectation and a positive verification window.
6. **The execution graph is a DAG.** Plans reject unknown dependencies, self-dependencies, duplicate step IDs, and cycles.
7. **Compensation is explicit.** A `Compensatable` plan step must define its compensation action.
8. **Simulation/trial is first-class.** `AutomationLifecycle::Trial` has an expiry. Learning systems should propose and trial routines rather than silently mutate permanent household policy.
9. **Telemetry is evidence, not authority.** Observations carry provenance, timestamps, confidence, and optional validity windows.
10. **Adapters remain local.** Nothing in this crate selects or embeds a physical-device adapter.

## Consequence ladder

| Class | Meaning | Default authority posture |
|---|---|---|
| A0 `Observe` | Read/observe only | May be automatic |
| A1 `Organize` | Change household information/task organization | May be automatic |
| A2 `Recommend` | Suggest an action to a person | May be automatic |
| A3 `ReversibleAct` | Reversible physical/digital side effect | Explicit capability or stronger |
| A4 `ConsequentialAct` | Purchase, access, energy export, similar consequence | Explicit capability/approval/decision |
| A5 `CriticalAct` | Safety-critical or otherwise critical | Explicit approval or household decision |

The policy kernel added in HTH-AUTO-002 should make authority *more* restrictive where household policy requires it; these types only define the minimum structural floor.

## Intended layering

```text
Hearth semantic domains
    care / rhythms / resources / autonomy / emergency
                          |
                          v
                household intent
                          |
                          v
               authority + policy
                          |
                          v
                 deterministic plan
                          |
                          v
                    hearth-edge
                          |
          +---------------+---------------+
          |               |               |
        Matter      Home Assistant       other
        adapter         adapter         adapters
```

The durable Hearth layer should store meaningful policy, intent, approvals, and receipts. High-frequency device telemetry and credentials should stay local to the edge runtime unless a household explicitly chooses to preserve a meaningful event.

## Capability naming

Capabilities use stable dotted semantic names, for example:

- `home.lighting.control`
- `home.hvac.control`
- `home.care.schedule`
- `home.shopping.prepare`
- `home.shopping.purchase`
- `home.access.lock`
- `home.access.unlock`
- `home.vehicle.charge`
- `home.energy.export`
- `home.water.shutoff`
- `home.emergency.notify`

These are **not** adapter API names. An edge adapter resolves a capability onto available local hardware or services.

## HTH-AUTO-001 scope

This tranche freezes the ontology only:

- intent
- triggers and conditions
- deterministic values
- evidence requirements
- consequence classes
- reversibility
- authority requirements
- semantic actions
- outcome verification
- deterministic plan DAGs
- command/verification receipts
- inhibition and execution state

It deliberately does **not** implement:

- authorization resolution against Hearth autonomy profiles
- scheduling/execution
- Home Assistant/Matter/OCPP/OpenADR adapters
- AI-generated plans
- automatic learning
- DHT entry/integrity types
- device credentials or secrets

Those belong to subsequent tranches.
