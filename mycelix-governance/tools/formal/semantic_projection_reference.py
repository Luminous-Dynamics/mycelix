#!/usr/bin/env python3
from dataclasses import dataclass, replace

KNOWN_FIELDS = frozenset({"operation", "target", "amount", "currency", "destination"})
REQUIRED_FIELDS = frozenset({"operation", "target", "amount", "currency"})

@dataclass(frozen=True)
class Projection:
    authorized_fields: frozenset[str]
    decision_value_fields: frozenset[str]
    effect_fields: frozenset[str]
    defaulted_fields: frozenset[str]
    authorized_schema: str
    effect_schema: str
    authorized_operation: str
    authorized_target: str
    authorized_amount: str
    authorized_currency: str
    projected_operation: str
    projected_target: str
    projected_amount: str
    projected_currency: str
    source_operation: str
    source_target: str
    source_amount: str
    source_currency: str

CANONICAL = Projection(
    authorized_fields=REQUIRED_FIELDS,
    decision_value_fields=REQUIRED_FIELDS,
    effect_fields=REQUIRED_FIELDS,
    defaulted_fields=frozenset(),
    authorized_schema="payment.v1",
    effect_schema="payment.v1",
    authorized_operation="transfer",
    authorized_target="acct-alice",
    authorized_amount="100",
    authorized_currency="USD",
    projected_operation="transfer",
    projected_target="acct-alice",
    projected_amount="100",
    projected_currency="USD",
    source_operation="operation",
    source_target="target",
    source_amount="amount",
    source_currency="currency",
)

def failures(p: Projection) -> list[str]:
    out: list[str] = []
    if not p.effect_fields <= KNOWN_FIELDS:
        out.append("EffectFieldsKnown")
    if not p.effect_fields <= p.authorized_fields:
        out.append("EffectFieldsAuthorized")
    if not REQUIRED_FIELDS <= p.effect_fields:
        out.append("RequiredFieldsPresent")
    if not (p.effect_fields - p.defaulted_fields) <= p.decision_value_fields:
        out.append("EffectFieldsDecisionBacked")
    if (
        p.source_operation,
        p.source_target,
        p.source_amount,
        p.source_currency,
    ) != ("operation", "target", "amount", "currency"):
        out.append("ProjectionSourcesExact")
    if "operation" in p.effect_fields and p.projected_operation != p.authorized_operation:
        out.append("ProjectedValuesConserved")
    if "target" in p.effect_fields and p.projected_target != p.authorized_target:
        out.append("ProjectedValuesConserved")
    if "amount" in p.effect_fields and p.projected_amount != p.authorized_amount:
        out.append("ProjectedValuesConserved")
    if "currency" in p.effect_fields and p.projected_currency != p.authorized_currency:
        out.append("ProjectedValuesConserved")
    if not p.defaulted_fields <= p.decision_value_fields:
        out.append("NoImplicitDefaultAuthority")
    if p.effect_schema != p.authorized_schema:
        out.append("SchemaProfileExact")
    return sorted(set(out))

assert failures(CANONICAL) == []

CASES = {
    "uncovered-field": replace(
        CANONICAL,
        effect_fields=CANONICAL.effect_fields | {"destination"},
        decision_value_fields=CANONICAL.decision_value_fields | {"destination"},
    ),
    "unbacked-field": replace(
        CANONICAL,
        authorized_fields=CANONICAL.authorized_fields | {"destination"},
        effect_fields=CANONICAL.effect_fields | {"destination"},
    ),
    "unknown-field": replace(
        CANONICAL,
        authorized_fields=CANONICAL.authorized_fields | {"x-extra"},
        decision_value_fields=CANONICAL.decision_value_fields | {"x-extra"},
        effect_fields=CANONICAL.effect_fields | {"x-extra"},
    ),
    "value-substitution": replace(CANONICAL, projected_amount="200"),
    "source-substitution": replace(CANONICAL, source_operation="target"),
    "implicit-default": replace(
        CANONICAL,
        decision_value_fields=CANONICAL.decision_value_fields - {"currency"},
        defaulted_fields=frozenset({"currency"}),
    ),
    "schema-profile": replace(CANONICAL, effect_schema="payment.v2"),
    "required-omission": replace(CANONICAL, effect_fields=CANONICAL.effect_fields - {"currency"}),
}

EXPECTED = {
    "uncovered-field": "EffectFieldsAuthorized",
    "unbacked-field": "EffectFieldsDecisionBacked",
    "unknown-field": "EffectFieldsKnown",
    "value-substitution": "ProjectedValuesConserved",
    "source-substitution": "ProjectionSourcesExact",
    "implicit-default": "NoImplicitDefaultAuthority",
    "schema-profile": "SchemaProfileExact",
    "required-omission": "RequiredFieldsPresent",
}

MARKERS = {
    "uncovered-field": "UNCOVERED field NEGATIVE PASS",
    "unbacked-field": "UNBACKED field NEGATIVE PASS",
    "unknown-field": "UNKNOWN field NEGATIVE PASS",
    "value-substitution": "VALUE substitution NEGATIVE PASS",
    "source-substitution": "SOURCE substitution NEGATIVE PASS",
    "implicit-default": "DEFAULT synthesis NEGATIVE PASS",
    "schema-profile": "SCHEMA substitution NEGATIVE PASS",
    "required-omission": "REQUIRED omission NEGATIVE PASS",
}

for name, case in CASES.items():
    actual = failures(case)
    assert actual == [EXPECTED[name]], (name, actual)
    print(MARKERS[name])

print("CANONICAL PASS: every executable field is covered by the exact authorized projection profile")
print("ISOLATION PASS: each negative mutates one semantic-projection condition")
print("NEGATIVE PASS: every semantic expansion/substitution is rejected")
