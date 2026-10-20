#!/usr/bin/env python3
from dataclasses import dataclass, replace

@dataclass(frozen=True)
class Scope:
    operations:frozenset[str]
    targets:frozenset[str]
    currencies:frozenset[str]
    max_amount:int

@dataclass(frozen=True)
class Projection:
    decision_operations:frozenset[str]
    decision_targets:frozenset[str]
    decision_currencies:frozenset[str]
    decision_max_amount:int
    profile_operations:frozenset[str]
    profile_targets:frozenset[str]
    profile_currencies:frozenset[str]
    profile_max_amount:int
    effect_operations:frozenset[str]
    effect_targets:frozenset[str]
    effect_currencies:frozenset[str]
    effect_max_amount:int
    known_fields:frozenset[str]
    decision_fields:frozenset[str]
    effect_fields:frozenset[str]
    profile_derived_fields:frozenset[str]
    declared_derivation_fields:frozenset[str]
    defaulted_fields:frozenset[str]
    external_fields:frozenset[str]
    source_operation:str
    source_target:str
    source_amount:str
    source_currency:str
    profile_id:str
    implementation_hash:str
    source_schema:str
    target_schema:str
    conversion_factor:int
    rendered_amount:int
    context_digest:str
    execution_context_digest:str
    used_derivation_rule:str

CANONICAL=Projection(
    frozenset({"transfer"}),frozenset({"acct-alice","acct-bob"}),frozenset({"USD"}),100,
    frozenset({"transfer"}),frozenset({"acct-alice"}),frozenset({"USD"}),50,
    frozenset({"transfer"}),frozenset({"acct-alice"}),frozenset({"USD"}),40,
    frozenset({"operation","target","amount","currency","destination","risk_score"}),
    frozenset({"operation","target","amount","currency"}),
    frozenset({"operation","target","amount","currency"}),
    frozenset(),frozenset(),frozenset(),frozenset(),
    "operation","target","amount","currency",
    "payment-attenuate-v1","sha256:transform-v1",
    "payment.v1","payment.v1",100,4000,
    "ctx-canonical-v1","ctx-canonical-v1","none",
)

def failures(p:Projection)->list[str]:
    out=[]
    if not p.profile_operations <= p.decision_operations or not p.profile_targets <= p.decision_targets or not p.profile_currencies <= p.decision_currencies or p.profile_max_amount > p.decision_max_amount:
        out.append("ProfileCeilingNarrowed")
    if not p.effect_operations <= p.profile_operations or not p.effect_targets <= p.profile_targets or not p.effect_currencies <= p.profile_currencies or p.effect_max_amount > p.profile_max_amount:
        out.append("EffectScopeNarrowed")
    if not p.effect_fields <= p.known_fields:
        out.append("EffectFieldsKnown")
    if not (p.effect_fields - p.profile_derived_fields - p.defaulted_fields) <= p.decision_fields:
        out.append("EffectFieldsBacked")
    if not p.profile_derived_fields <= p.declared_derivation_fields or ((p.profile_derived_fields and p.used_derivation_rule!="canonical") or (not p.profile_derived_fields and p.used_derivation_rule!="none")):
        out.append("DeclaredDerivationExact")
    if (p.source_operation,p.source_target,p.source_amount,p.source_currency)!=("operation","target","amount","currency"):
        out.append("FieldSourcesExact")
    if p.profile_id!="payment-attenuate-v1":
        out.append("ProfileIdentityExact")
    if p.implementation_hash!="sha256:transform-v1":
        out.append("ImplementationIdentityExact")
    if p.source_schema!="payment.v1" or p.target_schema!="payment.v1":
        out.append("SchemaIdentityExact")
    if p.conversion_factor!=100 or p.rendered_amount != p.effect_max_amount*p.conversion_factor:
        out.append("ConversionRuleExact")
    if p.defaulted_fields:
        out.append("NoImplicitDefaults")
    if p.external_fields:
        out.append("NoExternalEnrichment")
    if p.context_digest!="ctx-canonical-v1" or p.execution_context_digest!="ctx-canonical-v1":
        out.append("DeterministicContextExact")
    return sorted(set(out))

assert failures(CANONICAL)==[]

CASES={
"undeclared-derivation":replace(CANONICAL,profile_derived_fields=frozenset({"destination"})),
"privilege-widening":replace(CANONICAL,effect_max_amount=60,rendered_amount=6000),
"cross-field-contamination":replace(CANONICAL,source_target="amount"),
"profile-substitution":replace(CANONICAL,profile_id="payment-attenuate-v2"),
"implementation-hash-substitution":replace(CANONICAL,implementation_hash="sha256:transform-v2"),
"conversion-rule-substitution":replace(CANONICAL,conversion_factor=50,rendered_amount=2000),
"default-reconstruction":replace(CANONICAL,defaulted_fields=frozenset({"currency"})),
"external-enrichment":replace(CANONICAL,external_fields=frozenset({"risk_score"})),
"unbound-context":replace(CANONICAL,execution_context_digest="ctx-runtime-v7"),
}
EXPECTED={
"profile-ceiling-widening":["ProfileCeilingNarrowed"],
"undeclared-derivation":["DeclaredDerivationExact"],
"privilege-widening":["EffectScopeNarrowed"],
"cross-field-contamination":["FieldSourcesExact"],
"profile-substitution":["ProfileIdentityExact"],
"implementation-hash-substitution":["ImplementationIdentityExact"],
"conversion-rule-substitution":["ConversionRuleExact"],
"default-reconstruction":["NoImplicitDefaults"],
"external-enrichment":["NoExternalEnrichment"],
"unbound-context":["DeterministicContextExact"],
}
MARKERS={
"profile-ceiling-widening":"PROFILE CEILING widening NEGATIVE PASS",
"undeclared-derivation":"UNDECLARED derivation NEGATIVE PASS",
"privilege-widening":"PRIVILEGE widening NEGATIVE PASS",
"cross-field-contamination":"CROSS-FIELD contamination NEGATIVE PASS",
"profile-substitution":"PROFILE substitution NEGATIVE PASS",
"implementation-hash-substitution":"IMPLEMENTATION substitution NEGATIVE PASS",
"conversion-rule-substitution":"CONVERSION rule NEGATIVE PASS",
"default-reconstruction":"DEFAULT reconstruction NEGATIVE PASS",
"external-enrichment":"EXTERNAL enrichment NEGATIVE PASS",
"unbound-context":"CONTEXT dependence NEGATIVE PASS",
}
for name,case in CASES.items():
    assert failures(case)==EXPECTED[name],(name,failures(case))
    print(MARKERS[name])
print("CANONICAL PASS: effect authority is bounded by the transform profile and decision authority")
print("ISOLATION PASS: each negative mutates one transformation-semantic condition")
print("NEGATIVE PASS: semantic transformation cannot expand modeled authority")
