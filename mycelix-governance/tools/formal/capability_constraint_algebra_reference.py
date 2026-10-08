from __future__ import annotations
from dataclasses import dataclass
from itertools import product

OPS=("transfer","refund")
TARGETS=("alice","bob")
AUDIENCES=("payments","admin")
CURRENCIES=("USD","EUR")
ARGS=("standard","sensitive")
PURPOSES=("business","refund")
CONTEXTS=("trusted","untrusted")
AMOUNTS=range(0,4)
TIMES=range(0,4)
WILDCARD="*"
UNSUPPORTED={"unknown","future-v1"}

@dataclass(frozen=True)
class Constraint:
    operations:frozenset[str]; targets:frozenset[str]; audiences:frozenset[str]
    currencies:frozenset[str]; arguments:frozenset[str]
    min_amount:int; max_amount:int; min_time:int; max_time:int
    purposes:frozenset[str]; contexts:frozenset[str]; extension:str="none"

@dataclass(frozen=True)
class Policy:
    allows:frozenset[Constraint]; denies:frozenset[Constraint]; conflict_rule:str="deny-overrides"

Request=tuple[str,str,str,str,str,int,int,str,str]
UNIVERSE=frozenset(product(OPS,TARGETS,AUDIENCES,CURRENCIES,ARGS,AMOUNTS,TIMES,PURPOSES,CONTEXTS))

def norm_target(x:str)->str:
    return {"acct-alice":"alice","acct-bob":"bob"}.get(x,x)

def norm_set(xs,fn=lambda x:x): return frozenset(fn(x) for x in xs)
def supported(c:Constraint)->bool: return c.extension not in UNSUPPORTED and c.extension in {"none","tag-v1"}

def cat_match(value,allowed,norm=lambda x:x)->bool:
    n=norm(value)
    return WILDCARD in allowed or n in norm_set(allowed,norm)

def matches(c:Constraint,r:Request)->bool:
    if not supported(c): return False
    op,t,a,cur,arg,amt,tm,purpose,ctx=r
    return (
        cat_match(op,c.operations) and cat_match(t,c.targets,norm_target)
        and cat_match(a,c.audiences) and cat_match(cur,c.currencies)
        and cat_match(arg,c.arguments)
        and c.min_amount<=amt<=c.max_amount
        and c.min_time<=tm<=c.max_time
        and cat_match(purpose,c.purposes) and cat_match(ctx,c.contexts)
        and (c.extension!="tag-v1" or ctx=="trusted")
    )

def denotation(constraints): return frozenset(r for r in UNIVERSE if any(matches(c,r) for c in constraints))

def normalize(c:Constraint)->Constraint:
    return Constraint(norm_set(c.operations),norm_set(c.targets,norm_target),norm_set(c.audiences),
        norm_set(c.currencies),norm_set(c.arguments),c.min_amount,c.max_amount,c.min_time,c.max_time,
        norm_set(c.purposes),norm_set(c.contexts),c.extension)

def normalized_set(s): return frozenset(normalize(c) for c in s)
def syntax_equal(a,b): return a==b
def semantic_equivalent(a,b): return denotation(a)==denotation(b)

def satisfiable(c:Constraint)->bool:
    return (supported(c) and c.min_amount<=c.max_amount and c.min_time<=c.max_time
            and all(bool(x) for x in (c.operations,c.targets,c.audiences,c.currencies,c.arguments,c.purposes,c.contexts)))

def decidable_subsumes(child,parent)->bool:
    if not all(supported(c) for c in child|parent): return False
    return denotation(child) <= denotation(parent)

def allow_denotation(p): return denotation(p.allows)
def deny_denotation(p): return denotation(p.denies)
def effective_denotation(p):
    if p.conflict_rule=="deny-overrides": return allow_denotation(p)-deny_denotation(p)
    if p.conflict_rule=="allow-overrides": return allow_denotation(p)
    return frozenset()

def policy_attenuates(child,parent)->bool:
    return allow_denotation(child)<=allow_denotation(parent) and deny_denotation(parent)<=deny_denotation(child)

def safe_policy_subsumption(child,parent)->bool:
    return policy_attenuates(child,parent) and effective_denotation(child)<=effective_denotation(parent)

P_TRANSFER=Constraint(frozenset({"transfer"}),frozenset({"alice"}),frozenset({"payments"}),frozenset({"USD"}),frozenset({"standard"}),0,2,1,3,frozenset({"business"}),frozenset({"trusted"}))
P_REFUND=Constraint(frozenset({"refund"}),frozenset({"bob"}),frozenset({"payments"}),frozenset({"EUR"}),frozenset({"standard"}),0,3,1,3,frozenset({"refund"}),frozenset({"trusted"}))
P_DENY=Constraint(frozenset({"transfer"}),frozenset({"alice"}),frozenset({"payments"}),frozenset({"USD"}),frozenset({"standard"}),2,2,1,3,frozenset({"business"}),frozenset({"trusted"}))
C_TRANSFER=Constraint(frozenset({"transfer"}),frozenset({"alice"}),frozenset({"payments"}),frozenset({"USD"}),frozenset({"standard"}),0,1,1,3,frozenset({"business"}),frozenset({"trusted"}))
C_REFUND=Constraint(frozenset({"refund"}),frozenset({"bob"}),frozenset({"payments"}),frozenset({"EUR"}),frozenset({"standard"}),0,2,1,3,frozenset({"refund"}),frozenset({"trusted"}))

PARENT=Policy(frozenset({P_TRANSFER,P_REFUND}),frozenset({P_DENY}))
CHILD=Policy(frozenset({C_TRANSFER,C_REFUND}),frozenset({P_DENY}))
assert all(satisfiable(c) for c in PARENT.allows|PARENT.denies|CHILD.allows|CHILD.denies)
assert policy_attenuates(CHILD,PARENT) and safe_policy_subsumption(CHILD,PARENT)

alias_child=Constraint(C_TRANSFER.operations,frozenset({"acct-alice"}),C_TRANSFER.audiences,C_TRANSFER.currencies,C_TRANSFER.arguments,C_TRANSFER.min_amount,C_TRANSFER.max_amount,C_TRANSFER.min_time,C_TRANSFER.max_time,C_TRANSFER.purposes,C_TRANSFER.contexts)
assert syntax_equal({alias_child},{alias_child}) and not syntax_equal({alias_child},{C_TRANSFER})
assert semantic_equivalent({alias_child},{C_TRANSFER})
assert normalized_set({alias_child})==normalized_set({C_TRANSFER})

interval_widen=Constraint(C_TRANSFER.operations,C_TRANSFER.targets,C_TRANSFER.audiences,C_TRANSFER.currencies,C_TRANSFER.arguments,0,3,C_TRANSFER.min_time,C_TRANSFER.max_time,C_TRANSFER.purposes,C_TRANSFER.contexts)
wildcard=Constraint(C_TRANSFER.operations,frozenset({WILDCARD}),C_TRANSFER.audiences,C_TRANSFER.currencies,C_TRANSFER.arguments,C_TRANSFER.min_amount,C_TRANSFER.max_amount,C_TRANSFER.min_time,C_TRANSFER.max_time,C_TRANSFER.purposes,C_TRANSFER.contexts)
time_widen=Constraint(C_TRANSFER.operations,C_TRANSFER.targets,C_TRANSFER.audiences,C_TRANSFER.currencies,C_TRANSFER.arguments,C_TRANSFER.min_amount,C_TRANSFER.max_amount,0,3,C_TRANSFER.purposes,C_TRANSFER.contexts)
context_weak=Constraint(C_TRANSFER.operations,C_TRANSFER.targets,C_TRANSFER.audiences,C_TRANSFER.currencies,C_TRANSFER.arguments,C_TRANSFER.min_amount,C_TRANSFER.max_amount,C_TRANSFER.min_time,C_TRANSFER.max_time,C_TRANSFER.purposes,frozenset(CONTEXTS))
non_equiv=Constraint(C_TRANSFER.operations,frozenset({"acct-bob"}),C_TRANSFER.audiences,C_TRANSFER.currencies,C_TRANSFER.arguments,C_TRANSFER.min_amount,C_TRANSFER.max_amount,C_TRANSFER.min_time,C_TRANSFER.max_time,C_TRANSFER.purposes,C_TRANSFER.contexts)
unknown=Constraint(C_TRANSFER.operations,C_TRANSFER.targets,C_TRANSFER.audiences,C_TRANSFER.currencies,C_TRANSFER.arguments,C_TRANSFER.min_amount,C_TRANSFER.max_amount,C_TRANSFER.min_time,C_TRANSFER.max_time,C_TRANSFER.purposes,C_TRANSFER.contexts,"unknown")
future=Constraint(frozenset({"transfer","refund"}),frozenset({"alice","bob"}),frozenset({"payments"}),frozenset({"USD","EUR"}),frozenset({"standard"}),0,3,0,3,frozenset(PURPOSES),frozenset(CONTEXTS),"future-v1")

controls={
"interval-widening":not safe_policy_subsumption(Policy(frozenset({interval_widen,C_REFUND}),frozenset({P_DENY})),PARENT),
"interval-hole-filling":not safe_policy_subsumption(Policy(PARENT.allows,frozenset()),PARENT),
"wildcard-expansion":not safe_policy_subsumption(Policy(frozenset({wildcard,C_REFUND}),frozenset({P_DENY})),PARENT),
"temporal-window-widening":not safe_policy_subsumption(Policy(frozenset({time_widen,C_REFUND}),frozenset({P_DENY})),PARENT),
"context-weakening":not safe_policy_subsumption(Policy(frozenset({context_weak,C_REFUND}),frozenset({P_DENY})),PARENT),
"normalization-equivalence":semantic_equivalent({alias_child},{C_TRANSFER}) and not syntax_equal({alias_child},{C_TRANSFER}),
"non-equivalent-normalization":not semantic_equivalent({non_equiv},{C_TRANSFER}),
"deny-set-deletion":not safe_policy_subsumption(Policy(PARENT.allows,frozenset()),PARENT),
"conflict-rule-substitution":not safe_policy_subsumption(Policy(PARENT.allows,PARENT.denies,"allow-overrides"),PARENT),
"unknown-extension-treated-as-subsumed":not decidable_subsumes({unknown},{P_TRANSFER}),
"unsupported-compound-extension":not decidable_subsumes({future},{P_TRANSFER}),
}
assert controls["normalization-equivalence"]
assert all(v for k,v in controls.items() if k!="normalization-equivalence")
assert deny_denotation(CHILD)==deny_denotation(PARENT)
assert effective_denotation(Policy(PARENT.allows,PARENT.denies,"deny-overrides")) < effective_denotation(Policy(PARENT.allows,PARENT.denies,"allow-overrides"))
print("CANONICAL PASS: bounded constraint denotation, allow/deny separation, and policy attenuation")
print("NORMALIZATION PASS: syntax differs while denotation is preserved for canonical aliases")
for k,v in controls.items(): print(k.upper().replace("-"," ")+" "+("POSITIVE PASS" if k=="normalization-equivalence" else "NEGATIVE PASS"))
print("DENY MONOTONICITY PASS: parent denies are preserved")
print("FAIL CLOSED PASS: unsupported extension types are never treated as subsumed")
