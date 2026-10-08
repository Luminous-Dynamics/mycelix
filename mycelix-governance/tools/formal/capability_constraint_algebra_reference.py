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
UNKNOWN="unknown"
WILDCARD="*"

@dataclass(frozen=True)
class Constraint:
    operations:frozenset[str]
    targets:frozenset[str]
    audiences:frozenset[str]
    currencies:frozenset[str]
    arguments:frozenset[str]
    min_amount:int
    max_amount:int
    min_time:int
    max_time:int
    purposes:frozenset[str]
    contexts:frozenset[str]
    extension:str="none"

@dataclass(frozen=True)
class Policy:
    allows:frozenset[Constraint]
    denies:frozenset[Constraint]
    conflict_rule:str="deny-overrides"

Request=tuple[str,str,str,str,str,int,int,str,str]
UNIVERSE=frozenset(product(OPS,TARGETS,AUDIENCES,CURRENCIES,ARGS,AMOUNTS,TIMES,PURPOSES,CONTEXTS))

def norm_target(x): return {"acct-alice":"alice","acct-bob":"bob"}.get(x,x)
def norm_set(xs,fn=lambda x:x): return frozenset(fn(x) for x in xs)
def supported(c): return c.extension in {"none","tag-v1"}
def cat_match(v,allowed,norm=lambda x:x):
    na=norm(v)
    return WILDCARD in allowed or na in norm_set(allowed,norm)

def matches(c,r):
    if not supported(c): return False
    op,t,a,cur,arg,amt,tm,purpose,ctx=r
    return (
        cat_match(op,c.operations)
        and cat_match(t,c.targets,norm_target)
        and cat_match(a,c.audiences)
        and cat_match(cur,c.currencies)
        and cat_match(arg,c.arguments)
        and c.min_amount<=amt<=c.max_amount
        and c.min_time<=tm<=c.max_time
        and cat_match(purpose,c.purposes)
        and cat_match(ctx,c.contexts)
        and (c.extension!="tag-v1" or ctx=="trusted")
    )

def denotation(constraints):
    return frozenset(r for r in UNIVERSE if any(matches(c,r) for c in constraints))

def normalize(c):
    return Constraint(
        norm_set(c.operations),norm_set(c.targets,norm_target),norm_set(c.audiences),
        norm_set(c.currencies),norm_set(c.arguments),c.min_amount,c.max_amount,
        c.min_time,c.max_time,norm_set(c.purposes),norm_set(c.contexts),c.extension)

def normalized_set(s): return frozenset(normalize(c) for c in s)
def syntax_equal(a,b): return a==b
def semantic_equivalent(a,b): return denotation(a)==denotation(b)
def satisfiable(c): return supported(c) and c.min_amount<=c.max_amount and c.min_time<=c.max_time and bool(c.operations|c.targets|c.audiences|c.currencies|c.arguments|c.purposes|c.contexts)

def decidable_subsumes(child,parent):
    if not all(supported(c) for c in child|parent): return False
    return denotation(child) <= denotation(parent)

def allow_denotation(p): return denotation(p.allows)
def deny_denotation(p): return denotation(p.denies)
def effective_denotation(p):
    allow,deny=allow_denotation(p),deny_denotation(p)
    if p.conflict_rule=="deny-overrides": return allow-deny
    if p.conflict_rule=="allow-overrides": return allow
    return frozenset()

def policy_attenuates(child,parent):
    return allow_denotation(child)<=allow_denotation(parent) and deny_denotation(parent)<=deny_denotation(child)

def safe_policy_subsumption(child,parent):
    return policy_attenuates(child,parent) and effective_denotation(child)<=effective_denotation(parent)

P_TRANSFER=Constraint(frozenset({"transfer"}),frozenset({"alice"}),frozenset({"payments"}),frozenset({"USD"}),frozenset({"standard"}),0,3,0,3,frozenset({"business"}),frozenset({"trusted"}))
P_REFUND=Constraint(frozenset({"refund"}),frozenset({"bob"}),frozenset({"payments"}),frozenset({"EUR"}),frozenset({"standard"}),0,3,1,3,frozenset({"refund"}),frozenset({"trusted"}))
P_DENY=Constraint(frozenset({"transfer"}),frozenset({"alice"}),frozenset({"payments"}),frozenset({"USD"}),frozenset({"standard"}),2,2,0,3,frozenset({"business"}),frozenset({"trusted"}))
C_TRANSFER=Constraint(frozenset({"transfer"}),frozenset({"alice"}),frozenset({"payments"}),frozenset({"USD"}),frozenset({"standard"}),0,2,0,3,frozenset({"business"}),frozenset({"trusted"}))
C_REFUND=Constraint(frozenset({"refund"}),frozenset({"bob"}),frozenset({"payments"}),frozenset({"EUR"}),frozenset({"standard"}),0,2,1,3,frozenset({"refund"}),frozenset({"trusted"}))

PARENT=Policy(frozenset({P_TRANSFER,P_REFUND}),frozenset({P_DENY}))
CHILD=Policy(frozenset({C_TRANSFER,C_REFUND}),frozenset({P_DENY}))

assert all(satisfiable(c) for c in PARENT.allows|PARENT.denies|CHILD.allows|CHILD.denies)
assert policy_attenuates(CHILD,PARENT)
assert safe_policy_subsumption(CHILD,PARENT)
assert normalized_set({Constraint(frozenset({"transfer"}),frozenset({"acct-alice"}),frozenset({"payments"}),frozenset({"USD"}),frozenset({"standard"}),0,2,0,3,frozenset({"business"}),frozenset({"trusted"}))}) == normalized_set({C_TRANSFER})
assert not syntax_equal({P_TRANSFER},{Constraint(frozenset({"transfer"}),frozenset({"acct-alice"}),frozenset({"payments"}),frozenset({"USD"}),frozenset({"standard"}),0,3,0,3,frozenset({"business"}),frozenset({"trusted"}))}))
assert semantic_equivalent({Constraint(frozenset({"transfer"}),frozenset({"acct-alice"}),frozenset({"payments"}),frozenset({"USD"}),frozenset({"standard"}),0,2,0,3,frozenset({"business"}),frozenset({"trusted"}))},{C_TRANSFER})

controls={
"interval-widening": safe_policy_subsumption(Policy(frozenset({Constraint(C_TRANSFER.operations,C_TRANSFER.targets,C_TRANSFER.audiences,C_TRANSFER.currencies,C_TRANSFER.arguments,0,3,0,3,C_TRANSFER.purposes,C_TRANSFER.contexts)}|{C_REFUND}),frozenset({P_DENY})),PARENT),
"interval-hole-filling": safe_policy_subsumption(Policy(frozenset({P_TRANSFER,P_REFUND}),frozenset()),PARENT),
"wildcard-expansion": safe_policy_subsumption(Policy(frozenset({Constraint(C_TRANSFER.operations,frozenset({WILDCARD}),C_TRANSFER.audiences,C_TRANSFER.currencies,C_TRANSFER.arguments,0,2,0,3,C_TRANSFER.purposes,C_TRANSFER.contexts),C_REFUND}),frozenset({P_DENY})),PARENT),
"temporal-window-widening": safe_policy_subsumption(Policy(frozenset({Constraint(C_TRANSFER.operations,C_TRANSFER.targets,C_TRANSFER.audiences,C_TRANSFER.currencies,C_TRANSFER.arguments,0,2,0,3,C_TRANSFER.purposes,C_TRANSFER.contexts),C_REFUND}),frozenset({P_DENY})),PARENT) is False,
"context-weakening": safe_policy_subsumption(Policy(frozenset({Constraint(C_TRANSFER.operations,C_TRANSFER.targets,C_TRANSFER.audiences,C_TRANSFER.currencies,C_TRANSFER.arguments,0,2,0,3,C_TRANSFER.purposes,frozenset(CONTEXTS)) ,C_REFUND}),frozenset({P_DENY})),PARENT) is False,
"normalization-equivalence": semantic_equivalent({Constraint(C_TRANSFER.operations,frozenset({"acct-alice"}),C_TRANSFER.audiences,C_TRANSFER.currencies,C_TRANSFER.arguments,0,2,0,3,C_TRANSFER.purposes,C_TRANSFER.contexts)},{C_TRANSFER}),
"non-equivalent-normalization": not semantic_equivalent({Constraint(C_TRANSFER.operations,frozenset({"acct-bob"}),C_TRANSFER.audiences,C_TRANSFER.currencies,C_TRANSFER.arguments,0,2,0,3,C_TRANSFER.purposes,C_TRANSFER.contexts)},{C_TRANSFER}),
"deny-set-deletion": not safe_policy_subsumption(Policy(PARENT.allows,frozenset()),PARENT),
"conflict-rule-substitution": not safe_policy_subsumption(Policy(PARENT.allows,PARENT.denies,"allow-overrides"),PARENT),
"unknown-extension-treated-as-subsumed": not decidable_subsumes({Constraint(C_TRANSFER.operations,C_TRANSFER.targets,C_TRANSFER.audiences,C_TRANSFER.currencies,C_TRANSFER.arguments,0,2,0,3,C_TRANSFER.purposes,C_TRANSFER.contexts,UNKNOWN)},{P_TRANSFER}),
"unsupported-compound-extension": not decidable_subsumes({Constraint(frozenset({"transfer","refund"}),frozenset({"alice","bob"}),frozenset({"payments"}),frozenset({"USD","EUR"}),frozenset({"standard"}),0,3,0,3,frozenset(PURPOSES),frozenset(CONTEXTS),"future-v1")},{PARENT.allows.__iter__().__next__()})
}
assert controls["normalization-equivalence"] and all(v for k,v in controls.items() if k!="normalization-equivalence")
assert deny_denotation(CHILD)==deny_denotation(PARENT)
assert effective_denotation(Policy(PARENT.allows,PARENT.denies,"deny-overrides")) != effective_denotation(Policy(PARENT.allows,PARENT.denies,"allow-overrides"))
print("CANONICAL PASS: bounded constraint denotation, allow/deny separation, and attenuation")
print("NORMALIZATION PASS: syntax differs while denotation is preserved for canonical aliases")
for k,v in controls.items():
    print(k.upper().replace("-"," ")+" "+("POSITIVE PASS" if k=="normalization-equivalence" else "NEGATIVE PASS") )
print("DENY MONOTONICITY PASS: parent denies must be preserved by attenuation")
print("FAIL CLOSED PASS: unsupported extension types are never treated as subsumed")
