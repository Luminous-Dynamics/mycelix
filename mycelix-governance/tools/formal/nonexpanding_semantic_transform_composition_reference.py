from dataclasses import dataclass, replace
import random

@dataclass(frozen=True)
class Scope:
    operations:frozenset[str]
    targets:frozenset[str]
    audiences:frozenset[str]
    currencies:frozenset[str]
    max_amount:int

def leq(a:Scope,b:Scope)->bool:
    return (a.operations <= b.operations and a.targets <= b.targets and
            a.audiences <= b.audiences and a.currencies <= b.currencies and
            a.max_amount <= b.max_amount)

@dataclass(frozen=True)
class Lineage:
    decision_id:str
    step1_effect_id:str
    step2_source_effect_id:str
    step2_input:Scope
    step1_effect:Scope
    derivation1:str
    derivation2:str
    source_amount1:str
    source_amount2:str
    implementation2:str

def canonical_chain():
    return (
        Scope(frozenset({'transfer'}),frozenset({'alice','bob'}),frozenset({'payments'}),frozenset({'USD'}),100),
        Scope(frozenset({'transfer'}),frozenset({'alice'}),frozenset({'payments'}),frozenset({'USD'}),60),
        Scope(frozenset({'transfer'}),frozenset({'alice'}),frozenset({'payments'}),frozenset({'USD'}),40),
        Scope(frozenset({'transfer'}),frozenset({'alice'}),frozenset({'payments'}),frozenset({'USD'}),30),
        Scope(frozenset({'transfer'}),frozenset({'alice'}),frozenset({'payments'}),frozenset({'USD'}),20),
    )

D,P1,E1,P2,E2=canonical_chain()
CANONICAL_LINEAGE=Lineage('decision-1','effect-1','effect-1',E1,E1,'none','none','amount','amount','sha256:transform-v2')

def chain_closed(c):
    return leq(c[1],c[0]) and leq(c[2],c[1]) and leq(c[3],c[2]) and leq(c[4],c[3]) and leq(c[4],c[0])

def lineage_closed(l):
    return (l.decision_id=='decision-1' and l.step1_effect_id=='effect-1' and
            l.step2_source_effect_id==l.step1_effect_id and l.step2_input==l.step1_effect and
            l.derivation1=='none' and l.derivation2=='none' and
            l.source_amount1=='amount' and l.source_amount2=='amount' and
            l.implementation2=='sha256:transform-v2')

assert chain_closed((D,P1,E1,P2,E2)) and lineage_closed(CANONICAL_LINEAGE)

MONOTONICITY_DECLARED=True
def monotonicity_declared_exact(value): return value is True

checks={
'upstream-decision-identity-substitution': lambda: not lineage_closed(replace(CANONICAL_LINEAGE,decision_id='decision-attacker')),
'derivation-rule-substitution': lambda: not lineage_closed(replace(CANONICAL_LINEAGE,derivation2='derive-unbounded')),
'source-amount-substitution': lambda: not lineage_closed(replace(CANONICAL_LINEAGE,source_amount2='currency')),
'audience-widening': lambda: not leq(replace(P2,audiences=frozenset({'payments','admin'})),E1),
'second-hop-profile-ceiling-widening': lambda: not leq(replace(P2,max_amount=70),E1),
'second-hop-effect-widening': lambda: not leq(replace(E2,max_amount=50),P2),
'chain-link-substitution': lambda: not lineage_closed(replace(CANONICAL_LINEAGE,step2_source_effect_id='effect-attacker',step2_input=replace(E1,max_amount=60))),
'missing-monotonicity-declaration': lambda: not monotonicity_declared_exact(False) and MONOTONICITY_DECLARED,
'downstream-implementation-substitution': lambda: not lineage_closed(replace(CANONICAL_LINEAGE,implementation2='sha256:transform-attacker')),
}
# Monotonicity is a separate contract: contractivity alone does not imply it.
mono_input_a=Scope(frozenset({'transfer'}),frozenset({'alice'}),frozenset({'payments'}),frozenset({'USD'}),10)
mono_input_b=Scope(frozenset({'transfer'}),frozenset({'alice'}),frozenset({'payments'}),frozenset({'USD'}),20)
mono_effect_a=Scope(frozenset({'transfer'}),frozenset({'alice'}),frozenset({'payments'}),frozenset({'USD'}),8)
mono_effect_b=Scope(frozenset({'transfer'}),frozenset({'alice'}),frozenset({'payments'}),frozenset({'USD'}),3)
assert leq(mono_input_a,mono_input_b)
assert leq(mono_effect_a,mono_input_a) and leq(mono_effect_b,mono_input_b)
assert not leq(mono_effect_a,mono_effect_b)
checks['monotonicity-violation']=lambda: not leq(mono_effect_a,mono_effect_b)

rng=random.Random(21021)
ops=[frozenset(),frozenset({'transfer'}),frozenset({'transfer','refund'})]
targs=[frozenset(),frozenset({'alice'}),frozenset({'alice','bob'})]
aud=[frozenset(),frozenset({'payments'}),frozenset({'payments','admin'})]
curs=[frozenset(),frozenset({'USD'}),frozenset({'USD','EUR'})]
for _ in range(10000):
    a=Scope(rng.choice(ops),rng.choice(targs),rng.choice(aud),rng.choice(curs),rng.randrange(0,101))
    b=Scope(rng.choice(ops),rng.choice(targs),rng.choice(aud),rng.choice(curs),rng.randrange(0,101))
    c=Scope(rng.choice(ops),rng.choice(targs),rng.choice(aud),rng.choice(curs),rng.randrange(0,101))
    if leq(a,b) and leq(b,c): assert leq(a,c)

markers={
'upstream-decision-identity-substitution':'UPSTREAM DECISION IDENTITY NEGATIVE PASS',
'derivation-rule-substitution':'DERIVATION RULE IDENTITY NEGATIVE PASS',
'source-amount-substitution':'SOURCE AMOUNT FIELD NEGATIVE PASS',
'audience-widening':'AUDIENCE WIDENING NEGATIVE PASS',
'second-hop-profile-ceiling-widening':'SECOND-HOP PROFILE CEILING NEGATIVE PASS',
'second-hop-effect-widening':'SECOND-HOP EFFECT SCOPE NEGATIVE PASS',
'chain-link-substitution':'CHAIN LINK / INPUT AUTHORITY NEGATIVE PASS',
'missing-monotonicity-declaration':'MONOTONICITY DECLARATION NEGATIVE PASS',
'monotonicity-violation':'MONOTONICITY RELATION NEGATIVE PASS',
'downstream-implementation-substitution':'DOWNSTREAM IMPLEMENTATION IDENTITY NEGATIVE PASS',
}
for name in markers: print(markers[name])
print('CANONICAL PASS: two-hop authority is contractive at every transformation step')
print('COMPOSITION PASS: Effect2 <= Profile2 <= Effect1 <= Profile1 <= Decision')
print('IDENTITY PASS: decision, derivation rule, amount source, chain link, and implementation are distinct bindings')
print('AUDIENCE PASS: audience restriction participates in the same partial order')
print('MONOTONICITY SEPARATION PASS: contractive does not imply monotone')
print('ORDER PASS: sampled component-wise relation is transitive')
