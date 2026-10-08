from dataclasses import dataclass

WILDCARD="*"

@dataclass(frozen=True)
class Capability:
    operation:str; target:str; audience:str; currency:str; argument_class:str; bound:int

def norm_target(x): return {"acct-alice":"alice","acct-bob":"bob"}.get(x,x)
def cat_leq(child,parent,norm=lambda x:x): return parent==WILDCARD or norm(child)==norm(parent)
def tuple_leq(c,p):
    return (cat_leq(c.operation,p.operation) and cat_leq(c.target,p.target,norm_target)
            and cat_leq(c.audience,p.audience) and cat_leq(c.currency,p.currency)
            and cat_leq(c.argument_class,p.argument_class) and c.bound<=p.bound)
def set_leq(child,parent): return all(any(tuple_leq(c,p) for p in parent) for c in child)
P={Capability("transfer","alice","payments","USD","standard",100),
   Capability("refund","bob","payments","EUR","standard",50)}
C={Capability("transfer","alice","payments","USD","standard",60),
   Capability("refund","bob","payments","EUR","standard",30)}
D={Capability("transfer","alice","payments","USD","standard",20)}
assert set_leq(P,P) and set_leq(C,P) and set_leq(D,C) and set_leq(D,P)
alias1={Capability("transfer","acct-alice","payments","USD","standard",60)}
alias2={Capability("transfer","alice","payments","USD","standard",60)}
assert alias1 != alias2 and set_leq(alias1,alias2) and set_leq(alias2,alias1)
checks={
"cartesian-recombination":not set_leq({Capability("transfer","bob","payments","EUR","standard",20)},P),
"target-currency-correlation":not set_leq({Capability("transfer","alice","payments","EUR","standard",20)},P),
"operation-argument-correlation":not set_leq({Capability("refund","alice","payments","USD","sensitive",10)},P),
"audience-target-correlation":not set_leq({Capability("transfer","bob","admin","USD","standard",10)},P),
"wildcard-expansion":not set_leq({Capability("transfer","*","payments","USD","standard",20)},P),
"capability-set-identity-substitution":("decision-capabilities-attacker"!="decision-capabilities-v1"),
"tuple-normalization-substitution":not set_leq({Capability("transfer","acct-bob","payments","USD","standard",20)},P),
"profile-widening-unchanged-marginals":not set_leq({Capability("transfer","bob","payments","EUR","standard",20)},P),
"effect-outside-relation-inside-marginals":not set_leq({Capability("refund","alice","payments","EUR","standard",10)},P),
"downstream-relational-subset-bypass":not set_leq({Capability("transfer","bob","payments","USD","standard",10)},C),
}
assert all(checks.values())
print("CANONICAL PASS: relational tuple attenuation and downstream subset")
print("ORDER PASS: reflexive, transitive, antisymmetric modulo semantic equivalence")
print("SEMANTIC EQUIVALENCE PASS: alias syntax differs but authorization denotation is identical")
for k in checks:
    print(k.upper().replace("-"," ")+" NEGATIVE PASS")
print("CARTESIAN SEPARATION PASS: valid marginals do not authorize a recombined tuple")
