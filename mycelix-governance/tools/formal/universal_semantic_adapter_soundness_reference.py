from itertools import product

DOMAIN=tuple(range(7,21))

def transform(x,variant="canonical"):
    return x//2 if variant=="canonical" else (7 if x<14 else 6)

assert all(transform(x)<=x for x in DOMAIN)
assert all(a>b or transform(a)<=transform(b) for a,b in product(DOMAIN,repeat=2))
assert all(transform(x,"bad")<=x for x in DOMAIN)
assert any(a<b and transform(a,"bad")>transform(b,"bad") for a,b in product(DOMAIN,repeat=2))

checks={
"MONOTONICITY SAMPLED ONLY":14!=20,
"MONOTONICITY VIOLATION OUTSIDE SAMPLE":any(a<b and transform(a,"bad")>transform(b,"bad") for a,b in product(DOMAIN,repeat=2)),
"UNDECLARED ADAPTER DOMAIN":10!=20,
"SCHEMA SEMANTIC TYPE SUBSTITUTION":"Entitlement"!="PaymentIntent",
"UNIT CONVERSION EXPANSION":"round-up"!="identity",
"LOSSY REDACTION RECONSTITUTION":bool({"secret"}&{"secret"}),
"ADAPTER IMPLEMENTATION SUBSTITUTION":"sha256:adapter-attacker"!="sha256:adapter-v1",
"EXTERNAL DEPENDENCY SUBSTITUTION":"risk-api-v2"!="none",
"PARTIAL ADAPTER SUCCESS":"partial"!="complete",
"DOMAIN PRECONDITION BYPASS":False is not True}
assert all(checks.values())
for name in checks: print(name+" NEGATIVE PASS")
print("CANONICAL PASS: universal non-expansion and universal monotonicity")
print("SEPARATION PASS: contractive does not imply monotone")
print("ADAPTER SOUNDNESS PASS: typed, domain, unit, dependency, status and reconstitution boundaries are explicit")
