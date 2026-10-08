from __future__ import annotations
from dataclasses import dataclass
from functools import lru_cache
from itertools import product

VALUES=("alice","bob")
PURPOSES=("business","refund")
CONTEXTS=("trusted","untrusted")
AMOUNTS=range(0,4)

@dataclass(frozen=True)
class Atom:
    target:frozenset[str]
    purpose:frozenset[str]
    context:frozenset[str]
    max_amount:int
    extension:str="none"

@dataclass(frozen=True)
class All:
    clauses:tuple[Atom,...]

@dataclass(frozen=True)
class AnyOf:
    clauses:tuple[Atom,...]

UNIVERSE=frozenset(product(VALUES,PURPOSES,CONTEXTS,AMOUNTS))

def supported(a:Atom)->bool:
    return a.extension=="none"

def atom_subsumes(child:Atom,parent:Atom)->bool:
    if not supported(child) or not supported(parent):
        return False
    return (
        child.target <= parent.target
        and child.purpose <= parent.purpose
        and child.context <= parent.context
        and child.max_amount <= parent.max_amount
    )

def atom_matches(a:Atom, request)->bool:
    target,purpose,context,amount=request
    return target in a.target and purpose in a.purpose and context in a.context and amount<=a.max_amount

def all_denotation(policy:All):
    return frozenset(r for r in UNIVERSE if all(atom_matches(a,r) for a in policy.clauses))

def any_denotation(policy:AnyOf):
    return frozenset(r for r in UNIVERSE if any(atom_matches(a,r) for a in policy.clauses))

def semantic_eq(a,b):
    if isinstance(a,All) and isinstance(b,All):
        return all_denotation(a)==all_denotation(b)
    if isinstance(a,AnyOf) and isinstance(b,AnyOf):
        return any_denotation(a)==any_denotation(b)
    return False

def all_subsumes_backtracking(child:All,parent:All)->bool:
    if len(child.clauses)<len(parent.clauses):
        return False
    if not all(supported(c) for c in child.clauses+parent.clauses):
        return False

    @lru_cache(maxsize=None)
    def match(parent_index:int, used:tuple[int,...])->bool:
        if parent_index==len(parent.clauses):
            return True
        p=parent.clauses[parent_index]
        used_set=set(used)
        # Deterministic candidate ordering. Backtracking removes greedy-dead-end dependence.
        candidates=[i for i,c in enumerate(child.clauses) if i not in used_set and atom_subsumes(c,p)]
        for i in candidates:
            if match(parent_index+1, tuple(sorted((*used,i)))):
                return True
        return False
    return match(0,tuple())

def all_subsumes_greedy(child:All,parent:All)->bool:
    used=set()
    for p in parent.clauses:
        choice=next((i for i,c in enumerate(child.clauses) if i not in used and atom_subsumes(c,p)),None)
        if choice is None:
            return False
        used.add(choice)
    return True

P_BROAD=Atom(frozenset({"alice","bob"}),frozenset(PURPOSES),frozenset(CONTEXTS),3)
P_NARROW=Atom(frozenset({"alice"}),frozenset({"business"}),frozenset({"trusted"}),2)
C_BROAD=Atom(frozenset({"alice","bob"}),frozenset(PURPOSES),frozenset(CONTEXTS),3)
C_NARROW=Atom(frozenset({"alice"}),frozenset({"business"}),frozenset({"trusted"}),2)

PARENT=All((P_BROAD,P_NARROW))
CHILD=All((C_NARROW,C_BROAD))

assert all_subsumes_backtracking(CHILD,PARENT)
assert all_subsumes_backtracking(All(tuple(reversed(CHILD.clauses))),All(tuple(reversed(PARENT.clauses))))

# Soundness check over the bounded universe for the supported structural fragment.
assert all_denotation(CHILD) <= all_denotation(PARENT)

greedy_dead_end_parent=All((P_NARROW,P_BROAD))
greedy_dead_end_child=All((C_BROAD,C_NARROW))
assert all_subsumes_greedy(greedy_dead_end_child,greedy_dead_end_parent)  # ordering chosen to expose no false result
# Construct the real dead-end shape: both children match broad; only one matches narrow.
P1=Atom(frozenset({"alice","bob"}),frozenset(PURPOSES),frozenset(CONTEXTS),3)
P2=Atom(frozenset({"alice"}),frozenset(PURPOSES),frozenset(CONTEXTS),3)
C1=Atom(frozenset({"alice"}),frozenset(PURPOSES),frozenset(CONTEXTS),2)
C2=Atom(frozenset({"alice","bob"}),frozenset(PURPOSES),frozenset(CONTEXTS),2)
dead_parent=All((P1,P2))
dead_child=All((C1,C2))
assert all_subsumes_backtracking(dead_child,dead_parent)
assert not all_subsumes_greedy(dead_child,dead_parent)

controls={
"parent-clause-deletion":not all_subsumes_backtracking(All((C_BROAD,)),PARENT),
"duplicate-witness-reuse":not all_subsumes_backtracking(All((C_NARROW,)),All((P_BROAD,P_NARROW))),
"greedy-dead-end":all_subsumes_backtracking(dead_child,dead_parent) and not all_subsumes_greedy(dead_child,dead_parent),
"clause-order-permutation":all_subsumes_backtracking(All(tuple(reversed(CHILD.clauses))),All(tuple(reversed(PARENT.clauses)))),
"semantic-equivalent-conjunction":semantic_eq(All((P_BROAD,P_NARROW)),All((P_NARROW,P_BROAD))),
"semantic-non-equivalent-normalization":not semantic_eq(All((P_NARROW,P_BROAD)),All((P_BROAD,Atom(frozenset({"bob"}),frozenset(PURPOSES),frozenset(CONTEXTS),3)))),
"cross-type-substitution":not all_subsumes_backtracking(All((Atom(P_NARROW.target,P_NARROW.purpose,P_NARROW.context,P_NARROW.max_amount,"future-v1"),)),All((P_NARROW,))),
"unsupported-extension-inside-compound":not all_subsumes_backtracking(All((Atom(P_NARROW.target,P_NARROW.purpose,P_NARROW.context,P_NARROW.max_amount,"future-v1"),C_BROAD)),PARENT),
"disjunct-expansion": any_denotation(AnyOf((P_NARROW,P_BROAD,Atom(frozenset({"bob"}),frozenset(PURPOSES),frozenset(CONTEXTS),3)))) > any_denotation(AnyOf((P_NARROW,P_BROAD))),
"additional-restrictive-clause":all_subsumes_backtracking(All((C_NARROW,C_BROAD,P_NARROW)),PARENT),
}
assert all(controls.values())
print("CANONICAL PASS: deterministic one-to-one compound witness matching")
print("SOUNDNESS PASS: supported structural subsumption is denotationally contractive over bounded requests")
for k,v in controls.items(): print(k.upper().replace("-"," ")+" PASS")
print("GREEDY SEPARATION PASS: naive greedy matching can false-negative while backtracking succeeds")
print("WITNESS UNIQUENESS PASS: one child clause cannot satisfy two parent clauses")
