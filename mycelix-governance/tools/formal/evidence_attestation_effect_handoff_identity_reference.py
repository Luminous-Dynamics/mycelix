#!/usr/bin/env python3
from dataclasses import dataclass

@dataclass(frozen=True)
class Decision: id: str
@dataclass(frozen=True)
class Intent:
    id: str; decision: Decision; operation: str; target: str; adapter: str; invocation: str
@dataclass(frozen=True)
class Use:
    id: str; decision: Decision; intent: Intent; operation: str; target: str; adapter: str; invocation: str
@dataclass(frozen=True)
class Attempt:
    decision: Decision; intent: Intent; use: Use
    recorded_decision: str; recorded_intent: str; recorded_operation: str
    recorded_use: str; recorded_target: str; recorded_adapter: str; recorded_invocation: str

D=Decision("decision-1")
I=Intent("intent-1",D,"op-1","target-1","adapter-v1","inv-1")
U=Use("use-1",D,I,"op-1","target-1","adapter-v1","inv-1")
A=Attempt(D,I,U,D.id,I.id,I.operation,U.id,I.target,I.adapter,I.invocation)

def failures(a):
    out=[]
    if a.recorded_decision != a.decision.id: out.append("DecisionIdentityConserved")
    if a.recorded_intent != a.intent.id: out.append("IntentIdentityConserved")
    if a.recorded_operation != a.intent.operation: out.append("OperationIdentityConserved")
    if a.recorded_use != a.use.id: out.append("UseCommitmentConserved")
    if a.recorded_target != a.intent.target: out.append("TargetConserved")
    if a.recorded_adapter != a.intent.adapter: out.append("AdapterConserved")
    if a.recorded_invocation != a.intent.invocation: out.append("InvocationConserved")
    if a.use.decision != a.decision or a.use.intent != a.intent: out.append("EffectHandoffIdentityExact")
    return sorted(set(out))

assert failures(A)==[]

cases={
"decision-identity": Attempt(D,I,U,"decision-2",I.id,I.operation,U.id,I.target,I.adapter,I.invocation),
"intent-identity": Attempt(D,I,U,D.id,"intent-2",I.operation,U.id,I.target,I.adapter,I.invocation),
"operation-commitment": Attempt(D,I,U,D.id,I.id,"op-2",U.id,I.target,I.adapter,I.invocation),
"use-commitment": Attempt(D,I,U,D.id,I.id,I.operation,"use-2",I.target,I.adapter,I.invocation),
"target": Attempt(D,I,U,D.id,I.id,I.operation,U.id,"target-2",I.adapter,I.invocation),
"adapter": Attempt(D,I,U,D.id,I.id,I.operation,U.id,I.target,"adapter-v2",I.invocation),
"invocation-identity": Attempt(D,I,U,D.id,I.id,I.operation,U.id,I.target,I.adapter,"inv-2")
}
expected=dict(zip(cases,[
"DecisionIdentityConserved","IntentIdentityConserved","OperationIdentityConserved",
"UseCommitmentConserved","TargetConserved","AdapterConserved","InvocationConserved"
]))
for name,case in cases.items():
    f=failures(case); assert f==[expected[name]],(name,f)

print("CANONICAL PASS: exact decision, intent, use-commit, operation, target, adapter, and invocation identities are conserved")
print("ISOLATION PASS: each negative substitutes exactly one handoff identity")
print("NEGATIVE PASS: every substitution is detected before effect attempt")
