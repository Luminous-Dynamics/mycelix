#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, sys
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

SCHEMA="mycelix.ci-gov.001k.capacity-oracle.v0.2"
POLICY_SCHEMA="mycelix.ci-gov.001k.capacity-policy.v0.2"
EXPECTED_GROUP="mycelix-heavy-qualification-v1"
EXPECTED_QUEUE="max"
EXPECTED_LABEL="ci:qualify"
EXPECTED_MAX_ACTIVE=1
EXPECTED_MAX_PENDING=8
EXPECTED_PLATFORM_PENDING_CAP=100
EXPECTED_MAX_SNAPSHOT_AGE_SECONDS=30

class OracleError(ValueError): pass
class AdmissionState(str,Enum):
    AUTHORING_NOT_ADMITTED="AuthoringNotAdmitted"
    QUALIFICATION_ADMITTED="QualificationAdmitted"
    CLOSED_OUTSIDE_AUTHORITY="ClosedOutsideCapacityAuthority"
    SUPERSEDED_OUTSIDE_AUTHORITY="SupersededOutsideCapacityAuthority"
    METADATA_INVALID="MetadataInvalid"
class CapacityState(str,Enum):
    NOT_ADMITTED="NotAdmitted"
    EXECUTION_ELIGIBLE="ExecutionEligible"
    QUEUE_ADMISSION_ELIGIBLE="QueueAdmissionEligible"
    ADMISSION_DEFERRED_BUDGET_FULL="AdmissionDeferredBudgetFull"
    ADMISSION_DEFERRED_OBSERVATION_UNKNOWN="AdmissionDeferredObservationUnknown"
    ADMISSION_DEFERRED_OBSERVATION_INVALID="AdmissionDeferredObservationInvalid"

@dataclass(frozen=True)
class CapacityPolicy:
    schema:str; shared_group:str; queue:str; cancel_in_progress:bool; max_active:int
    max_pending:int; platform_pending_cap:int; explicit_label:str; ready_admits:bool
    max_snapshot_age_seconds:int
    @classmethod
    def from_mapping(cls,value:dict[str,Any])->"CapacityPolicy":
        expected={"schema","shared_group","queue","cancel_in_progress","max_active","max_pending","platform_pending_cap","explicit_label","ready_admits","max_snapshot_age_seconds"}
        if set(value)!=expected: raise OracleError("policy fields mismatch")
        p=cls(**value); p.validate(); return p
    def validate(self)->None:
        if self.schema!=POLICY_SCHEMA: raise OracleError("unsupported policy schema")
        if self.shared_group!=EXPECTED_GROUP: raise OracleError("unexpected shared concurrency group")
        if self.queue!=EXPECTED_QUEUE: raise OracleError("v0.2 requires queue=max")
        if self.cancel_in_progress is not False: raise OracleError("v0.2 forbids cancel-in-progress")
        if type(self.max_active) is not int or self.max_active!=EXPECTED_MAX_ACTIVE: raise OracleError("v0.2 requires exactly one active heavy qualification")
        if type(self.max_pending) is not int or self.max_pending!=EXPECTED_MAX_PENDING: raise OracleError("unexpected soft pending budget")
        if type(self.platform_pending_cap) is not int or self.platform_pending_cap!=EXPECTED_PLATFORM_PENDING_CAP: raise OracleError("unexpected platform pending cap")
        if not (0<self.max_pending<self.platform_pending_cap): raise OracleError("soft pending budget must remain below platform cap")
        if self.explicit_label!=EXPECTED_LABEL: raise OracleError("unexpected explicit admission label")
        if self.ready_admits is not False: raise OracleError("v0.2 requires explicit-token-only admission while #697 is unresolved")
        if type(self.max_snapshot_age_seconds) is not int or self.max_snapshot_age_seconds!=EXPECTED_MAX_SNAPSHOT_AGE_SECONDS: raise OracleError("unexpected snapshot freshness bound")

DEFAULT_POLICY=CapacityPolicy(POLICY_SCHEMA,EXPECTED_GROUP,EXPECTED_QUEUE,False,EXPECTED_MAX_ACTIVE,EXPECTED_MAX_PENDING,EXPECTED_PLATFORM_PENDING_CAP,EXPECTED_LABEL,False,EXPECTED_MAX_SNAPSHOT_AGE_SECONDS)

def canonical_json(v:Any)->bytes: return json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=True).encode()
def commitment(v:Any)->str: return hashlib.sha256(canonical_json(v)).hexdigest()
def _valid_sha(v:Any)->bool: return isinstance(v,str) and len(v)==40 and all(c in "0123456789abcdef" for c in v)
def _normalize_labels(v:Any):
    if not isinstance(v,list) or any(not isinstance(x,str) or not x for x in v): return None
    if len(v)!=len(set(v)): return None
    return tuple(sorted(v))

def decide_admission(subject:dict[str,Any],policy:CapacityPolicy=DEFAULT_POLICY)->AdmissionState:
    policy.validate(); required={"event","pr_open","draft","subject_head","current_head","labels"}
    if set(subject)!=required or subject.get("event")!="pull_request": return AdmissionState.METADATA_INVALID
    if type(subject.get("pr_open")) is not bool or type(subject.get("draft")) is not bool: return AdmissionState.METADATA_INVALID
    if not _valid_sha(subject.get("subject_head")) or not _valid_sha(subject.get("current_head")): return AdmissionState.METADATA_INVALID
    labels=_normalize_labels(subject.get("labels"))
    if labels is None: return AdmissionState.METADATA_INVALID
    if not subject["pr_open"]: return AdmissionState.CLOSED_OUTSIDE_AUTHORITY
    if subject["subject_head"]!=subject["current_head"]: return AdmissionState.SUPERSEDED_OUTSIDE_AUTHORITY
    if policy.explicit_label in labels: return AdmissionState.QUALIFICATION_ADMITTED
    return AdmissionState.AUTHORING_NOT_ADMITTED

def decide_capacity(admission:AdmissionState,observation:dict[str,Any],policy:CapacityPolicy=DEFAULT_POLICY)->CapacityState:
    policy.validate()
    if admission is not AdmissionState.QUALIFICATION_ADMITTED: return CapacityState.NOT_ADMITTED
    expected={"complete","age_seconds","active_count","pending_count"}
    if not isinstance(observation,dict) or set(observation)!=expected: return CapacityState.ADMISSION_DEFERRED_OBSERVATION_INVALID
    if type(observation["complete"]) is not bool: return CapacityState.ADMISSION_DEFERRED_OBSERVATION_INVALID
    if observation["complete"] is not True: return CapacityState.ADMISSION_DEFERRED_OBSERVATION_UNKNOWN
    for k in ("age_seconds","active_count","pending_count"):
        if type(observation[k]) is not int or observation[k]<0: return CapacityState.ADMISSION_DEFERRED_OBSERVATION_INVALID
    if observation["age_seconds"]>policy.max_snapshot_age_seconds: return CapacityState.ADMISSION_DEFERRED_OBSERVATION_UNKNOWN
    if observation["active_count"]>policy.max_active or observation["pending_count"]>policy.platform_pending_cap: return CapacityState.ADMISSION_DEFERRED_OBSERVATION_INVALID
    if observation["pending_count"]>=policy.max_pending: return CapacityState.ADMISSION_DEFERRED_BUDGET_FULL
    if observation["active_count"]==0 and observation["pending_count"]==0: return CapacityState.EXECUTION_ELIGIBLE
    return CapacityState.QUEUE_ADMISSION_ELIGIBLE

def build_receipt(subject:dict[str,Any],observation:dict[str,Any],policy:CapacityPolicy=DEFAULT_POLICY)->dict[str,Any]:
    admission=decide_admission(subject,policy); capacity=decide_capacity(admission,observation,policy)
    labels=subject.get("labels"); normalized=sorted(labels) if isinstance(labels,list) and all(isinstance(x,str) for x in labels) else None
    body={"schema":SCHEMA,"policy":policy.__dict__,"subject":{"event":subject.get("event"),"pr_open":subject.get("pr_open"),"draft":subject.get("draft"),"subject_head":subject.get("subject_head"),"current_head":subject.get("current_head"),"labels":normalized},"capacity_observation":observation,"admission_state":admission.value,"capacity_state":capacity.value,"grants_product_pass":False,"grants_scientific_pass":False,"grants_cancellation_authority":False,"grants_live_scheduler_qualification":False,"proposition":"Offline CI-GOV-001K explicit-token admission and bounded pending-budget semantics only.","nonclaims":["No GitHub workflow or job was mutated.","Ready state alone does not admit heavy qualification in v0.2.","Capacity eligibility is not proof of runner assignment.","No product or scientific PASS is established.","SupersededOutsideCapacityAuthority does not authorize cancellation.","Snapshot completeness/freshness must be established by a separately qualified observer."]}
    body["receipt_commitment"]=commitment(body); return body

def parse_args():
    p=argparse.ArgumentParser(); p.add_argument("subject_json",type=Path); p.add_argument("capacity_json",type=Path); return p.parse_args()
def main()->int:
    a=parse_args()
    try:
        s=json.loads(a.subject_json.read_text()); o=json.loads(a.capacity_json.read_text())
        if not isinstance(s,dict) or not isinstance(o,dict): raise OracleError("inputs must be objects")
        r=build_receipt(s,o)
    except (OSError,json.JSONDecodeError,OracleError) as e:
        print(f"ORACLE ERROR: {e}",file=sys.stderr); return 2
    print(json.dumps(r,indent=2,sort_keys=True)); return 0
if __name__=="__main__": raise SystemExit(main())
