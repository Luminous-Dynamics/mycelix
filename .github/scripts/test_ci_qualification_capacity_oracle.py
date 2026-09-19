import importlib.util, pathlib, sys, unittest
HERE=pathlib.Path(__file__).parent
SPEC=importlib.util.spec_from_file_location("o",str(HERE/"ci_qualification_capacity_oracle.py"))
o=importlib.util.module_from_spec(SPEC); sys.modules[SPEC.name]=o; SPEC.loader.exec_module(o)
A="a"*40; B="b"*40
def subj(draft=True,labels=None,current=A,frozen=A,open_=True):
    return {"event":"pull_request","pr_open":open_,"draft":draft,"subject_head":frozen,"current_head":current,"labels":[] if labels is None else labels}
def obs(complete=True,age=0,active=0,pending=0):
    return {"complete":complete,"age_seconds":age,"active_count":active,"pending_count":pending}
class T(unittest.TestCase):
    def test_policy_exact(self):
        p=o.DEFAULT_POLICY; p.validate(); self.assertEqual(p.max_pending,8); self.assertEqual(p.platform_pending_cap,100); self.assertFalse(p.ready_admits)
    def test_reject_cancel(self):
        v=dict(o.DEFAULT_POLICY.__dict__); v["cancel_in_progress"]=True; self.assertRaises(o.OracleError,o.CapacityPolicy.from_mapping,v)
    def test_reject_ready_auto(self):
        v=dict(o.DEFAULT_POLICY.__dict__); v["ready_admits"]=True; self.assertRaises(o.OracleError,o.CapacityPolicy.from_mapping,v)
    def test_reject_pending_budget_drift(self):
        v=dict(o.DEFAULT_POLICY.__dict__); v["max_pending"]=9; self.assertRaises(o.OracleError,o.CapacityPolicy.from_mapping,v)
    def test_reject_platform_cap_drift(self):
        v=dict(o.DEFAULT_POLICY.__dict__); v["platform_pending_cap"]=101; self.assertRaises(o.OracleError,o.CapacityPolicy.from_mapping,v)
    def test_draft_no_token(self): self.assertEqual(o.decide_admission(subj()).value,"AuthoringNotAdmitted")
    def test_ready_no_token_still_not_admitted(self): self.assertEqual(o.decide_admission(subj(draft=False)).value,"AuthoringNotAdmitted")
    def test_token_admits_draft(self): self.assertEqual(o.decide_admission(subj(labels=["ci:qualify"])).value,"QualificationAdmitted")
    def test_token_admits_ready(self): self.assertEqual(o.decide_admission(subj(draft=False,labels=["ci:qualify"])).value,"QualificationAdmitted")
    def test_closed(self): self.assertEqual(o.decide_admission(subj(open_=False,labels=["ci:qualify"])).value,"ClosedOutsideCapacityAuthority")
    def test_superseded(self): self.assertEqual(o.decide_admission(subj(current=B,labels=["ci:qualify"])).value,"SupersededOutsideCapacityAuthority")
    def test_unknown_observation(self): self.assertEqual(o.decide_capacity(o.AdmissionState.QUALIFICATION_ADMITTED,obs(complete=False)).value,"AdmissionDeferredObservationUnknown")
    def test_stale_observation(self): self.assertEqual(o.decide_capacity(o.AdmissionState.QUALIFICATION_ADMITTED,obs(age=31)).value,"AdmissionDeferredObservationUnknown")
    def test_invalid_observation_shape(self): self.assertEqual(o.decide_capacity(o.AdmissionState.QUALIFICATION_ADMITTED,{"active_count":0}).value,"AdmissionDeferredObservationInvalid")
    def test_invalid_active_width(self): self.assertEqual(o.decide_capacity(o.AdmissionState.QUALIFICATION_ADMITTED,obs(active=2)).value,"AdmissionDeferredObservationInvalid")
    def test_invalid_over_platform_cap(self): self.assertEqual(o.decide_capacity(o.AdmissionState.QUALIFICATION_ADMITTED,obs(pending=101)).value,"AdmissionDeferredObservationInvalid")
    def test_budget_full_at_8(self): self.assertEqual(o.decide_capacity(o.AdmissionState.QUALIFICATION_ADMITTED,obs(pending=8)).value,"AdmissionDeferredBudgetFull")
    def test_budget_full_above_8(self): self.assertEqual(o.decide_capacity(o.AdmissionState.QUALIFICATION_ADMITTED,obs(pending=99)).value,"AdmissionDeferredBudgetFull")
    def test_immediate(self): self.assertEqual(o.decide_capacity(o.AdmissionState.QUALIFICATION_ADMITTED,obs()).value,"ExecutionEligible")
    def test_queue_eligible_when_active(self): self.assertEqual(o.decide_capacity(o.AdmissionState.QUALIFICATION_ADMITTED,obs(active=1,pending=0)).value,"QueueAdmissionEligible")
    def test_queue_eligible_when_pending(self): self.assertEqual(o.decide_capacity(o.AdmissionState.QUALIFICATION_ADMITTED,obs(active=1,pending=7)).value,"QueueAdmissionEligible")
    def test_nonadmitted_ignores_capacity(self): self.assertEqual(o.decide_capacity(o.AdmissionState.AUTHORING_NOT_ADMITTED,obs(active=9,pending=999)).value,"NotAdmitted")
    def test_duplicate_labels_invalid(self): self.assertEqual(o.decide_admission(subj(labels=["ci:qualify","ci:qualify"])).value,"MetadataInvalid")
    def test_unknown_subject_field_invalid(self):
        s=subj(); s["priority"]="urgent"; self.assertEqual(o.decide_admission(s).value,"MetadataInvalid")
    def test_receipt_binds_budget(self):
        r=o.build_receipt(subj(labels=["ci:qualify"]),obs(active=1,pending=7)); self.assertEqual(r["policy"]["max_pending"],8); self.assertEqual(r["capacity_state"],"QueueAdmissionEligible")
    def test_receipt_non_authority(self):
        r=o.build_receipt(subj(labels=["ci:qualify"]),obs()); self.assertFalse(r["grants_product_pass"]); self.assertFalse(r["grants_scientific_pass"]); self.assertFalse(r["grants_cancellation_authority"]); self.assertFalse(r["grants_live_scheduler_qualification"])
    def test_receipt_commitment_changes_with_counts(self):
        a=o.build_receipt(subj(labels=["ci:qualify"]),obs(pending=0)); b=o.build_receipt(subj(labels=["ci:qualify"]),obs(pending=1)); self.assertNotEqual(a["receipt_commitment"],b["receipt_commitment"])
    def test_no_network_mutation_client(self):
        text=(HERE/"ci_qualification_capacity_oracle.py").read_text()
        for s in ("api.github.com","urllib.request","requests.","cancel_run(","rerun","merge_pull_request","add_labels"): self.assertNotIn(s,text)
if __name__=="__main__": unittest.main()
