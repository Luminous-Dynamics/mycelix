import importlib.util, json, pathlib, sys, unittest
HERE=pathlib.Path(__file__).parent
SPEC=importlib.util.spec_from_file_location("o",str(HERE/"ci_qualification_capacity_observer.py"))
o=importlib.util.module_from_spec(SPEC); sys.modules[SPEC.name]=o; SPEC.loader.exec_module(o)

def member(run=1,status="pending",job=None):
    m={"run_id":run,"run_name":"Q","run_url":f"https://api.github.com/repos/{o.OWNER}/{o.REPO}/actions/runs/{run}",
       "run_html_url":f"https://github.com/{o.OWNER}/{o.REPO}/actions/runs/{run}","status":status}
    if job is not None:
        m.update(job_id=job,job_name="qualify",job_url=f"https://api.github.com/repos/{o.OWNER}/{o.REPO}/actions/jobs/{job}",
                 job_html_url=f"https://github.com/{o.OWNER}/{o.REPO}/actions/runs/{run}/job/{job}")
    return m
def payload(ms):
    return json.dumps({"group_name":o.GROUP,"group_url":o.GROUP_URL,"total_count":len(ms),"group_members":ms}).encode()

class T(unittest.TestCase):
    def test_404_empty_live_group(self):
        r=o.parse_live_group(404,b"{}"); self.assertTrue(r["complete"]); self.assertFalse(r["group_present"]); self.assertFalse(r["configuration_established"]); self.assertEqual(r["pending_count"],0)
    def test_non200_non404_incomplete(self): self.assertFalse(o.parse_live_group(422,b"{}")["complete"])
    def test_one_active_two_pending(self):
        r=o.parse_live_group(200,payload([member(1,"in_progress"),member(2),member(3,job=9)])); self.assertEqual((r["active_count"],r["pending_count"]),(1,2))
    def test_wrong_group_rejected(self):
        b=json.dumps({"group_name":"other","group_url":o.GROUP_URL,"total_count":1,"group_members":[member()]}).encode(); self.assertRaises(o.ObserverError,o.parse_live_group,200,b)
    def test_wrong_group_url_rejected(self):
        b=json.dumps({"group_name":o.GROUP,"group_url":"https://evil.invalid","total_count":1,"group_members":[member()]}).encode(); self.assertRaises(o.ObserverError,o.parse_live_group,200,b)
    def test_total_mismatch_rejected(self):
        b=json.dumps({"group_name":o.GROUP,"group_url":o.GROUP_URL,"total_count":2,"group_members":[member()]}).encode(); self.assertRaises(o.ObserverError,o.parse_live_group,200,b)
    def test_zero_count_200_rejected(self): self.assertRaises(o.ObserverError,o.parse_live_group,200,payload([]))
    def test_unknown_status_rejected(self): self.assertRaises(o.ObserverError,o.parse_live_group,200,payload([member(status="queued")]))
    def test_duplicate_identity_rejected(self): self.assertRaises(o.ObserverError,o.parse_live_group,200,payload([member(),member()]))
    def test_two_active_rejected(self): self.assertRaises(o.ObserverError,o.parse_live_group,200,payload([member(1,"in_progress"),member(2,"in_progress")]))
    def test_over_pending_cap_rejected(self): self.assertRaises(o.ObserverError,o.parse_live_group,200,payload([member(i+1) for i in range(101)]))
    def test_missing_run_id_rejected(self):
        m=member(); del m["run_id"]; self.assertRaises(o.ObserverError,o.parse_live_group,200,payload([m]))
    def test_bool_run_id_rejected(self):
        m=member(); m["run_id"]=True; self.assertRaises(o.ObserverError,o.parse_live_group,200,payload([m]))
    def test_job_urls_required_together(self):
        m=member(job=9); del m["job_url"]; self.assertRaises(o.ObserverError,o.parse_live_group,200,payload([m]))
    def test_run_url_repo_bound(self):
        m=member(); m["run_url"]="https://api.github.com/repos/other/repo/actions/runs/1"; self.assertRaises(o.ObserverError,o.parse_live_group,200,payload([m]))
    def test_malformed_json_rejected(self): self.assertRaises(o.ObserverError,o.parse_live_group,200,b"{")
    def test_observe_transport_fail_closed(self):
        def bad(): raise o.ObserverError("transport_error")
        r=o.observe(bad,lambda:1000); self.assertFalse(r["complete"]); self.assertEqual(r["reason"],"transport_error")
    def test_observe_timestamp_and_commitment(self):
        r=o.observe(lambda:(404,b"{}"),lambda:1000); self.assertEqual(r["observed_at_epoch_seconds"],1000); self.assertEqual(len(r["receipt_commitment"]),64)
    def test_invalid_clock_fails_closed(self):
        r=o.observe(lambda:(404,b"{}"),lambda:-1); self.assertFalse(r["complete"]); self.assertEqual(r["reason"],"invalid_clock"); self.assertEqual(len(r["receipt_commitment"]),64)
    def test_oversized_http_error_propagates(self):
        class E:
            def read(self,n): return b"x"*(o.MAX_BODY_BYTES+1)
        with self.assertRaises(o.ObserverError): o._read_bounded(E())
    def test_headers_exact_without_token(self):
        h=o._headers(None); self.assertNotIn("Authorization",h); self.assertEqual(h["X-GitHub-Api-Version"],"2026-03-10")
    def test_headers_token_not_in_receipt_surface(self):
        h=o._headers("secret"); self.assertEqual(h["Authorization"],"Bearer secret"); r=o.observe(lambda:(404,b"{}"),lambda:1000); self.assertNotIn("secret",json.dumps(r))
    def test_no_redirect_handler(self): self.assertIsNone(o.NoRedirect().redirect_request(None,None,302,"x",{},"https://evil.invalid"))
    def test_get_only_source(self):
        text=(HERE/"ci_qualification_capacity_observer.py").read_text(); self.assertIn('method="GET"',text)
        for s in ('method="POST"','method="PATCH"','method="PUT"','method="DELETE"','cancel_run(','rerun','merge_pull_request'): self.assertNotIn(s,text)
    def test_fixed_endpoint(self): self.assertEqual(o.API_URL,"https://api.github.com/repos/Luminous-Dynamics/mycelix/actions/concurrency_groups/mycelix-heavy-qualification-v1")
    def test_non_authority_fields(self):
        r=o.observe(lambda:(404,b"{}"),lambda:1000); self.assertFalse(r["grants_queue_admission"]); self.assertFalse(r["grants_cancellation_authority"]); self.assertFalse(r["grants_product_pass"])
if __name__=="__main__": unittest.main()
