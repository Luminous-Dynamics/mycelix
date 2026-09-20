import copy, hashlib, importlib.util, json, pathlib, unittest
P=pathlib.Path(__file__).with_name("github_rest_adapter_integrity.py")
S=importlib.util.spec_from_file_location("g",P); g=importlib.util.module_from_spec(S); S.loader.exec_module(g)

def valid():
    r={
        "schema":"mycelix-github-rest-evidence-ci-core-observation-v1",
        "adapter_profile_id":"p",
        "adapter_profile_commitment_sha256":"1"*64,
        "adapter_implementation_commitment_sha256":g.CORE_ADAPTER_IMPLEMENTATION,
        "supported_core_head":g.SEMANTIC_CORE_HEAD,
        "observed_at_utc":"2026-09-20T14:05:24Z",
        "core_manifest_v1":{"theorem_id":"T","repository_id":1,"workflow_path":"w","qualification_head":"h","required_jobs":[{"job_key":"q"}]},
        "core_observation_v1":{"repository_id":1,"workflow_run_id":2,"qualification_head":"h","workflow_path":"w","jobs":[{"job_id":3,"job_key":"q","status":"Queued","conclusion":None,"gate_execution":"NoTheoremStepExecuted","dependency_state":"EligibleForRunner","queue_age_seconds":60,"failure_class":"NotApplicable"}]},
        "provider_diagnostics":[{"job_key":"q","observed_name":"q","runner_assignment":"NotObserved","step_metadata":"Present","provider_state":"QueuedNoTheoremStart"}],
        "observation_source_authenticity_verified":False,
        "github_api_response_authenticity_verified":False,
        "runner_identity_attested":False,
        "semantic_classification_performed":False,
        "qualification_result":None,"theorem_result":None,
        "qualification_authority":False,"evidence_authority":False,
        "failover_authority":False,"rerun_authority":False,"dispatch_authority":False,
    }
    r["adapter_observation_commitment_sha256"]=hashlib.sha256(g.CORE_OBSERVATION_DOMAIN+g.canonical_json(r)).hexdigest()
    r["adapter_policy_schema"]="mycelix-github-rest-evidence-ci-core-adapter-policy-v1"
    r["adapter_policy_implementation_commitment_sha256"]=g.POLICY_IMPLEMENTATION
    r["adapter_core_git_blob_sha1"]=g.CORE_ADAPTER_GIT_BLOB
    r["adapter_core_contract_validated"]=True
    p={k:v for k,v in r.items() if k!="adapter_policy_commitment_sha256"}
    r["adapter_policy_commitment_sha256"]=hashlib.sha256(g.POLICY_DOMAIN+g.canonical_json(p)).hexdigest()
    return r

class T(unittest.TestCase):
    def test_valid(self): self.assertEqual(g._verify_result(valid())["core_observation_v1"]["jobs"][0]["status"],"Queued")
    def test_core_mutation_rejected(self):
        x=valid(); x["core_observation_v1"]["jobs"][0]["failure_class"]="Unknown"
        p={k:v for k,v in x.items() if k!="adapter_policy_commitment_sha256"}
        x["adapter_policy_commitment_sha256"]=hashlib.sha256(g.POLICY_DOMAIN+g.canonical_json(p)).hexdigest()
        with self.assertRaises(g.IntegrityError): g._verify_result(x)
    def test_policy_mutation_rejected(self):
        x=valid(); x["adapter_core_contract_validated"]=False
        with self.assertRaises(g.IntegrityError): g._verify_result(x)
    def test_policy_commitment_tamper_rejected(self):
        x=valid(); x["adapter_policy_commitment_sha256"]="0"*64
        with self.assertRaises(g.IntegrityError): g._verify_result(x)
    def test_unknown_field_rejected(self):
        x=valid(); x["authority"]="yes"
        with self.assertRaises(g.IntegrityError): g._verify_result(x)
    def test_authority_rejected(self):
        x=valid(); x["dispatch_authority"]=True
        with self.assertRaises(g.IntegrityError): g._verify_result(x)
    def test_float_rejected(self):
        with self.assertRaises(g.IntegrityError): g.canonical_json({"x":1.5})
    def test_constants(self):
        self.assertEqual(g.POLICY_GIT_BLOB,"2bfbaba7696347cce016cd2344b7127af873c9d8")
        self.assertEqual(g.CORE_ADAPTER_GIT_BLOB,"603fd2b77bc588701dbcdae376ae31f5b49ce13b")
    def test_impl_frozen(self):
        old=g.IMPLEMENTATION_COMMITMENT_SHA256; data=P.read_bytes(); P.write_bytes(data+b"\n#x")
        try: self.assertEqual(g.IMPLEMENTATION_COMMITMENT_SHA256,old)
        finally: P.write_bytes(data)
if __name__=="__main__": unittest.main()
