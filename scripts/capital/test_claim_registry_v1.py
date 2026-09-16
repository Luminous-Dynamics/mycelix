from __future__ import annotations
import copy, json, sys, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from claim_registry_v1 import REGISTRY_VERSION, CLAIM_CLASS, RegistryError, qualify, sha256_hex

PARENT_SUBJECT="270b852e0ac744dfca3a2cb966bf53fce78f2ab9"
PARENT_RECEIPT=json.loads('{"counted_investor_distributions_units": 60000000, "created_entitlement_units": 133000000, "event_chain_tip_sha256": "f766650cbc8233f016ae3482f46708f395264cbc0baeb869ad7c09dd662d240d", "event_count": 8, "event_history_sha256": "b1e75101137bcecc60daba64aa0d232398b78d65e4e946723bd25e5ab23184b7", "financial_state": "CLAIM_ACTIVE", "grant_or_subsidy_units": 7000000, "handback_accepted": false, "impairments_units": 0, "initial_principal_units": 100000000, "legal_transition_complete": false, "nonclaims": ["financial satisfaction is not legal title transfer", "financial satisfaction is not democratic legitimacy", "financial satisfaction is not handback acceptance", "receipt validity is not accounting-standard compliance", "receipt validity is not tax or securities-law compliance", "receipt validity is not infrastructure safety or performance"], "preferred_return_cap_units": 20000000, "profile_sha256": "6929b9e089e3f7b272713f6ff8587bf429594a51a9d67d67d0d4468ba72b52e7", "profile_version": "mycelix-capital-to-commons-fixed-preferred-v1", "project_id": "fiber:jhb:test-001", "qualified_new_capital_units": 10000000, "receipt_version": "mycelix-commons-transition-receipt-v1", "recoverable_lifecycle_units": 3000000, "remaining_claim_units": 73000000, "required_reserve_units": 5000000, "reserve_balance_units": 5000000, "reserve_compliant": true, "retired_claim_ppm": 451127, "retired_claim_units": 60000000, "unit": "ZAR-cent"}')
def profile():
    return {
      "registry_version":REGISTRY_VERSION,"project_id":"fiber:jhb:test-001","unit":"ZAR-cent",
      "claim_class_id":CLAIM_CLASS,"parent_subject_sha":PARENT_SUBJECT,
      "parent_transition_receipt_sha256":sha256_hex(PARENT_RECEIPT),
      "parent_financial_profile_sha256":PARENT_RECEIPT["profile_sha256"],
      "parent_remaining_claim_units":PARENT_RECEIPT["remaining_claim_units"],
      "max_face_units":1_000_000_000,"max_transaction_price_units":1_000_000_000,"max_operations":100}

def op(seq,kind,inputs,outputs,price=0,p=None):
    p=p or profile()
    return {"seq":seq,"operation_id":f"op-{seq}","project_id":p["project_id"],"profile_sha256":sha256_hex(p),
            "prev_operation_sha256":None,"kind":kind,"inputs":inputs,"outputs":outputs,
            "transaction_price_units":price,"authority_ref":f"authority:{seq}","evidence_ref":f"evidence:{seq}"}
def chain(ops):
    out=[]
    for x in ops:
        x=copy.deepcopy(x)
        x["prev_operation_sha256"]=None if not out else sha256_hex(out[-1])
        out.append(x)
    return out
def positive(p=None):
    p=p or profile()
    return chain([
      op(0,"GenesisIssue",[],[{"slice_id":"s0","holder_ref":"investor:a","face_units":73_000_000}],p=p),
      op(1,"Transfer",["s0"],[{"slice_id":"s1","holder_ref":"investor:b","face_units":73_000_000}],91_000_000,p),
      op(2,"Split",["s1"],[{"slice_id":"s2","holder_ref":"investor:b","face_units":40_000_000},{"slice_id":"s3","holder_ref":"investor:b","face_units":33_000_000}],0,p),
      op(3,"Transfer",["s3"],[{"slice_id":"s4","holder_ref":"investor:c","face_units":33_000_000}],35_000_000,p),
      op(4,"Transfer",["s4"],[{"slice_id":"s5","holder_ref":"investor:b","face_units":33_000_000}],30_000_000,p),
      op(5,"Merge",["s2","s5"],[{"slice_id":"s6","holder_ref":"investor:b","face_units":73_000_000}],0,p),
    ])

class Tests(unittest.TestCase):
  def test_positive(self):
    r=qualify(profile(),PARENT_RECEIPT,positive()).receipt()
    self.assertEqual(r["active_claim_total_units"],73_000_000)
    self.assertEqual(r["holder_totals"],[{"holder_ref":"investor:b","face_units":73_000_000}])
    self.assertFalse(r["commons_asset_transferable"])
  def test_premium_sale_does_not_change_liability(self):
    ops=chain([op(0,"GenesisIssue",[],[{"slice_id":"s0","holder_ref":"a","face_units":73_000_000}]),
               op(1,"Transfer",["s0"],[{"slice_id":"s1","holder_ref":"b","face_units":73_000_000}],100_000_000)])
    self.assertEqual(qualify(profile(),PARENT_RECEIPT,ops).receipt()["active_claim_total_units"],73_000_000)
  def test_discount_sale_does_not_change_liability(self):
    ops=chain([op(0,"GenesisIssue",[],[{"slice_id":"s0","holder_ref":"a","face_units":73_000_000}]),
               op(1,"Transfer",["s0"],[{"slice_id":"s1","holder_ref":"b","face_units":73_000_000}],60_000_000)])
    self.assertEqual(qualify(profile(),PARENT_RECEIPT,ops).receipt()["active_claim_total_units"],73_000_000)
  def test_split_cannot_mint(self):
    ops=chain([op(0,"GenesisIssue",[],[{"slice_id":"s0","holder_ref":"a","face_units":73_000_000}]),
               op(1,"Split",["s0"],[{"slice_id":"s1","holder_ref":"a","face_units":40_000_000},{"slice_id":"s2","holder_ref":"a","face_units":34_000_000}])])
    with self.assertRaisesRegex(RegistryError,"face conservation"): qualify(profile(),PARENT_RECEIPT,ops)
  def test_merge_cannot_mint(self):
    ops=chain([op(0,"GenesisIssue",[],[{"slice_id":"s0","holder_ref":"a","face_units":73_000_000}]),
               op(1,"Split",["s0"],[{"slice_id":"s1","holder_ref":"a","face_units":40_000_000},{"slice_id":"s2","holder_ref":"a","face_units":33_000_000}]),
               op(2,"Merge",["s1","s2"],[{"slice_id":"s3","holder_ref":"a","face_units":74_000_000}])])
    with self.assertRaisesRegex(RegistryError,"face conservation"): qualify(profile(),PARENT_RECEIPT,ops)
  def test_stale_serialized_slice_rejected(self):
    ops=chain([op(0,"GenesisIssue",[],[{"slice_id":"s0","holder_ref":"a","face_units":73_000_000}]),
               op(1,"Transfer",["s0"],[{"slice_id":"s1","holder_ref":"b","face_units":73_000_000}]),
               op(2,"Transfer",["s0"],[{"slice_id":"s2","holder_ref":"c","face_units":73_000_000}])])
    with self.assertRaisesRegex(RegistryError,"stale or unknown"): qualify(profile(),PARENT_RECEIPT,ops)
  def test_slice_id_cannot_be_reused(self):
    ops=chain([op(0,"GenesisIssue",[],[{"slice_id":"s0","holder_ref":"a","face_units":73_000_000}]),
               op(1,"Transfer",["s0"],[{"slice_id":"s1","holder_ref":"b","face_units":73_000_000}]),
               op(2,"Transfer",["s1"],[{"slice_id":"s0","holder_ref":"c","face_units":73_000_000}])])
    with self.assertRaisesRegex(RegistryError,"reused"): qualify(profile(),PARENT_RECEIPT,ops)
  def test_unknown_asset_transfer_kind_fails_closed(self):
    ops=chain([op(0,"GenesisIssue",[],[{"slice_id":"s0","holder_ref":"a","face_units":73_000_000}]),
               op(1,"TransferCommonsAsset",["s0"],[{"slice_id":"s1","holder_ref":"b","face_units":73_000_000}])])
    with self.assertRaisesRegex(RegistryError,"unsupported kind"): qualify(profile(),PARENT_RECEIPT,ops)
  def test_parent_receipt_substitution_rejected(self):
    parent=copy.deepcopy(PARENT_RECEIPT); parent["remaining_claim_units"]+=1
    with self.assertRaisesRegex(RegistryError,"semantic digest mismatch"): qualify(profile(),parent,positive())
  def test_profile_substitution_rejected(self):
    ops=positive(); ops[0]["profile_sha256"]="0"*64
    with self.assertRaisesRegex(RegistryError,"profile substitution"): qualify(profile(),PARENT_RECEIPT,ops)
  def test_project_substitution_rejected(self):
    ops=positive(); ops[0]["project_id"]="fiber:other"
    with self.assertRaisesRegex(RegistryError,"project substitution"): qualify(profile(),PARENT_RECEIPT,ops)
  def test_operation_mutation_breaks_chain(self):
    ops=positive(); ops[2]["transaction_price_units"]=1
    with self.assertRaisesRegex(RegistryError,"broken operation chain"): qualify(profile(),PARENT_RECEIPT,ops)
  def test_merge_requires_single_holder(self):
    ops=chain([op(0,"GenesisIssue",[],[{"slice_id":"s0","holder_ref":"a","face_units":73_000_000}]),
               op(1,"Split",["s0"],[{"slice_id":"s1","holder_ref":"a","face_units":40_000_000},{"slice_id":"s2","holder_ref":"a","face_units":33_000_000}]),
               op(2,"Transfer",["s2"],[{"slice_id":"s3","holder_ref":"b","face_units":33_000_000}]),
               op(3,"Merge",["s1","s3"],[{"slice_id":"s4","holder_ref":"a","face_units":73_000_000}])])
    with self.assertRaisesRegex(RegistryError,"one current holder"): qualify(profile(),PARENT_RECEIPT,ops)
  def test_duplicate_operation_id_rejected(self):
    ops=positive(); ops[2]["operation_id"]=ops[1]["operation_id"]
    ops=chain(ops)
    with self.assertRaisesRegex(RegistryError,"operation_id: duplicate"): qualify(profile(),PARENT_RECEIPT,ops)
  def test_duplicate_output_slice_id_rejected(self):
    ops=chain([op(0,"GenesisIssue",[],[{"slice_id":"s0","holder_ref":"a","face_units":73_000_000}]),
               op(1,"Split",["s0"],[{"slice_id":"s1","holder_ref":"a","face_units":40_000_000},{"slice_id":"s1","holder_ref":"a","face_units":33_000_000}])])
    with self.assertRaisesRegex(RegistryError,"slice_id: reused"): qualify(profile(),PARENT_RECEIPT,ops)
  def test_split_cannot_implicitly_transfer(self):
    ops=chain([op(0,"GenesisIssue",[],[{"slice_id":"s0","holder_ref":"a","face_units":73_000_000}]),
               op(1,"Split",["s0"],[{"slice_id":"s1","holder_ref":"a","face_units":40_000_000},{"slice_id":"s2","holder_ref":"b","face_units":33_000_000}])])
    with self.assertRaisesRegex(RegistryError,"Split must preserve holder"): qualify(profile(),PARENT_RECEIPT,ops)

if __name__=="__main__": unittest.main()
