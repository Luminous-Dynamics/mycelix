#!/usr/bin/env python3
"""Independent standard-library claim-fidelity checks for the SAFRB fixture."""
import copy, json, sys
from pathlib import Path

SINGLE="0a1c507d3bff9b3062e24ca8718e102244473d839c2a31c5ead31812ee249be2"
TEMPORAL="edfd1c2a5a3eb52a0026a0e63abb38ab2ca8109c4de3d96fbff1215ad78f0308"

def require(condition, message):
    if not condition:
        raise AssertionError(message)

def verify(doc):
    require(doc["fixture_id"]=="MYCELIX-SAFRB-SHARED-PUMP-001","fixture identity")
    require(doc["maturity"]=="synthetic_test_fixture","maturity promotion")
    require(doc["source_scenario"]["scenario_sha256"]==SINGLE,"single-window source reference changed")
    scenarios={x["case_id"]:x["scenario_sha256"] for x in doc["source_scenarios"]}
    require(scenarios.get("SAFRB-SHARED-PUMP-2ROUTE-001")==SINGLE,"single-window digest mismatch")
    require(scenarios.get("SAFRB-TWO-WINDOW-SHARED-PUMP-001")==TEMPORAL,"two-window digest mismatch")
    resource=doc["resource"]; issuer=doc["issuer"]
    require(issuer["key_authentication"]=="synthetic_authenticated","fixture key attribution missing")
    require(issuer["recognition_for_claim_purpose"]=="unknown","issuer purpose-recognition promotion")
    require(doc["participant"]["recognition_for_purpose"]=="unknown","participant recognition promotion")
    require(issuer["claim_scope"]=="capacity_observation","issuer claim-scope promotion")
    require(resource["reported_capacity"]["claim_status"]=="reported_unverified","reported capacity promoted to verified")
    require(resource["current_availability"]=="unknown","reported observation promoted to current availability")
    require(resource["title"]=="unknown","title promoted without evidence")
    require(resource["custody"]=="unknown","custody promoted without evidence")
    require(resource["contract_access"]=="unknown","contract access promoted without evidence")
    require(resource["dispatch_authority"]=="not_granted","dispatch authority promoted")
    physical=resource["physical_resource_id"]
    aliases=doc["route_aliases"]
    require(len(aliases)==2,"expected two logical routes")
    require({a["logical_route_ref"] for a in aliases}=={"synthetic:route-a:shared-pump","synthetic:route-b:shared-pump"},"route identities")
    require(all(a["physical_resource_id"]==physical for a in aliases),"logical route aliases split one physical resource")
    require(all(a["dependency_role"]=="claim_critical" for a in aliases),"claim-critical dependency dropped")
    dep=doc["dependency"]
    require(dep["both_route_paths_depend_on"]==[physical],"shared-resource dependency closure lost")
    require(dep["failure_domain_id"]=="synthetic:failure-domain:pump-01","common-cause identity changed")
    claims=doc["time_indexed_resource_claims"]
    require(len(claims)==2 and {c["time_index"] for c in claims}=={0,1},"time windows collapsed or duplicated")
    for c in claims:
        require(c["physical_resource_id"]==physical,"time-indexed claim resource identity mismatch")
        require(c["unit"]=="t","unit promotion or mismatch")
        require(type(c["lower"]) is int and type(c["upper"]) is int and 0<=c["lower"]<=c["upper"],"invalid uncertainty interval")
        require(c["claim_status"]=="reported_unverified_in_real_world_synthetic_admitted_for_fixture","synthetic fixture claim promoted into field evidence")
        require(c["valid_interval"]==f'[{c["time_index"]},{c["time_index"]+1})',"capacity claim valid interval mismatch")
        expected_bounds={0:(6,8),1:(3,8)}[c["time_index"]]
        require((c["lower"],c["upper"])==expected_bounds,"time-indexed capacity claim differs from frozen source scenario")
    return True

def main():
    path=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).with_name("claim-fidelity-known-answer.json")
    doc=json.loads(path.read_text(encoding="utf-8"))
    verify(doc)
    attacks=[]
    a=copy.deepcopy(doc); a["route_aliases"][1]["physical_resource_id"]="synthetic:physical:pump-02"; attacks.append(a)
    a=copy.deepcopy(doc); a["issuer"]["recognition_for_claim_purpose"]="recognized"; attacks.append(a)
    a=copy.deepcopy(doc); a["resource"]["current_availability"]="available"; attacks.append(a)
    a=copy.deepcopy(doc); a["resource"]["title"]="established"; attacks.append(a)
    a=copy.deepcopy(doc); a["resource"]["dispatch_authority"]="granted"; attacks.append(a)
    a=copy.deepcopy(doc); a["time_indexed_resource_claims"][1]["valid_interval"]="[0,1)"; attacks.append(a)
    a=copy.deepcopy(doc); a["time_indexed_resource_claims"][1]["lower"]=8; attacks.append(a)
    a=copy.deepcopy(doc); a["dependency"]["both_route_paths_depend_on"]=["synthetic:physical:pump-02"]; attacks.append(a)
    rejected=0
    for attack in attacks:
        try: verify(attack)
        except (AssertionError,KeyError,TypeError): rejected+=1
    require(rejected==len(attacks),"a semantic-promotion mutation escaped the independent verifier")
    print(f"PASS: 2 frozen scenario references; 1 shared physical resource; 2 separate capacity windows; 8 authority/currentness/dependency mutations rejected")

if __name__=="__main__":
    main()
