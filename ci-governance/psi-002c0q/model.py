import json,tomllib
import spec,guard
def product(r):
 if (guard.g(r,"rev-parse",f"{spec.P}^{{tree}}"),guard.g(r,"rev-parse",f"{spec.P}^"),guard.g(r,"rev-parse",f"{spec.PP}^{{tree}}"))!=(spec.PT,spec.PP,spec.PPT):raise guard.E("product topology mismatch")
 if tuple(sorted(guard.g(r,"diff","--name-only",spec.PP,spec.P).splitlines()))!=tuple(sorted(spec.PB)):raise guard.E("product paths mismatch")
 for p,o in spec.PB.items():
  if guard.g(r,"rev-parse",f"{spec.P}:{p}")!=o:raise guard.E(f"product blob mismatch: {p}")
def blob(r,p):return guard.gb(r,"show",f"{spec.P}:{p}")
def model_lock(b):
 x=json.loads(b);a=x["authority_ceiling"];c=x["runtime_capabilities"];s=x["semantics"]
 if x["schema"]!="mycelix.psi.002c0.model-lock.v0.1" or x["parent"]!={"psi_002a_commit":"2be72da2acfd9903bfca168035c9ee087059f46f","psi_002b_commit":spec.PP,"psi_002b_runtime_contract_blob":"128821eb0d7cfc2715560c6310f91d9bac1fee2e"}:raise guard.E("model lock lineage")
 if not all(s[k] for k in ("failed_transition_state_unchanged","release_idempotent","released_commitment_remains_consumed","successful_admission_consumes_budget_and_commitment_together")):raise guard.E("model semantics")
 if any(c[k] for k in ("database","filesystem","network","randomness","threads","wall_clock")) or any(a[k] for k in ("runtime_atomicity_measured","crash_recovery_measured","distributed_consistency_measured","enumeration_resistance_established","production_admitted","sybil_resistance_established")):raise guard.E("authority/capability widened")
def probe(r):
 s=blob(r,spec.X+"psi-admission-reference-model/src/lib.rs").decode();c=tomllib.loads(blob(r,spec.X+"psi-admission-reference-model/Cargo.toml").decode());model_lock(blob(r,spec.X+"psi-admission-reference-model/MODEL.lock.json"))
 if set(c.get("dependencies",{}))!={"psi-abuse-control-profiles"} or set(c.get("dev-dependencies",{}))!={"privacy-computation-core"}:raise guard.E("dependency surface widened")
 req=(f'pub const PSI_002B_SUBJECT: &str = "{spec.PP}";',"pub profile: PsiAbuseControlProfile","return Ok(rejected(state, AdmissionDecision::MalformedPolicyState));","next.requests_used += 1;","next.consumed_request_commitments","next.active_reservations.insert(request.request_commitment);","next.active_reservations.remove(&request_commitment);","AdmissionDecision::EpochClosed","AdmissionDecision::EpochNotCurrent")
 if any(x not in s for x in req):raise guard.E("required model invariant absent")
 if any(x in s for x in ("std::fs","std::net","std::process","std::thread","tokio","rusqlite","sqlx","sled","redb","rand::","SystemTime","Instant::now","unsafe {",'extern "C"',"voprf::","SyntheticIdentifier","BlindedRequest")):raise guard.E("runtime/backend capability present")
 if s.count("#[test]")!=spec.T:raise guard.E("test count mismatch")
 return {"tests":spec.T,"pure_reference_model":True}
