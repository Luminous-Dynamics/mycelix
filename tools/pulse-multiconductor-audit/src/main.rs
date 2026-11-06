use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::alloc::{GlobalAlloc, Layout, System};
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};

struct CountingAllocator;

static LIVE_ALLOCATED_BYTES: AtomicUsize = AtomicUsize::new(0);
static PEAK_LIVE_ALLOCATED_BYTES: AtomicUsize = AtomicUsize::new(0);
static TOTAL_ALLOCATED_BYTES: AtomicUsize = AtomicUsize::new(0);

#[derive(Debug, Clone, Copy)]
struct AllocationSnapshot {
    live_bytes: usize,
    peak_live_bytes: usize,
    total_allocated_bytes: usize,
}

fn update_peak_live_bytes(current: usize) {
    let mut observed = PEAK_LIVE_ALLOCATED_BYTES.load(Ordering::Relaxed);
    while current > observed {
        match PEAK_LIVE_ALLOCATED_BYTES.compare_exchange_weak(
            observed,
            current,
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            Ok(_) => break,
            Err(actual) => observed = actual,
        }
    }
}

fn record_allocation(size: usize) {
    if size == 0 {
        return;
    }
    TOTAL_ALLOCATED_BYTES.fetch_add(size, Ordering::Relaxed);
    let current = LIVE_ALLOCATED_BYTES.fetch_add(size, Ordering::Relaxed) + size;
    update_peak_live_bytes(current);
}

fn record_deallocation(size: usize) {
    if size > 0 {
        LIVE_ALLOCATED_BYTES.fetch_sub(size, Ordering::Relaxed);
    }
}

fn allocation_snapshot() -> AllocationSnapshot {
    AllocationSnapshot {
        live_bytes: LIVE_ALLOCATED_BYTES.load(Ordering::Relaxed),
        peak_live_bytes: PEAK_LIVE_ALLOCATED_BYTES.load(Ordering::Relaxed),
        total_allocated_bytes: TOTAL_ALLOCATED_BYTES.load(Ordering::Relaxed),
    }
}

fn reset_peak_to_current_live_bytes() {
    PEAK_LIVE_ALLOCATED_BYTES.store(
        LIVE_ALLOCATED_BYTES.load(Ordering::Relaxed),
        Ordering::Relaxed,
    );
}

// This allocator is used only to expose benchmark phase allocation/high-water
// metrics. Correctness remains covered by the normal test suite; RSS is still
// measured independently by the workflow's /usr/bin/time wrapper.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        // SAFETY: delegated to the system allocator with the original layout.
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() {
            record_allocation(layout.size());
        }
        pointer
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        // SAFETY: delegated to the system allocator with the original layout.
        let pointer = unsafe { System.alloc_zeroed(layout) };
        if !pointer.is_null() {
            record_allocation(layout.size());
        }
        pointer
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        if !pointer.is_null() {
            record_deallocation(layout.size());
        }
        // SAFETY: pointer/layout are passed back unchanged to the system allocator.
        unsafe { System.dealloc(pointer, layout) };
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        // SAFETY: pointer/layout are passed back unchanged to the system allocator.
        let new_pointer = unsafe { System.realloc(pointer, layout, new_size) };
        if !new_pointer.is_null() {
            if new_size > layout.size() {
                record_allocation(new_size - layout.size());
            } else {
                record_deallocation(layout.size() - new_size);
            }
        }
        new_pointer
    }
}

#[global_allocator]
static GLOBAL_ALLOCATOR: CountingAllocator = CountingAllocator;

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct History {
    schema_version: u32,
    trace_id: String,
    evidence_class: String,
    source_commit: String,
    conductors: Vec<Conductor>,
    events: Vec<Event>,
    causal_edges: Vec<CausalEdge>,
}
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Conductor {
    conductor_id: String,
    agent_key_ref: String,
    conductor_build: String,
    dna_hash: String,
    cell_id: String,
    integrity_wasm_sha256: String,
    role: String,
}
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Event {
    event_id: String,
    conductor_id: String,
    local_sequence: u64,
    source: String,
    evidence_ref: String,
    event_type: String,
    data: Value,
}
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct CausalEdge {
    from_event: String,
    to_event: String,
    relation: String,
    evidence_ref: String,
}
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "UPPERCASE")]
enum Status { Pass, Fail, Incomplete }
#[derive(Debug, Clone, Serialize)]
struct Finding {
    code: String,
    severity: String,
    event_id: Option<String>,
    detail: String,
}
#[derive(Debug, Clone, Default, Serialize)]
struct Counts {
    conductors: usize,
    events: usize,
    causal_edges: usize,
    grants_issued: usize,
    grants_revoked: usize,
    claims_accepted: usize,
    host_calls: usize,
    domain_effects: usize,
    actions_authored: usize,
    validator_dispositions: usize,
    projections_observed: usize,
    remote_signals: usize,
    caller_outcomes: usize,
}
#[derive(Debug, Clone, Serialize)]
struct Report {
    schema_version: u32,
    status: Status,
    trace_id: String,
    evidence_class: String,
    input_sha256: String,
    counts: Counts,
    findings: Vec<Finding>,
    claim_ceiling: String,
}
#[derive(Debug, Clone)]
struct Grant {
    conductor_id: String,
    action_hash: String,
    grantor: String,
    grantee: String,
    assigned: bool,
    functions: BTreeSet<String>,
    issue_index: u64,
    revoke_index: Option<u64>,
}
#[derive(Debug, Clone)]
struct Call {
    event_id: String,
    conductor_id: String,
    host_authorized: bool,
    domain_allowed: bool,
}

fn finding(findings: &mut Vec<Finding>, code: &str, severity: &str, event_id: Option<&str>, detail: impl Into<String>) {
    findings.push(Finding {
        code: code.to_string(),
        severity: severity.to_string(),
        event_id: event_id.map(ToString::to_string),
        detail: detail.into(),
    });
}
fn valid_sha256(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
}

fn valid_sha(value: &str) -> bool {
    value.len() == 40 && value.bytes().all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
}
fn str_value<'a>(data: &'a Value, key: &str) -> Option<&'a str> {
    data.get(key).and_then(Value::as_str)
}
fn nonempty_str(data: &Value, key: &str) -> bool {
    str_value(data, key).is_some_and(|v| !v.trim().is_empty())
}
fn u64_value(data: &Value, key: &str) -> Option<u64> {
    data.get(key).and_then(Value::as_u64)
}
fn bool_value(data: &Value, key: &str) -> Option<bool> {
    data.get(key).and_then(Value::as_bool)
}
fn str_array(data: &Value, key: &str) -> Option<Vec<String>> {
    data.get(key)?.as_array()?.iter().map(|v| v.as_str().map(ToString::to_string)).collect()
}
fn exact_fields(data: &Value, expected: &[&str]) -> bool {
    let Some(map) = data.as_object() else { return false; };
    map.len() == expected.len() && expected.iter().all(|key| map.contains_key(*key))
}

fn exact_fields_with_optional(data: &Value, required: &[&str], optional: &[&str]) -> bool {
    let Some(map) = data.as_object() else { return false; };
    required.iter().all(|key| map.contains_key(*key))
        && map.keys().all(|key| required.contains(&key.as_str()) || optional.contains(&key.as_str()))
}

fn valid_event_shape(event: &Event) -> bool {
    let (fields, strings, numbers, booleans, arrays): (&[&str], &[&str], &[&str], &[&str], &[&str]) =
        match event.event_type.as_str() {
            "host_grant_issued" => (
                &["grant_ref","grant_action_hash","grantor","grantee","assigned","functions","idempotency_key","request_digest_sha256","host_authority_index"],
                &["grant_ref","grant_action_hash","grantor","grantee","idempotency_key","request_digest_sha256"],
                &["host_authority_index"], &["assigned"], &["functions"]),
            "host_grant_revoked" => (
                &["grant_ref","target_grant_action_hash","actor","revocation_action_hash","host_authority_index"],
                &["grant_ref","target_grant_action_hash","actor","revocation_action_hash"],
                &["host_authority_index"], &[], &[]),
            "capability_claim_accepted" => (
                &["grant_ref","grant_action_hash","recipient","claim_action_hash"],
                &["grant_ref","grant_action_hash","recipient","claim_action_hash"], &[], &[], &[]),
            "host_call" => (
                &["operation_id","attempt_id","grant_ref","grant_action_hash","caller","function","host_authority_index","host_decision","host_state_at_decision","domain_decision","projection_used_for_authorization"],
                &["operation_id","grant_ref","grant_action_hash","caller","function","host_decision","host_state_at_decision","domain_decision"],
                &["attempt_id","host_authority_index"], &["projection_used_for_authorization"], &[]),
            "domain_effect" => (
                &["operation_id","attempt_id","effect_id","action_hash"],
                &["operation_id","effect_id","action_hash"], &["attempt_id"], &[], &[]),
            "action_authored" => (
                &["action_hash","author","entry_type","payload_sha256"],
                &["action_hash","author","entry_type","payload_sha256"], &[], &[], &[]),
            "validation_disposition" => (
                &["action_hash","author_conductor_id","receiving_conductor_id","disposition","validator_wasm_sha256","reason_code"],
                &["action_hash","author_conductor_id","receiving_conductor_id","disposition","validator_wasm_sha256"], &[], &[], &[]),
            "projection_observed" => (
                &["action_hash","author_conductor_id","projection_state","decision","required_dna_hash","observed_dna_hash","required_cell_id","observed_cell_id","required_author","observed_author","required_action_hash","observed_action_hash","required_epoch","observed_epoch","descends_from_required"],
                &["action_hash","author_conductor_id","projection_state","decision","required_dna_hash","observed_dna_hash","required_cell_id","observed_cell_id","required_author","observed_author","required_action_hash","observed_action_hash"],
                &["required_epoch","observed_epoch"], &["descends_from_required"], &[]),
            "remote_signal" => (
                &["event_type","trust_class","transport_sender","claimed_sender","backing_action_hash","record_author","verified_payload_sha256","verifier_build","backing_author_conductor_id","backing_validator_event_id","trusted_state_mutation","authorizes_operation"],
                &["event_type","trust_class"], &[], &["trusted_state_mutation","authorizes_operation"], &[]),
            "caller_outcome" => (
                &["operation_id","attempt_id","outcome"], &["operation_id","outcome"], &["attempt_id"], &[], &[]),
            _ => return false,
        };
    let exact_shape = if event.event_type == "projection_observed" {
        exact_fields_with_optional(&event.data, fields, &["validator_event_id"])
    } else {
        exact_fields(&event.data, fields)
    };
    let optional_projection_ref_valid = event.event_type != "projection_observed"
        || event.data.get("validator_event_id").is_none_or(|v| v.as_str().is_some_and(|s| !s.trim().is_empty()));
    if !exact_shape
        || !optional_projection_ref_valid
        || !strings.iter().all(|key| nonempty_str(&event.data,key))
        || !numbers.iter().all(|key| u64_value(&event.data,key).is_some())
        || !booleans.iter().all(|key| bool_value(&event.data,key).is_some())
        || !arrays.iter().all(|key| str_array(&event.data,key).is_some_and(|v| !v.is_empty() && v.iter().all(|s| !s.trim().is_empty())))
    { return false; }

    match event.event_type.as_str() {
        "host_grant_issued" => u64_value(&event.data,"host_authority_index").is_some_and(|v| v > 0)
            && str_value(&event.data,"request_digest_sha256").is_some_and(valid_sha256),
        "host_grant_revoked" => u64_value(&event.data,"host_authority_index").is_some_and(|v| v > 0),
        "action_authored" => str_value(&event.data,"payload_sha256").is_some_and(valid_sha256),
        "host_call" => {
            u64_value(&event.data,"attempt_id").is_some_and(|v| v > 0)
                && u64_value(&event.data,"host_authority_index").is_some_and(|v| v > 0)
                && matches!(str_value(&event.data,"host_decision"),Some("authorized"|"unauthorized"|"unknown"))
                && matches!(str_value(&event.data,"host_state_at_decision"),Some("active"|"revoked"|"absent"|"unknown"))
                && matches!(str_value(&event.data,"domain_decision"),Some("allow"|"deny"|"not_run"|"unknown"))
        }
        "domain_effect" => u64_value(&event.data,"attempt_id").is_some_and(|v| v > 0),
        "caller_outcome" => u64_value(&event.data,"attempt_id").is_some_and(|v| v > 0)
            && matches!(str_value(&event.data,"outcome"),Some("acknowledged"|"unknown"|"committed"|"not_committed"|"rejected")),
        "validation_disposition" => {
            matches!(str_value(&event.data,"disposition"),Some("accepted"|"rejected"))
                && str_value(&event.data,"validator_wasm_sha256").is_some_and(valid_sha256)
                && (event.data["reason_code"].is_null() || event.data["reason_code"].is_string())
        }
        "projection_observed" => {
            matches!(str_value(&event.data,"projection_state"),Some("integrated"|"not_integrated"|"pending"|"conflicting"|"unknown"))
                && matches!(str_value(&event.data,"decision"),Some("trusted"|"stale_advisory"|"unknown"|"conflicting"))
        }
        "remote_signal" => {
            matches!(str_value(&event.data,"trust_class"),Some("verified_record"|"authenticated_peer_hint"|"unverified_hint"|"local_only"|"unknown"|"rejected"))
                && ["transport_sender","claimed_sender","backing_action_hash","record_author","verified_payload_sha256","verifier_build","backing_author_conductor_id","backing_validator_event_id"]
                    .iter().all(|k| event.data[*k].is_null() || event.data[*k].is_string())
                && (event.data["verified_payload_sha256"].is_null()
                    || str_value(&event.data,"verified_payload_sha256").is_some_and(valid_sha256))
        }
        _ => true,
    }
}

fn relation_valid(relation: &str) -> bool {
    matches!(relation,"program_order"|"request_response"|"source_action_dependency"|"validator_receipt"|"observer_confirmation"|"test_harness_barrier")
}
fn path_exists(adjacency: &BTreeMap<String,BTreeSet<String>>, from: &str, to: &str) -> bool {
    if from == to { return true; }
    let mut seen = BTreeSet::new();
    let mut stack = vec![from.to_string()];
    while let Some(node) = stack.pop() {
        if !seen.insert(node.clone()) { continue; }
        if let Some(next) = adjacency.get(&node) {
            for target in next {
                if target == to { return true; }
                stack.push(target.clone());
            }
        }
    }
    false
}
fn graph_has_cycle(adjacency: &BTreeMap<String,BTreeSet<String>>, nodes: &BTreeSet<String>) -> bool {
    let mut indegree: BTreeMap<String,usize> = nodes.iter().map(|n|(n.clone(),0)).collect();
    for targets in adjacency.values() { for target in targets { *indegree.entry(target.clone()).or_default() += 1; } }
    let mut queue: VecDeque<String> = indegree.iter().filter(|(_,d)| **d == 0).map(|(n,_)|n.clone()).collect();
    let mut visited=0;
    while let Some(node)=queue.pop_front() {
        visited+=1;
        if let Some(next)=adjacency.get(&node) {
            for target in next {
                if let Some(d)=indegree.get_mut(target) { *d-=1; if *d==0 { queue.push_back(target.clone()); } }
            }
        }
    }
    visited != nodes.len()
}
fn direct_relation(edges:&[CausalEdge],from:&str,to:&str,relation:&str)->bool {
    edges.iter().any(|e|e.from_event==from && e.to_event==to && e.relation==relation)
}

fn audit(history:&History,input_sha256:String)->Report {
    let mut findings=Vec::new();
    let mut counts=Counts{conductors:history.conductors.len(),events:history.events.len(),causal_edges:history.causal_edges.len(),..Counts::default()};
    if history.schema_version!=2 { finding(&mut findings,"UNSUPPORTED_SCHEMA_VERSION","incomplete",None,"expected schema version 2"); }
    if history.trace_id.trim().is_empty() || !valid_sha(&history.source_commit) { finding(&mut findings,"HISTORY_HEADER_INCOMPLETE","incomplete",None,"trace_id and lowercase 40-character source_commit are required"); }
    if !matches!(history.evidence_class.as_str(),"synthetic"|"runtime_capture") { finding(&mut findings,"EVIDENCE_CLASS_INVALID","incomplete",None,"evidence_class must be synthetic or runtime_capture"); }
    if history.conductors.len()<2 { finding(&mut findings,"MULTICONDUCTOR_HISTORY_REQUIRED","incomplete",None,"at least two conductors are required"); }

    let mut conductors:BTreeMap<String,&Conductor>=BTreeMap::new();
    for c in &history.conductors {
        let valid=!c.conductor_id.trim().is_empty()&&!c.agent_key_ref.trim().is_empty()&&!c.conductor_build.trim().is_empty()&&!c.dna_hash.trim().is_empty()&&!c.cell_id.trim().is_empty()
            && valid_sha256(&c.integrity_wasm_sha256)
            && matches!(c.role.as_str(),"grant_owner"|"caller"|"validator"|"observer"|"peer");
        if !valid { finding(&mut findings,"CONDUCTOR_IDENTITY_INCOMPLETE","incomplete",None,"conductor identity, build, DNA, cell and supported role are required"); }
        if conductors.insert(c.conductor_id.clone(),c).is_some() { finding(&mut findings,"DUPLICATE_CONDUCTOR_ID","violation",None,"conductor_id must be unique"); }
    }

    let mut event_map:BTreeMap<String,&Event>=BTreeMap::new();
    let mut nodes=BTreeSet::new();
    let mut by_conductor:BTreeMap<String,Vec<&Event>>=BTreeMap::new();
    for e in &history.events {
        nodes.insert(e.event_id.clone());
        if e.event_id.trim().is_empty()||e.evidence_ref.trim().is_empty()||e.local_sequence==0 { finding(&mut findings,"EVENT_IDENTITY_INCOMPLETE","incomplete",Some(&e.event_id),"event_id, evidence_ref and positive local_sequence are required"); }
        if !conductors.contains_key(&e.conductor_id) { finding(&mut findings,"UNKNOWN_CONDUCTOR","incomplete",Some(&e.event_id),format!("unknown conductor {}",e.conductor_id)); }
        if !matches!(e.source.as_str(),"conductor_host"|"source_chain"|"validator"|"observer"|"remote_peer"|"client"|"coordinator"|"synthetic") {
            finding(&mut findings,"EVENT_SOURCE_INVALID","incomplete",Some(&e.event_id),"source is outside the v1 evidence vocabulary");
        }
        if !valid_event_shape(e) { finding(&mut findings,"EVENT_DATA_SHAPE_INVALID","incomplete",Some(&e.event_id),format!("{} data is missing, mistyped or contains unknown fields",e.event_type)); }
        if history.evidence_class=="runtime_capture"&&e.source=="synthetic" { finding(&mut findings,"SYNTHETIC_EVENT_IN_RUNTIME_CAPTURE","incomplete",Some(&e.event_id),"runtime capture cannot contain synthetic evidence"); }
        if event_map.insert(e.event_id.clone(),e).is_some() { finding(&mut findings,"DUPLICATE_EVENT_ID","violation",Some(&e.event_id),"event IDs must be globally unique"); }
        by_conductor.entry(e.conductor_id.clone()).or_default().push(e);
    }

    // Local sequence numbers create order only within their own conductor.
    let mut adjacency:BTreeMap<String,BTreeSet<String>>=BTreeMap::new();
    for (conductor,events) in &mut by_conductor {
        events.sort_by_key(|e|e.local_sequence);
        let mut seen=BTreeSet::new();
        let mut previous:Option<&Event>=None;
        for e in events.iter() {
            if !seen.insert(e.local_sequence) { finding(&mut findings,"DUPLICATE_LOCAL_SEQUENCE","violation",Some(&e.event_id),format!("conductor {conductor} reused local_sequence {}",e.local_sequence)); }
            if let Some(prev)=previous { adjacency.entry(prev.event_id.clone()).or_default().insert(e.event_id.clone()); }
            previous=Some(*e);
        }
    }

    let mut edge_keys=BTreeSet::new();
    for edge in &history.causal_edges {
        if !edge_keys.insert((edge.from_event.clone(),edge.to_event.clone(),edge.relation.clone())) { finding(&mut findings,"DUPLICATE_CAUSAL_EDGE","violation",None,"duplicate causal edge"); }
        if edge.from_event==edge.to_event { finding(&mut findings,"SELF_CAUSAL_EDGE","violation",None,"causal edge cannot point to itself"); }
        if edge.evidence_ref.trim().is_empty()||!relation_valid(&edge.relation) { finding(&mut findings,"CAUSAL_EDGE_EVIDENCE_INVALID","incomplete",None,"edge relation and evidence_ref are required"); }
        let (Some(from),Some(to))=(event_map.get(&edge.from_event),event_map.get(&edge.to_event)) else {
            finding(&mut findings,"CAUSAL_EDGE_ENDPOINT_MISSING","incomplete",None,"edge references a missing event"); continue;
        };
        if edge.relation=="program_order" && (from.conductor_id!=to.conductor_id||from.local_sequence>=to.local_sequence) {
            finding(&mut findings,"PROGRAM_ORDER_CROSSES_CONDUCTORS","violation",Some(&to.event_id),"program_order is valid only within one conductor with increasing local_sequence");
        }
        adjacency.entry(edge.from_event.clone()).or_default().insert(edge.to_event.clone());
    }
    if graph_has_cycle(&adjacency,&nodes) { finding(&mut findings,"CAUSAL_GRAPH_CYCLE","violation",None,"local order plus evidence-backed edges must form a DAG"); }

    let mut grants:BTreeMap<(String,String),Grant>=BTreeMap::new();
    let mut grant_hashes=BTreeSet::new();
    let mut idempotency:BTreeMap<(String,String,String),(String,String)>=BTreeMap::new();
    let mut grant_events=Vec::new();
    let mut revoke_events=Vec::new();
    let mut claim_events=Vec::new();
    let mut call_events=Vec::new();
    let mut effect_events=Vec::new();
    // Action hashes are scoped by their authoring conductor/cell lineage. The same
    // byte-level hash value in two independent cells must not alias histories.
    let mut authored:BTreeMap<(String,String),&Event>=BTreeMap::new();
    let mut validation_events=Vec::new();
    let mut projection_events=Vec::new();
    let mut outcome_events=Vec::new();

    for e in &history.events {
        let d=&e.data;
        match e.event_type.as_str() {
            "host_grant_issued"=>{
                counts.grants_issued+=1;grant_events.push(e);
                let grant_ref=str_value(d,"grant_ref").unwrap_or_default().to_string();
                let action_hash=str_value(d,"grant_action_hash").unwrap_or_default().to_string();
                let grantor=str_value(d,"grantor").unwrap_or_default().to_string();
                let grantee=str_value(d,"grantee").unwrap_or_default().to_string();
                let assigned=bool_value(d,"assigned").unwrap_or(false);
                let functions=str_array(d,"functions").unwrap_or_default();
                let index=u64_value(d,"host_authority_index").unwrap_or(0);
                if !grant_hashes.insert((e.conductor_id.clone(),action_hash.clone())) { finding(&mut findings,"DUPLICATE_GRANT_ACTION_HASH","violation",Some(&e.event_id),"grant ActionHash reused within one conductor"); }
                if let Some(c)=conductors.get(&e.conductor_id) {
                    if c.agent_key_ref!=grantor { finding(&mut findings,"GRANTOR_CONDUCTOR_MISMATCH","violation",Some(&e.event_id),"grantor does not match host conductor agent key"); }
                    if c.role!="grant_owner" { finding(&mut findings,"GRANT_ISSUED_BY_NON_OWNER_CONDUCTOR","incomplete",Some(&e.event_id),"grant issuance should be observed on grant_owner conductor"); }
                }
                let request_digest=str_value(d,"request_digest_sha256").unwrap_or_default().to_string();
                let idem=(e.conductor_id.clone(),grantor.clone(),str_value(d,"idempotency_key").unwrap_or_default().to_string());
                if let Some((old_grant,old_digest))=idempotency.get(&idem) {
                    if old_digest!=&request_digest {
                        finding(&mut findings,"IDEMPOTENCY_KEY_REUSED_WITH_DIFFERENT_REQUEST","violation",Some(&e.event_id),"same conductor/grantor idempotency key was reused with a different canonical request digest");
                    }
                    if old_grant!=&grant_ref {
                        finding(&mut findings,"IDEMPOTENCY_KEY_MINTED_MULTIPLE_GRANTS","violation",Some(&e.event_id),"same conductor/grantor idempotency key minted multiple grant references");
                    }
                } else {
                    idempotency.insert(idem,(grant_ref.clone(),request_digest));
                }
                let key=(e.conductor_id.clone(),grant_ref.clone());
                if grants.contains_key(&key) { finding(&mut findings,"GRANT_REFERENCE_REUSED","violation",Some(&e.event_id),"grant_ref reused on one host conductor"); }
                else { grants.insert(key,Grant{conductor_id:e.conductor_id.clone(),action_hash,grantor,grantee,assigned,functions:functions.into_iter().collect(),issue_index:index,revoke_index:None}); }
            }
            "host_grant_revoked"=>{counts.grants_revoked+=1;revoke_events.push(e);}
            "capability_claim_accepted"=>{counts.claims_accepted+=1;claim_events.push(e);}
            "host_call"=>{counts.host_calls+=1;call_events.push(e);}
            "domain_effect"=>{counts.domain_effects+=1;effect_events.push(e);}
            "action_authored"=>{
                counts.actions_authored+=1;
                let hash=str_value(d,"action_hash").unwrap_or_default().to_string();
                let identity=(e.conductor_id.clone(),hash);
                if let Some(old)=authored.insert(identity,e) { finding(&mut findings,"ACTION_HASH_AUTHORED_TWICE","violation",Some(&e.event_id),format!("action hash also authored in the same conductor/cell as {}",old.event_id)); }
                if let Some(c)=conductors.get(&e.conductor_id) {
                    if str_value(d,"author")!=Some(c.agent_key_ref.as_str()) { finding(&mut findings,"ACTION_AUTHOR_CONDUCTOR_MISMATCH","violation",Some(&e.event_id),"action author differs from author conductor identity"); }
                }
            }
            "validation_disposition"=>{counts.validator_dispositions+=1;validation_events.push(e);}
            "projection_observed"=>{counts.projections_observed+=1;projection_events.push(e);}
            "remote_signal"=>{ counts.remote_signals+=1; }
            "caller_outcome"=>{counts.caller_outcomes+=1;outcome_events.push(e);}
            _=>{}
        }
    }

    // Remote signal trust is a claim about evidence, not authority. Bind a
    // verified_record assertion to exact source-chain authorship, payload digest,
    // validator WASM and observer lineage before accepting its structural shape.
    for e in history.events.iter().filter(|e| e.event_type == "remote_signal") {
        let d = &e.data;
        let trust = str_value(d, "trust_class").unwrap_or_default();
        let transport = str_value(d, "transport_sender");
        let claimed = str_value(d, "claimed_sender");
        let record_author = str_value(d, "record_author");
        let backing_hash = str_value(d, "backing_action_hash");
        let digest = str_value(d, "verified_payload_sha256");
        let verifier_build = str_value(d, "verifier_build");
        let author_conductor_id = str_value(d, "backing_author_conductor_id");
        let validator_event_id = str_value(d, "backing_validator_event_id");

        if bool_value(d, "authorizes_operation") == Some(true) {
            finding(&mut findings, "REMOTE_SIGNAL_USED_AS_AUTHORITY", "violation", Some(&e.event_id), "remote signal cannot authorize an operation");
        }
        if trust != "verified_record" && bool_value(d, "trusted_state_mutation") == Some(true) {
            finding(&mut findings, "UNVERIFIED_SIGNAL_MUTATED_TRUSTED_STATE", "violation", Some(&e.event_id), "unverified or advisory signal cannot mutate trusted state");
        }

        match trust {
            "verified_record" => {
                let observer = conductors.get(&e.conductor_id);
                if e.source != "observer" || observer.is_none_or(|c| c.role != "observer") {
                    finding(&mut findings, "VERIFIED_SIGNAL_WRONG_OBSERVER", "incomplete", Some(&e.event_id), "verified_record must be emitted by an observer conductor");
                }
                if transport.is_none() || transport != claimed || claimed != record_author {
                    finding(&mut findings, "VERIFIED_SIGNAL_SENDER_MISMATCH", "violation", Some(&e.event_id), "transport sender, claimed sender and record author must match exactly");
                }
                if backing_hash.is_none_or(str::is_empty)
                    || digest.is_none_or(|v| !valid_sha256(v))
                    || verifier_build.is_none_or(str::is_empty)
                    || author_conductor_id.is_none_or(str::is_empty)
                    || validator_event_id.is_none_or(str::is_empty) {
                    finding(&mut findings, "VERIFIED_SIGNAL_EVIDENCE_INCOMPLETE", "incomplete", Some(&e.event_id), "verified_record requires exact author conductor, backing ActionHash, validator event, payload digest and verifier build");
                    continue;
                }

                let Some(author_conductor) = conductors.get(author_conductor_id.unwrap()) else {
                    finding(&mut findings, "VERIFIED_SIGNAL_AUTHOR_CONDUCTOR_UNKNOWN", "incomplete", Some(&e.event_id), "backing author conductor is not declared");
                    continue;
                };
                if author_conductor.agent_key_ref != record_author.unwrap()
                    || observer.is_none_or(|o| o.dna_hash != author_conductor.dna_hash) {
                    finding(&mut findings, "VERIFIED_SIGNAL_AUTHOR_LINEAGE_MISMATCH", "violation", Some(&e.event_id), "backing author and observer must share the same DNA network; their CellIds are agent-specific");
                }

                let authored_key = (author_conductor_id.unwrap().to_string(), backing_hash.unwrap().to_string());
                let Some(authored_event) = authored.get(&authored_key) else {
                    finding(&mut findings, "VERIFIED_SIGNAL_ACTION_NOT_AUTHORED", "incomplete", Some(&e.event_id), "backing ActionHash does not resolve to authored source-chain action in the claimed author conductor");
                    continue;
                };
                if authored_event.source != "source_chain"
                    || str_value(&authored_event.data, "author") != record_author
                    || str_value(&authored_event.data, "payload_sha256") != digest {
                    finding(&mut findings, "VERIFIED_SIGNAL_PAYLOAD_OR_AUTHOR_MISMATCH", "violation", Some(&e.event_id), "authored source action author and canonical payload digest must match the verified record");
                }

                let Some(validator_event) = validation_events.iter().find(|v| v.event_id == validator_event_id.unwrap()) else {
                    finding(&mut findings, "VERIFIED_SIGNAL_VALIDATOR_EVENT_MISSING", "incomplete", Some(&e.event_id), "backing_validator_event_id does not identify a validation disposition");
                    continue;
                };
                let validator_conductor = conductors.get(&validator_event.conductor_id);
                let exact_validation = str_value(&validator_event.data, "action_hash") == backing_hash
                    && str_value(&validator_event.data, "author_conductor_id") == author_conductor_id
                    && str_value(&validator_event.data, "receiving_conductor_id") == Some(validator_event.conductor_id.as_str())
                    && str_value(&validator_event.data, "disposition") == Some("accepted")
                    && validator_conductor.is_some_and(|v| v.role == "validator"
                        && v.dna_hash == author_conductor.dna_hash
                        && str_value(&validator_event.data, "validator_wasm_sha256") == Some(v.integrity_wasm_sha256.as_str()));
                if !exact_validation {
                    if str_value(&validator_event.data, "disposition") == Some("rejected") {
                        finding(&mut findings, "REJECTED_RECORD_MARKED_VERIFIED", "violation", Some(&e.event_id), "a rejected source action cannot be promoted to verified_record");
                    } else {
                        finding(&mut findings, "VERIFIED_SIGNAL_VALIDATOR_MISMATCH", "violation", Some(&e.event_id), "validator receipt must accept the exact action from the declared author and matching DNA/WASM lineage");
                    }
                }
                if !direct_relation(&history.causal_edges, &authored_event.event_id, &validator_event.event_id, "validator_receipt") {
                    finding(&mut findings, "VERIFIED_SIGNAL_AUTHOR_VALIDATOR_EDGE_MISSING", "incomplete", Some(&e.event_id), "verified record requires an explicit author-to-validator receipt edge");
                }
                if !direct_relation(&history.causal_edges, &validator_event.event_id, &e.event_id, "observer_confirmation") {
                    finding(&mut findings, "VERIFIED_SIGNAL_VALIDATOR_OBSERVER_EDGE_MISSING", "incomplete", Some(&e.event_id), "verified record requires an explicit validator-to-observer confirmation edge");
                }
            }
            "authenticated_peer_hint" => {
                if e.source != "remote_peer" || transport.is_none_or(str::is_empty) {
                    finding(&mut findings, "AUTHENTICATED_HINT_SENDER_MISSING", "incomplete", Some(&e.event_id), "authenticated_peer_hint requires transport-level sender evidence");
                }
                if claimed.is_some() && claimed != transport {
                    finding(&mut findings, "AUTHENTICATED_HINT_SENDER_MISMATCH", "violation", Some(&e.event_id), "payload sender claim conflicts with transport provenance");
                }
            }
            "unverified_hint" => {
                if e.source != "remote_peer" || transport.is_none_or(str::is_empty) {
                    finding(&mut findings, "UNVERIFIED_HINT_TRANSPORT_MISSING", "incomplete", Some(&e.event_id), "unverified_hint should retain transport sender evidence when received remotely");
                }
            }
            "local_only" => {
                if e.source == "remote_peer" || transport.is_some() || claimed.is_some() {
                    finding(&mut findings, "LOCAL_ONLY_HAS_REMOTE_SENDER", "violation", Some(&e.event_id), "local_only events cannot originate from remote-peer evidence or claim remote transport/payload sender identity");
                }
            }
            "unknown" | "rejected" => {}
            _ => finding(&mut findings, "REMOTE_SIGNAL_TRUST_CLASS_INVALID", "incomplete", Some(&e.event_id), "unsupported signal trust class"),
        }
    }

    // Never compare local counters from different conductors.
    for (conductor,events) in &by_conductor {
        let mut indexed:Vec<(&Event,u64)>=events.iter().filter_map(|e|match e.event_type.as_str(){
            "host_grant_issued"|"host_grant_revoked"|"host_call"=>Some((*e,u64_value(&e.data,"host_authority_index").unwrap_or(0))),
            _=>None
        }).collect();
        indexed.sort_by_key(|(e,_)|e.local_sequence);
        let mut previous=0;
        for (e,index) in indexed {
            if index==0||index<=previous { finding(&mut findings,"HOST_AUTHORITY_INDEX_NOT_MONOTONIC","violation",Some(&e.event_id),format!("host_authority_index did not increase within conductor {conductor}")); }
            previous=index;
        }
    }

    for e in &revoke_events {
        let d=&e.data;
        let key=(e.conductor_id.clone(),str_value(d,"grant_ref").unwrap_or_default().to_string());
        match grants.get_mut(&key) {
            None=>finding(&mut findings,"REVOCATION_WITHOUT_GRANT","incomplete",Some(&e.event_id),"revocation does not match a grant on the same conductor"),
            Some(g)=>{
                if str_value(d,"target_grant_action_hash")!=Some(g.action_hash.as_str()) { finding(&mut findings,"REVOCATION_TARGET_HASH_MISMATCH","violation",Some(&e.event_id),"revocation must target exact host grant ActionHash"); }
                if str_value(d,"actor")!=Some(g.grantor.as_str()) { finding(&mut findings,"REVOCATION_ACTOR_MISMATCH","violation",Some(&e.event_id),"only original grantor may revoke"); }
                let index=u64_value(d,"host_authority_index").unwrap_or(0);
                if index<=g.issue_index||g.revoke_index.is_some() { finding(&mut findings,"INVALID_GRANT_REVOCATION_ORDER","violation",Some(&e.event_id),"revocation must follow issuance once"); }
                g.revoke_index=Some(index);
            }
        }
    }

    let mut accepted_claims=BTreeSet::new();
    for e in &claim_events {
        let d=&e.data;
        let hash=str_value(d,"grant_action_hash").unwrap_or_default();
        let recipient=str_value(d,"recipient").unwrap_or_default();
        let candidates:Vec<_>=grants.values().filter(|g|g.action_hash==hash&&g.grantee==recipient).collect();
        let Some(g)=candidates.first() else { finding(&mut findings,"CLAIM_WITHOUT_MATCHING_GRANT","incomplete",Some(&e.event_id),"claim does not match issued grant ActionHash and grantee"); continue; };
        if !g.assigned { finding(&mut findings,"CLAIM_FOR_UNASSIGNED_GRANT","violation",Some(&e.event_id),"private cap claim only applies to Assigned grant"); }
        if conductors.get(&e.conductor_id).is_none_or(|c|c.agent_key_ref!=recipient) { finding(&mut findings,"CLAIM_RECIPIENT_CONDUCTOR_MISMATCH","violation",Some(&e.event_id),"claim must be accepted on recipient conductor"); }
        if e.source != "source_chain" { finding(&mut findings,"CLAIM_WRONG_EVIDENCE_SOURCE","incomplete",Some(&e.event_id),"claim acceptance must be evidenced by recipient source-chain state"); }
        let issue_id=grant_events.iter().find(|x|str_value(&x.data,"grant_action_hash")==Some(g.action_hash.as_str())).map(|x|x.event_id.as_str()).unwrap_or("");
        if !direct_relation(&history.causal_edges,issue_id,&e.event_id,"source_action_dependency") { finding(&mut findings,"GRANT_TO_CLAIM_CAUSAL_EDGE_MISSING","incomplete",Some(&e.event_id),"grant issue and private claim require an evidence-backed causal edge"); }
        if !accepted_claims.insert((g.conductor_id.clone(),g.action_hash.clone(),recipient.to_string())) { finding(&mut findings,"DUPLICATE_CAPABILITY_CLAIM","violation",Some(&e.event_id),"same assigned grant accepted more than one claim"); }
        let actual_ref=grant_events.iter().find(|x|str_value(&x.data,"grant_action_hash")==Some(g.action_hash.as_str())).and_then(|x|str_value(&x.data,"grant_ref"));
        if str_value(d,"grant_ref")!=actual_ref { finding(&mut findings,"CLAIM_GRANT_REFERENCE_MISMATCH","violation",Some(&e.event_id),"claim grant_ref conflicts with host issuance"); }
    }

    let mut calls:BTreeMap<(String,u64),Call>=BTreeMap::new();
    for e in &call_events {
        let d=&e.data;
        let grant_ref=str_value(d,"grant_ref").unwrap_or_default();
        let key=(e.conductor_id.clone(),grant_ref.to_string());
        let Some(g)=grants.get(&key) else { finding(&mut findings,"HOST_CALL_WITHOUT_LOCAL_GRANT","incomplete",Some(&e.event_id),"host call has no matching grant on receiving conductor"); continue; };
        let index=u64_value(d,"host_authority_index").unwrap_or(0);
        let expected_state=if index<=g.issue_index {"absent"}else if g.revoke_index.is_some_and(|r|index>=r){"revoked"}else{"active"};
        if str_value(d,"host_state_at_decision")!=Some(expected_state) { finding(&mut findings,"HOST_STATE_CONFLICT","violation",Some(&e.event_id),"host state conflicts with conductor-local authority index"); }
        if str_value(d,"grant_action_hash")!=Some(g.action_hash.as_str()) { finding(&mut findings,"CALL_GRANT_ACTION_HASH_MISMATCH","violation",Some(&e.event_id),"call must bind exact local grant ActionHash"); }
        if bool_value(d,"projection_used_for_authorization")==Some(true) { finding(&mut findings,"PROJECTION_USED_AS_AUTHORITY","violation",Some(&e.event_id),"projection cannot authorize host call"); }
        let host=str_value(d,"host_decision").unwrap_or_default();
        let domain=str_value(d,"domain_decision").unwrap_or_default();
        let scope_ok=str_value(d,"caller")==Some(g.grantee.as_str())&&str_value(d,"function").is_some_and(|f|g.functions.contains(f));
        let authorized=host=="authorized";
        if authorized&&(expected_state!="active"||!scope_ok) { finding(&mut findings,"HOST_ALLOW_OUTSIDE_GRANT","violation",Some(&e.event_id),"host allowed with inactive grant or wrong assigned caller/function"); }
        if authorized&&g.assigned&&!accepted_claims.contains(&(g.conductor_id.clone(),g.action_hash.clone(),g.grantee.clone())) { finding(&mut findings,"ASSIGNED_CALL_BEFORE_PRIVATE_CLAIM","violation",Some(&e.event_id),"assigned host grant used before recipient claim"); }
        if domain=="allow"&&!authorized { finding(&mut findings,"DOMAIN_ALLOW_WITHOUT_HOST_ALLOW","violation",Some(&e.event_id),"domain allow cannot follow host denial/unknown"); }
        if e.source!="conductor_host" { finding(&mut findings,"HOST_CALL_WRONG_EVIDENCE_SOURCE","incomplete",Some(&e.event_id),"host decision must come from conductor evidence"); }
        if authorized&&g.assigned {
            let claim=claim_events.iter().find(|c|str_value(&c.data,"grant_action_hash")==Some(g.action_hash.as_str())&&str_value(&c.data,"recipient")==Some(g.grantee.as_str()));
            if let Some(claim)=claim { if !path_exists(&adjacency,&claim.event_id,&e.event_id) { finding(&mut findings,"CLAIM_NOT_CAUSAL_BEFORE_HOST_CALL","incomplete",Some(&e.event_id),"claim must causally precede assigned call"); } }
        }
        let op=str_value(d,"operation_id").unwrap_or_default().to_string();
        let attempt=u64_value(d,"attempt_id").unwrap_or(0);
        if calls.insert((op,attempt),Call{event_id:e.event_id.clone(),conductor_id:e.conductor_id.clone(),host_authorized:authorized,domain_allowed:domain=="allow"}).is_some() { finding(&mut findings,"DUPLICATE_CALL_ATTEMPT","violation",Some(&e.event_id),"operation/attempt pair must be unique"); }
    }

    let mut effects_by_operation=BTreeSet::new();
    let mut effects_by_attempt=BTreeSet::new();
    let mut effect_bindings=BTreeSet::new();
    for e in &effect_events {
        let d=&e.data;
        let op=str_value(d,"operation_id").unwrap_or_default().to_string();
        let attempt=u64_value(d,"attempt_id").unwrap_or(0);
        if !effects_by_operation.insert(op.clone()) { finding(&mut findings,"MULTIPLE_EFFECTS_PER_LOGICAL_OPERATION","violation",Some(&e.event_id),"logical operation has more than one durable effect"); }
        if !effects_by_attempt.insert((op.clone(),attempt)) { finding(&mut findings,"MULTIPLE_EFFECTS_PER_ATTEMPT","violation",Some(&e.event_id),"attempt has more than one durable effect"); }
        let Some(call)=calls.get(&(op.clone(),attempt)) else { finding(&mut findings,"EFFECT_WITHOUT_HOST_CALL","incomplete",Some(&e.event_id),"domain effect has no matching host call"); continue; };
        if !call.host_authorized||!call.domain_allowed { finding(&mut findings,"EFFECT_WITHOUT_AUTHORIZED_DOMAIN_CALL","violation",Some(&e.event_id),"effect requires host authorization and domain allow"); }
        if e.conductor_id!=call.conductor_id||!path_exists(&adjacency,&call.event_id,&e.event_id) { finding(&mut findings,"EFFECT_NOT_CAUSAL_AFTER_HOST_CALL","incomplete",Some(&e.event_id),"effect must causally follow call on the same conductor"); }
        if !effect_bindings.insert((op,attempt)) { finding(&mut findings,"DUPLICATE_EFFECT_BINDING","violation",Some(&e.event_id),"effect bound twice to one attempt"); }
    }

    for e in &validation_events {
        let d=&e.data;
        let hash=str_value(d,"action_hash").unwrap_or_default();
        let author_conductor=str_value(d,"author_conductor_id").unwrap_or_default();
        let receiver=str_value(d,"receiving_conductor_id").unwrap_or_default();
        let authored_key=(author_conductor.to_string(),hash.to_string());
        let Some(authored_event)=authored.get(&authored_key) else { finding(&mut findings,"VALIDATION_WITHOUT_AUTHORED_ACTION","incomplete",Some(&e.event_id),"validation has no authored action with this exact author-conductor and ActionHash identity"); continue; };
        if authored_event.conductor_id!=author_conductor||authored_event.conductor_id==e.conductor_id||receiver!=e.conductor_id { finding(&mut findings,"VALIDATION_CONDUCTOR_BINDING_MISMATCH","violation",Some(&e.event_id),"author/receiving conductor IDs do not match event provenance"); }
        let author_cell=conductors.get(author_conductor);
        let validator_cell=conductors.get(&e.conductor_id);
        if let (Some(author_cell),Some(validator_cell))=(author_cell,validator_cell) {
            if author_cell.dna_hash!=validator_cell.dna_hash {
                finding(&mut findings,"VALIDATION_CELL_LINEAGE_MISMATCH","violation",Some(&e.event_id),"receiving validator must belong to the same DNA network; its agent-specific CellId is expected to differ");
            }
        }
        if conductors.get(&e.conductor_id).is_none_or(|c|c.role!="validator") { finding(&mut findings,"VALIDATION_NON_VALIDATOR_CONDUCTOR","incomplete",Some(&e.event_id),"validation disposition must come from validator role"); }
        if conductors.get(&e.conductor_id).is_some_and(|c|str_value(d,"validator_wasm_sha256")!=Some(c.integrity_wasm_sha256.as_str())) {
            finding(&mut findings,"VALIDATOR_WASM_DIGEST_MISMATCH","violation",Some(&e.event_id),"validator receipt WASM digest must equal the receiving conductor's declared integrity-WASM digest");
        }
        if e.source!="validator" { finding(&mut findings,"VALIDATION_WRONG_EVIDENCE_SOURCE","incomplete",Some(&e.event_id),"validation must be reported by validator plane"); }
        if authored_event.source != "source_chain" { finding(&mut findings,"ACTION_AUTHOR_WRONG_EVIDENCE_SOURCE","incomplete",Some(&authored_event.event_id),"action authoring must come from source-chain evidence"); }
        if conductors.get(&authored_event.conductor_id).is_none_or(|c| c.role != "caller" && c.role != "peer") { finding(&mut findings,"ACTION_AUTHOR_CONDUCTOR_ROLE_MISMATCH","incomplete",Some(&authored_event.event_id),"action should originate from caller or peer conductor"); }
        if !direct_relation(&history.causal_edges,&authored_event.event_id,&e.event_id,"validator_receipt") { finding(&mut findings,"VALIDATOR_RECEIPT_CAUSAL_EDGE_MISSING","incomplete",Some(&e.event_id),"authored action and validation require explicit validator_receipt edge"); }
    }

    for e in &projection_events {
        let d=&e.data;
        let decision=str_value(d,"decision").unwrap_or_default();
        let state=str_value(d,"projection_state").unwrap_or_default();
        if e.source!="observer"||conductors.get(&e.conductor_id).is_none_or(|c|c.role!="observer") { finding(&mut findings,"PROJECTION_WRONG_OBSERVER","incomplete",Some(&e.event_id),"projection must be reported by observer role"); }
        if conductors.get(&e.conductor_id).is_some_and(|c| c.dna_hash != str_value(d,"observed_dna_hash").unwrap_or_default()) { finding(&mut findings,"OBSERVER_DNA_MISMATCH","violation",Some(&e.event_id),"observer DNA differs from observed projection DNA"); }
        let required_hash=str_value(d,"required_action_hash").unwrap_or_default();
        let action_hash=str_value(d,"action_hash").unwrap_or_default();
        let author_conductor_id=str_value(d,"author_conductor_id").unwrap_or_default();
        let author_cell=conductors.get(author_conductor_id);
        let observer_cell=conductors.get(&e.conductor_id);
        let exact=action_hash==required_hash
            &&str_value(d,"required_dna_hash")==str_value(d,"observed_dna_hash")
            &&str_value(d,"required_author")==str_value(d,"observed_author")
            &&str_value(d,"required_action_hash")==str_value(d,"observed_action_hash");
        let fresh=u64_value(d,"observed_epoch").unwrap_or(0)>=u64_value(d,"required_epoch").unwrap_or(u64::MAX);
        let descends=bool_value(d,"descends_from_required")==Some(true);

        if !exact {
            finding(&mut findings,"PROJECTION_ACTION_IDENTITY_MISMATCH","violation",Some(&e.event_id),"projection action hash, checkpoint hash, DNA, cell, and observed identity must match exactly");
        }
        let Some(source_event)=authored.get(&(author_conductor_id.to_string(),required_hash.to_string())) else {
            finding(&mut findings,"PROJECTION_SOURCE_ACTION_MISSING","incomplete",Some(&e.event_id),"projection checkpoint does not resolve to an authored action in the exact authoring conductor");
            continue;
        };
        if let Some(author_cell)=author_cell {
            if author_cell.dna_hash!=str_value(d,"required_dna_hash").unwrap_or_default()
                ||author_cell.cell_id!=str_value(d,"required_cell_id").unwrap_or_default()
                ||author_cell.agent_key_ref!=str_value(d,"required_author").unwrap_or_default()
                ||str_value(&source_event.data,"author")!=Some(author_cell.agent_key_ref.as_str()) {
                finding(&mut findings,"PROJECTION_SOURCE_LINEAGE_MISMATCH","violation",Some(&e.event_id),"checkpoint DNA/author does not match the exact authored source action; author CellId is checked independently");
            }
        } else {
            finding(&mut findings,"PROJECTION_AUTHOR_CONDUCTOR_UNKNOWN","incomplete",Some(&e.event_id),"projection references an unknown authoring conductor");
        }
        if let Some(observer_cell)=observer_cell {
            if observer_cell.dna_hash!=str_value(d,"observed_dna_hash").unwrap_or_default()
                ||observer_cell.cell_id!=str_value(d,"observed_cell_id").unwrap_or_default() {
                finding(&mut findings,"OBSERVER_CELL_MISMATCH","violation",Some(&e.event_id),"observer must report its own agent-specific CellId and the exact DNA network it hosts");
            }
        }
        if decision=="trusted"&&(state!="integrated"||!exact||!fresh||!descends) { finding(&mut findings,"FALSE_FRESH_PROJECTION","violation",Some(&e.event_id),"trusted projection requires integrated state, exact identity, adequate epoch and ancestry"); }
        if state=="conflicting"&&!matches!(decision,"conflicting"|"unknown") { finding(&mut findings,"CONFLICTING_PROJECTION_MISCLASSIFIED","violation",Some(&e.event_id),"conflicting projection must remain conflicting or unknown"); }

        let validator_ref=str_value(d,"validator_event_id");
        match validator_ref {
            Some(validator_id) => {
                let Some(validator_event)=validation_events.iter().find(|v|v.event_id==validator_id) else {
                    finding(&mut findings,"PROJECTION_VALIDATOR_RECEIPT_MISSING","incomplete",Some(&e.event_id),"validator_event_id does not reference a validation disposition in this history");
                    continue;
                };
                let validator_author=str_value(&validator_event.data,"author_conductor_id").unwrap_or_default();
                let validator_hash=str_value(&validator_event.data,"action_hash").unwrap_or_default();
                if validator_author!=author_conductor_id||validator_hash!=required_hash {
                    finding(&mut findings,"PROJECTION_VALIDATOR_RECEIPT_IDENTITY_MISMATCH","violation",Some(&e.event_id),"referenced validator receipt must concern the exact author conductor and ActionHash");
                }
                if !path_exists(&adjacency,&validator_event.event_id,&e.event_id) {
                    finding(&mut findings,"VALIDATION_NOT_CAUSAL_BEFORE_PROJECTION","incomplete",Some(&e.event_id),"projection must causally follow its exact referenced validator disposition");
                }
                let disposition=str_value(&validator_event.data,"disposition").unwrap_or_default();
                if disposition=="rejected"&&decision=="trusted" {
                    finding(&mut findings,"REJECTED_ACTION_PROJECTED_AS_TRUSTED","violation",Some(&e.event_id),"a rejected action cannot be trusted as integrated");
                }
                if decision=="trusted"&&(disposition!="accepted"||state!="integrated") {
                    finding(&mut findings,"TRUSTED_PROJECTION_WITHOUT_ACCEPTED_VALIDATION","violation",Some(&e.event_id),"trusted integrated projection requires an accepted validator disposition");
                }
            }
            None if decision=="trusted" => {
                finding(&mut findings,"TRUSTED_PROJECTION_WITHOUT_VALIDATOR_REFERENCE","incomplete",Some(&e.event_id),"trusted projection must name the exact validator disposition that supports it");
            }
            None => {}
        }

        // If any validation disposition is supplied for this exact action, it must
        // have the required author-conductor identity; foreign-cell receipts cannot
        // be borrowed merely because an ActionHash string matches.
        let matching:Vec<_>=validation_events.iter().filter(|v|
            str_value(&v.data,"action_hash")==Some(required_hash)
                &&str_value(&v.data,"author_conductor_id")==Some(author_conductor_id)
        ).collect();
        if matching.is_empty()&&decision=="trusted" {
            finding(&mut findings,"TRUSTED_PROJECTION_WITHOUT_VALIDATION","incomplete",Some(&e.event_id),"trusted projection lacks a matching exact-lineage validator disposition");
        }
        if matching.iter().all(|v|v.event_id!=validator_ref.unwrap_or_default())&&validator_ref.is_some() {
            finding(&mut findings,"PROJECTION_VALIDATOR_NOT_IN_EXACT_LINEAGE","violation",Some(&e.event_id),"the named validator receipt is not one of the matching exact-lineage validation events");
        }
        if source_event.conductor_id!=author_conductor_id {
            finding(&mut findings,"PROJECTION_AUTHOR_CONDUCTOR_MISMATCH","violation",Some(&e.event_id),"source action conductor differs from the checkpoint author conductor");
        }
    }

    for e in &outcome_events {
        let d=&e.data;
        let op=str_value(d,"operation_id").unwrap_or_default().to_string();
        let attempt=u64_value(d,"attempt_id").unwrap_or(0);
        let outcome=str_value(d,"outcome").unwrap_or_default();
        let effect=effect_bindings.contains(&(op.clone(),attempt));
        match calls.get(&(op.clone(),attempt)) {
            None=>finding(&mut findings,"OUTCOME_WITHOUT_HOST_CALL","incomplete",Some(&e.event_id),"caller outcome has no host-call attempt"),
            Some(call)=>{
                if !path_exists(&adjacency,&call.event_id,&e.event_id) { finding(&mut findings,"OUTCOME_NOT_CAUSAL_AFTER_CALL","incomplete",Some(&e.event_id),"caller outcome must causally follow the host call"); }
                let caller = call_events.iter().find(|c| str_value(&c.data,"operation_id") == Some(op.as_str()) && u64_value(&c.data,"attempt_id") == Some(attempt)).and_then(|c| str_value(&c.data,"caller"));
                if conductors.get(&e.conductor_id).is_none_or(|c| Some(c.agent_key_ref.as_str()) != caller) { finding(&mut findings,"OUTCOME_WRONG_CALLER_CONDUCTOR","incomplete",Some(&e.event_id),"caller outcome should be observed on caller conductor"); }
                if outcome=="committed"&&!effect { finding(&mut findings,"COMMITTED_WITHOUT_EFFECT","incomplete",Some(&e.event_id),"committed outcome has no durable effect"); }
                if matches!(outcome,"not_committed"|"rejected")&&effect { finding(&mut findings,"REJECTED_OUTCOME_WITH_EFFECT","violation",Some(&e.event_id),"rejected outcome contradicts durable effect"); }
            }
        }
    }

    let violation=findings.iter().any(|f|f.severity=="violation");
    let incomplete=findings.iter().any(|f|f.severity=="incomplete");
    let status=if violation{Status::Fail}else if incomplete{Status::Incomplete}else{Status::Pass};
    Report{
        schema_version:2,status,trace_id:history.trace_id.clone(),evidence_class:history.evidence_class.clone(),
        input_sha256,counts,findings,
        claim_ceiling:"STRUCTURAL_PARTIAL_ORDER_ONLY: this verifier checks a redacted multi-conductor event DAG and conductor-local order; it does not authenticate receipts, prove conductor honesty, establish DHT consensus, or prove unbounded linearizability".to_string()
    }
}

fn benchmark_history(event_count: usize) -> Result<History, Box<dyn Error>> {
    if event_count == 0 || event_count > 1_000_000 {
        return Err("--benchmark-events must be between 1 and 1000000".into());
    }

    let conductors = vec![
        Conductor {
            conductor_id: "peer-a".into(),
            agent_key_ref: "agent-peer-a".into(),
            conductor_build: "synthetic-benchmark".into(),
            dna_hash: "uhCkk-benchmark-dna".into(),
            cell_id: "benchmark-cell-a".into(),
            integrity_wasm_sha256: "0".repeat(64),
            role: "peer".into(),
        },
        Conductor {
            conductor_id: "peer-b".into(),
            agent_key_ref: "agent-peer-b".into(),
            conductor_build: "synthetic-benchmark".into(),
            dna_hash: "uhCkk-benchmark-dna".into(),
            cell_id: "benchmark-cell-b".into(),
            integrity_wasm_sha256: "0".repeat(64),
            role: "peer".into(),
        },
    ];

    let events = (0..event_count)
        .map(|index| {
            let conductor = index % 2;
            let local_sequence = (index / 2 + 1) as u64;
            let conductor_id = if conductor == 0 { "peer-a" } else { "peer-b" };
            let agent = if conductor == 0 { "agent-peer-a" } else { "agent-peer-b" };
            Event {
                event_id: format!("signal-{conductor_id}-{local_sequence:010}"),
                conductor_id: conductor_id.into(),
                local_sequence,
                source: "remote_peer".into(),
                evidence_ref: format!("synthetic://benchmark/{conductor_id}/{local_sequence}"),
                event_type: "remote_signal".into(),
                data: serde_json::json!({
                    "event_type": "benchmark.wakeup.v1",
                    "trust_class": "unverified_hint",
                    "transport_sender": agent,
                    "claimed_sender": agent,
                    "backing_action_hash": null,
                    "record_author": null,
                    "verified_payload_sha256": null,
                    "verifier_build": null,
                    "backing_author_conductor_id": null,
                    "backing_validator_event_id": null,
                    "trusted_state_mutation": false,
                    "authorizes_operation": false
                }),
            }
        })
        .collect();

    Ok(History {
        schema_version: 2,
        trace_id: format!("synthetic-benchmark-{event_count}"),
        evidence_class: "synthetic".into(),
        source_commit: "0".repeat(40),
        conductors,
        events,
        causal_edges: Vec::new(),
    })
}

fn write_json_output(value: &Value, output: Option<PathBuf>) -> Result<(), Box<dyn Error>> {
    let json = serde_json::to_string_pretty(value)?;
    if let Some(path) = output {
        if let Some(parent) = path.parent() { fs::create_dir_all(parent)?; }
        fs::write(path, json)?;
    } else {
        println!("{json}");
    }
    Ok(())
}

fn run_benchmark(event_count: usize, output: Option<PathBuf>) -> Result<(), Box<dyn Error>> {
    use std::time::Instant;

    let build_baseline = allocation_snapshot();
    reset_peak_to_current_live_bytes();
    let build_started = Instant::now();
    let history = benchmark_history(event_count)?;
    let build_ns = build_started.elapsed().as_nanos();
    let after_build = allocation_snapshot();

    let spec = format!(
        "pulse-multiconductor-audit-benchmark-v1|events={event_count}|conductors=2|event=unverified_remote_signal"
    );
    let generator_commitment_sha256 = format!("{:x}", Sha256::digest(spec.as_bytes()));

    let audit_live_baseline = allocation_snapshot().live_bytes;
    reset_peak_to_current_live_bytes();
    let audit_started = Instant::now();
    let report = audit(&history, generator_commitment_sha256.clone());
    let audit_ns = audit_started.elapsed().as_nanos();
    let after_audit = allocation_snapshot();
    let throughput = if audit_ns == 0 {
        0.0
    } else {
        event_count as f64 / (audit_ns as f64 / 1_000_000_000.0)
    };

    let status = match &report.status {
        Status::Pass => "PASS",
        Status::Fail => "FAIL",
        Status::Incomplete => "INCOMPLETE",
    };
    let output_value = serde_json::json!({
        "benchmark_version": 1,
        "benchmark_name": "pulse-multiconductor-audit",
        "evidence_class": "synthetic",
        "status": status,
        "events": event_count,
        "conductors": history.conductors.len(),
        "causal_edges": history.causal_edges.len(),
        "findings": report.findings.len(),
        "history_build_ns": build_ns,
        "audit_ns": audit_ns,
        "audit_events_per_second": throughput,
        "allocation_metrics_version": 1,
        "allocation_baseline_live_bytes": build_baseline.live_bytes,
        "allocation_live_bytes_after_build": after_build.live_bytes,
        "allocation_peak_live_bytes_during_build": after_build.peak_live_bytes,
        "allocation_peak_added_bytes_during_build": after_build.peak_live_bytes.saturating_sub(build_baseline.live_bytes),
        "allocation_bytes_requested_during_build": after_build.total_allocated_bytes.saturating_sub(build_baseline.total_allocated_bytes),
        "allocation_live_bytes_before_audit": audit_live_baseline,
        "allocation_live_bytes_after_audit": after_audit.live_bytes,
        "allocation_peak_live_bytes_during_audit": after_audit.peak_live_bytes,
        "allocation_peak_added_bytes_during_audit": after_audit.peak_live_bytes.saturating_sub(audit_live_baseline),
        "allocation_bytes_requested_during_audit": after_audit.total_allocated_bytes.saturating_sub(after_build.total_allocated_bytes),
        "generator_commitment_sha256": generator_commitment_sha256,
        "claim_ceiling": "BENCHMARK_ONLY: synthetic remote-signal histories measure this verifier build and fixture generator; this is not Holochain runtime throughput or a production workload claim"
    });
    write_json_output(&output_value, output)?;

    if matches!(&report.status, Status::Pass) {
        Ok(())
    } else {
        Err("synthetic benchmark fixture did not pass structural audit".into())
    }
}

fn main()->Result<(),Box<dyn Error>>{
    let mut input=None;
    let mut output=None;
    let mut benchmark_events=None;
    let mut args=env::args().skip(1);
    while let Some(arg)=args.next(){
        match arg.as_str(){
            "--input"=>input=Some(PathBuf::from(args.next().ok_or("--input requires a path")?)),
            "--output"=>output=Some(PathBuf::from(args.next().ok_or("--output requires a path")?)),
            "--benchmark-events"=>{
                let value=args.next().ok_or("--benchmark-events requires a count")?;
                benchmark_events=Some(value.parse::<usize>()?);
            }
            "--help"|"-h"=>{
                println!("pulse-multiconductor-audit --input HISTORY.json [--output REPORT.json]");
                println!("pulse-multiconductor-audit --benchmark-events COUNT [--output BENCHMARK.json] (1..=1000000)");
                return Ok(());
            }
            other=>return Err(format!("unknown argument: {other}").into())
        }
    }

    if let Some(count)=benchmark_events {
        if input.is_some() { return Err("--input and --benchmark-events are mutually exclusive".into()); }
        return run_benchmark(count, output);
    }

    let input=input.ok_or("--input is required unless --benchmark-events is used")?;
    let raw=fs::read(&input)?;
    let digest=format!("{:x}",Sha256::digest(&raw));
    let history:History=serde_json::from_slice(&raw)?;
    let report=audit(&history,digest);
    let json=serde_json::to_string_pretty(&report)?;
    if let Some(path)=output{if let Some(parent)=path.parent(){fs::create_dir_all(parent)?;}fs::write(path,&json)?;}else{println!("{json}");}
    match report.status{Status::Pass=>Ok(()),Status::Fail=>Err("multiconductor history violation".into()),Status::Incomplete=>Err("multiconductor evidence incomplete".into())}
}

#[cfg(test)]
mod tests{
    use super::*;
    fn fixture()->History{serde_json::from_str(include_str!("../examples/safe-history.json")).unwrap()}
    fn report(h:&History)->Report{audit(h,"0".repeat(64))}
    fn verified_record_fixture() -> History {
        let mut h = fixture();
        h.events.push(Event {
            event_id: "verified-attestation-authored".into(),
            conductor_id: "caller-host".into(),
            local_sequence: 6,
            source: "source_chain".into(),
            evidence_ref: "fixture://caller/verified-attestation".into(),
            event_type: "action_authored".into(),
            data: serde_json::json!({
                "action_hash": "uhCkk-verified-attestation",
                "author": "agent-caller",
                "entry_type": "TrustAttestation",
                "payload_sha256": "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee"
            }),
        });
        h.events.push(Event {
            event_id: "verified-attestation-accepted".into(),
            conductor_id: "validator-host".into(),
            local_sequence: 2,
            source: "validator".into(),
            evidence_ref: "fixture://validator/verified-attestation".into(),
            event_type: "validation_disposition".into(),
            data: serde_json::json!({
                "action_hash": "uhCkk-verified-attestation",
                "author_conductor_id": "caller-host",
                "receiving_conductor_id": "validator-host",
                "disposition": "accepted",
                "validator_wasm_sha256": "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
                "reason_code": null
            }),
        });
        h.events.push(Event {
            event_id: "verified-record-signal".into(),
            conductor_id: "observer-host".into(),
            local_sequence: 2,
            source: "observer".into(),
            evidence_ref: "fixture://observer/verified-attestation".into(),
            event_type: "remote_signal".into(),
            data: serde_json::json!({
                "event_type": "TrustAttestationReceived",
                "trust_class": "verified_record",
                "transport_sender": "agent-caller",
                "claimed_sender": "agent-caller",
                "backing_action_hash": "uhCkk-verified-attestation",
                "record_author": "agent-caller",
                "verified_payload_sha256": "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee",
                "verifier_build": "pulse-verifier-1",
                "backing_author_conductor_id": "caller-host",
                "backing_validator_event_id": "verified-attestation-accepted",
                "trusted_state_mutation": true,
                "authorizes_operation": false
            }),
        });
        h.causal_edges.push(CausalEdge {
            from_event: "verified-attestation-authored".into(),
            to_event: "verified-attestation-accepted".into(),
            relation: "validator_receipt".into(),
            evidence_ref: "fixture://causal/verified-author-to-validator".into(),
        });
        h.causal_edges.push(CausalEdge {
            from_event: "verified-attestation-accepted".into(),
            to_event: "verified-record-signal".into(),
            relation: "observer_confirmation".into(),
            evidence_ref: "fixture://causal/verified-validator-to-observer".into(),
        });
        h
    }
    #[test]fn safe_multiconductor_partial_order_passes(){let r=report(&fixture());assert!(matches!(r.status,Status::Pass),"{:?}",r.findings);}
    #[test]
    fn same_dna_peers_have_distinct_cell_ids_and_valid_cross_peer_receipts() {
        let h = fixture();
        let cells: BTreeSet<_> = h.conductors.iter().map(|c| c.cell_id.as_str()).collect();
        assert_eq!(cells.len(), h.conductors.len(), "each agent's CellId must remain distinct");
        let dnas: BTreeSet<_> = h.conductors.iter().map(|c| c.dna_hash.as_str()).collect();
        assert_eq!(dnas.len(), 1, "the peer conductors share one DNA network");
        let r = report(&h);
        assert!(matches!(r.status, Status::Pass), "{:?}", r.findings);
        assert!(!r.findings.iter().any(|f| f.code == "VALIDATION_CELL_LINEAGE_MISMATCH"));
    }
    #[test]
    fn synthetic_benchmark_generator_produces_safe_histories() {
        for count in [1usize, 2, 999, 1000] {
            let history = benchmark_history(count).unwrap();
            assert_eq!(history.events.len(), count);
            let r = report(&history);
            assert!(matches!(r.status, Status::Pass), "{:?}", r.findings);
        }
    }
    #[test]
    fn synthetic_benchmark_rejects_zero_and_oversized_inputs() {
        assert!(benchmark_history(0).is_err());
        assert!(benchmark_history(1_000_001).is_err());
    }
    #[test]fn same_idempotency_key_with_different_request_digest_fails(){
        let mut h=fixture();
        let mut duplicate=h.events.iter().find(|e|e.event_type=="host_grant_issued").unwrap().clone();
        duplicate.event_id="grant-retry-changed-request".into();
        duplicate.local_sequence=6;
        duplicate.data["grant_ref"]=Value::String("grant-B".into());
        duplicate.data["grant_action_hash"]=Value::String("uhCkk-grant-B".into());
        duplicate.data["host_authority_index"]=Value::Number(5u64.into());
        duplicate.data["request_digest_sha256"]=Value::String("e".repeat(64));
        h.events.push(duplicate);
        let r=report(&h);
        assert!(r.findings.iter().any(|f|f.code=="IDEMPOTENCY_KEY_REUSED_WITH_DIFFERENT_REQUEST"),"{:?}",r.findings);
    }
    #[test]fn malformed_request_digest_is_incomplete(){
        let mut h=fixture();
        let issue=h.events.iter_mut().find(|e|e.event_type=="host_grant_issued").unwrap();
        issue.data["request_digest_sha256"]=Value::String("not-a-sha256".into());
        let r=report(&h);
        assert!(r.findings.iter().any(|f|f.code=="EVENT_DATA_SHAPE_INVALID"),"{:?}",r.findings);
    }
    #[test]fn local_sequences_are_not_compared_across_conductors(){let h=fixture();let r=report(&h);assert!(matches!(r.status,Status::Pass),"{:?}",r.findings);assert_eq!(h.events.iter().filter(|e|e.local_sequence==1).count(),4);}
    #[test]fn cross_conductor_program_order_is_rejected(){let mut h=fixture();h.causal_edges.push(CausalEdge{from_event:"grant-issue".into(),to_event:"claim-accepted".into(),relation:"program_order".into(),evidence_ref:"fixture://invalid/scalar-order".into()});let r=report(&h);assert!(r.findings.iter().any(|f|f.code=="PROGRAM_ORDER_CROSSES_CONDUCTORS"));}
    #[test]fn missing_claim_edge_is_incomplete(){let mut h=fixture();h.causal_edges.retain(|e|!(e.from_event=="grant-issue"&&e.to_event=="claim-accepted"));let r=report(&h);assert!(r.findings.iter().any(|f|f.code=="GRANT_TO_CLAIM_CAUSAL_EDGE_MISSING"));}
    #[test]fn causal_cycle_is_rejected(){let mut h=fixture();h.causal_edges.push(CausalEdge{from_event:"projection-after-reject".into(),to_event:"mailbox-action-authored".into(),relation:"test_harness_barrier".into(),evidence_ref:"fixture://invalid/cycle".into()});let r=report(&h);assert!(r.findings.iter().any(|f|f.code=="CAUSAL_GRAPH_CYCLE"));}
    #[test]fn post_revoke_allow_is_rejected(){let mut h=fixture();let e=h.events.iter_mut().find(|e|e.event_id=="host-call-after-revoke").unwrap();e.data["host_decision"]=Value::String("authorized".into());e.data["domain_decision"]=Value::String("allow".into());let r=report(&h);assert!(r.findings.iter().any(|f|f.code=="HOST_ALLOW_OUTSIDE_GRANT"));}
    #[test]fn rejected_action_cannot_be_trusted(){let mut h=fixture();let e=h.events.iter_mut().find(|e|e.event_id=="projection-after-reject").unwrap();e.data["projection_state"]=Value::String("integrated".into());e.data["decision"]=Value::String("trusted".into());let r=report(&h);assert!(r.findings.iter().any(|f|f.code=="REJECTED_ACTION_PROJECTED_AS_TRUSTED"));}
    #[test]
    fn verified_record_signal_resolves_exact_action_digest_and_validator_receipt() {
        let h = verified_record_fixture();
        let r = report(&h);
        assert!(matches!(r.status, Status::Pass), "{:?}", r.findings);
    }

    #[test]
    fn verified_record_cannot_point_to_an_unknown_action_hash() {
        let mut h = verified_record_fixture();
        let signal = h.events.iter_mut().find(|e| e.event_id == "verified-record-signal").unwrap();
        signal.data["backing_action_hash"] = Value::String("uhCkk-unknown-action".into());
        let r = report(&h);
        assert!(r.findings.iter().any(|f| f.code == "VERIFIED_SIGNAL_ACTION_NOT_AUTHORED"), "{:?}", r.findings);
    }

    #[test]
    fn verified_record_payload_digest_must_match_authored_payload() {
        let mut h = verified_record_fixture();
        let signal = h.events.iter_mut().find(|e| e.event_id == "verified-record-signal").unwrap();
        signal.data["verified_payload_sha256"] = Value::String("ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff".into());
        let r = report(&h);
        assert!(r.findings.iter().any(|f| f.code == "VERIFIED_SIGNAL_PAYLOAD_OR_AUTHOR_MISMATCH"), "{:?}", r.findings);
    }

    #[test]
    fn verified_record_requires_both_authorship_and_validator_causal_edges() {
        let mut h = verified_record_fixture();
        h.causal_edges.retain(|e| e.from_event != "verified-attestation-accepted");
        let r = report(&h);
        assert!(r.findings.iter().any(|f| f.code == "VERIFIED_SIGNAL_VALIDATOR_OBSERVER_EDGE_MISSING"), "{:?}", r.findings);
    }

    #[test]
    fn authenticated_peer_hint_cannot_claim_a_different_transport_sender() {
        let mut h = fixture();
        let signal = h.events.iter_mut().find(|e| e.event_type == "remote_signal").unwrap();
        signal.data["trust_class"] = Value::String("authenticated_peer_hint".into());
        signal.data["claimed_sender"] = Value::String("agent-owner".into());
        let r = report(&h);
        assert!(r.findings.iter().any(|f| f.code == "AUTHENTICATED_HINT_SENDER_MISMATCH"), "{:?}", r.findings);
    }

    #[test]
    fn local_only_signal_cannot_claim_remote_transport_identity() {
        let mut h = fixture();
        let signal = h.events.iter_mut().find(|e| e.event_type == "remote_signal").unwrap();
        signal.data["trust_class"] = Value::String("local_only".into());
        let r = report(&h);
        assert!(r.findings.iter().any(|f| f.code == "LOCAL_ONLY_HAS_REMOTE_SENDER"), "{:?}", r.findings);
    }

    #[test]
    fn validator_wasm_digest_must_match_declared_conductor_artifact() {
        let mut h = fixture();
        let event = h.events.iter_mut().find(|e| e.event_type == "validation_disposition").unwrap();
        event.data["validator_wasm_sha256"] = Value::String("ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff".into());
        let r = report(&h);
        assert!(r.findings.iter().any(|f| f.code == "VALIDATOR_WASM_DIGEST_MISMATCH"), "{:?}", r.findings);
    }

    #[test]fn forged_remote_signal_cannot_mutate_trusted_state(){let mut h=fixture();let e=h.events.iter_mut().find(|e|e.event_type=="remote_signal").unwrap();e.data["trusted_state_mutation"]=Value::Bool(true);let r=report(&h);assert!(r.findings.iter().any(|f|f.code=="UNVERIFIED_SIGNAL_MUTATED_TRUSTED_STATE"));}
    #[test]fn event_data_rejects_unknown_fields(){let mut h=fixture();let e=h.events.iter_mut().find(|e|e.event_type=="host_call").unwrap();e.data["debug_secret"]=Value::String("never-allowed".into());let r=report(&h);assert!(r.findings.iter().any(|f|f.code=="EVENT_DATA_SHAPE_INVALID"));}

    #[test]
    fn same_action_hash_in_different_dna_cells_is_not_a_collision() {
        let mut h = fixture();
        h.conductors.push(Conductor {
            conductor_id: "foreign-caller".into(),
            agent_key_ref: "agent-foreign".into(),
            conductor_build: "holochain-test".into(),
            dna_hash: "uhCkk-foreign-dna".into(),
            cell_id: "foreign-cell".into(),
            integrity_wasm_sha256: "f".repeat(64),
            role: "caller".into(),
        });
        let mut foreign_action = h.events.iter().find(|e| e.event_type == "action_authored").unwrap().clone();
        foreign_action.event_id = "same-hash-other-cell".into();
        foreign_action.conductor_id = "foreign-caller".into();
        foreign_action.local_sequence = 1;
        foreign_action.data["author"] = Value::String("agent-foreign".into());
        h.events.push(foreign_action);

        let r = report(&h);
        assert!(!r.findings.iter().any(|f| f.code == "ACTION_HASH_AUTHORED_TWICE"), "{:?}", r.findings);
    }

    #[test]
    fn projection_action_hash_must_equal_checkpoint_action_hash() {
        let mut h = fixture();
        let projection = h.events.iter_mut().find(|e| e.event_type == "projection_observed").unwrap();
        projection.data["action_hash"] = Value::String("uhCkk-some-other-action".into());
        let r = report(&h);
        assert!(r.findings.iter().any(|f| f.code == "PROJECTION_ACTION_IDENTITY_MISMATCH"), "{:?}", r.findings);
    }

    #[test]
    fn projection_must_reference_an_exact_validator_event() {
        let mut h = fixture();
        let projection = h.events.iter_mut().find(|e| e.event_type == "projection_observed").unwrap();
        projection.data["validator_event_id"] = Value::String("missing-validator-receipt".into());
        projection.data["decision"] = Value::String("trusted".into());
        projection.data["projection_state"] = Value::String("integrated".into());
        projection.data["observed_epoch"] = Value::Number(8u64.into());
        projection.data["descends_from_required"] = Value::Bool(true);
        let r = report(&h);
        assert!(r.findings.iter().any(|f| f.code == "PROJECTION_VALIDATOR_RECEIPT_MISSING"), "{:?}", r.findings);
    }

    #[test]
    fn validator_from_foreign_dna_cannot_back_a_projection() {
        let mut h = fixture();
        h.conductors.push(Conductor {
            conductor_id: "foreign-validator".into(),
            agent_key_ref: "agent-foreign-validator".into(),
            conductor_build: "holochain-test".into(),
            dna_hash: "uhCkk-foreign-dna".into(),
            cell_id: "foreign-cell".into(),
            integrity_wasm_sha256: "e".repeat(64),
            role: "validator".into(),
        });
        h.events.push(Event {
            event_id: "foreign-validator-accepted".into(),
            conductor_id: "foreign-validator".into(),
            local_sequence: 1,
            source: "validator".into(),
            evidence_ref: "fixture://foreign-validator/receipt".into(),
            event_type: "validation_disposition".into(),
            data: serde_json::json!({
                "action_hash": "uhCkk-mailbox-forged",
                "author_conductor_id": "caller-host",
                "receiving_conductor_id": "foreign-validator",
                "disposition": "accepted",
                "validator_wasm_sha256": "e".repeat(64),
                "reason_code": null
            }),
        });
        h.causal_edges.push(CausalEdge {
            from_event: "mailbox-action-authored".into(),
            to_event: "foreign-validator-accepted".into(),
            relation: "validator_receipt".into(),
            evidence_ref: "fixture://foreign-validator/receipt-edge".into(),
        });
        h.causal_edges.push(CausalEdge {
            from_event: "foreign-validator-accepted".into(),
            to_event: "projection-after-reject".into(),
            relation: "observer_confirmation".into(),
            evidence_ref: "fixture://foreign-validator/projection-edge".into(),
        });
        let projection = h.events.iter_mut().find(|e| e.event_type == "projection_observed").unwrap();
        projection.data["validator_event_id"] = Value::String("foreign-validator-accepted".into());
        projection.data["decision"] = Value::String("trusted".into());
        projection.data["projection_state"] = Value::String("integrated".into());
        projection.data["observed_epoch"] = Value::Number(8u64.into());
        projection.data["descends_from_required"] = Value::Bool(true);

        let r = report(&h);
        assert!(r.findings.iter().any(|f| f.code == "VALIDATION_CELL_LINEAGE_MISMATCH"), "{:?}", r.findings);
    }
}
