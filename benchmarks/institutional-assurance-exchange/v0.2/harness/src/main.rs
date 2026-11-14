use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{collections::{HashMap, HashSet}, env, error::Error, fs, io};

fn invalid(message: impl Into<String>) -> Box<dyn Error> {
    Box::new(io::Error::new(io::ErrorKind::InvalidData, message.into()))
}
fn read_json(path: &str) -> Result<Value, Box<dyn Error>> {
    Ok(serde_json::from_slice(&fs::read(path)?)?)
}
fn sha256_file(path: &str) -> Result<String, Box<dyn Error>> {
    Ok(format!("{:x}", Sha256::digest(fs::read(path)?)))
}
fn sha256_current_executable() -> Result<String, Box<dyn Error>> {
    Ok(format!("{:x}", Sha256::digest(fs::read(env::current_exe()?)?)))
}
fn required_str<'a>(v: &'a Value, key: &str, context: &str) -> Result<&'a str, Box<dyn Error>> {
    v.get(key).and_then(Value::as_str).filter(|s| !s.is_empty())
        .ok_or_else(|| invalid(format!("{context}: missing or empty string '{key}'")))
}
fn required_array<'a>(v: &'a Value, key: &str, context: &str) -> Result<&'a Vec<Value>, Box<dyn Error>> {
    v.get(key).and_then(Value::as_array)
        .ok_or_else(|| invalid(format!("{context}: missing array '{key}'")))
}
fn string_set(v: &Value, key: &str, context: &str) -> Result<HashSet<String>, Box<dyn Error>> {
    let values = required_array(v, key, context)?;
    let mut result = HashSet::new();
    for value in values {
        let s = value.as_str().filter(|s| !s.is_empty())
            .ok_or_else(|| invalid(format!("{context}: '{key}' must contain nonempty strings")))?;
        if !result.insert(s.to_owned()) {
            return Err(invalid(format!("{context}: duplicate value in '{key}'")));
        }
    }
    Ok(result)
}
fn index<'a>(items: &'a [Value], key: &str, label: &str) -> Result<HashMap<String, &'a Value>, Box<dyn Error>> {
    let mut result = HashMap::new();
    for item in items {
        let id = required_str(item, key, label)?.to_owned();
        if result.insert(id.clone(), item).is_some() {
            return Err(invalid(format!("{label}: duplicate identifier '{id}'")));
        }
    }
    Ok(result)
}
fn is_supported_claim(claim: &Value, required_dimension: &str) -> bool {
    if claim.get("dimension").and_then(Value::as_str) != Some(required_dimension) { return false; }
    if claim.get("disposition").and_then(Value::as_str) != Some("supported_within_scope") { return false; }
    claim.get("competent_for").and_then(Value::as_array)
        .map(|values| values.iter().any(|v| v.as_str() == Some(required_dimension)))
        .unwrap_or(false)
}
fn is_hard_modality(modality: &str) -> bool {
    matches!(modality, "mandatory" | "conditional")
}
fn is_non_blocking_modality(modality: &str) -> bool {
    matches!(modality, "preferred" | "unresolved")
}
fn is_allow(disposition: &str) -> bool {
    matches!(disposition, "ALLOW_SCOPED" | "ALLOW_PARTIAL")
}
fn write_is_authorized(write: &Value, boundaries: &[Value]) -> bool {
    let Ok(action) = required_str(write,"action","write trace") else { return false; };
    let Ok(purpose) = required_str(write,"purpose","write trace") else { return false; };
    let Ok(asset) = required_str(write,"asset_ref","write trace") else { return false; };
    let Ok(actor) = required_str(write,"actor_ref","write trace") else { return false; };
    let Ok(auth) = required_str(write,"authorization_ref","write trace") else { return false; };
    let Ok(fields) = string_set(write,"fields","write trace") else { return false; };
    let Some(external) = write.get("possible_external_side_effect").and_then(Value::as_bool) else { return false; };
    boundaries.iter().any(|boundary| {
        let Ok(allowed_fields) = string_set(boundary,"fields","authorized write boundary") else { return false; };
        boundary.get("action").and_then(Value::as_str)==Some(action)
            && boundary.get("purpose").and_then(Value::as_str)==Some(purpose)
            && boundary.get("asset_ref").and_then(Value::as_str)==Some(asset)
            && boundary.get("actor_ref").and_then(Value::as_str)==Some(actor)
            && boundary.get("authorization_ref").and_then(Value::as_str)==Some(auth)
            && boundary.get("possible_external_side_effect").and_then(Value::as_bool)==Some(external)
            && fields.is_subset(&allowed_fields)
    })
}
fn canonical_utc(value: &str) -> bool {
    value.ends_with('Z') && value.len() >= 20
}
fn byte_span_matches(source: &str, start: usize, end: usize, quote: &str) -> bool {
    start <= end && end <= source.len()
        && source.is_char_boundary(start)
        && source.is_char_boundary(end)
        && &source.as_bytes()[start..end] == quote.as_bytes()
}
fn eligible_observation<'a>(
    packet: &'a Value,
    query: &Value,
    dimension: &str,
    explicit_ref: Option<&str>,
) -> Option<&'a Value> {
    let excluded: HashSet<&str> = query.get("excluded_claim_refs")
        .and_then(Value::as_array)
        .map(|xs| xs.iter().filter_map(Value::as_str).collect())
        .unwrap_or_default();
    let effective_at = query.get("effective_at").and_then(Value::as_str)?;
    let known_at = query.get("known_at").and_then(Value::as_str)?;
    let observations = packet.get("claim_observations")?.as_array()?;
    if let Some(reference) = explicit_ref {
        return observations.iter().find(|observation| {
            observation.get("claim_ref").and_then(Value::as_str) == Some(reference)
                && !excluded.contains(reference)
                && observation.get("asset_ref").and_then(Value::as_str) == query.get("asset_ref").and_then(Value::as_str)
                && observation.get("effective_from").and_then(Value::as_str).map(|s| s <= effective_at).unwrap_or(false)
                && observation.get("observed_at").and_then(Value::as_str).map(|s| s <= known_at).unwrap_or(false)
        });
    }
    observations.iter()
        .filter(|observation| {
            observation.get("dimension").and_then(Value::as_str) == Some(dimension)
                && !observation.get("claim_ref").and_then(Value::as_str).map(|s| excluded.contains(s)).unwrap_or(false)
                && observation.get("asset_ref").and_then(Value::as_str) == query.get("asset_ref").and_then(Value::as_str)
                && observation.get("effective_from").and_then(Value::as_str).map(|s| s <= effective_at).unwrap_or(false)
                && observation.get("observed_at").and_then(Value::as_str).map(|s| s <= known_at).unwrap_or(false)
        })
        .max_by(|a, b| {
            let a_effective = a.get("effective_from").and_then(Value::as_str).unwrap_or("");
            let b_effective = b.get("effective_from").and_then(Value::as_str).unwrap_or("");
            let a_observed = a.get("observed_at").and_then(Value::as_str).unwrap_or("");
            let b_observed = b.get("observed_at").and_then(Value::as_str).unwrap_or("");
            a_effective.cmp(b_effective).then(a_observed.cmp(b_observed))
        })
}
fn authority_state_at(packet: &Value, query: &Value) -> String {
    let authority = &packet["authority"];
    let action_at = query.get("action_at").and_then(Value::as_str).unwrap_or("");
    let known_at = query.get("known_at").and_then(Value::as_str).unwrap_or("");
    let events = authority.get("status_events").and_then(Value::as_array);
    let Some(events) = events else { return "unknown".to_owned(); };
    events.iter()
        .filter(|event| {
            event.get("effective_at").and_then(Value::as_str).map(|s| s <= action_at).unwrap_or(false)
                && event.get("observed_at").and_then(Value::as_str).map(|s| s <= known_at).unwrap_or(false)
        })
        .max_by(|a, b| {
            let ae = a.get("effective_at").and_then(Value::as_str).unwrap_or("");
            let be = b.get("effective_at").and_then(Value::as_str).unwrap_or("");
            let ak = a.get("observed_at").and_then(Value::as_str).unwrap_or("");
            let bk = b.get("observed_at").and_then(Value::as_str).unwrap_or("");
            ae.cmp(be).then(ak.cmp(bk))
        })
        .and_then(|event| event.get("status").and_then(Value::as_str))
        .unwrap_or("unknown").to_owned()
}
#[derive(Debug)]
struct Derived {
    disposition: String,
    authority_state: String,
    permitted_use: String,
    reason_codes: Vec<String>,
    supporting_claim_refs: Vec<String>,
    source_receipt_refs: Vec<String>,
}
fn add_claim(result: &mut Derived, claim: &Value) {
    if let Some(id) = claim.get("claim_ref").and_then(Value::as_str) {
        if !result.supporting_claim_refs.iter().any(|s| s == id) {
            result.supporting_claim_refs.push(id.to_owned());
        }
    }
    if let Some(id) = claim.get("source_receipt_ref").and_then(Value::as_str) {
        if !result.source_receipt_refs.iter().any(|s| s == id) {
            result.source_receipt_refs.push(id.to_owned());
        }
    }
}
fn derive_query(packet: &Value, query: &Value) -> Result<Derived, Box<dyn Error>> {
    let action = required_str(query, "action", "query")?;
    let purpose = required_str(query, "purpose", "query")?;
    let asset = required_str(query, "asset_ref", "query")?;
    let actor = required_str(query, "actor_ref", "query")?;
    let entity = required_str(query, "contracting_entity_ref", "query")?;
    let authority = &packet["authority"];
    let status = authority_state_at(packet, query);
    let mut result = Derived {
        disposition: "ALLOW_SCOPED".to_owned(),
        authority_state: status.clone(),
        permitted_use: "not_applicable".to_owned(),
        reason_codes: Vec::new(),
        supporting_claim_refs: Vec::new(),
        source_receipt_refs: Vec::new(),
    };
    let mut blocked = false;
    if authority.get("subject_ref").and_then(Value::as_str) != Some(actor)
        || authority.get("entity_ref").and_then(Value::as_str) != Some(entity)
        || authority.get("verification_state").and_then(Value::as_str) != Some("verified")
        || authority.get("action_scope").and_then(Value::as_array).map(|xs| !xs.iter().any(|x| x.as_str()==Some(action))).unwrap_or(true)
        || authority.get("purpose_scope").and_then(Value::as_array).map(|xs| !xs.iter().any(|x| x.as_str()==Some(purpose))).unwrap_or(true)
        || authority.get("asset_scope").and_then(Value::as_array).map(|xs| !xs.iter().any(|x| x.as_str()==Some(asset))).unwrap_or(true) {
        blocked = true;
        result.reason_codes.push("authority.scope.mismatch".to_owned());
    }
    let action_at = required_str(query, "action_at", "query")?;
    let valid_from = required_str(authority, "valid_from", "authority")?;
    let valid_through = required_str(authority, "valid_through", "authority")?;
    if action_at < valid_from || action_at > valid_through {
        blocked = true;
        result.reason_codes.push("authority.outside.validity".to_owned());
    }
    if status != "active" {
        blocked = true;
        result.reason_codes.push(if status == "revoked" {
            "authority.revoked_before_action".to_owned()
        } else {
            "authority.status.not_active".to_owned()
        });
    }
    let applicable = packet.get("intent_candidates").and_then(Value::as_array)
        .ok_or_else(|| invalid("fixture intent_candidates missing"))?;
    let mut applied_count = 0usize;
    if action == "approve_commercial_lease" {
        let title_requirement = applicable.iter().find(|r|
            r.get("requirement_ref").and_then(Value::as_str)==Some("REQ-LEASE-TITLE")
                && r.get("applies_to_actions").and_then(Value::as_array).map(|xs|xs.iter().any(|x|x.as_str()==Some(action))).unwrap_or(false)
        );
        if let Some(requirement) = title_requirement {
            applied_count += 1;
            let explicit_binding = query.get("claim_bindings").and_then(Value::as_array)
                .and_then(|xs| xs.iter().find(|x|x.get("requirement_ref").and_then(Value::as_str)==Some("REQ-LEASE-TITLE")))
                .and_then(|x|x.get("claim_ref")).and_then(Value::as_str);
            let title = eligible_observation(packet, query, "title", explicit_binding);
            let title_ok = title.map(|claim| {
                is_supported_claim(claim, "title")
                    && claim.get("value").and_then(|v|v.get("registered_entity_ref")).and_then(Value::as_str)==Some(entity)
            }).unwrap_or(false);
            if let Some(claim) = title { add_claim(&mut result, claim); }
            if !title_ok {
                blocked = true;
                let wrong_dimension = title.map(|c| c.get("dimension").and_then(Value::as_str)!=Some("title")
                    || !is_supported_claim(c,"title")).unwrap_or(false);
                result.reason_codes.push(if wrong_dimension {
                    "requirement.title.source_competence_mismatch".to_owned()
                } else {
                    "requirement.title.not_established".to_owned()
                });
            } else {
                result.reason_codes.push("requirement.title.verified".to_owned());
            }
            let modality = required_str(requirement, "modality", "title requirement")?;
            if !is_hard_modality(modality) { return Err(invalid("title requirement unexpectedly non-hard")); }
        }
        let custody = eligible_observation(packet, query, "custody", None);
        if let Some(claim) = custody {
            if is_supported_claim(claim, "custody") { add_claim(&mut result, claim); }
        }
        let permitted = eligible_observation(packet, query, "permitted_use", None);
        let permitted_is_usable = permitted.map(|claim| is_supported_claim(claim, "permitted_use")).unwrap_or(false);
        let use_allowed = if permitted_is_usable {
            permitted.and_then(|c|c.get("value"))
                .and_then(|v|v.get("commercial_occupancy_allowed"))
                .and_then(Value::as_bool)
        } else {
            None
        };
        if permitted_is_usable {
            if let Some(claim) = permitted { add_claim(&mut result, claim); }
        }
        match use_allowed {
            Some(true) => {
                result.permitted_use = "commercial_allowed".to_owned();
                result.reason_codes.push("requirement.conditional.not_triggered".to_owned());
            }
            Some(false) => {
                result.permitted_use = "commercial_prohibited".to_owned();
                result.reason_codes.push("requirement.conditional.triggered".to_owned());
                let variance = eligible_observation(packet, query, "land_use_variance", None);
                let variance_ok = variance.map(|c|is_supported_claim(c,"land_use_variance")).unwrap_or(false);
                if let Some(claim) = variance { add_claim(&mut result, claim); }
                if !variance_ok {
                    blocked = true;
                    result.reason_codes.push("requirement.land_use_variance.missing".to_owned());
                }
            }
            None => {
                result.permitted_use = "unknown".to_owned();
                blocked = true;
                result.reason_codes.push("requirement.permitted_use.not_established".to_owned());
            }
        }
        let _ = applied_count;
        result.reason_codes.push("preference.non_blocking".to_owned());
        result.reason_codes.push("intent.baseline.human_accepted".to_owned());
    } else if action == "schedule_condition_inspection" {
        result.reason_codes.push("action.independent_of_title".to_owned());
        result.reason_codes.push("authority.scope.verified".to_owned());
    } else {
        blocked = true;
        result.reason_codes.push("action.not_in_reference_policy".to_owned());
    }
    result.disposition = if blocked { "BLOCK".to_owned() } else { "ALLOW_SCOPED".to_owned() };
    result.reason_codes.sort();
    result.supporting_claim_refs.sort();
    result.source_receipt_refs.sort();
    Ok(result)
}
fn has_unknown_operation(outcome: &Value) -> bool {
    outcome.get("operation_history").and_then(Value::as_array)
        .map(|events| events.iter().any(|e|e.get("outcome").and_then(Value::as_str)==Some("unknown")))
        .unwrap_or(false)
}
fn blind_retry_in_history(outcome: &Value) -> bool {
    let Some(events) = outcome.get("operation_history").and_then(Value::as_array) else { return false; };
    let unknown_times: Vec<&str> = events.iter()
        .filter(|e|e.get("outcome").and_then(Value::as_str)==Some("unknown"))
        .filter_map(|e|e.get("occurred_at").and_then(Value::as_str)).collect();
    events.iter().any(|event| {
        if event.get("event_type").and_then(Value::as_str)!=Some("submit_attempt") { return false; }
        let when = event.get("occurred_at").and_then(Value::as_str).unwrap_or("");
        unknown_times.iter().any(|unknown| when > *unknown)
    })
}
fn check_requirement_spans(packet: &Value) -> Result<usize, Box<dyn Error>> {
    let content = required_str(&packet["source_brief"], "content", "source_brief")?;
    let requirements = required_array(packet, "intent_candidates", "fixture")?;
    let mut ids = HashSet::new();
    for requirement in requirements {
        let id = required_str(requirement,"requirement_ref","requirement")?;
        if !ids.insert(id) { return Err(invalid(format!("duplicate requirement {id}"))); }
        let source_ref = required_str(requirement,"source_document_ref","requirement")?;
        if source_ref != packet["source_brief"]["document_ref"].as_str().unwrap_or("") {
            return Err(invalid(format!("{id}: source document mismatch")));
        }
        if required_str(requirement,"source_version","requirement")? != packet["source_brief"]["version"].as_str().unwrap_or("") {
            return Err(invalid(format!("{id}: source version mismatch")));
        }
        let start = requirement["span_start_byte"].as_u64().ok_or_else(||invalid("span_start_byte missing"))? as usize;
        let end = requirement["span_end_byte"].as_u64().ok_or_else(||invalid("span_end_byte missing"))? as usize;
        let quote = required_str(requirement,"exact_text","requirement")?;
        if !byte_span_matches(content,start,end,quote) {
            return Err(invalid(format!("{id}: byte span does not match exact source quotation")));
        }
    }
    Ok(requirements.len())
}
fn check_baseline(packet: &Value) -> Result<(), Box<dyn Error>> {
    let baseline = &packet["intent_baseline"];
    if required_str(baseline,"system_id","baseline")? != "symthaea" { return Err(invalid("intent baseline owner must be Symthaea")); }
    if baseline["approval"]["state"].as_str()!=Some("accepted")
        || baseline["approval"]["approval_kind"].as_str()!=Some("human_record_not_cryptographically_verified")
        || baseline["approval"]["approved_by_ref"].as_str().unwrap_or("").starts_with("SYSTEM-") {
        return Err(invalid("intent baseline requires an explicitly human-owned acceptance record"));
    }
    let baseline_refs = string_set(baseline,"requirement_refs","baseline")?;
    let requirements: HashSet<String> = packet["intent_candidates"].as_array().unwrap().iter()
        .filter_map(|x|x.get("requirement_ref").and_then(Value::as_str).map(str::to_owned)).collect();
    if baseline_refs != requirements { return Err(invalid("accepted baseline requirement references do not exactly match candidate set")); }
    Ok(())
}
fn check_receipts(packet: &Value) -> Result<(), Box<dyn Error>> {
    let receipts = required_array(packet,"evidence_receipts","fixture")?;
    let map = index(receipts,"receipt_ref","evidence_receipts")?;
    for observation in required_array(packet,"claim_observations","fixture")? {
        let receipt_ref = required_str(observation,"source_receipt_ref","claim")?;
        let receipt = map.get(receipt_ref).ok_or_else(||invalid(format!("missing source receipt {receipt_ref}")))?;
        if required_str(receipt,"source_ref","receipt")? != required_str(observation,"source_ref","claim")?
            || required_str(receipt,"source_version","receipt")? != required_str(observation,"source_version","claim")? {
            return Err(invalid(format!("{receipt_ref}: receipt does not bind claim source/version")));
        }
        if required_str(receipt,"digest_verification","receipt")? != "synthetic_placeholder_not_cryptographically_verified" {
            return Err(invalid("fixture digest must be explicitly marked as unverified synthetic placeholder"));
        }
    }
    Ok(())
}
fn check_query_bindings(packet: &Value) -> Result<(), Box<dyn Error>> {
    let authority = &packet["authority"];
    for query in required_array(packet,"queries","fixture")? {
        for key in ["effective_at","known_at","action_at"] {
            let value = required_str(query,key,"query")?;
            if !canonical_utc(value) { return Err(invalid(format!("{}: {key} must be canonical UTC Z",required_str(query,"query_ref","query")?))); }
        }
        if query["asset_ref"].as_str()!=packet["asset"]["asset_ref"].as_str()
            || query["contracting_entity_ref"].as_str()!=packet["asset"]["contracting_entity_ref"].as_str() {
            return Err(invalid("query asset/entity binding mismatch"));
        }
        if query["actor_ref"].as_str()!=authority["subject_ref"].as_str() {
            return Err(invalid("query actor does not bind to authority subject"));
        }
    }
    Ok(())
}
fn compare_submission(packet: &Value, oracle: &Value, submission: &Value) -> Result<Value, Box<dyn Error>> {
    let queries = required_array(packet,"queries","fixture")?;
    let gold_rows = required_array(oracle,"query_expectations","oracle")?;
    let actual_rows = required_array(submission,"decisions","submission")?;
    let gold = index(gold_rows,"query_ref","oracle query expectations")?;
    let actual = index(actual_rows,"query_ref","submission decisions")?;
    if gold.len()!=queries.len() || actual.len()!=queries.len()
        || queries.iter().any(|q| {
            let id=q.get("query_ref").and_then(Value::as_str).unwrap_or("");
            !gold.contains_key(id) || !actual.contains_key(id)
        }) {
        return Err(invalid("query, oracle and submission must have exact one-to-one query coverage"));
    }
    let mut rows=Vec::new();
    let mut correct=0usize;
    let mut false_clears=0usize;
    let mut authority_correct=0usize;
    let mut temporal_correct=0usize;
    let mut reason_tp=0usize;
    let mut reason_gold=0usize;
    let mut reason_actual=0usize;
    let mut missing_trace=0usize;
    for query in queries {
        let id=required_str(query,"query_ref","query")?;
        let expected=gold[id];
        let output=actual[id];
        let derived=derive_query(packet,query)?;
        let disposition=required_str(output,"disposition",id)?;
        let authority_state=required_str(output,"authority_state",id)?;
        let permitted_use=required_str(output,"permitted_use",id)?;
        let expected_disposition=required_str(expected,"expected_disposition","oracle")?;
        let expected_authority=required_str(expected,"expected_authority_state","oracle")?;
        let expected_use=required_str(expected,"expected_permitted_use","oracle")?;
        let actual_reasons=string_set(output,"reason_codes",id)?;
        let required_reasons=string_set(expected,"required_reason_codes","oracle")?;
        let required_claims=string_set(expected,"required_claim_refs","oracle")?;
        let submitted_claims=string_set(output,"supporting_claim_refs",id)?;
        let expected_reasons_present=required_reasons.is_subset(&actual_reasons)
            && derived.reason_codes.iter().all(|reason| actual_reasons.contains(reason));
        let expected_claims_present=required_claims.is_subset(&submitted_claims)
            && derived.supporting_claim_refs.iter().cloned().collect::<HashSet<String>>() == submitted_claims
            && derived.source_receipt_refs.iter().cloned().collect::<HashSet<String>>() == string_set(output,"source_receipt_refs",id)?;
        let disposition_correct=disposition==expected_disposition && disposition==derived.disposition;
        let authority_ok=authority_state==expected_authority && authority_state==derived.authority_state;
        let use_ok=permitted_use==expected_use && permitted_use==derived.permitted_use;
        if disposition_correct && authority_ok && use_ok && expected_reasons_present && expected_claims_present { correct+=1; }
        if !is_allow(expected_disposition) && is_allow(disposition) { false_clears+=1; }
        if authority_ok { authority_correct+=1; }
        if use_ok { temporal_correct+=1; }
        let overlap=actual_reasons.intersection(&required_reasons).count();
        reason_tp+=overlap; reason_gold+=required_reasons.len(); reason_actual+=actual_reasons.len();
        if !expected_reasons_present || !expected_claims_present { missing_trace+=1; }

        // Every cited claim must exist, be applicable at the query's two times,
        // and resolve to the same source receipt as the claim observation.
        for claim_ref in &submitted_claims {
            let claim=packet["claim_observations"].as_array().unwrap().iter()
                .find(|c|c.get("claim_ref").and_then(Value::as_str)==Some(claim_ref))
                .ok_or_else(||invalid(format!("{id}: submission cites unknown claim {claim_ref}")))?;
            let start=required_str(claim,"effective_from","claim")?;
            let observed=required_str(claim,"observed_at","claim")?;
            if start>required_str(query,"effective_at","query")? || observed>required_str(query,"known_at","query")? {
                return Err(invalid(format!("{id}: submission cites claim outside query's bitemporal slice")));
            }
        }
        // Compare source/receipt refs for each submitted claim.
        let receipts=string_set(output,"source_receipt_refs",id)?;
        for claim_ref in &submitted_claims {
            let claim=packet["claim_observations"].as_array().unwrap().iter()
                .find(|c|c.get("claim_ref").and_then(Value::as_str)==Some(claim_ref)).unwrap();
            let r=required_str(claim,"source_receipt_ref","claim")?;
            if !receipts.contains(r) { missing_trace+=1; }
        }
        if output.get("authority_ref").and_then(Value::as_str)!=packet["authority"]["authorization_ref"].as_str()
            || output.get("policy_ref").and_then(Value::as_str)!=packet["policy_ref"].as_str()
            || output.get("evaluator_ref").and_then(Value::as_str)!=packet["evaluator_ref"].as_str() {
            missing_trace+=1;
        }
        rows.push(json!({
            "query_ref":id,
            "disposition_correct":disposition_correct,
            "authority_state_correct":authority_ok,
            "permitted_use_projection_correct":use_ok,
            "required_reason_codes_present":expected_reasons_present,
            "required_claim_refs_present":expected_claims_present,
            "derived_disposition":derived.disposition,
            "derived_authority_state":derived.authority_state,
            "derived_permitted_use":derived.permitted_use,
            "critical_false_clear":!is_allow(expected_disposition)&&is_allow(disposition),
            "derived_reason_codes":derived.reason_codes
        }));
    }
    let n=queries.len().max(1) as f64;
    Ok(json!({
        "query_count":queries.len(),
        "query_conformance_rate":correct as f64/n,
        "decision_accuracy":correct as f64/n,
        "authority_state_accuracy":authority_correct as f64/n,
        "bitemporal_projection_accuracy":temporal_correct as f64/n,
        "reason_code_precision":if reason_actual==0 {0.0}else{reason_tp as f64/reason_actual as f64},
        "reason_code_recall":if reason_gold==0 {0.0}else{reason_tp as f64/reason_gold as f64},
        "critical_false_clears":false_clears,
        "trace_completeness_failures":missing_trace,
        "per_query":rows
    }))
}
fn operation_binding_matches(approved: &Value, pending: &Value) -> bool {
    approved == pending
}
fn unknown_finality_retry_is_blocked(prior_effect: &str, retry_count: u64, reattempt_permitted: bool) -> bool {
    prior_effect == "outcome_unknown_after_dispatch" && (retry_count > 0 || reattempt_permitted)
}
fn check_operation_finality(packet: &Value, submission: &Value) -> Result<bool, Box<dyn Error>> {
    let binding=&packet["operation_finality"];
    let approved=&binding["approved_operation"];
    let pending=&binding["pending_operation"];
    if !operation_binding_matches(approved,pending) {
        return Err(invalid("approved and pending exact-operation projections differ in the fixture"));
    }
    let execution=&packet["execution_control"];
    let write=required_array(submission,"write_trace","submission")?
        .first().ok_or_else(||invalid("missing normalized write trace"))?;
    let checks=[
        (required_str(approved,"logical_operation_id","approved operation")?,required_str(execution,"operation_ref","execution control")?,"operation reference"),
        (required_str(approved,"idempotency_ref","approved operation")?,required_str(execution,"idempotency_ref","execution control")?,"idempotency reference"),
        (required_str(approved,"target_ref","approved operation")?,required_str(&packet["asset"],"asset_ref","asset")?,"target asset"),
        (required_str(approved,"action","approved operation")?,required_str(write,"action","write trace")?,"operation action"),
        (required_str(approved,"principal_ref","approved operation")?,required_str(write,"actor_ref","write trace")?,"operation principal"),
        (required_str(approved,"purpose","approved operation")?,required_str(write,"purpose","write trace")?,"operation purpose"),
        (required_str(approved,"canonical_operation_digest","approved operation")?,required_str(write,"canonical_operation_digest","write trace")?,"canonical operation digest"),
        (required_str(approved,"action_digest","approved operation")?,required_str(write,"action_digest","write trace")?,"action digest"),
        (required_str(binding,"policy_revision","operation finality")?,required_str(packet,"policy_ref","policy")?,"policy revision"),
    ];
    for (a,b,label) in checks {
        if a!=b { return Err(invalid(format!("operation finality {label} mismatch"))); }
    }
    if approved != pending
        || required_str(binding,"approval_state","operation finality")? != "authorized"
        || required_str(binding,"approval_kind","operation finality")? != "human_record_not_cryptographically_verified"
        || required_str(binding,"authority_state_before_dispatch","operation finality")? != "current"
        || required_str(binding,"resource_state_before_dispatch","operation finality")? != "matches"
        || required_str(binding,"prior_effect_after_timeout","operation finality")? != "outcome_unknown_after_dispatch"
        || required_str(binding,"retry_safety_after_timeout","operation finality")? != "unknown"
        || required_str(binding,"expected_post_timeout_disposition","operation finality")? != "HOLD_FOR_RECONCILIATION"
        || binding["expected_retry_permitted"].as_bool()!=Some(false) {
        return Err(invalid("fixture operation-finality state is internally inconsistent"));
    }
    let reported=&submission["operation_finality_result"];
    let constitution=&binding["constitution"];
    let expected_fields=[
        ("operation_ref",required_str(approved,"logical_operation_id","approved operation")?),
        ("canonical_operation_digest",required_str(approved,"canonical_operation_digest","approved operation")?),
        ("action_digest",required_str(approved,"action_digest","approved operation")?),
        ("approval_ref",required_str(binding,"approval_ref","operation finality")?),
        ("constitution_digest",required_str(constitution,"constitution_digest","constitution")?),
        ("action",required_str(approved,"action","approved operation")?),
        ("principal_ref",required_str(approved,"principal_ref","approved operation")?),
        ("target_ref",required_str(approved,"target_ref","approved operation")?),
        ("purpose",required_str(approved,"purpose","approved operation")?),
        ("idempotency_ref",required_str(approved,"idempotency_ref","approved operation")?),
    ];
    let mut exact=true;
    for (field,value) in expected_fields {
        if reported.get(field).and_then(Value::as_str)!=Some(value) { exact=false; }
    }
    exact &= reported["constitution_epoch"].as_u64()==constitution["epoch"].as_u64();
    exact &= reported["authority_state"].as_str()==Some("current");
    exact &= reported["resource_state"].as_str()==Some("matches");
    exact &= reported["prior_effect_state"].as_str()==Some("outcome_unknown_after_dispatch");
    exact &= reported["retry_safety"].as_str()==Some("unknown");
    exact &= reported["disposition"].as_str()==Some("HOLD_FOR_RECONCILIATION");
    exact &= reported["retry_count"].as_u64()==Some(0);
    exact &= reported["reattempt_permitted"].as_bool()==Some(false);
    if unknown_finality_retry_is_blocked(
        required_str(binding,"prior_effect_after_timeout","operation finality")?,
        reported["retry_count"].as_u64().unwrap_or(0),
        reported["reattempt_permitted"].as_bool().unwrap_or(true),
    ) {
        return Err(invalid("operation-finality submission permits retry after unknown outcome"));
    }
    Ok(exact)
}
fn run() -> Result<(), Box<dyn Error>> {
    let args:Vec<String>=env::args().collect();
    if args.len()!=4 { return Err(invalid("usage: iae-vertical-slice-v0-2 <fixture.json> <oracle.json> <submission.json>")); }
    let fixture=read_json(&args[1])?;
    let oracle=read_json(&args[2])?;
    let submission=read_json(&args[3])?;
    if fixture.get("exchange_version").and_then(Value::as_str)!=Some("0.2")
        || fixture.get("synthetic").and_then(Value::as_bool)!=Some(true)
        || oracle.get("oracle_version").and_then(Value::as_str)!=Some("0.2")
        || oracle.get("synthetic").and_then(Value::as_bool)!=Some(true) {
        return Err(invalid("fixture and oracle must be synthetic IAE v0.2"));
    }
    if fixture["environment"]["mode"].as_str()!=Some("synthetic_fixture_only")
        || fixture["environment"]["external_io_enabled"].as_bool()!=Some(false)
        || fixture["environment"]["network_enabled"].as_bool()!=Some(false)
        || fixture["environment"]["mutations_enabled"].as_bool()!=Some(false)
        || submission["environment"]["mode"].as_str()!=Some("synthetic_fixture_only")
        || submission["environment"]["external_io_enabled"].as_bool()!=Some(false)
        || submission["environment"]["network_enabled"].as_bool()!=Some(false)
        || submission["environment"]["mutations_enabled"].as_bool()!=Some(false) {
        return Err(invalid("refusing non-fixture or externally enabled execution"));
    }
    required_str(&submission,"system_id","submission")?;
    required_str(&submission,"build_sha","submission")?;
    required_str(&submission,"configuration_sha256","submission")?;
    check_requirement_spans(&fixture)?;
    check_baseline(&fixture)?;
    check_receipts(&fixture)?;
    check_query_bindings(&fixture)?;
    // Exact operation authorization is distinct from human acceptance of the intent baseline.
    let operation_finality_ok=check_operation_finality(&fixture,&submission)?;
    if submission["intent_baseline_ref"].as_str()!=fixture["intent_baseline"]["baseline_id"].as_str() {
        return Err(invalid("submission baseline reference mismatch"));
    }
    // The baseline is accepted only as a human-owned reference. Verify modality is
    // preserved exactly and that preferred/unresolved requirements are not hard blockers.
    let requirement_index=index(required_array(&fixture,"intent_candidates","fixture")?,"requirement_ref","requirements")?;
    let modality_expectations=index(required_array(&oracle,"requirement_expectations","oracle")?,"requirement_ref","requirement oracle")?;
    if requirement_index.len()!=modality_expectations.len() {
        return Err(invalid("requirement candidate/oracle coverage mismatch"));
    }
    for (id,expected) in modality_expectations {
        let actual=requirement_index.get(&id).ok_or_else(||invalid(format!("oracle refers to absent requirement {id}")))?;
        if required_str(actual,"modality",&id)? != required_str(expected,"modality","requirement oracle")? {
            return Err(invalid(format!("{id}: modality mismatch")));
        }
        if is_non_blocking_modality(required_str(actual,"modality",&id)?)
            && expected.get("hard_blocker").and_then(Value::as_bool)==Some(true) {
            return Err(invalid(format!("{id}: preferred/unresolved requirement incorrectly designated hard blocker")));
        }
    }
    let metrics=compare_submission(&fixture,&oracle,&submission)?;
    let execution=&fixture["execution_control"];
    let expected_exec=&oracle["execution_expectation"];
    let submitted_exec=&submission["execution_control"];
    let unknown_history=has_unknown_operation(&fixture);
    let blind_retry=blind_retry_in_history(&fixture);
    let operation_events=required_array(&fixture,"operation_history","fixture")?;
    let operation_ref=required_str(execution,"operation_ref","execution_control")?;
    let idempotency_ref=required_str(execution,"idempotency_ref","execution_control")?;
    let same_operation_events=operation_events.iter().all(|event|
        event.get("operation_ref").and_then(Value::as_str)==Some(operation_ref)
            && event.get("idempotency_ref").and_then(Value::as_str)==Some(idempotency_ref));
    let submit_attempts=operation_events.iter().filter(|event|
        event.get("event_type").and_then(Value::as_str)==Some("submit_attempt")).count();
    let operation_consistent=unknown_history && same_operation_events && submit_attempts==1
        && execution["state"].as_str()==Some("UNKNOWN")
        && submitted_exec["state"].as_str()==expected_exec["state"].as_str()
        && submitted_exec["disposition"].as_str()==expected_exec["disposition"].as_str()
        && submitted_exec["operation_ref"].as_str()==execution["operation_ref"].as_str()
        && submitted_exec["idempotency_ref"].as_str()==execution["idempotency_ref"].as_str()
        && submitted_exec["mutation_count"].as_u64()==Some(1)
        && submitted_exec["retry_count"].as_u64()==Some(0)
        && submitted_exec["reattempt_permitted"].as_bool()==Some(false)
        && submitted_exec["reconciliation_required"].as_bool()==Some(true)
        && !blind_retry;
    if submitted_exec["retry_count"].as_u64().unwrap_or(0)>0
        || submitted_exec["reattempt_permitted"].as_bool()==Some(true)
        || blind_retry {
        return Err(invalid("blind retry detected while an operation outcome is or was UNKNOWN"));
    }
    let authorized_boundaries=required_array(execution,"authorized_write_boundary","execution_control")?;
    let write_trace=required_array(&submission,"write_trace","submission")?;
    let mut write_ids=HashSet::new();
    let mut unauthorized_side_effects=0usize;
    let mut unauthorized_external_side_effects=0usize;
    let mut possible_external_writes=0usize;
    let operation_events=required_array(&fixture,"operation_history","fixture")?;
    for write in write_trace {
        let event_ref=required_str(write,"event_ref","write trace")?;
        if !write_ids.insert(event_ref.to_owned()) {
            return Err(invalid(format!("duplicate write trace event {event_ref}")));
        }
        if !write_is_authorized(write,authorized_boundaries) {
            unauthorized_side_effects+=1;
            if write.get("external_visibility").and_then(Value::as_str)==Some("external") {
                unauthorized_external_side_effects+=1;
            }
        }
        if write.get("possible_external_side_effect").and_then(Value::as_bool)==Some(true) {
            possible_external_writes+=1;
            if write.get("effect_status").and_then(Value::as_str)!=Some("possible")
                || write.get("external_visibility").and_then(Value::as_str)!=Some("unknown") {
                return Err(invalid("UNKNOWN operation must preserve possible effect and unknown external visibility"));
            }
        }
        let matching_event=operation_events.iter().find(|event|
            event.get("event_ref").and_then(Value::as_str)==Some(event_ref));
        let event=matching_event.ok_or_else(||invalid(format!("write trace event {event_ref} absent from operation history")))?;
        for field in ["operation_ref","idempotency_ref","action","purpose","asset_ref","actor_ref","canonical_operation_digest","action_digest"] {
            if write.get(field)!=event.get(field) {
                return Err(invalid(format!("write trace {event_ref}: {field} differs from operation event")));
            }
        }
    }
    let submit_event_count=operation_events.iter().filter(|event|
        event.get("event_type").and_then(Value::as_str)==Some("submit_attempt")).count();
    if write_trace.len()!=submit_event_count {
        return Err(invalid("every submit_attempt must have exactly one normalized write-trace record"));
    }
    let write_authorization_ok=unauthorized_side_effects==0;
    let trace=&submission["trace_completeness"];
    let required_trace=[
        ("human_approval_ref",fixture["intent_baseline"]["approval"]["approval_ref"].as_str().unwrap_or("")),
        ("baseline_ref",fixture["intent_baseline"]["baseline_id"].as_str().unwrap_or("")),
        ("authority_ref",fixture["authority"]["authorization_ref"].as_str().unwrap_or("")),
        ("policy_ref",fixture["policy_ref"].as_str().unwrap_or("")),
        ("evaluator_ref",fixture["evaluator_ref"].as_str().unwrap_or(""))
    ];
    let mut trace_ok=true;
    for (field,value) in required_trace {
        if trace.get(field).and_then(Value::as_str)!=Some(value) { trace_ok=false; }
    }
    let trace_receipts=string_set(trace,"source_receipt_refs","trace_completeness")?;
    for receipt in required_array(&fixture,"evidence_receipts","fixture")? {
        let ref_id=required_str(receipt,"receipt_ref","receipt")?;
        if !trace_receipts.contains(ref_id) { trace_ok=false; }
    }
    // Verify that the metamorphic inputs actually represent the declared perturbation.
    for query in required_array(&fixture,"queries","fixture")? {
        let query_ref=required_str(query,"query_ref","query")?;
        if query_ref=="Q-HISTORICAL" {
            let excluded=string_set(query,"excluded_claim_refs",query_ref)?;
            if !excluded.contains("CLM-CONDITION-UNRELATED") {
                return Err(invalid("base query must exclude the unrelated-condition perturbation"));
            }
        }
        if query_ref=="Q-UNRELATED-CLAIM" {
            let added=string_set(query,"added_claim_refs",query_ref)?;
            let base=required_array(&fixture,"queries","fixture")?.iter().find(|q|
                q.get("query_ref").and_then(Value::as_str)==Some("Q-HISTORICAL")).unwrap();
            let base_excluded=string_set(base,"excluded_claim_refs","Q-HISTORICAL")?;
            if !added.contains("CLM-CONDITION-UNRELATED") || !base_excluded.contains("CLM-CONDITION-UNRELATED") {
                return Err(invalid("unrelated-claim metamorphic pair is not a true add-only perturbation"));
            }
            let unrelated=fixture["claim_observations"].as_array().unwrap().iter().find(|c|
                c.get("claim_ref").and_then(Value::as_str)==Some("CLM-CONDITION-UNRELATED"))
                .ok_or_else(||invalid("unrelated claim observation missing"))?;
            if required_str(unrelated,"observed_at","unrelated claim")? > required_str(query,"known_at",query_ref)? {
                return Err(invalid("unrelated claim is not known in the metamorphic query's knowledge view"));
            }
        }
    }
    let metamorphic_gold=required_array(&oracle,"metamorphic_expectations","oracle")?;
    let metamorphic_actual=required_array(&submission,"metamorphic_results","submission")?;
    let gold_map=index(metamorphic_gold,"probe_ref","metamorphic oracle")?;
    let actual_map=index(metamorphic_actual,"probe_ref","metamorphic submission")?;
    let mut metamorphic_ok=true;
    if gold_map.len()!=actual_map.len() { metamorphic_ok=false; }
    for (id,gold) in gold_map {
        let Some(actual)=actual_map.get(&id) else { metamorphic_ok=false; continue; };
        let base=required_str(actual,"base_disposition","metamorphic result")?;
        let variant=required_str(actual,"variant_disposition","metamorphic result")?;
        if gold["expected"].as_str()==Some("identical") && base!=variant { metamorphic_ok=false; }
        if gold["invariant"].as_str()==Some("disposition_and_reason_codes") {
            let a=string_set(actual,"base_reason_codes","metamorphic result")?;
            let b=string_set(actual,"variant_reason_codes","metamorphic result")?;
            if a!=b { metamorphic_ok=false; }
        }
    }
    let query_coverage=metrics["query_count"].as_u64().unwrap_or(0) as usize==fixture["queries"].as_array().map(Vec::len).unwrap_or(0);
    let all_ok=query_coverage
        && operation_finality_ok
        && write_authorization_ok
        && metrics["critical_false_clears"].as_u64()==Some(0)
        && metrics["trace_completeness_failures"].as_u64()==Some(0)
        && operation_consistent && trace_ok && metamorphic_ok;
    let report=json!({
        "evaluator_version":"iae-vertical-slice-v0.2-rust-0.1",
        "fixture_version":"0.2",
        "packet_id":fixture["packet_id"],
        "system_id":submission["system_id"],
        "build_sha":submission["build_sha"],
        "configuration_sha256":submission["configuration_sha256"],
        "requirement_source_spans_verified":fixture["intent_candidates"].as_array().map(Vec::len).unwrap_or(0),
        "bitemporal_query_rules":"canonical UTC lexical ordering: effective_from <= effective_at and observed_at <= known_at",
        "metrics":metrics,
        "operation_finality":{
            "approved_operation_equals_pending_operation":operation_binding_matches(&fixture["operation_finality"]["approved_operation"],&fixture["operation_finality"]["pending_operation"]),
            "operation_finality_matches_submission":operation_finality_ok,
            "distinct_human_operation_approval":fixture["operation_finality"]["approval_ref"]!=fixture["intent_baseline"]["approval"]["approval_ref"],
            "constitution_epoch_bound":fixture["operation_finality"]["constitution"]["epoch"],
            "retry_safety":"unknown",
            "required_disposition":"HOLD_FOR_RECONCILIATION"
        },
        "execution_control":{
            "unknown_event_in_history":unknown_history,
            "blind_retry_detected":blind_retry,
            "execution_control_matches_oracle":operation_consistent
        },
        "trace_completeness":trace_ok,
        "side_effect_authorization":{
            "write_trace_count":write_trace.len(),
            "possible_external_write_count":possible_external_writes,
            "unauthorized_side_effects":unauthorized_side_effects,
            "unauthorized_external_side_effects":unauthorized_external_side_effects,
            "write_trace_within_explicit_boundary":write_authorization_ok
        },
        "metamorphic_invariance":metamorphic_ok,
        "fixture_only_safety":{
            "external_io_disabled":fixture["environment"]["external_io_enabled"]==false,
            "network_disabled":fixture["environment"]["network_enabled"]==false,
            "mutations_disabled":fixture["environment"]["mutations_enabled"]==false
        },
        "reproducibility_receipt":{
            "fixture_file_sha256":sha256_file(&args[1])?,
            "oracle_file_sha256":sha256_file(&args[2])?,
            "submission_file_sha256":sha256_file(&args[3])?,
            "evaluator_binary_sha256":sha256_current_executable()?
        },
        "qualification":{
            "development_conformance_pass":all_ok,
            "independent_domain_adjudication":false,
            "cryptographic_authenticity_verified":false,
            "legal_or_operational_authority_established":false,
            "production_qualification":false
        },
        "claim_limits":[
            "Synthetic fixture and provisional oracle only; no independent legal or domain adjudication.",
            "Placeholder source and baseline digests are explicitly not cryptographically verified.",
            "The evaluator checks fixture semantics and declared output; it is not a real source adapter, trust-root validator or external transaction provider.",
            "A development conformance pass is not production safety or legal correctness."
        ]
    });
    println!("{}",serde_json::to_string_pretty(&report)?);
    if !all_ok { std::process::exit(1); }
    Ok(())
}
fn main() {
    if let Err(error)=run() {
        eprintln!("iae-vertical-slice-v0.2: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::{byte_span_matches,is_hard_modality,is_non_blocking_modality,canonical_utc,blind_retry_in_history,authority_state_at,write_is_authorized};
    use serde_json::json;

    #[test]
    fn approved_and_pending_operation_bindings_must_match_exactly() {
        let approved=json!({"action":"approve","target":"asset-a","purpose":"lease"});
        let pending=json!({"action":"approve","target":"asset-a","purpose":"lease"});
        let substituted=json!({"action":"approve","target":"asset-b","purpose":"lease"});
        assert!(operation_binding_matches(&approved,&pending));
        assert!(!operation_binding_matches(&approved,&substituted));
    }
    #[test]
    fn unknown_outcome_finality_blocks_any_reattempt() {
        assert!(unknown_finality_retry_is_blocked("outcome_unknown_after_dispatch",0,true));
        assert!(unknown_finality_retry_is_blocked("outcome_unknown_after_dispatch",1,false));
        assert!(!unknown_finality_retry_is_blocked("outcome_unknown_after_dispatch",0,false));
        assert!(!unknown_finality_retry_is_blocked("never_attempted",0,false));
    }

    #[test]
    fn source_spans_use_utf8_bytes_not_character_offsets() {
        let source="The café lease";
        let start=source.find("café").unwrap();
        assert!(byte_span_matches(source,start,start+"café".len(),"café"));
        assert!(!byte_span_matches(source,start+1,start+3,"afé"));
    }
    #[test]
    fn mandatory_and_conditional_are_distinct_from_preferred_and_unresolved() {
        assert!(is_hard_modality("mandatory"));
        assert!(is_hard_modality("conditional"));
        assert!(is_non_blocking_modality("preferred"));
        assert!(is_non_blocking_modality("unresolved"));
    }
    #[test]
    fn only_canonical_utc_strings_are_accepted_for_lexical_time_ordering() {
        assert!(canonical_utc("2026-02-06T10:00:00Z"));
        assert!(!canonical_utc("2026-02-06T12:00:00+02:00"));
    }
    #[test]
    fn late_observed_correction_is_invisible_to_earlier_knowledge_view() {
        let packet=json!({"authority":{"status_events":[]},"claim_observations":[
            {"claim_ref":"v4","asset_ref":"A","dimension":"permitted_use","effective_from":"2026-01-01T00:00:00Z","observed_at":"2026-01-02T00:00:00Z"},
            {"claim_ref":"v5","asset_ref":"A","dimension":"permitted_use","effective_from":"2026-01-15T00:00:00Z","observed_at":"2026-02-05T00:00:00Z"}
        ]});
        let q=json!({"asset_ref":"A","effective_at":"2026-01-25T00:00:00Z","known_at":"2026-01-25T00:00:00Z","excluded_claim_refs":[]});
        let obs=super::eligible_observation(&packet,&q,"permitted_use",None).unwrap();
        assert_eq!(obs["claim_ref"],"v4");
    }
    #[test]
    fn later_knowledge_view_selects_retroactive_correction() {
        let packet=json!({"authority":{"status_events":[]},"claim_observations":[
            {"claim_ref":"v4","asset_ref":"A","dimension":"permitted_use","effective_from":"2026-01-01T00:00:00Z","observed_at":"2026-01-02T00:00:00Z"},
            {"claim_ref":"v5","asset_ref":"A","dimension":"permitted_use","effective_from":"2026-01-15T00:00:00Z","observed_at":"2026-02-05T00:00:00Z"}
        ]});
        let q=json!({"asset_ref":"A","effective_at":"2026-01-25T00:00:00Z","known_at":"2026-02-06T00:00:00Z","excluded_claim_refs":[]});
        let obs=super::eligible_observation(&packet,&q,"permitted_use",None).unwrap();
        assert_eq!(obs["claim_ref"],"v5");
    }
    #[test]
    fn revocation_is_not_visible_before_known_time() {
        let packet=json!({"authority":{"status_events":[
            {"status":"active","effective_at":"2025-01-01T00:00:00Z","observed_at":"2025-01-01T00:00:00Z"},
            {"status":"revoked","effective_at":"2026-02-01T00:00:00Z","observed_at":"2026-02-02T12:00:00Z"}
        ]}});
        let old=json!({"action_at":"2026-01-25T10:00:00Z","known_at":"2026-01-25T10:05:00Z"});
        let new=json!({"action_at":"2026-02-06T10:00:00Z","known_at":"2026-02-06T10:00:00Z"});
        assert_eq!(authority_state_at(&packet,&old),"active");
        assert_eq!(authority_state_at(&packet,&new),"revoked");
    }
    #[test]
    fn retry_after_unknown_is_detected_even_if_receipt_later_arrives() {
        let packet=json!({"operation_history":[
            {"event_type":"timeout","outcome":"unknown","occurred_at":"2026-01-25T10:01:05Z"},
            {"event_type":"submit_attempt","outcome":"submitted","occurred_at":"2026-01-25T10:02:00Z"},
            {"event_type":"completion_receipt","outcome":"success","occurred_at":"2026-01-25T10:01:02Z"}
        ]});
        assert!(blind_retry_in_history(&packet));
    }
    #[test]
    fn explicitly_scoped_write_is_authorized() {
        let write=json!({"action":"approve_commercial_lease","purpose":"new_tenant_commercial_lease","asset_ref":"PROPERTY-001","actor_ref":"ACTOR-A","fields":["lease_submission_request"],"authorization_ref":"AUTHZ-1","possible_external_side_effect":true});
        let boundaries=vec![json!({"action":"approve_commercial_lease","purpose":"new_tenant_commercial_lease","asset_ref":"PROPERTY-001","actor_ref":"ACTOR-A","fields":["lease_submission_request","lease_status"],"authorization_ref":"AUTHZ-1","possible_external_side_effect":true})];
        assert!(write_is_authorized(&write,&boundaries));
    }
    #[test]
    fn wrong_asset_or_extra_field_is_unauthorized() {
        let write=json!({"action":"approve_commercial_lease","purpose":"new_tenant_commercial_lease","asset_ref":"PROPERTY-002","actor_ref":"ACTOR-A","fields":["lease_submission_request","notify_external_parties"],"authorization_ref":"AUTHZ-1","possible_external_side_effect":true});
        let boundaries=vec![json!({"action":"approve_commercial_lease","purpose":"new_tenant_commercial_lease","asset_ref":"PROPERTY-001","actor_ref":"ACTOR-A","fields":["lease_submission_request"],"authorization_ref":"AUTHZ-1","possible_external_side_effect":true})];
        assert!(!write_is_authorized(&write,&boundaries));
    }
    #[test]
    fn contested_permitted_use_is_not_promoted_to_a_decision_fact() {
        let claim=json!({"dimension":"permitted_use","disposition":"contested","competent_for":["permitted_use"]});
        assert!(!is_supported_claim(&claim,"permitted_use"));
    }

    #[test]
    fn unrelated_condition_claim_does_not_change_requirement_dimension() {
        let claim=json!({"dimension":"condition","disposition":"supported_within_scope","competent_for":["condition"]});
        assert!(!is_supported_claim(&claim,"title"));
    }
}
