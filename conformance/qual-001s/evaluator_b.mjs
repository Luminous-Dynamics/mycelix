#!/usr/bin/env node
// QUAL-001S evaluator B: independent semantic contract checker.
// It reconstructs expectations from proposition/theorem vocabulary and scans
// raw JSON before JSON.parse so duplicate-key normalization cannot hide input.
import fs from "node:fs";
import { createHash } from "node:crypto";
import path from "node:path";
import { fileURLToPath } from "node:url";

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const raw = fs.readFileSync(path.join(ROOT, "qualification_vectors_v1.json"), "utf8");

function isWhitespace(code) { return code === 9 || code === 10 || code === 13 || code === 32; }

function scanDuplicateObjectKeys(json) {
  let i = 0;
  const error = (message) => { throw new Error(message + " at byte " + i); };
  const skipWhitespace = () => { while (i < json.length && isWhitespace(json.charCodeAt(i))) i++; };
  const scanString = () => {
    if (json[i] !== String.fromCharCode(34)) error("expected string");
    i++;
    while (i < json.length) {
      const code = json.charCodeAt(i);
      if (code === 92) { i += 2; continue; }
      if (code === 34) { i++; return; }
      i++;
    }
    error("unterminated string");
  };
  const scanValue = () => {
    skipWhitespace();
    if (json[i] === "{") return scanObject();
    if (json[i] === "[") return scanArray();
    if (json[i] === String.fromCharCode(34)) return scanString();
    while (i < json.length) {
      const ch = json[i];
      if (ch === "," || ch === "]" || ch === "}") return;
      i++;
    }
  };
  function scanObject() {
    i++;
    const keys = new Set();
    skipWhitespace();
    if (json[i] === "}") { i++; return; }
    while (true) {
      skipWhitespace();
      if (json[i] !== String.fromCharCode(34)) error("expected object key");
      const start = i + 1;
      scanString();
      const end = i - 1;
      const key = JSON.parse(json.slice(start - 1, i));
      if (keys.has(key)) error("duplicate key: " + key);
      keys.add(key);
      skipWhitespace();
      if (json[i] !== ":") error("expected colon");
      i++;
      scanValue();
      skipWhitespace();
      if (json[i] === "}") { i++; return; }
      if (json[i] !== ",") error("expected comma");
      i++;
    }
  }
  function scanArray() {
    i++;
    skipWhitespace();
    if (json[i] === "]") { i++; return; }
    while (true) {
      scanValue();
      skipWhitespace();
      if (json[i] === "]") { i++; return; }
      if (json[i] !== ",") error("expected comma");
      i++;
    }
  }
  skipWhitespace();
  scanValue();
  skipWhitespace();
  if (i !== json.length) error("trailing data");
}

try {
  scanDuplicateObjectKeys(raw);
} catch (err) {
  throw new Error("corpus duplicate-key rejection failed: " + String(err));
}

const corpus = JSON.parse(raw);

function canonicalAsciiJson(value) {
  if (Array.isArray(value)) return "[" + value.map(canonicalAsciiJson).join(",") + "]";
  if (value !== null && typeof value === "object") {
    return "{" + Object.keys(value).sort().map((key) => JSON.stringify(key) + ":" + canonicalAsciiJson(value[key])).join(",") + "}";
  }
  return JSON.stringify(value);
}
const ao = JSON.parse(fs.readFileSync(path.join(ROOT, "s0_authority_observation_v1.example.json"), "utf8"));
if (ao.schema !== "mycelix.qual-001s.s0-authority-observation-v1") throw new Error("wrong S0 authority observation schema");
for (const key of [
  "repository_id","repository_full_name","observation_sha256","s0_workflow_source_commit_sha","s1_workflow_source_commit_sha",
  "dispatch_envelope_sha256","dispatch_ref","dispatch_input_commitment_sha256","dispatch_input_canonical_json","authenticated_principal_identity",
  "policy_scope_identity","observation_timestamp"
]) if (!(key in ao)) throw new Error("S0 authority observation missing " + key);
if (!/^[0-9a-f]{64}$/.test(ao.observation_sha256)) throw new Error("invalid authority observation commitment");
const aoPreimage = {...ao};
delete aoPreimage.observation_sha256;
const computedAuthoritySha256 = createHash("sha256").update(canonicalAsciiJson(aoPreimage), "utf8").digest("hex");
if (computedAuthoritySha256 !== ao.observation_sha256) throw new Error("S0 authority observation commitment does not match canonical preimage");
if (ao.event_type !== "workflow_dispatch") throw new Error("wrong S0 authority observation event type");
if (!["OBSERVED", "UNAVAILABLE", "CONTRADICTED"].includes(ao.observation_state)) throw new Error("invalid S0 observation state");
if (!["OBSERVED", "UNAVAILABLE", "CONTRADICTED"].includes(ao.request_authentication.state)) throw new Error("invalid request authentication state");
if (!["OBSERVED", "UNAVAILABLE", "CONTRADICTED"].includes(ao.actor_authorization.state)) throw new Error("invalid actor authorization state");
if (!["OBSERVED", "UNAVAILABLE", "CONTRADICTED"].includes(ao.event_authorization.state)) throw new Error("invalid event authorization state");
if (!["OBSERVED", "UNAVAILABLE", "CONTRADICTED"].includes(ao.workflow_source_authentication.state)) throw new Error("invalid workflow source authentication state");
if (!["ACCEPTED", "REJECTED", "UNOBSERVED"].includes(ao.dispatch_result)) throw new Error("invalid dispatch result");
if (!["OBSERVED", "UNOBSERVED", "CONTRADICTED"].includes(ao.run_attribution.state)) throw new Error("invalid run attribution state");
if (!/^[0-9a-f]{64}$/.test(ao.dispatch_input_commitment_sha256)) throw new Error("invalid S0 dispatch input commitment");
scanDuplicateObjectKeys(ao.dispatch_input_canonical_json);
const dispatchInputs = JSON.parse(ao.dispatch_input_canonical_json);
if (dispatchInputs === null || Array.isArray(dispatchInputs) || typeof dispatchInputs !== "object") throw new Error("canonical dispatch inputs must be a JSON object");
if (canonicalAsciiJson(dispatchInputs) !== ao.dispatch_input_canonical_json) throw new Error("dispatch input bytes are not canonical JCS");
const dispatchInputHash = createHash("sha256").update(ao.dispatch_input_canonical_json, "utf8").digest("hex");
if (dispatchInputHash !== ao.dispatch_input_commitment_sha256) throw new Error("dispatch input commitment does not match canonical input bytes");
if (ao.dispatch_ref !== "main") throw new Error("wrong S0 dispatch ref");
if (ao.run_attribution.state === "UNOBSERVED" && Object.keys(ao.run_attribution).length !== 1) throw new Error("unobserved run attribution must not carry run identifiers");
for (const field of ["request_authentication","actor_authorization","event_authorization","workflow_source_authentication"]) {
  const d = ao[field];
  if (d.state === "OBSERVED" && (!d.rule_id || !d.principal_identity)) throw new Error("observed authority identity missing for " + field);
}
if (ao.request_authentication.state === "OBSERVED" && ao.request_authentication.principal_identity !== ao.authenticated_principal_identity) throw new Error("top-level authenticated principal disagrees with request authentication");

const s0 = JSON.parse(fs.readFileSync(path.join(ROOT, "s0_dispatch_envelope_v1.example.json"), "utf8"));
if (s0.schema !== "mycelix.qual-001s.s0-dispatch-envelope-v1") throw new Error("wrong S0 schema");
if (s0.canonicalization_profile !== "RFC8785-JCS-IJSON-v1") throw new Error("wrong S0 canonicalization profile");
if (typeof s0.epoch_id !== "string" || s0.epoch_id.length === 0) throw new Error("invalid S0 epoch");
if (s0.candidate_code_executed !== false) throw new Error("S0 candidate execution must be false");
if (typeof s0.envelope_sha256 !== "string" || !/^[0-9a-f]{64}$/.test(s0.envelope_sha256)) throw new Error("invalid S0 envelope commitment");
const s0Preimage = {...s0};
delete s0Preimage.envelope_sha256;
const computedEnvelopeSha256 = createHash("sha256").update(canonicalAsciiJson(s0Preimage), "utf8").digest("hex");
if (computedEnvelopeSha256 !== s0.envelope_sha256) throw new Error("S0 envelope commitment does not match canonical preimage");
if (ao.dispatch_envelope_sha256 !== s0.envelope_sha256) throw new Error("authority observation is not bound to the exact S0 envelope commitment");
for (const field of ["epoch_id","repository_id","repository_full_name","dispatch_nonce_hex","s0_workflow_source_commit_sha","s1_workflow_source_commit_sha"]) {
  if (ao[field] !== s0[field]) throw new Error("authority observation " + field + " disagrees with S0 envelope");
}

let fixtureRejected = false;
try {
  scanDuplicateObjectKeys('{"authority_outcome":"NONE","\\u0061uthority_outcome":"AUTHORITY_AUTHORIZED"}');
} catch (err) {
  fixtureRejected = String(err).includes("duplicate key");
}
if (!fixtureRejected) throw new Error("duplicate-key negative control was accepted");

const expect = new Map([
  ["EXECUTION_AUTHENTICATED", ["VERIFIED", "EXECUTION_OBSERVED"]],
  ["REQUIRED_DEPENDENCY", ["UNAVAILABLE", "EVIDENCE_MISSING"]],
  ["CLAIM_CEILING", ["VERIFIED", "NONE"]],
  ["EPOCH_CONTINUITY", ["CONTRADICTED", "PROVENANCE_MISMATCH"]],
  ["SOURCE_CLASS", ["CONTRADICTED", "PROVENANCE_MISMATCH"]],
  ["TEMPORAL_FRONTIER", ["CONTRADICTED", "NOT_ACCEPTED"]],
  ["RECEIPT_CEILING", ["VERIFIED", "NONE"]],
  ["CORROBORATION_INDEPENDENCE", ["VERIFIED", "NONE"]],
  ["SCHEMA_RECOGNITION", ["UNAVAILABLE", "NOT_ACCEPTED"]],
  ["CANONICAL_JSON", ["CONTRADICTED", "NOT_ACCEPTED"]],
  ["CLAIM_CEILING_REQUEST", ["CONTRADICTED", "AUTHORITY_REJECTED"]],
  ["AUTHORITY_REJECTION", ["VERIFIED", "AUTHORITY_REJECTED"]],
  ["EXECUTION_VS_QUALIFICATION", ["OBSERVED", "EXECUTION_OBSERVED"]],
  ["CANDIDATE_LOCAL_RECEIPT", ["CONTRADICTED", "NOT_ACCEPTED"]],
  ["WORKFLOW_IDENTITY", ["CONTRADICTED", "PROVENANCE_MISMATCH"]],
  ["PRIOR_ATTEMPT", ["CONTRADICTED", "PROVENANCE_MISMATCH"]],
  ["ALWAYS_PASS_IMPOSTOR", ["CONTRADICTED", "CONFORMANCE_CONTRADICTED"]],
  ["ALWAYS_FAIL_IMPOSTOR", ["CONTRADICTED", "CONFORMANCE_CONTRADICTED"]],
  ["POSTFLIGHT_IMMUTABILITY", ["CONTRADICTED", "PROVENANCE_MISMATCH"]],
  ["CURRENT_QUALIFICATION", ["VERIFIED", "AUTHORITY_AUTHORIZED"]],
  ["CANONICALIZATION_ORDER", ["VERIFIED", "NONE"]],
  ["DISPATCH_ACCEPTANCE_VS_EXECUTION", ["UNAVAILABLE", "NOT_ACCEPTED"]],
  ["AUTHORITY_ENVELOPE_JOIN", ["CONTRADICTED", "PROVENANCE_MISMATCH"]],
  ["RUN_ATTEMPT_PAIR", ["VERIFIED", "EXECUTION_OBSERVED"]],
]);

if (corpus.schema !== "mycelix.qual-001s.semantic-corpus-v1") throw new Error("wrong corpus schema");
if (corpus.corpus_id !== "QUAL-001S") throw new Error("wrong corpus id");
if (corpus.vectors.length !== 24) throw new Error("expected 24 vectors");

for (const v of corpus.vectors) {
  const pair = expect.get(v.proposition_id);
  if (!pair) throw new Error(v.vector_id + ": no independent theorem expectation");
  if (v.expected_state !== pair[0] || v.authority_outcome !== pair[1]) {
    throw new Error(v.vector_id + ": expected " + pair[0] + "/" + pair[1] + ", got " + v.expected_state + "/" + v.authority_outcome);
  }
  const requested = new Set(v.requested_claims);
  const admitted = new Set(v.admitted_claims);
  for (const claim of admitted) {
    if (!requested.has(claim)) throw new Error(v.vector_id + ": admitted claim was never requested");
  }
  if (v.kind === "availability" && v.expected_state !== "UNAVAILABLE") throw new Error(v.vector_id + ": availability vector must remain unavailable");
  if (v.proposition_id === "RECEIPT_CEILING" && v.expected_claims.includes("rotation_adoption")) throw new Error("receipt ceiling violated");
  if (v.proposition_id === "CLAIM_CEILING" && v.mutation?.operation === "add_required_constraint" && v.mutation.expected_effect !== "rotation_authorization remains unadmitted") throw new Error("claim ceiling monotonicity contract missing");
  if (v.proposition_id === "CANONICALIZATION_ORDER") {
    if (v.mutation?.expected_effect !== "canonical commitment bytes unchanged") throw new Error("canonicalization metamorphic contract missing");
    const before = JSON.parse(v.mutation.before_wire);
    const after = JSON.parse(v.mutation.after_wire);
    if (canonicalAsciiJson(before) !== v.mutation.expected_canonical || canonicalAsciiJson(after) !== v.mutation.expected_canonical) throw new Error("canonicalization bytes differ after property reordering");
  }
  if (v.proposition_id === "CANONICAL_JSON") {
    let rejected = false;
    try { scanDuplicateObjectKeys(v.mutation.fixture); } catch (err) { rejected = String(err).includes("duplicate key"); }
    if (!rejected) throw new Error("duplicate-key fixture was not rejected");
  }
}

const s1 = JSON.parse(fs.readFileSync(path.join(ROOT, "s1_conformance_receipt_v1.example.json"), "utf8"));
for (const key of [
  "schema","profile","epoch_id","repository_id","repository_full_name","pull_request_number",
  "s0_dispatch_envelope_sha256","s0_authority_observation_sha256","dispatch_input_commitment_sha256",
  "subject_base_sha","subject_head_sha","subject_tree_sha","current_verifier_head_sha",
  "current_verifier_tree_sha","current_verifier_bundle_sha256","current_verifier_profile",
  "proposed_bundle_sha256","proposed_verifier_sha256","proposed_gate_sha256","registered_successor_profile",
  "s1_workflow_identity","s1_workflow_ref","s1_workflow_source_commit_sha","run_id","run_attempt",
  "run_head_sha","workflow_event","builder_id","builder_version","harness_source_commit_sha",
  "harness_bundle_sha256","corpus_sha256","candidate_code_executed","execution_outcome",
  "conformance_outcome","admitted_claims","nonclaims","receipt_sha256"
]) if (!(key in s1)) throw new Error("S1 receipt missing " + key);
if (s1.schema !== "mycelix.qual-001s.s1-conformance-receipt-v1") throw new Error("wrong S1 receipt schema");
if (s1.candidate_code_executed !== true) throw new Error("S1 receipt must record candidate execution");
if (!/^[0-9a-f]{64}$/.test(s1.receipt_sha256)) throw new Error("invalid S1 receipt commitment");
const s1Preimage = {...s1};
delete s1Preimage.receipt_sha256;
const computedS1Sha256 = createHash("sha256").update(canonicalAsciiJson(s1Preimage), "utf8").digest("hex");
if (computedS1Sha256 !== s1.receipt_sha256) throw new Error("S1 receipt commitment does not match canonical preimage");
if (s1.conformance_outcome === "PASS" && !s1.admitted_claims.includes("successor_conformance")) throw new Error("S1 PASS must admit successor_conformance");
if (s1.conformance_outcome !== "PASS" && s1.admitted_claims.includes("successor_conformance")) throw new Error("non-PASS S1 receipt cannot admit successor_conformance");
if (s1.s0_dispatch_envelope_sha256 !== s0.envelope_sha256) throw new Error("S1 receipt detached from S0 envelope");
if (s1.s0_authority_observation_sha256 !== ao.observation_sha256) throw new Error("S1 receipt detached from S0 authority observation");
for (const field of ["epoch_id","repository_id","repository_full_name","pull_request_number","s1_workflow_source_commit_sha"]) {
  if (s1[field] !== s0[field]) throw new Error("S1 receipt " + field + " disagrees with S0 envelope");
}
if (s1.dispatch_input_commitment_sha256 !== ao.dispatch_input_commitment_sha256) throw new Error("S1 receipt dispatch input commitment disagrees with S0 authority observation");
if (!Number.isInteger(s1.run_id) || s1.run_id < 1 || !Number.isInteger(s1.run_attempt) || s1.run_attempt < 1) throw new Error("invalid S1 run identity");

console.log("QUAL-001S evaluator B: PASS");
