#!/usr/bin/env node
// QUAL-001S evaluator B: independent semantic contract checker.
// It reconstructs expectations from proposition/theorem vocabulary and scans
// raw JSON before JSON.parse so duplicate-key normalization cannot hide input.
import fs from "node:fs";
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
      const key = json.slice(start, end);
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
const ao = JSON.parse(fs.readFileSync(path.join(ROOT, "s0_authority_observation_v1.example.json"), "utf8"));
if (ao.schema !== "mycelix.qual-001s.s0-authority-observation-v1") throw new Error("wrong S0 authority observation schema");
if (ao.event_type !== "workflow_dispatch") throw new Error("wrong S0 authority observation event type");
if (!["OBSERVED", "UNAVAILABLE", "CONTRADICTED"].includes(ao.observation_state)) throw new Error("invalid S0 observation state");
if (!["OBSERVED", "UNAVAILABLE", "CONTRADICTED"].includes(ao.request_authentication.state)) throw new Error("invalid request authentication state");
if (!["OBSERVED", "UNAVAILABLE", "CONTRADICTED"].includes(ao.actor_authorization.state)) throw new Error("invalid actor authorization state");
if (!["OBSERVED", "UNAVAILABLE", "CONTRADICTED"].includes(ao.event_authorization.state)) throw new Error("invalid event authorization state");
if (!["OBSERVED", "UNAVAILABLE", "CONTRADICTED"].includes(ao.workflow_source_authentication.state)) throw new Error("invalid workflow source authentication state");
if (!["ACCEPTED", "REJECTED", "UNOBSERVED"].includes(ao.dispatch_result)) throw new Error("invalid dispatch result");
if (!["OBSERVED", "UNOBSERVED", "CONTRADICTED"].includes(ao.run_attribution.state)) throw new Error("invalid run attribution state");

const s0 = JSON.parse(fs.readFileSync(path.join(ROOT, "s0_dispatch_envelope_v1.example.json"), "utf8"));
if (s0.schema !== "mycelix.qual-001s.s0-dispatch-envelope-v1") throw new Error("wrong S0 schema");
if (s0.canonicalization_profile !== "RFC8785-JCS-IJSON-v1") throw new Error("wrong S0 canonicalization profile");
if (typeof s0.epoch_id !== "string" || s0.epoch_id.length === 0) throw new Error("invalid S0 epoch");
if (s0.candidate_code_executed !== false) throw new Error("S0 candidate execution must be false");
if (typeof s0.envelope_sha256 !== "string" || s0.envelope_sha256.length !== 64) throw new Error("invalid S0 envelope commitment");

let fixtureRejected = false;
try {
  scanDuplicateObjectKeys('{"authority_outcome":"NONE","authority_outcome":"AUTHORITY_AUTHORIZED"}');
} catch (err) {
  fixtureRejected = String(err).includes("duplicate key");
}
if (!fixtureRejected) throw new Error("duplicate-key negative control was accepted");

function canonicalAsciiJson(value) {
  if (Array.isArray(value)) return "[" + value.map(canonicalAsciiJson).join(",") + "]";
  if (value !== null && typeof value === "object") {
    return "{" + Object.keys(value).sort().map((key) => JSON.stringify(key) + ":" + canonicalAsciiJson(value[key])).join(",") + "}";
  }
  return JSON.stringify(value);
}
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
]);

if (corpus.schema !== "mycelix.qual-001s.semantic-corpus-v1") throw new Error("wrong corpus schema");
if (corpus.corpus_id !== "QUAL-001S") throw new Error("wrong corpus id");
if (corpus.vectors.length !== 22) throw new Error("expected 22 vectors");

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

console.log("QUAL-001S evaluator B: PASS");
