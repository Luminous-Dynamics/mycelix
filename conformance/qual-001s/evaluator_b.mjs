#!/usr/bin/env node
// QUAL-001S evaluator B: independent semantic contract checker.
// It reconstructs expectations from proposition/theorem vocabulary and uses
// an independent duplicate-key scan rather than sharing evaluator A code.
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const raw = fs.readFileSync(path.join(ROOT, "qualification_vectors_v1.json"), "utf8");

function duplicateObjectKeys(json) {
  let i = 0;
  const walkObject = () => {
    if (json[i] !== "{") throw new Error("expected object");
    i++;
    const keys = new Set();
    while (true) {
      while (/s/.test(json[i] ?? "")) i++;
      if (json[i] === "}") { i++; return; }
      if (json[i] !== '"') throw new Error("expected object key");
      i++;
      let key = "";
      while (i < json.length) {
        const ch = json[i++];
        if (ch === "\") { key += json[i++] ?? ""; continue; }
        if (ch === '"') break;
        key += ch;
      }
      if (keys.has(key)) throw new Error(`duplicate key: ${key}`);
      keys.add(key);
      while (/s/.test(json[i] ?? "")) i++;
      if (json[i++] !== ":") throw new Error("expected colon");
      walkValue();
      while (/s/.test(json[i] ?? "")) i++;
      if (json[i] === "}") { i++; return; }
      if (json[i++] !== ",") throw new Error("expected comma");
    }
  };
  const walkValue = () => {
    while (/s/.test(json[i] ?? "")) i++;
    if (json[i] === "{") return walkObject();
    if (json[i] === "[") {
      i++;
      while (true) {
        while (/s/.test(json[i] ?? "")) i++;
        if (json[i] === "]") { i++; return; }
        walkValue();
        while (/s/.test(json[i] ?? "")) i++;
        if (json[i] === "]") { i++; return; }
        if (json[i++] !== ",") throw new Error("expected comma");
      }
    }
    if (json[i] === '"') {
      i++;
      while (i < json.length) {
        const ch = json[i++];
        if (ch === "\") i++;
        else if (ch === '"') return;
      }
      throw new Error("unterminated string");
    }
    const m = json.slice(i).match(/^(true|false|null|-?(?:0|[1-9]d*)(?:.d+)?(?:[eE][+-]?d+)?)/);
    if (!m) throw new Error("invalid JSON scalar");
    i += m[0].length;
  };
  walkObject();
  while (/s/.test(json[i] ?? "")) i++;
  if (i !== json.length) throw new Error("trailing data");
}

// Check the entire corpus before any parser can normalize duplicate keys.
try {
  duplicateObjectKeys(raw);
} catch (err) {
  throw new Error(`corpus duplicate-key rejection failed: ${err}`);
}

const corpus = JSON.parse(raw);

let fixtureRejected = false;
try {
  duplicateObjectKeys('{"authority_outcome":"NONE","authority_outcome":"AUTHORITY_AUTHORIZED"}');
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
]);

if (corpus.schema !== "mycelix.qual-001s.semantic-corpus-v1") throw new Error("wrong corpus schema");
if (corpus.corpus_id !== "QUAL-001S") throw new Error("wrong corpus id");
if (corpus.vectors.length !== 20) throw new Error("expected 20 vectors");

for (const v of corpus.vectors) {
  const pair = expect.get(v.proposition_id);
  if (!pair) throw new Error(`${v.vector_id}: no independent theorem expectation`);
  if (v.expected_state !== pair[0] || v.authority_outcome !== pair[1]) {
    throw new Error(
      `${v.vector_id}: expected ${pair[0]}/${pair[1]}, got ${v.expected_state}/${v.authority_outcome}`
    );
  }
  const requested = new Set(v.requested_claims);
  const admitted = new Set(v.admitted_claims);
  for (const claim of admitted) {
    if (!requested.has(claim)) throw new Error(`${v.vector_id}: admitted claim was never requested`);
  }
  if (v.kind === "availability" && v.expected_state !== "UNAVAILABLE") {
    throw new Error(`${v.vector_id}: availability vector must remain unavailable`);
  }
  if (v.proposition_id === "RECEIPT_CEILING" && v.expected_claims.includes("rotation_adoption")) {
    throw new Error("receipt ceiling violated");
  }
  if (v.proposition_id === "CLAIM_CEILING" && v.mutation?.operation === "add_required_constraint") {
    if (v.mutation.expected_effect !== "rotation_authorization remains unadmitted") {
      throw new Error("claim ceiling monotonicity contract missing");
    }
  }
  if (v.proposition_id === "CANONICAL_JSON") {
    let rejected = false;
    try {
      duplicateObjectKeys(v.mutation.fixture);
    } catch (err) {
      rejected = String(err).includes("duplicate key");
    }
    if (!rejected) throw new Error("duplicate-key fixture was not rejected");
  }
}

console.log("QUAL-001S evaluator B: PASS");
