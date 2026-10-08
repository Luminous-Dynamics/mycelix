#!/usr/bin/env node
/**
 * Independent Node.js reference verifier for the continual-adaptation ledger fixtures.
 * Research fixture only; not a production trust root.
 */

import fs from "node:fs";
import crypto from "node:crypto";

const REQUIRED_EDGES = new Set([
  "result|claim|qualifies",
  "result|evaluator|generated_by",
  "evaluator|reference|uses",
  "evaluator|attempts|uses",
  "attempts|campaign|derived_from",
  "campaign|subject|applies_to",
  "campaign|intervention|uses",
  "campaign|observation|uses",
  "campaign|measurement|uses",
  "claim|transport|requires",
  "claim|freshness|requires",
  "claim|subject|applies_to"
]);

function hasNumber(value) {
  if (typeof value === "number") return true;
  if (Array.isArray(value)) return value.some(hasNumber);
  if (value && typeof value === "object") return Object.values(value).some(hasNumber);
  return false;
}

function canonicalRecursive(value) {
  if (value === null || typeof value !== "object") return JSON.stringify(value);
  if (Array.isArray(value)) return "[" + value.map(canonicalRecursive).join(",") + "]";
  const keys = Object.keys(value).sort();
  return "{" + keys.map(k => JSON.stringify(k) + ":" + canonicalRecursive(value[k])).join(",") + "}";
}

function graphDigest(graph) {
  if (hasNumber(graph)) {
    throw new Error("numeric scalar found; extend RFC 8785-compatible number handling first");
  }
  const bytes = Buffer.from(canonicalRecursive(graph), "utf8");
  return "sha256:" + crypto.createHash("sha256").update(bytes).digest("hex");
}

function applyMutations(base, mutations) {
  const graph = structuredClone(base);

  for (const op of mutations) {
    const kind = op[0];
    const nodes = new Map(graph.nodes.map(n => [n.id, n]));

    if (kind === "set") {
      const [, id, field, value] = op;
      const node = nodes.get(id);
      if (!node) throw new Error("unknown node: " + id);
      node[field] = value;
    } else if (kind === "remove_edge") {
      const target = JSON.stringify(op[1]);
      const index = graph.edges.findIndex(e => JSON.stringify(e) === target);
      if (index < 0) throw new Error("missing edge: " + target);
      graph.edges.splice(index, 1);
    } else if (kind === "add_node") {
      const node = op[1];
      if (nodes.has(node.id)) throw new Error("duplicate node: " + node.id);
      graph.nodes.push(node);
    } else if (kind === "add_edge") {
      graph.edges.push(op[1]);
    } else {
      throw new Error("unknown mutation operation: " + kind);
    }
  }

  return graph;
}

function verify(graph) {
  const nodes = new Map(graph.nodes.map(n => [n.id, n]));
  const edges = new Set(graph.edges.map(e => e.join("|")));
  const required = [
    "claim", "subject", "campaign", "attempts", "evaluator",
    "reference", "intervention", "observation", "measurement",
    "result", "transport", "freshness"
  ];

  for (const id of required) {
    if (!nodes.has(id)) return "unresolved";
  }
  for (const edge of REQUIRED_EDGES) {
    if (!edges.has(edge)) return "unresolved";
  }

  if (nodes.get("claim").subject !== nodes.get("subject").commitment) return "unqualified";
  if (nodes.get("claim").target !== nodes.get("transport").target) return "unqualified";

  const evaluator = nodes.get("evaluator");
  if (evaluator.commitment !== "E1" || evaluator.state !== "fresh") return "unqualified";
  if (nodes.get("intervention").semantic_id !== "U1") return "unqualified";
  if (nodes.get("measurement").semantic_id !== "M1") return "unqualified";

  const qualifyingResults = graph.nodes.filter(
    n => n.type === "Result" && (n.id === "result" || edges.has(n.id + "|claim|qualifies"))
  );
  if (qualifyingResults.length > 1) return "unresolved";

  const derivedCopies = graph.nodes.filter(
    n => n.type === "Result" && n.derived_from === "result"
  );
  if (derivedCopies.length > 0) return "qualified-with-dependence";

  return "qualified";
}

const path = process.argv[2];
if (!path) {
  console.error("usage: verify_continual_adaptation_ledger.mjs CORPUS.json");
  process.exit(2);
}

const corpus = JSON.parse(fs.readFileSync(path, "utf8"));
const failures = [];

for (const testCase of corpus.cases) {
  const graph = applyMutations(corpus.base_graph, testCase.mutation);
  const digest = graphDigest(graph);
  const verdict = verify(graph);

  if (digest !== testCase.expected_graph_digest_sha256) {
    failures.push([testCase.case_id, "digest", testCase.expected_graph_digest_sha256, digest]);
  }
  if (verdict !== testCase.expected_verdict) {
    failures.push([testCase.case_id, "verdict", testCase.expected_verdict, verdict]);
  }
}

console.log("cases=" + corpus.cases.length + " failures=" + failures.length);
for (const failure of failures) console.log("FAIL", failure);

process.exit(failures.length ? 1 : 0);
