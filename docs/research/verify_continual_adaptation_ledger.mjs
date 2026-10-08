#!/usr/bin/env node
/**
 * Independent policy-driven Node.js reference verifier for the ledger fixtures.
 * Research fixture only; not a production trust root.
 */
import fs from "node:fs";
import crypto from "node:crypto";

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
  return "{" + keys.map(key => JSON.stringify(key) + ":" + canonicalRecursive(value[key])).join(",") + "}";
}

function graphDigest(graph) {
  if (hasNumber(graph)) {
    throw new Error("numeric scalar found; extend RFC 8785-compatible number handling first");
  }
  return "sha256:" + crypto.createHash("sha256")
    .update(Buffer.from(canonicalRecursive(graph), "utf8"))
    .digest("hex");
}

function nodeIndex(graph) {
  const index = new Map();
  for (const node of graph.nodes) {
    if (!node || typeof node.id !== "string" || index.has(node.id)) return null;
    index.set(node.id, node);
  }
  return index;
}

function edgeSet(graph) {
  return new Set(graph.edges.map(edge => edge.join("|")));
}

function applyMutations(base, mutations) {
  const graph = structuredClone(base);

  for (const op of mutations) {
    const kind = op[0];
    const nodes = nodeIndex(graph);
    if (!nodes) throw new Error("invalid or duplicate node id");

    if (kind === "set") {
      const [, id, field, value] = op;
      const node = nodes.get(id);
      if (!node) throw new Error("unknown node: " + id);
      node[field] = value;
    } else if (kind === "remove_edge") {
      const target = JSON.stringify(op[1]);
      const index = graph.edges.findIndex(edge => JSON.stringify(edge) === target);
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

function verify(graph, policy) {
  const nodes = nodeIndex(graph);
  if (!nodes) return "unresolved";
  const edges = edgeSet(graph);

  for (const spec of policy.required_nodes) {
    const node = nodes.get(spec.id);
    if (!node || node.type !== spec.type) return "unresolved";
  }

  for (const edge of policy.required_edges) {
    const key = edge.join("|");
    if (!edges.has(key) || !nodes.has(edge[0]) || !nodes.has(edge[1])) return "unresolved";
  }

  for (const constraint of policy.equality_constraints) {
    const left = nodes.get(constraint.left[0])?.[constraint.left[1]];
    const right = nodes.get(constraint.right[0])?.[constraint.right[1]];
    if (JSON.stringify(left) !== JSON.stringify(right)) return constraint.failure_verdict;
  }

  for (const rule of policy.fixed_fields) {
    if (JSON.stringify(nodes.get(rule.node)?.[rule.field]) !== JSON.stringify(rule.value)) {
      return rule.failure_verdict;
    }
  }

  const conflict = policy.result_conflict;
  const qualifyingResults = graph.nodes.filter(node =>
    node.type === conflict.node_type &&
    (
      node.id === "result" ||
      edges.has(node.id + "|" + conflict.qualifies_edge_to + "|qualifies")
    )
  );
  if (qualifyingResults.length > conflict.max_qualifying_results) {
    return conflict.overflow_verdict;
  }

  const dependence = policy.derived_dependence;
  const hasDerivedCopy = graph.nodes.some(node =>
    node.type === dependence.node_type &&
    node[dependence.derived_from_field] === dependence.source_node
  );
  if (hasDerivedCopy) return dependence.verdict;

  return "qualified";
}

const policyPath = process.argv[2];
const corpusPath = process.argv[3];
if (!policyPath || !corpusPath) {
  console.error("usage: verify_continual_adaptation_ledger.mjs POLICY.json CORPUS.json");
  process.exit(2);
}

const policy = JSON.parse(fs.readFileSync(policyPath, "utf8"));
const corpus = JSON.parse(fs.readFileSync(corpusPath, "utf8"));
const failures = [];

for (const testCase of corpus.cases) {
  const graph = applyMutations(corpus.base_graph, testCase.mutation);
  const digest = graphDigest(graph);
  const verdict = verify(graph, policy);

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
