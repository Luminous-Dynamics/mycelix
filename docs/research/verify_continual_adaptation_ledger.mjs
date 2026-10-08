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

function digest(value) {
  if (hasNumber(value)) throw new Error("numeric scalar found");
  return "sha256:" + crypto.createHash("sha256")
    .update(Buffer.from(canonicalRecursive(value), "utf8"))
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

function semanticNormalize(graph, policy) {
  const nodes = nodeIndex(graph);
  if (!nodes) return null;

  const seenEdges = new Set();
  for (const edge of graph.edges) {
    if (!Array.isArray(edge) || edge.length !== 3 || !nodes.has(edge[0]) || !nodes.has(edge[1])) {
      return null;
    }
    const key = edge.join("|");
    if (policy.graph_canonicalization.reject_duplicate_edges && seenEdges.has(key)) return null;
    seenEdges.add(key);
  }

  const normalized = structuredClone(graph);
  if (policy.graph_canonicalization.node_collection === "unordered-by-id") {
    normalized.nodes.sort((a,b) => a.id.localeCompare(b.id));
  }
  if (policy.graph_canonicalization.edge_collection === "unordered-by-tuple") {
    normalized.edges.sort((a,b) => canonicalRecursive(a).localeCompare(canonicalRecursive(b)));
  }
  return normalized;
}

function semanticDigest(graph, policy) {
  const normalized = semanticNormalize(graph, policy);
  return normalized === null ? "invalid" : digest(normalized);
}

function claimLocalProjection(graph, policy) {
  const normalized = semanticNormalize(graph, policy);
  if (normalized === null) return null;

  const root = policy.claim_local_projection.root;
  const allowed = new Set(policy.claim_local_projection.relation_allowlist);
  const included = new Set([root]);

  let changed = true;
  while (changed) {
    changed = false;
    for (const edge of normalized.edges) {
      if (!allowed.has(edge[2])) continue;
      const [left, right] = edge;
      if (included.has(left) && !included.has(right)) {
        included.add(right);
        changed = true;
      } else if (included.has(right) && !included.has(left)) {
        included.add(left);
        changed = true;
      }
    }
  }

  return {
    nodes: normalized.nodes.filter(node => included.has(node.id)),
    edges: normalized.edges.filter(
      edge => included.has(edge[0]) && included.has(edge[1]) && allowed.has(edge[2])
    )
  };
}

function claimLocalDigest(graph, policy) {
  const projection = claimLocalProjection(graph, policy);
  return projection === null ? "invalid" : digest(projection);
}

function validateGraphStructure(graph, policy) {
  const nodes = nodeIndex(graph);
  if (!nodes) return false;
  const seenEdges = new Set();
  for (const edge of graph.edges) {
    if (!Array.isArray(edge) || edge.length !== 3 || !nodes.has(edge[0]) || !nodes.has(edge[1])) {
      return false;
    }
    const key = edge.join("|");
    if (policy.graph_canonicalization.reject_duplicate_edges && seenEdges.has(key)) {
      return false;
    }
    seenEdges.add(key);
  }
  return true;
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
      if (index < 0) throw new Error("missing edge");
      graph.edges.splice(index, 1);
    } else if (kind === "add_node") {
      const node = op[1];
      if (nodes.has(node.id)) throw new Error("duplicate node: " + node.id);
      graph.nodes.push(node);
    } else if (kind === "add_edge") {
      graph.edges.push(op[1]);
    } else if (kind === "reverse_collection") {
      if (op[1] !== "nodes" && op[1] !== "edges") throw new Error("unsupported collection");
      graph[op[1]].reverse();
    } else {
      throw new Error("unknown mutation operation: " + kind);
    }
  }
  return graph;
}

function verify(graph, policy) {
  if (!validateGraphStructure(graph, policy)) return "unresolved";
  const nodes = nodeIndex(graph);
  if (!nodes) return "unresolved";
  const edges = edgeSet(graph);

  for (const spec of policy.required_nodes) {
    const node = nodes.get(spec.id);
    if (!node || node.type !== spec.type) return "unresolved";
  }

  for (const edge of policy.required_edges) {
    if (!edges.has(edge.join("|")) || !nodes.has(edge[0]) || !nodes.has(edge[1])) {
      return "unresolved";
    }
  }

  for (const c of policy.equality_constraints) {
    const left = nodes.get(c.left[0])?.[c.left[1]];
    const right = nodes.get(c.right[0])?.[c.right[1]];
    if (JSON.stringify(left) !== JSON.stringify(right)) return c.failure_verdict;
  }

  for (const rule of policy.fixed_fields) {
    if (JSON.stringify(nodes.get(rule.node)?.[rule.field]) !== JSON.stringify(rule.value)) {
      return rule.failure_verdict;
    }
  }

  const conflict = policy.result_conflict;
  const qualifying = graph.nodes.filter(node =>
    node.type === conflict.node_type &&
    (node.id === "result" || edges.has(node.id + "|" + conflict.qualifies_edge_to + "|qualifies"))
  );
  if (qualifying.length > conflict.max_qualifying_results) return conflict.overflow_verdict;

  const dependence = policy.derived_dependence;
  if (graph.nodes.some(node =>
    node.type === dependence.node_type &&
    edges.has(node.id + "|" + dependence.source_node + "|" + dependence.edge_relation)
  )) return dependence.verdict;

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
  const serialized = digest(graph);
  const semantic = semanticDigest(graph, policy);
  const claimLocal = claimLocalDigest(graph, policy);
  const verdict = verify(graph, policy);

  for (const [field, actual] of [
    ["expected_graph_digest_sha256", serialized],
    ["expected_semantic_graph_digest_sha256", semantic],
    ["expected_claim_local_graph_digest_sha256", claimLocal],
    ["expected_verdict", verdict]
  ]) {
    if (actual !== testCase[field]) {
      failures.push([testCase.case_id, field, testCase[field], actual]);
    }
  }
}

console.log("cases=" + corpus.cases.length + " failures=" + failures.length);
for (const failure of failures) console.log("FAIL", failure);
process.exit(failures.length ? 1 : 0);
