#!/usr/bin/env node
/**
 * Independent Node.js evaluator for generated continual-adaptation ledger properties.
 * Research fixture only; not a production trust root.
 */
import fs from "node:fs";
import crypto from "node:crypto";

function gitBlobSha(path) {
  const data = fs.readFileSync(path);
  const header = Buffer.from("blob " + data.length + "\\0", "ascii");
  return crypto.createHash("sha1").update(Buffer.concat([header, data])).digest("hex");
}

function hasNumber(v) {
  if (typeof v === "number") return true;
  if (Array.isArray(v)) return v.some(hasNumber);
  if (v && typeof v === "object") return Object.values(v).some(hasNumber);
  return false;
}

function canonical(v) {
  if (v === null || typeof v !== "object") return JSON.stringify(v);
  if (Array.isArray(v)) return "[" + v.map(canonical).join(",") + "]";
  return "{" + Object.keys(v).sort().map(k => JSON.stringify(k) + ":" + canonical(v[k])).join(",") + "}";
}

function digest(v) {
  if (hasNumber(v)) throw new Error("numeric scalar");
  return "sha256:" + crypto.createHash("sha256").update(Buffer.from(canonical(v), "utf8")).digest("hex");
}

function nodeIndex(graph) {
  const out = new Map();
  for (const node of graph.nodes) {
    if (!node || typeof node.id !== "string" || out.has(node.id)) return null;
    out.set(node.id, node);
  }
  return out;
}

function edgeSchemaValid(edge, nodes, policy) {
  const constraints = (policy.edge_schema_constraints ?? []).filter(rule => rule.relation === edge[2]);
  return constraints.some(rule =>
    rule.source_types.includes(nodes.get(edge[0])?.type) &&
    rule.target_types.includes(nodes.get(edge[1])?.type)
  );
}

function normalize(graph, policy) {
  const nodes = nodeIndex(graph);
  if (!nodes) return null;
  const seen = new Set();
  for (const edge of graph.edges) {
    if (!Array.isArray(edge) || edge.length !== 3 || !nodes.has(edge[0]) || !nodes.has(edge[1])) return null;
    if (!edgeSchemaValid(edge, nodes, policy)) return null;
    const key = edge.join("|");
    if (policy.graph_canonicalization.reject_duplicate_edges && seen.has(key)) return null;
    seen.add(key);
  }
  const out = structuredClone(graph);
  if (policy.graph_canonicalization.node_collection === "unordered-by-id") out.nodes.sort((a,b) => a.id < b.id ? -1 : a.id > b.id ? 1 : 0);
  if (policy.graph_canonicalization.edge_collection === "unordered-by-tuple") out.edges.sort((a,b) => canonical(a).localeCompare(canonical(b)));
  return out;
}

function semanticDigest(graph, policy) {
  const n = normalize(graph, policy);
  return n === null ? "invalid" : digest(n);
}

function claimLocalDigest(graph, policy) {
  const n = normalize(graph, policy);
  if (n === null) return "invalid";
  const root = policy.claim_local_projection.root;
  const allowed = new Set(policy.claim_local_projection.relation_allowlist);
  const directions = policy.claim_local_projection.relation_directions;
  const included = new Set([root]);
  let changed = true;
  while (changed) {
    changed = false;
    for (const edge of n.edges) {
      if (!allowed.has(edge[2])) continue;
      const relationDirections = new Set(directions[edge[2]] ?? []);
      if (relationDirections.has("outgoing") && included.has(edge[0]) && !included.has(edge[1])) {
        included.add(edge[1]); changed = true;
      }
      if (relationDirections.has("incoming") && included.has(edge[1]) && !included.has(edge[0])) {
        included.add(edge[0]); changed = true;
      }
    }
  }
  return digest({
    nodes: n.nodes.filter(n => included.has(n.id)),
    edges: n.edges.filter(e => included.has(e[0]) && included.has(e[1]) && allowed.has(e[2]))
  });
}

function verify(graph, policy) {
  if (normalize(graph, policy) === null) return "unresolved";
  const nodes = nodeIndex(graph);
  const edges = new Set(graph.edges.map(e => e.join("|")));
  for (const s of policy.required_nodes) {
    const n = nodes.get(s.id);
    if (!n || n.type !== s.type) return "unresolved";
  }
  for (const e of policy.required_edges) {
    if (!edges.has(e.join("|"))) return "unresolved";
  }
  for (const c of policy.equality_constraints) {
    if (JSON.stringify(nodes.get(c.left[0])?.[c.left[1]]) !== JSON.stringify(nodes.get(c.right[0])?.[c.right[1]])) return c.failure_verdict;
  }
  for (const r of policy.fixed_fields) {
    if (JSON.stringify(nodes.get(r.node)?.[r.field]) !== JSON.stringify(r.value)) return r.failure_verdict;
  }
  const cf = policy.result_conflict;
  const qualifying = graph.nodes.filter(n => n.type === cf.node_type && (n.id === "result" || edges.has(n.id + "|" + cf.qualifies_edge_to + "|qualifies")));
  if (qualifying.length > cf.max_qualifying_results) return cf.overflow_verdict;
  const dep = policy.derived_dependence;
  if (graph.nodes.some(n => n.type === dep.node_type && edges.has(n.id + "|" + dep.source_node + "|" + dep.edge_relation))) return dep.verdict;
  return "qualified";
}

function rotate(items, offset) {
  if (!items.length) return;
  const n = offset % items.length;
  items.push(...items.splice(0,n));
}

function applyMutations(base, mutations) {
  const graph = structuredClone(base);
  for (const op of mutations) {
    const nodes = nodeIndex(graph);
    if (!nodes) throw new Error("invalid node structure");
    switch (op[0]) {
      case "rotate_collection":
        if (op[1] !== "nodes" && op[1] !== "edges") throw new Error("bad collection");
        rotate(graph[op[1]], op[2]);
        break;
      case "set":
        if (!nodes.has(op[1])) throw new Error("unknown node");
        nodes.get(op[1])[op[2]] = op[3];
        break;
      case "add_node":
        if (nodes.has(op[1].id)) throw new Error("duplicate node");
        graph.nodes.push(op[1]);
        break;
      case "add_edge":
        graph.edges.push(op[1]);
        break;
      case "remove_edge": {
        const target = JSON.stringify(op[1]);
        const idx = graph.edges.findIndex(e => JSON.stringify(e) === target);
        if (idx < 0) throw new Error("missing edge");
        graph.edges.splice(idx,1);
        break;
      }
      default:
        throw new Error("unknown mutation: " + op[0]);
    }
  }
  return graph;
}

function checkCase(base, policy, testCase) {
  const graph = applyMutations(base, testCase.mutation);
  const baseSemantic = semanticDigest(base, policy);
  const baseLocal = claimLocalDigest(base, policy);
  const baseVerdict = verify(base, policy);
  const row = {
    case_id: testCase.case_id,
    property: testCase.property,
    verdict: verify(graph, policy),
    semantic_digest: semanticDigest(graph, policy),
    claim_local_digest: claimLocalDigest(graph, policy),
    base_verdict: baseVerdict,
    base_semantic_digest: baseSemantic,
    base_claim_local_digest: baseLocal
  };
  switch (testCase.property) {
    case "representation_invariance":
      row.status = row.verdict === baseVerdict && row.semantic_digest === baseSemantic && row.claim_local_digest === baseLocal ? "pass" : "fail";
      break;
    case "claim_local_invariance":
      row.status = row.verdict === baseVerdict && row.claim_local_digest === baseLocal && row.semantic_digest !== baseSemantic ? "pass" : "fail";
      break;
    case "identity_sensitivity":
      row.status = row.verdict !== "qualified" && row.semantic_digest !== baseSemantic ? "pass" : "fail";
      break;
    case "required_dependency_rejection":
      row.status = row.verdict === "unresolved" && row.semantic_digest !== "invalid" ? "pass" : "fail";
      break;
    case "endpoint_rejection":
      row.status = row.verdict === "unresolved" && row.semantic_digest === "invalid" ? "pass" : "fail";
      break;
    case "structural_rejection":
      row.status = row.verdict === "unresolved" && row.semantic_digest === "invalid" ? "pass" : "fail";
      break;
    case "ordered_array_sensitivity":
      row.status = row.verdict === "qualified" && row.semantic_digest !== baseSemantic && row.claim_local_digest !== baseLocal ? "pass" : "fail";
      break;
    case "provenance_dependence":
      row.status = row.verdict === "qualified-with-dependence" && row.claim_local_digest !== baseLocal ? "pass" : "fail";
      break;
    case "result_conflict":
      row.status = row.verdict === "unresolved" && row.claim_local_digest !== baseLocal ? "pass" : "fail";
      break;
    default:
      throw new Error("unknown property: " + testCase.property);
  }
  return row;
}

const [expectedPolicyBlobSha, policyPath, corpusPath, reportPath] = process.argv.slice(2);
if (!expectedPolicyBlobSha || !policyPath || !corpusPath || !reportPath) {
  console.error("usage: verify_generated_properties.mjs EXPECTED_POLICY_BLOB_SHA POLICY.json GENERATED.json REPORT.json");
  process.exit(2);
}
const policy = JSON.parse(fs.readFileSync(policyPath, "utf8"));
const corpus = JSON.parse(fs.readFileSync(corpusPath, "utf8"));
const actualPolicyBlobSha = gitBlobSha(policyPath);
if (
  actualPolicyBlobSha !== expectedPolicyBlobSha ||
  corpus.policy_binding?.git_blob_sha !== actualPolicyBlobSha
) {
  console.error("policy binding mismatch");
  process.exit(1);
}
const report = corpus.cases.map(c => checkCase(corpus.base_graph, policy, c));
const sortedReport = report
  .sort((a,b) => a.case_id < b.case_id ? -1 : a.case_id > b.case_id ? 1 : 0)
  .map(row => Object.fromEntries(Object.entries(row).sort(([a],[b]) => a.localeCompare(b))));
fs.writeFileSync(reportPath, JSON.stringify(sortedReport) + "\n");
const failures = report.filter(r => r.status !== "pass");
console.log("cases=" + report.length + " failures=" + failures.length);
for (const failure of failures.slice(0,10)) console.log("FAIL", failure);
process.exit(failures.length ? 1 : 0);
