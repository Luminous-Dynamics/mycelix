#!/usr/bin/env node
/**
 * Independent Node.js verifier for shared-claim projection scope.
 * Research fixture only.
 */
import fs from "node:fs";
import crypto from "node:crypto";

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

function gitBlobSha(path) {
  const data = fs.readFileSync(path);
  const header = Buffer.from("blob " + data.length + "\0", "ascii");
  return crypto.createHash("sha1").update(Buffer.concat([header, data])).digest("hex");
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
  const rules = (policy.edge_schema_constraints ?? []).filter(rule => rule.relation === edge[2]);
  return rules.some(rule =>
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
    if (seen.has(key)) return null;
    seen.add(key);
  }
  const out = structuredClone(graph);
  out.nodes.sort((a,b) => a.id < b.id ? -1 : a.id > b.id ? 1 : 0);
  out.edges.sort((a,b) => canonical(a) < canonical(b) ? -1 : canonical(a) > canonical(b) ? 1 : 0);
  return out;
}

function project(graph, policy, claimId) {
  const normalized = normalize(graph, policy);
  if (!normalized) return null;
  const ids = new Set(normalized.nodes.map(n => n.id));
  if (!ids.has(claimId)) return null;
  const allowed = new Set(policy.claim_local_projection.relation_allowlist);
  const directions = policy.claim_local_projection.relation_directions;
  const included = new Set([claimId]);
  let changed = true;
  while (changed) {
    changed = false;
    for (const edge of normalized.edges) {
      if (!allowed.has(edge[2])) continue;
      const ds = new Set(directions[edge[2]] ?? []);
      if (ds.has("outgoing") && included.has(edge[0]) && !included.has(edge[1])) {
        included.add(edge[1]);
        changed = true;
      }
      if (ds.has("incoming") && included.has(edge[1]) && !included.has(edge[0])) {
        included.add(edge[0]);
        changed = true;
      }
    }
  }
  return {
    nodes: normalized.nodes.filter(n => included.has(n.id)),
    edges: normalized.edges.filter(e => included.has(e[0]) && included.has(e[1]) && allowed.has(e[2]))
  };
}

function apply(base, mutations) {
  const graph = structuredClone(base);
  for (const op of mutations) {
    const nodes = nodeIndex(graph);
    if (!nodes) throw new Error("invalid graph");
    if (op[0] === "add_node") {
      if (nodes.has(op[1].id)) throw new Error("duplicate node");
      graph.nodes.push(op[1]);
    } else if (op[0] === "add_edge") {
      graph.edges.push(op[1]);
    } else if (op[0] === "reverse_collection") {
      if (op[1] !== "nodes" && op[1] !== "edges") throw new Error("bad collection");
      graph[op[1]].reverse();
    } else {
      throw new Error("unsupported mutation");
    }
  }
  return graph;
}

const [expectedPolicy, policyPath, fixturePath] = process.argv.slice(2);
if (!expectedPolicy || !policyPath || !fixturePath) {
  console.error("usage: verify_claim_local_projection_scope.mjs EXPECTED_POLICY_BLOB_SHA POLICY.json FIXTURES.json");
  process.exit(2);
}
const policy = JSON.parse(fs.readFileSync(policyPath, "utf8"));
const fixture = JSON.parse(fs.readFileSync(fixturePath, "utf8"));
const actualPolicy = gitBlobSha(policyPath);
if (actualPolicy !== expectedPolicy || fixture.policy_binding?.git_blob_sha !== actualPolicy) {
  console.error("policy binding mismatch");
  process.exit(1);
}

const failures = [];
const base = fixture.base_graph;

for (const testCase of fixture.cases) {
  const baseDigest = digest(project(base, policy, testCase.claim));
  const current = project(apply(base, testCase.mutation), policy, testCase.claim);
  if (!current) {
    failures.push([testCase.case_id, "invalid-projection"]);
    continue;
  }
  const currentDigest = digest(current);
  const expected =
    testCase.expected === "projection-unchanged" ? currentDigest === baseDigest :
    testCase.expected === "projection-changed" ? currentDigest !== baseDigest :
    testCase.expected === "qualified" ? true : false;
  if (!expected) failures.push([testCase.case_id, testCase.expected, currentDigest, baseDigest]);
}

console.log("cases=" + fixture.cases.length + " failures=" + failures.length);
for (const failure of failures) console.log("FAIL", failure);
process.exit(failures.length ? 1 : 0);
