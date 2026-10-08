#!/usr/bin/env node
/** Research-only claim-local censoring-classification provenance verifier. */
import fs from "node:fs";
import crypto from "node:crypto";

const IDENTITY_FIELDS = [
  "attempt_id",
  "censoring_reason",
  "classification_epoch",
  "frozen_epoch",
  "policy_blob_sha",
  "basis_id",
  "revision",
  "claim_scope_anchor"
];
const RELATIONS = new Set([
  "requires","uses","classifies","frozen_by","supported_by","supersedes","invalidated_by"
]);

function lexCompare(a, b) {
  return a < b ? -1 : a > b ? 1 : 0;
}
function containsNonAscii(value) {
  if (typeof value === "string") return [...value].some(ch => ch.codePointAt(0) > 0x7f);
  if (Array.isArray(value)) return value.some(containsNonAscii);
  if (value && typeof value === "object") return Object.values(value).some(containsNonAscii);
  return false;
}
function canonical(value) {
  if (value === null || typeof value !== "object") return JSON.stringify(value);
  if (Array.isArray(value)) return "[" + value.map(canonical).join(",") + "]";
  return "{" + Object.keys(value).sort(lexCompare).map(k => JSON.stringify(k) + ":" + canonical(value[k])).join(",") + "}";
}
function digest(value) {
  return "sha256:" + crypto.createHash("sha256").update(Buffer.from(canonical(value), "utf8")).digest("hex");
}
function gitBlobSha(path) {
  const data = fs.readFileSync(path);
  const header = Buffer.from("blob " + data.length + "\0", "ascii");
  return crypto.createHash("sha1").update(Buffer.concat([header, data])).digest("hex");
}
function nodeIndex(graph) {
  if (!Array.isArray(graph.nodes)) return null;
  const out = new Map();
  for (const node of graph.nodes) {
    if (!node || typeof node.id !== "string" || out.has(node.id)) return null;
    out.set(node.id, node);
  }
  return out;
}
function classCommitment(node) {
  const input = {};
  for (const field of IDENTITY_FIELDS) input[field] = node[field];
  return digest(input);
}
function semanticNormalize(graph) {
  const out = structuredClone(graph);
  out.nodes.sort((a,b) => lexCompare(a.id, b.id));
  out.edges.sort((a,b) => lexCompare(canonical(a), canonical(b)));
  return out;
}
function validateStructure(graph, policy) {
  if (containsNonAscii(graph)) return [false, "non-ascii"];
  const nodes = nodeIndex(graph);
  if (!nodes) return [false, "structure"];
  const claimRoots = [...nodes.values()].filter(n => n.type === "Claim");
  if (claimRoots.length !== 1 || !nodes.has("claim") || nodes.get("claim").type !== "Claim") {
    return [false, "claim-root"];
  }
  const scopeCfg = policy.classification;
  if (scopeCfg.claim_scope_binding_required ?? true) {
    const claim = nodes.get("claim");
    const claimScopeAnchor = claim.claim_scope_anchor;
    const expectedScopeAnchor = digest({id: claim.id, type: claim.type});
    if (claimScopeAnchor !== expectedScopeAnchor) return [false, "claim-scope-root"];
    for (const node of nodes.values()) {
      if (["AttemptCensus", "Attempt", "CensoringClassification"].includes(node.type) &&
          node.claim_scope_anchor !== claimScopeAnchor) {
        return [false, "claim-scope-mismatch"];
      }
    }
  }
  if (!Array.isArray(graph.edges)) return [false, "edge-structure"];
  const seen = new Set();
  for (const edge of graph.edges) {
    if (!Array.isArray(edge) || edge.length !== 3 || edge.some(v => typeof v !== "string") ||
        !nodes.has(edge[0]) || !nodes.has(edge[1])) return [false, "dangling-edge"];
    const key = JSON.stringify(edge);
    if (seen.has(key)) return [false, "duplicate-edge"];
    seen.add(key);
    if (!RELATIONS.has(edge[2])) return [false, "unknown-relation"];
    const rule = policy.relations?.[edge[2]];
    if (!rule || !rule.source_types.includes(nodes.get(edge[0]).type) ||
        !rule.target_types.includes(nodes.get(edge[1]).type)) return [false, "typed-endpoint"];
  }

  const claimEntryEdges = graph.edges.filter(e =>
    e[0] === "claim" && e[2] === "requires" &&
    nodes.get(e[1]).type === "AttemptCensus"
  );
  if (claimEntryEdges.length !== 1) return [false, "claim-root-edge"];

  const order = policy.epoch_order;
  const cfg = policy.classification;
  for (const node of nodes.values()) {
    if (node.type !== "CensoringClassification") continue;
    const outgoing = graph.edges.filter(e => e[0] === node.id);
    const classifies = outgoing.filter(e => e[2] === "classifies");
    const frozenBy = outgoing.filter(e => e[2] === "frozen_by");
    const supportedBy = outgoing.filter(e => e[2] === "supported_by");
    const basisEdges = supportedBy.filter(e => nodes.get(e[1]).type === "ClassificationBasis");
    const supersedes = outgoing.filter(e => e[2] === "supersedes");

    if (cfg.one_target_per_classification && classifies.length !== 1) return [false, "target-cardinality"];
    if (cfg.one_policy_per_classification && frozenBy.length !== 1) return [false, "policy-cardinality"];
    if (cfg.one_basis_per_classification && basisEdges.length !== 1) return [false, "basis-cardinality"];
    if (classifies.length !== 1 || frozenBy.length !== 1 || basisEdges.length < 1) return [false, "required-provenance-missing"];

    const attempt = nodes.get(classifies[0][1]);
    const policyNode = nodes.get(frozenBy[0][1]);
    const basisNode = nodes.get(basisEdges[0][1]);

    if (node.attempt_id !== attempt.id) return [false, "attempt-binding"];
    if (cfg.class_must_match_attempt_reason && node.censoring_reason !== attempt.censoring_reason) return [false, "reason-mismatch"];
    if (node.basis_id !== basisNode.basis_commitment) return [false, "basis-binding"];
    if (node.policy_blob_sha !== policyNode.policy_blob_sha) return [false, "policy-binding"];
    if (node.frozen_epoch !== policyNode.frozen_epoch) return [false, "freeze-binding"];
    if (!order.includes(node.classification_epoch) || !order.includes(node.frozen_epoch) ||
        !order.includes(attempt.outcome_epoch)) return [false, "epoch-unknown"];

    if (cfg.policy_must_be_frozen_before_outcome &&
        order.indexOf(node.frozen_epoch) >= order.indexOf(attempt.outcome_epoch)) return [false, "policy-late"];
    if (cfg.classification_recorded_at_or_before_outcome &&
        order.indexOf(node.classification_epoch) > order.indexOf(attempt.outcome_epoch)) return [false, "classification-late"];

    if (node.commitment !== classCommitment(node)) return [false, "commitment-mismatch"];

    const revision = node.revision;
    if (typeof revision !== "string" || !/^[0-9]+$/.test(revision)) return [false, "revision-format"];
    if (revision === "0" && supersedes.length) return [false, "base-supersedes"];
    if (revision !== "0" && cfg.require_supersedes_for_nonzero_revision && !supersedes.length) return [false, "revision-without-supersession"];
    if (supersedes.length > 1) return [false, "multiple-predecessors"];

    for (const edge of supersedes) {
      const old = nodes.get(edge[1]);
      if (old.type !== "CensoringClassification") return [false, "supersedes-type"];
      const oldTargets = graph.edges.filter(e => e[0] === old.id && e[2] === "classifies").map(e => e[1]);
      if (oldTargets.length !== 1 || oldTargets[0] !== attempt.id) return [false, "supersedes-target"];
      const oldRevision = old.revision;
      if (cfg.require_sequential_revision &&
          typeof oldRevision === "string" && /^[0-9]+$/.test(oldRevision) &&
          Number(revision) !== Number(oldRevision) + 1) return [false, "revision-gap"];
    }

    if (cfg.result_cannot_support_classification &&
        supportedBy.some(e => nodes.get(e[1]).type === "Result")) return [false, "result-support"];
  }
  return [true, "ok"];
}
function claimLocalNodes(graph, policy) {
  const nodes = nodeIndex(graph);
  if (!nodes || !nodes.has("claim") || nodes.get("claim").type !== "Claim") return new Set();
  const included = new Set(["claim"]);
  let changed = true;
  while (changed) {
    changed = false;
    for (const edge of graph.edges) {
      if (!policy.relations?.[edge[2]] || !included.has(edge[0]) || included.has(edge[1])) continue;
      included.add(edge[1]);
      changed = true;
    }
  }
  return included;
}
function supersededIds(graph) {
  return new Set(graph.edges.filter(e => e[2] === "supersedes").map(e => e[1]));
}
function hasSupersedesCycle(graph) {
  const next = new Map();
  for (const e of graph.edges.filter(x => x[2] === "supersedes")) {
    if (next.has(e[0])) return true;
    next.set(e[0], e[1]);
  }
  const visiting = new Set(), visited = new Set();
  function visit(id) {
    if (visiting.has(id)) return true;
    if (visited.has(id)) return false;
    visiting.add(id);
    const n = next.get(id);
    if (n && visit(n)) return true;
    visiting.delete(id);
    visited.add(id);
    return false;
  }
  for (const id of next.keys()) if (visit(id)) return true;
  return false;
}
function verify(graph, policy, actualPolicySha, historyAnchors, objectIdentityAnchors) {
  const [ok, reason] = validateStructure(graph, policy);
  if (!ok) {
    return new Set([
      "reason-mismatch","basis-binding","policy-binding","freeze-binding",
      "commitment-mismatch","attempt-binding","result-support","revision-format"
    ]).has(reason) ? "unqualified" : "unresolved";
  }

  const nodes = nodeIndex(graph);
  const objectAnchors = objectIdentityAnchors || {};
  for (const [nodeId, anchor] of Object.entries(objectAnchors)) {
    const node = nodes.get(nodeId);
    if (!node) continue;
    if (node.commitment === undefined) return "unresolved";
    const identity = digest({id: nodeId, type: node.type, commitment: node.commitment});
    if (identity !== anchor) return "unqualified";
  }
  const local = claimLocalNodes(graph, policy);
  if (policy.classification.supersession_must_be_claim_local ?? true) {
    for (const edge of graph.edges) {
      if (edge[2] !== "supersedes") continue;
      if (!local.has(edge[0]) || !local.has(edge[1])) return "unresolved";
    }
  }
  if (hasSupersedesCycle(graph)) return "unresolved";

  if (policy.classification.active_classification_required) {
    const superseded = supersededIds(graph);
    for (const attempt of [...nodes.values()].filter(n => n.type === "Attempt" && local.has(n.id))) {
      const classifications = [...nodes.values()].filter(c =>
        c.type === "CensoringClassification" &&
        local.has(c.id) &&
        graph.edges.some(e => e[0] === c.id && e[1] === attempt.id && e[2] === "classifies")
      );
      const active = classifications.filter(c => !superseded.has(c.id));
      if (active.length !== 1) return "unresolved";
    }
  }

  if (policy.classification.history_is_immutable && policy.classification.base_revision_anchor_required) {
    for (const [cid, anchor] of Object.entries(historyAnchors)) {
      const node = nodes.get(cid);
      if (!node) return "unresolved";
      if (node.type !== "CensoringClassification") return "unresolved";
      if (node.revision !== "0") return "unresolved";
      if (node.commitment !== anchor) return "unqualified";
      if (!local.has(cid)) return "unresolved";
      const anchorTargets = graph.edges
        .filter(e => e[0] === cid && e[2] === "classifies")
        .map(e => e[1]);
      if (anchorTargets.length !== 1) return "unresolved";
      const anchorAttempt = nodes.get(anchorTargets[0]);
      if (!anchorAttempt || anchorAttempt.type !== "Attempt" ||
          !local.has(anchorTargets[0]) || node.attempt_id !== anchorTargets[0]) {
        return "unresolved";
      }
    }
  }

  for (const node of nodes.values()) {
    if (node.type === "ClassificationPolicy" && node.policy_blob_sha !== actualPolicySha) return "unqualified";
  }

  const superseded = supersededIds(graph);
  for (const node of nodes.values()) {
    if (node.type !== "CensoringClassification") continue;
    const invalidated = graph.edges.some(e => e[0] === node.id && e[2] === "invalidated_by");
    if (invalidated && !superseded.has(node.id)) return "unresolved";
  }

  return "qualified";
}
function applyMutations(base, mutations) {
  const out = structuredClone(base);
  for (const op of mutations) {
    if (op[0] === "set_node") {
      const node = out.nodes.find(n => n.id === op[1]);
      if (!node) throw new Error("unknown node: " + op[1]);
      node[op[2]] = op[3];
    } else if (op[0] === "add_node") out.nodes.push(structuredClone(op[1]));
    else if (op[0] === "add_edge") out.edges.push(structuredClone(op[1]));
    else if (op[0] === "remove_edge") {
      const target = JSON.stringify(op[1]);
      const i = out.edges.findIndex(e => JSON.stringify(e) === target);
      if (i < 0) throw new Error("missing edge");
      out.edges.splice(i, 1);
    } else if (op[0] === "remove_node") {
      const nodeId = op[1];
      out.nodes = out.nodes.filter(n => n.id !== nodeId);
      out.edges = out.edges.filter(e => e[0] !== nodeId && e[1] !== nodeId);
    } else if (op[0] === "reverse_collection") out[op[1]].reverse();
    else throw new Error("unknown mutation: " + op[0]);
  }
  return out;
}

const [expectedSha, policyPath, fixturePath, reportPath] = process.argv.slice(2);
if (!expectedSha || !policyPath || !fixturePath || !reportPath) {
  console.error("usage: verify_censoring_classification.mjs EXPECTED_POLICY_SHA POLICY.json FIXTURES.json REPORT.json");
  process.exit(2);
}
const actualSha = gitBlobSha(policyPath);
if (actualSha !== expectedSha) process.exit(1);
const policy = JSON.parse(fs.readFileSync(policyPath, "utf8"));
const fixture = JSON.parse(fs.readFileSync(fixturePath, "utf8"));
if (fixture.policy_binding?.git_blob_sha !== actualSha) process.exit(1);

const failures = [];
const rows = [];
for (const c of fixture.cases) {
  const graph = applyMutations(fixture.base_graph, c.mutation);
  const verdict = verify(graph, policy, actualSha, fixture.history_anchors, fixture.object_identity_anchors);
  const normalized = semanticNormalize(graph);
  const local = claimLocalNodes(graph, policy);
  const projection = {
    nodes: normalized.nodes.filter(n => local.has(n.id)),
    edges: normalized.edges.filter(e => local.has(e[0]) && local.has(e[1]))
  };
  rows.push({
    actual_verdict: verdict,
    case_id: c.case_id,
    expected_verdict: c.expected_verdict,
    claim_local_graph_digest_sha256: digest(projection)
  });
  if (verdict !== c.expected_verdict) failures.push([c.case_id, c.expected_verdict, verdict]);
}
const report = {
  cases: rows,
  failures,
  policy_blob_sha: actualSha,
  schema: "mycelix.continual-adaptation.censoring-classification-provenance-report.v2",
  status: "research-evidence-only"
};
fs.writeFileSync(reportPath, canonical(report) + "\n");
console.log("cases=" + rows.length + " failures=" + failures.length);
process.exit(failures.length ? 1 : 0);
