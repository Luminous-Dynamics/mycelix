#!/usr/bin/env node
/**
 * Independent dependency-light research verifier for adaptive selection/censoring.
 * Research fixture only; not a production trust root.
 */
import fs from "node:fs";
import crypto from "node:crypto";

function gitBlobSha(path) {
  const data = fs.readFileSync(path);
  const header = Buffer.from("blob " + data.length + "\0", "ascii");
  return crypto.createHash("sha1").update(Buffer.concat([header, data])).digest("hex");
}

function canonicalRecursive(value) {
  if (value === null || typeof value !== "object") return JSON.stringify(value);
  if (Array.isArray(value)) return "[" + value.map(canonicalRecursive).join(",") + "]";
  return "{" + Object.keys(value).sort().map(k => JSON.stringify(k) + ":" + canonicalRecursive(value[k])).join(",") + "}";
}

function digest(value) {
  return "sha256:" + crypto.createHash("sha256").update(Buffer.from(canonicalRecursive(value), "utf8")).digest("hex");
}

function semanticNormalize(caseData) {
  const out = structuredClone(caseData);
  out.attempts.sort((a,b) => a.id < b.id ? -1 : a.id > b.id ? 1 : 0);
  out.decision_points.sort((a,b) => a.id < b.id ? -1 : a.id > b.id ? 1 : 0);
  out.analysis.included_attempt_ids.sort();
  out.analysis.trigger_denominator_ids.sort();
  return out;
}

const ATTEMPT_REQUIRED = [
  "id","eligible_at_time_zero","time_zero_epoch","action","terminal_state",
  "observation_end_epoch","observation_at_horizon","censoring_reason","censoring_basis",
  "action_induced_censoring","outcome_dependent_censoring","horizon_completed",
  "failure_is_outcome"
];
const DECISION_REQUIRED = ["id","attempt_id","epoch","eligible","triggered"];

function idsUnique(items) {
  const ids = items.map(x => x?.id);
  return ids.every(x => typeof x === "string") && new Set(ids).size === ids.length;
}

function structureValid(caseData, protocol, policy) {
  if (!caseData || !Array.isArray(caseData.attempts) || !Array.isArray(caseData.decision_points) || !caseData.analysis) return false;
  if (policy.attempt_completeness.one_record_per_attempt && !idsUnique(caseData.attempts)) return false;
  if ((policy.decision_completeness.one_record_per_attempt ?? true) && !idsUnique(caseData.decision_points)) return false;

  const attemptIds = new Set(caseData.attempts.map(a => a.id));
  if (policy.attempt_completeness.require_exact_id_set) {
    if (attemptIds.size !== new Set(protocol.expected_attempt_ids).size) return false;
    for (const id of protocol.expected_attempt_ids) if (!attemptIds.has(id)) return false;
  } else if ([...attemptIds].some(id => !protocol.expected_attempt_ids.includes(id))) {
    return false;
  }

  const decisionIds = new Set(caseData.decision_points.map(d => d.id));
  if (policy.decision_completeness.require_exact_id_set) {
    if (decisionIds.size !== new Set(protocol.expected_decision_ids).size) return false;
    for (const id of protocol.expected_decision_ids) if (!decisionIds.has(id)) return false;
  } else if ([...decisionIds].some(id => !protocol.expected_decision_ids.includes(id))) {
    return false;
  }

  for (const a of caseData.attempts) {
    if (!ATTEMPT_REQUIRED.every(k => Object.prototype.hasOwnProperty.call(a,k))) return false;
    if (policy.time_zero.must_be_frozen_before_action && a.time_zero_epoch !== protocol.time_zero_epoch) return false;
    if (a.horizon_completed && a.observation_at_horizon !== true) return false;
    if (a.observation_end_epoch === protocol.horizon_epoch && a.observation_at_horizon !== true) return false;
    if (a.terminal_state === "OutcomeFailure" && a.failure_is_outcome !== true) return false;
    if (a.terminal_state === "OutcomeFailure" && a.censoring_reason !== "None") return false;
  }

  for (const d of caseData.decision_points) {
    if (!DECISION_REQUIRED.every(k => Object.prototype.hasOwnProperty.call(d,k))) return false;
    if (!attemptIds.has(d.attempt_id)) return false;
    if (d.epoch === protocol.time_zero_epoch && d.eligible !== true) return false;
  }

  const included = caseData.analysis.included_attempt_ids;
  const denominator = caseData.analysis.trigger_denominator_ids;
  if (!Array.isArray(included) || new Set(included).size !== included.length || [...included].some(id => !attemptIds.has(id))) return false;
  if (!Array.isArray(denominator) || new Set(denominator).size !== denominator.length || [...denominator].some(id => !decisionIds.has(id))) return false;
  return true;
}

function verify(caseData, policy, protocol) {
  if (!structureValid(caseData, protocol, policy)) return "unresolved";

  const attempts = new Map(caseData.attempts.map(a => [a.id, a]));
  const analysis = caseData.analysis;
  const estimand = analysis.estimand;
  const selectionRule = analysis.selection_rule;
  const included = new Set(analysis.included_attempt_ids);
  const eligibleAttempts = new Set(caseData.attempts.filter(a => a.eligible_at_time_zero === true).map(a => a.id));
  const eligibleDecisions = new Set(caseData.decision_points.filter(d => d.eligible === true).map(d => d.id));
  const denominator = new Set(analysis.trigger_denominator_ids);

  if (policy.analysis.pre_outcome_classification_required && analysis.classification_timing !== "pre_outcome") return "unqualified";
  if (policy.decision_completeness.eligible_decisions_must_be_counted &&
      (denominator.size !== eligibleDecisions.size || [...eligibleDecisions].some(x => !denominator.has(x)))) return "unqualified";

  if (analysis.complete_case_filter === true && estimand !== policy.analysis.complete_case_filter_allowed_only) return "unqualified";

  const survivorRule = policy.population.survivor_selection_rule;
  const fullRule = policy.population.full_episode_selection_rule;
  const fullEstimands = new Set(policy.analysis.full_population_estimands);

  if (selectionRule === survivorRule && estimand !== policy.analysis.survivor_selection_allowed_only) return "unqualified";
  if (selectionRule !== fullRule && selectionRule !== survivorRule) return "unqualified";

  if (fullEstimands.has(estimand)) {
    if (selectionRule !== fullRule || included.size !== eligibleAttempts.size || [...eligibleAttempts].some(x => !included.has(x))) return "unqualified";
  }
  if (estimand === "SurvivorConditionalValue") {
    if (selectionRule !== survivorRule || [...included].some(x => !eligibleAttempts.has(x))) return "unqualified";
  }
  if (estimand === "CensoringUnresolved") return "unresolved";

  let informativeWithoutAdjustment = false;
  let explicitBasisMissing = false;
  for (const a of attempts.values()) {
    const reason = a.censoring_reason;
    if (reason === "ActionInducedCensoring" && a.action_induced_censoring !== true) return "unresolved";
    if (reason === "OutcomeDependentCensoring" && a.outcome_dependent_censoring !== true) return "unresolved";
    if (reason === "None" && a.action_induced_censoring === true) return "unresolved";
    if (policy.censoring.requires_explicit_basis.includes(reason) && !a.censoring_basis) explicitBasisMissing = true;
    if (policy.censoring.always_requires_adjustment_or_block.includes(reason)) informativeWithoutAdjustment = true;
    if (a.terminal_state === "OutcomeFailure" && (!a.failure_is_outcome || a.censoring_reason !== "None")) return "unresolved";
  }

  let adjustedOk = false;
  if (analysis.adjustment && typeof analysis.adjustment === "object") {
    const req = policy.censoring.adjustment_requirements;
    adjustedOk = req.every(k => analysis.adjustment[k] !== null && analysis.adjustment[k] !== undefined)
      && analysis.adjustment.pre_specified_before_outcomes === true
      && analysis.adjustment.frozen === true;
  }

  if (explicitBasisMissing) return "unresolved";
  if (informativeWithoutAdjustment && !adjustedOk) return "unresolved";
  if (estimand === "CensoringAdjustedValue" && !adjustedOk) return "unresolved";
  for (const a of attempts.values()) {
    if (policy.censoring.requires_explicit_basis.includes(a.censoring_reason) && !a.censoring_basis) return "unresolved";
  }
  return "qualified";
}

function applyMutations(base, mutations) {
  const out = structuredClone(base);
  for (const op of mutations) {
    switch (op[0]) {
      case "set_attempt": {
        const node = out.attempts.find(a => a.id === op[1]);
        if (!node) throw new Error("unknown attempt");
        node[op[2]] = op[3];
        break;
      }
      case "set_analysis":
        out.analysis[op[1]] = op[2];
        break;
      case "set_decision": {
        const node = out.decision_points.find(d => d.id === op[1]);
        if (!node) throw new Error("unknown decision");
        node[op[2]] = op[3];
        break;
      }
      case "remove_attempt":
        out.attempts = out.attempts.filter(a => a.id !== op[1]);
        break;
      case "remove_decision":
        out.decision_points = out.decision_points.filter(d => d.id !== op[1]);
        break;
      case "reverse_collection":
        if (!["attempts","decision_points"].includes(op[1])) throw new Error("unsupported collection");
        out[op[1]].reverse();
        break;
      default:
        throw new Error("unknown mutation: " + op[0]);
    }
  }
  return out;
}

const [expectedPolicySha, policyPath, corpusPath, reportPath] = process.argv.slice(2);
if (!expectedPolicySha || !policyPath || !corpusPath || !reportPath) {
  console.error("usage: verify_selection_censoring.mjs EXPECTED_POLICY_SHA POLICY.json FIXTURES.json REPORT.json");
  process.exit(2);
}
const policyBytes = fs.readFileSync(policyPath);
const actualPolicySha = gitBlobSha(policyPath);
if (actualPolicySha !== expectedPolicySha) {
  console.error("policy binding mismatch");
  process.exit(1);
}
const policy = JSON.parse(policyBytes.toString("utf8"));
const corpus = JSON.parse(fs.readFileSync(corpusPath, "utf8"));
if (corpus.policy_binding?.git_blob_sha !== actualPolicySha) {
  console.error("fixture policy binding mismatch");
  process.exit(1);
}

const rows = [];
const failures = [];
for (const testCase of corpus.cases) {
  const mutated = applyMutations(corpus.base_case, testCase.mutation);
  const actual = verify(mutated, policy, corpus.protocol);
  const semantic = digest(semanticNormalize(mutated));
  rows.push({actual_verdict:actual, case_id:testCase.case_id, expected_verdict:testCase.expected_verdict, semantic_digest_sha256:semantic});
  if (actual !== testCase.expected_verdict) failures.push([testCase.case_id,testCase.expected_verdict,actual]);
}
fs.writeFileSync(reportPath, JSON.stringify({
  cases:rows,
  failures,
  policy_blob_sha:actualPolicySha,
  schema:"mycelix.continual-adaptation.selection-censoring-report.v1",
  status:"research-evidence-only"
}, null, 2)+"\n");
console.log(`cases=${rows.length} failures=${failures.length}`);
process.exit(failures.length ? 1 : 0);
