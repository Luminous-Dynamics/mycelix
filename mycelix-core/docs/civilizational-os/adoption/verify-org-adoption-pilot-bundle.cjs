#!/usr/bin/env node
'use strict';

// Zero-dependency cross-invariant verifier for the portable organization adoption
// pilot bundle. JSON Schema validation is a separate lane; this evaluator checks
// invariants requiring relationships across fields, rows, and evidence references.

const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');

function fail(message) {
  process.stderr.write(`FAIL: ${message}\n`);
  process.exit(1);
}
function requireCondition(ok, message) {
  if (!ok) fail(message);
}
function readJson(filename) {
  try {
    return JSON.parse(fs.readFileSync(filename, 'utf8'));
  } catch (error) {
    fail(`cannot parse ${filename}: ${error.message}`);
  }
}
function sha256(value) {
  return crypto.createHash('sha256').update(value).digest('hex');
}
function canonical(value) {
  if (Array.isArray(value)) return '[' + value.map(canonical).join(',') + ']';
  if (value !== null && typeof value === 'object') {
    return '{' + Object.keys(value).sort()
      .map(key => JSON.stringify(key) + ':' + canonical(value[key])).join(',') + '}';
  }
  return JSON.stringify(value);
}
function hasSha(value) {
  return typeof value === 'string' && /^[0-9a-f]{64}$/.test(value);
}
function expectedMetricDisposition(metric) {
  switch (metric.comparison_rule) {
    case 'candidate_gte_baseline': return metric.candidate_value >= metric.baseline_value ? 'pass' : 'fail';
    case 'candidate_gt_baseline': return metric.candidate_value > metric.baseline_value ? 'pass' : 'fail';
    case 'candidate_lte_baseline': return metric.candidate_value <= metric.baseline_value ? 'pass' : 'fail';
    case 'candidate_zero': return metric.candidate_value === 0 ? 'pass' : 'fail';
    case 'informational': return 'indeterminate';
    default: return 'invalid';
  }
}

function validateBundle(bundle) {
  const errors = [];
  const add = (condition, message) => { if (!condition) errors.push(message); };
  if (!bundle || typeof bundle !== 'object' || Array.isArray(bundle)) {
    return ['bundle must be a JSON object'];
  }

  add(bundle.schema_version === '1.0.0', 'unsupported schema_version');
  add(typeof bundle.bundle_id === 'string' && /^[a-z0-9]+(?:-[a-z0-9]+)*$/.test(bundle.bundle_id), 'invalid bundle_id');
  add(typeof bundle.profile_id === 'string' && /^[a-z0-9]+(?:-[a-z0-9]+)*$/.test(bundle.profile_id), 'invalid profile_id');
  add(typeof bundle.pilot_id === 'string' && /^[a-z0-9]+(?:-[a-z0-9]+)*$/.test(bundle.pilot_id), 'invalid pilot_id');
  add(bundle.relationship_status === 'not_recorded', 'bundle must not assert a relationship');
  add(bundle.relationship_claim_guard === 'No partnership, customer relationship, endorsement, or integration is implied by this bundle.',
    'relationship claim guard missing or changed');
  add(bundle.effects_mode === 'disabled', 'reference bundle must have external effects disabled');
  add(bundle.claim_ceiling === 'SyntheticReferenceOnly', 'example fixture must retain the synthetic-reference claim ceiling');
  add(bundle.data_classification === 'synthetic', 'reference bundle must be synthetic');
  add(Array.isArray(bundle.nonclaims) && bundle.nonclaims.length >= 2, 'at least two nonclaims are required');

  const manifest = bundle.input_manifest;
  add(!!manifest && typeof manifest === 'object' && !Array.isArray(manifest), 'input_manifest missing');
  if (!manifest || typeof manifest !== 'object' || Array.isArray(manifest)) return errors.concat('input_manifest invalid');

  add(manifest.data_classification === bundle.data_classification, 'input and bundle data classifications differ');
  add(manifest.rights_disposition === 'synthetic_no_external_rights_required', 'synthetic rights disposition must be explicit');
  add(Array.isArray(manifest.source_entries) && manifest.source_entries.length > 0, 'source entries missing');
  add(Array.isArray(manifest.events) && manifest.events.length > 0, 'event fixture missing');

  const { manifest_sha256: declaredManifestSha, ...manifestBody } = manifest;
  const computedManifestSha = sha256(canonical(manifestBody));
  add(hasSha(declaredManifestSha) && declaredManifestSha === computedManifestSha, 'input manifest digest mismatch');

  const sourceById = new Map();
  const sourceRecordIds = new Map();
  for (const [index, entry] of (manifest.source_entries || []).entries()) {
    const label = `source_entries[${index}]`;
    add(entry && typeof entry.source_id === 'string' && entry.source_id.length >= 3, `${label} source_id missing`);
    if (!entry || typeof entry.source_id !== 'string') continue;
    add(!sourceById.has(entry.source_id), `duplicate source_id: ${entry.source_id}`);
    sourceById.set(entry.source_id, entry);
    add(hasSha(entry.content_sha256), `${label} content digest malformed`);
    add(typeof entry.rights_disposition === 'string' && entry.rights_disposition.length > 0,
      `${label} rights disposition missing`);
    add(Array.isArray(entry.record_ids) && entry.record_ids.length > 0, `${label} record_ids missing`);
    if (entry.source_kind === 'synthetic_fixture') {
      add(entry.rights_disposition === 'synthetic_no_external_rights_required',
        `${label} synthetic rights disposition mismatch`);
    } else {
      add(typeof entry.source_url === 'string' && entry.source_url.startsWith('https://'),
        `${label} non-synthetic source requires an HTTPS source_url`);
      add(entry.rights_disposition !== 'synthetic_no_external_rights_required',
        `${label} non-synthetic source cannot use synthetic rights disposition`);
    }
    for (const recordId of entry.record_ids || []) {
      add(!sourceRecordIds.has(recordId), `record id attributed to multiple source entries: ${recordId}`);
      sourceRecordIds.set(recordId, entry.source_id);
    }
  }

  const eventById = new Map();
  const sequences = new Set();
  for (const [index, event] of (manifest.events || []).entries()) {
    const label = `events[${index}]`;
    add(event && typeof event.event_id === 'string' && event.event_id.length >= 2, `${label} event_id missing`);
    if (!event || typeof event.event_id !== 'string') continue;
    add(!eventById.has(event.event_id), `duplicate event_id: ${event.event_id}`);
    eventById.set(event.event_id, event);
    add(Number.isInteger(event.sequence) && event.sequence >= 0, `${label} sequence invalid`);
    add(!sequences.has(event.sequence), `duplicate event sequence: ${event.sequence}`);
    sequences.add(event.sequence);
    add(sourceById.has(event.source_id), `${label} references unknown source_id`);
    add(sourceRecordIds.get(event.event_id) === event.source_id,
      `${label} source mapping missing or inconsistent`);
  }
  for (const recordId of sourceRecordIds.keys()) {
    add(eventById.has(recordId), `source entry references missing event: ${recordId}`);
  }

  // Baseline and candidate must run against the same inputs, resources, authority,
  // measurement window, and declared measurement profile.
  const comparison = bundle.comparison;
  add(!!comparison && comparison.baseline && comparison.candidate, 'comparison arms missing');
  if (comparison && comparison.baseline && comparison.candidate) {
    const baseline = comparison.baseline;
    const candidate = comparison.candidate;
    add(baseline.input_manifest_sha256 === declaredManifestSha, 'baseline input digest differs from fixture');
    add(candidate.input_manifest_sha256 === declaredManifestSha, 'candidate input digest differs from fixture');
    add(baseline.input_manifest_sha256 === candidate.input_manifest_sha256, 'baseline/candidate inputs are not matched');
    add(baseline.resource_budget_sha256 === candidate.resource_budget_sha256, 'baseline/candidate resource budgets differ');
    add(baseline.authority_budget_sha256 === candidate.authority_budget_sha256, 'baseline/candidate authority budgets differ');
    add(hasSha(baseline.implementation_sha256) && hasSha(candidate.implementation_sha256),
      'baseline/candidate implementation digest malformed');
    add(hasSha(baseline.resource_budget_sha256) && hasSha(baseline.authority_budget_sha256),
      'budget digest malformed');
    add(typeof comparison.measurement_profile_id === 'string' && comparison.measurement_profile_id.length >= 3,
      'measurement_profile_id missing');
    add(typeof comparison.hardware_profile_id === 'string' && comparison.hardware_profile_id.length >= 3,
      'hardware_profile_id missing');
    const start = Date.parse(comparison.window_start_utc);
    const end = Date.parse(comparison.window_end_utc);
    add(Number.isFinite(start) && Number.isFinite(end) && start <= end, 'measurement window invalid');
  }

  const claimIds = new Set();
  for (const [index, claim] of (bundle.evidence_claims || []).entries()) {
    add(claim && typeof claim.claim_id === 'string', `evidence_claims[${index}] invalid`);
    if (!claim) continue;
    add(!claimIds.has(claim.claim_id), `duplicate claim_id: ${claim.claim_id}`);
    claimIds.add(claim.claim_id);
    add(Array.isArray(claim.evidence_refs) && claim.evidence_refs.length > 0, `claim ${claim.claim_id} has no evidence refs`);
    for (const ref of claim.evidence_refs || []) {
      add(eventById.has(ref), `claim ${claim.claim_id} references missing event ${ref}`);
    }
    if (claim.claim_type === 'physical_delivery_verified' && claim.disposition === 'supported_within_fixture') {
      add((manifest.events || []).some(event =>
        event.event_type === 'physical_delivery_measurement' && event.disposition === 'observed'),
      'physical delivery cannot be supported without a declared physical-delivery measurement');
    }
    if (claim.claim_type === 'regulatory_compliance' && claim.disposition === 'supported_within_fixture') {
      add(false, 'synthetic event fixtures cannot establish regulatory compliance');
    }
    if (['physical_delivery_verified', 'regulatory_compliance', 'product_authenticity'].includes(claim.claim_type)) {
      add(claim.disposition !== 'supported_within_fixture' || bundle.claim_ceiling !== 'SyntheticReferenceOnly',
        `claim ${claim.claim_id} exceeds the bundle claim ceiling`);
    }
  }

  const metricIds = new Set();
  for (const [index, metric] of (bundle.metrics || []).entries()) {
    add(metric && typeof metric.metric_id === 'string', `metrics[${index}] invalid`);
    if (!metric) continue;
    add(!metricIds.has(metric.metric_id), `duplicate metric_id: ${metric.metric_id}`);
    metricIds.add(metric.metric_id);
    add(Number.isFinite(metric.baseline_value) && Number.isFinite(metric.candidate_value),
      `metric ${metric.metric_id} values must be finite numbers`);
    add(metric.disposition === expectedMetricDisposition(metric),
      `metric ${metric.metric_id} disposition disagrees with comparison_rule`);
  }

  const controls = bundle.negative_controls || [];
  const controlIds = new Set();
  const requiredControls = new Set([
    'rejected_duplicate', 'rejected_subject_mismatch', 'claim_indeterminate',
    'rejected_digest_mismatch', 'rejected_missing_rights'
  ]);
  const observedControls = new Set();
  for (const [index, control] of controls.entries()) {
    add(control && typeof control.control_id === 'string', `negative_controls[${index}] invalid`);
    if (!control) continue;
    add(!controlIds.has(control.control_id), `duplicate control_id: ${control.control_id}`);
    controlIds.add(control.control_id);
    const shouldPass = control.expected_disposition === control.observed_disposition &&
      control.observed_disposition !== 'accepted_incorrectly';
    add(control.passed === shouldPass, `control ${control.control_id} pass flag incorrect`);
    if (shouldPass) observedControls.add(control.expected_disposition);
  }
  for (const requiredControl of requiredControls) {
    add(observedControls.has(requiredControl), `required negative control not evidenced: ${requiredControl}`);
  }

  const exportTest = bundle.export_test;
  add(!!exportTest && exportTest.performed === true, 'export round-trip test not performed');
  if (exportTest) {
    add(hasSha(exportTest.expected_sha256) && hasSha(exportTest.observed_sha256), 'export digest malformed');
    add(exportTest.expected_sha256 === declaredManifestSha, 'export expected digest differs from source manifest');
    add(exportTest.observed_sha256 === exportTest.expected_sha256 && exportTest.round_trip_matches === true,
      'export/round-trip digest mismatch');
  }

  const rollback = bundle.rollback_test;
  add(!!rollback && rollback.performed === true, 'rollback test not performed');
  if (rollback) {
    add(rollback.result === 'rollback_completed', 'rollback was not completed');
    add(rollback.retained_external_effects === false, 'rollback leaves unaccounted external effects');
  }

  add(bundle.result && bundle.result.disposition === 'ReferenceFixtureConforms',
    'bundle result must remain a fixture-only conformance result');
  add(bundle.result && bundle.result.bundle_scope === 'synthetic_reference_fixture_only',
    'bundle scope must remain fixture-only');
  add(bundle.result && hasSha(bundle.result.evaluator_sha256), 'evaluator digest malformed');

  return errors;
}

function assertRejects(mutator, label, fixture) {
  const changed = JSON.parse(JSON.stringify(fixture));
  mutator(changed);
  const errors = validateBundle(changed);
  if (errors.length === 0) {
    throw new Error(`mutation was not rejected: ${label}`);
  }
  return { mutation: label, rejected: true, reasons: errors.length };
}

const root = __dirname;
const bundlePath = process.argv[2] || path.join(root, 'org-adoption-pilot-bundle-example-v1.json');
const schemaPath = process.argv[3] || path.join(root, 'org-adoption-pilot-bundle-v1.schema.json');
const bundleBytes = fs.readFileSync(bundlePath);
const schemaBytes = fs.readFileSync(schemaPath);
const bundle = JSON.parse(bundleBytes.toString('utf8'));
const schema = JSON.parse(schemaBytes.toString('utf8'));
requireCondition(schema.$schema === 'https://json-schema.org/draft/2020-12/schema',
  'schema must declare JSON Schema Draft 2020-12');

const errors = validateBundle(bundle);
if (errors.length) fail(errors.join('\n'));

const mutationResults = [
  assertRejects(b => { b.comparison.candidate.input_manifest_sha256 = 'dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd'; }, 'candidate-input-digest-mismatch', bundle),
  assertRejects(b => { b.comparison.candidate.resource_budget_sha256 = 'dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd'; }, 'resource-budget-mismatch', bundle),
  assertRejects(b => { b.comparison.candidate.authority_budget_sha256 = 'dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd'; }, 'authority-budget-mismatch', bundle),
  assertRejects(b => { b.input_manifest.manifest_sha256 = 'dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd'; }, 'manifest-tamper', bundle),
  assertRejects(b => { b.evidence_claims[0].evidence_refs[0] = 'evt-missing-999'; }, 'missing-evidence-reference', bundle),
  assertRejects(b => { b.evidence_claims.find(c => c.claim_type === 'physical_delivery_verified').disposition = 'supported_within_fixture'; }, 'scan-promoted-to-physical-delivery', bundle),
  assertRejects(b => { b.negative_controls[0].observed_disposition = 'accepted_incorrectly'; }, 'accepted-duplicate-replay', bundle),
  assertRejects(b => { b.export_test.observed_sha256 = 'dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd'; }, 'export-roundtrip-mismatch', bundle),
  assertRejects(b => { b.effects_mode = 'enabled'; }, 'external-effects-enabled', bundle),
  assertRejects(b => { b.relationship_status = 'partner'; }, 'fabricated-relationship-status', bundle),
  assertRejects(b => { b.rollback_test.retained_external_effects = true; }, 'rollback-retained-effects', bundle),
  assertRejects(b => { b.input_manifest.events.push({ ...b.input_manifest.events[0] }); }, 'duplicate-event-replay', bundle)
];

const receipt = {
  verifier: 'verify-org-adoption-pilot-bundle.cjs',
  qualification_result: 'ReferenceFixtureConforms',
  claim_ceiling: bundle.claim_ceiling,
  bundle_id: bundle.bundle_id,
  profile_id: bundle.profile_id,
  event_count: bundle.input_manifest.events.length,
  source_count: bundle.input_manifest.source_entries.length,
  evidence_claim_count: bundle.evidence_claims.length,
  metric_count: bundle.metrics.length,
  negative_controls_checked: bundle.negative_controls.length,
  negative_controls_passed: bundle.negative_controls.filter(control => control.passed).length,
  internal_mutation_controls: mutationResults,
  bundle_sha256: sha256(bundleBytes),
  schema_sha256: sha256(schemaBytes),
  evaluator_sha256: sha256(fs.readFileSync(__filename)),
  input_manifest_sha256: bundle.input_manifest.manifest_sha256,
  matched_inputs: bundle.comparison.baseline.input_manifest_sha256 === bundle.comparison.candidate.input_manifest_sha256,
  matched_resource_budget: bundle.comparison.baseline.resource_budget_sha256 === bundle.comparison.candidate.resource_budget_sha256,
  matched_authority_budget: bundle.comparison.baseline.authority_budget_sha256 === bundle.comparison.candidate.authority_budget_sha256,
  external_effects_disabled: bundle.effects_mode === 'disabled',
  nonclaims: [
    'This receipt proves only cross-invariant conformance of this synthetic fixture.',
    'It does not run or independently validate the baseline/candidate implementation named by their digests.',
    'JSON Schema structural validation is a separate lane.',
    'It does not establish vendor integration, regulatory compliance, physical delivery, product authenticity, or efficacy.'
  ]
};
process.stdout.write(JSON.stringify(receipt, null, 2) + '\n');
