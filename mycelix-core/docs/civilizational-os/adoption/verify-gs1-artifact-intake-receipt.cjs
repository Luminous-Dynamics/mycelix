#!/usr/bin/env node
'use strict';

// Validates the GS1 artifact-intake receipt contract and its relationship to
// the pinned source manifest. It does not perform network access, JSON Schema
// validation, JSON-LD expansion, SHACL validation, or an EPCIS adapter run.

const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');

const root = __dirname;
const receiptDefault = path.join(root, 'gs1-normative-artifact-intake-example-v1.json');
const manifestDefault = path.join(root, 'source-snapshots/gs1-epcis-cbv-v1.json');
const MANIFEST_SHA = 'd96381be7a7bbfdd6cafb6286834a575268a1f5b423a5e1345d8d1af4c3156e6';
const ARTIFACTS = Object.freeze([
  {
    artifact_id: 'gs1-epcis-2-0-1-context',
    artifact_kind: 'jsonld_context',
    requested_url: 'https://ref.gs1.org/standards/epcis/2.0.1/artefacts/epcis-context.jsonld'
  },
  {
    artifact_id: 'gs1-epcis-2-0-1-json-schema',
    artifact_kind: 'json_schema_draft07',
    requested_url: 'https://ref.gs1.org/standards/epcis/2.0.1/artefacts/epcis-json-schema.json'
  },
  {
    artifact_id: 'gs1-epcis-2-0-1-shacl',
    artifact_kind: 'shacl_shapes',
    requested_url: 'https://ref.gs1.org/standards/epcis/2.0.1/artefacts/epcis-shacl.ttl'
  },
  {
    artifact_id: 'gs1-cbv-2-0-0-ontology-jsonld',
    artifact_kind: 'cbv_ontology_jsonld',
    requested_url: 'https://ref.gs1.org/standards/epcis/2.0.1/artefacts/cbv-ontology.jsonld'
  }
]);
const SHA = /^[0-9a-f]{64}$/;
function sha256(bytes) {
  return crypto.createHash('sha256').update(bytes).digest('hex');
}
function readJson(filename) {
  try { return JSON.parse(fs.readFileSync(filename, 'utf8')); }
  catch (error) { throw new Error('cannot read JSON ' + filename + ': ' + error.message); }
}
function add(errors, condition, message) {
  if (!condition) errors.push(message);
}
function safeChild(rootDir, relative) {
  const resolvedRoot = path.resolve(rootDir);
  const target = path.resolve(resolvedRoot, relative);
  return target.startsWith(resolvedRoot + path.sep) ? target : null;
}
function validate(receipt, manifest, manifestBytes) {
  const errors = [];
  add(errors, receipt && receipt.schema_version === '1.0.0', 'unsupported receipt schema version');
  add(errors, receipt && receipt.profile_id === 'gs1-epcis-cbv', 'profile id mismatch');
  add(errors, receipt && receipt.claim_ceiling === 'ArtifactIntakeReceiptOnly', 'claim ceiling mismatch');
  add(errors, receipt && receipt.source_manifest_sha256 === MANIFEST_SHA, 'receipt manifest pin mismatch');
  add(errors, sha256(manifestBytes) === MANIFEST_SHA, 'source manifest exact-byte SHA-256 mismatch');
  add(errors, manifest && manifest.manifest_id === 'gs1-epcis-cbv-v1' &&
    manifest.profile_id === 'gs1-epcis-cbv', 'source manifest identity mismatch');

  const sources = new Map((manifest && Array.isArray(manifest.sources) ? manifest.sources : [])
    .map(item => [item.source_url, item]));
  add(errors, sources.size === (manifest.sources || []).length, 'source manifest has duplicate URLs');
  for (const expected of ARTIFACTS) {
    const source = sources.get(expected.requested_url);
    add(errors, !!source, 'source manifest missing corrected URL for ' + expected.artifact_id);
    if (source) {
      add(errors, source.source_id === expected.artifact_id, 'source id mismatch for ' + expected.artifact_id);
      add(errors, source.capture_state === 'exact_revision_pinned', 'source reference should remain version-pinned for ' + expected.artifact_id);
    }
  }

  const rows = Array.isArray(receipt && receipt.artifacts) ? receipt.artifacts : [];
  add(errors, rows.length === ARTIFACTS.length, 'receipt must contain exactly four normative artifact rows');
  const byId = new Map();
  for (const [index, row] of rows.entries()) {
    const label = 'artifacts[' + index + ']';
    if (!row || typeof row !== 'object') {
      errors.push(label + ' must be an object');
      continue;
    }
    add(errors, !byId.has(row.artifact_id), label + ' duplicate artifact_id');
    byId.set(row.artifact_id, row);
    const expected = ARTIFACTS.find(item => item.artifact_id === row.artifact_id);
    add(errors, !!expected, label + ' has an unknown artifact_id');
    if (!expected) continue;
    add(errors, row.artifact_kind === expected.artifact_kind, label + ' artifact_kind mismatch');
    add(errors, row.requested_url === expected.requested_url, label + ' requested_url mismatch');
    add(errors, typeof row.source_revision === 'string' && row.source_revision.length >= 3,
      label + ' source revision/reference must be stated');

    if (row.fetch_state === 'not_fetched') {
      add(errors, row.http_status === null && row.final_url === null && row.media_type === null &&
        row.byte_length === null && row.content_sha256 === null && row.artifact_path === null,
        label + ' not_fetched row cannot invent an HTTP response or content digest');
      add(errors, ['network_unavailable', 'tool_could_not_fetch_raw_artifact', 'not_attempted'].includes(row.failure_class),
        label + ' not_fetched requires a specific non-origin failure class');
    } else if (row.fetch_state === 'fetch_failed') {
      add(errors, row.content_sha256 === null && row.artifact_path === null,
        label + ' failed fetch cannot claim captured content bytes');
      add(errors, ['network_unavailable', 'tool_could_not_fetch_raw_artifact', 'http_error',
        'unexpected_redirect', 'unexpected_media_type', 'rights_or_terms_blocked'].includes(row.failure_class),
        label + ' failed fetch requires a failure class');
      if (row.http_status === null) {
        add(errors, row.final_url === null && row.media_type === null && row.byte_length === null,
          label + ' without an HTTP status cannot invent response metadata');
      }
    } else if (row.fetch_state === 'fetched_and_hashed') {
      add(errors, row.http_status === 200, label + ' captured artifact must have HTTP 200');
      add(errors, typeof row.final_url === 'string' && row.final_url === expected.requested_url,
        label + ' final URL must match the pinned requested URL; redirects require review');
      add(errors, typeof row.media_type === 'string' && row.media_type.length >= 3,
        label + ' captured artifact must declare media type');
      add(errors, Number.isInteger(row.byte_length) && row.byte_length > 0,
        label + ' captured artifact must declare byte length');
      add(errors, typeof row.content_sha256 === 'string' && SHA.test(row.content_sha256),
        label + ' captured artifact must declare SHA-256');
      const fullPath = typeof row.artifact_path === 'string' ?
        safeChild(root, row.artifact_path) : null;
      add(errors, !!fullPath, label + ' artifact path must be a safe relative path');
      add(errors, row.failure_class === null, label + ' successful fetch cannot retain a failure class');
      if (fullPath) {
        add(errors, fs.existsSync(fullPath), label + ' local artifact bytes are missing');
        if (fs.existsSync(fullPath)) {
          const bytes = fs.readFileSync(fullPath);
          add(errors, bytes.length === row.byte_length, label + ' captured byte length mismatch');
          add(errors, sha256(bytes) === row.content_sha256, label + ' captured bytes SHA-256 mismatch');
        }
      }
    } else {
      errors.push(label + ' has an unknown fetch_state');
    }
  }

  for (const expected of ARTIFACTS) {
    add(errors, byId.has(expected.artifact_id), 'missing artifact row ' + expected.artifact_id);
  }
  const complete = rows.length === ARTIFACTS.length &&
    rows.every(row => row && row.fetch_state === 'fetched_and_hashed');
  add(errors, receipt.receipt_state === (complete ? 'complete' : 'incomplete'),
    'receipt_state must agree with per-artifact fetch states');
  if (receipt.receipt_state === 'complete') {
    add(errors, receipt.runtime_access_state === 'network_available',
      'complete intake requires network_access_state=network_available');
  }
  return { errors, complete };
}
function mutationMustFail(receipt, manifest, manifestBytes, name, mutate) {
  const changed = JSON.parse(JSON.stringify(receipt));
  mutate(changed);
  const result = validate(changed, manifest, manifestBytes);
  if (result.errors.length === 0) throw new Error('mutation was not rejected: ' + name);
  return { name, rejected: true, invariant_errors: result.errors.length };
}
function main() {
  const args = process.argv.slice(2);
  const requireComplete = args.includes('--require-complete');
  const selfTest = args.includes('--self-test');
  const positional = args.filter(arg => !arg.startsWith('--'));
  const receiptPath = positional[0] || receiptDefault;
  const manifestPath = positional[1] || manifestDefault;
  const receiptBytes = fs.readFileSync(receiptPath);
  const manifestBytes = fs.readFileSync(manifestPath);
  const receipt = JSON.parse(receiptBytes.toString('utf8'));
  const manifest = JSON.parse(manifestBytes.toString('utf8'));
  const result = validate(receipt, manifest, manifestBytes);
  if (result.errors.length) {
    process.stderr.write('FAIL: ' + result.errors.join('\n') + '\n');
    process.exit(1);
  }
  const mutations = selfTest ? [
    mutationMustFail(receipt, manifest, manifestBytes, 'invented-hash-without-bytes',
      r => { r.artifacts[0].content_sha256 = 'a'.repeat(64); }),
    mutationMustFail(receipt, manifest, manifestBytes, 'invented-http-404-without-response',
      r => { r.artifacts[0].http_status = 404; }),
    mutationMustFail(receipt, manifest, manifestBytes, 'falsely-complete',
      r => { r.receipt_state = 'complete'; }),
    mutationMustFail(receipt, manifest, manifestBytes, 'duplicate-artifact-row',
      r => { r.artifacts[1] = JSON.parse(JSON.stringify(r.artifacts[0])); }),
    mutationMustFail(receipt, manifest, manifestBytes, 'wrong-pinned-url',
      r => { r.artifacts[0].requested_url = 'https://ref.gs1.org/epcis-context.jsonld'; }),
    mutationMustFail(receipt, manifest, manifestBytes, 'manifest-digest-tamper',
      r => { r.source_manifest_sha256 = '0'.repeat(64); }),
    mutationMustFail(receipt, manifest, manifestBytes, 'missing-artifact-row',
      r => { r.artifacts.pop(); }),
    mutationMustFail(receipt, manifest, manifestBytes, 'failed-fetch-claims-hash',
      r => { r.artifacts[0].fetch_state = 'fetch_failed'; r.artifacts[0].content_sha256 = 'a'.repeat(64); }),
    mutationMustFail(receipt, manifest, manifestBytes, 'unsafe-artifact-path',
      r => { r.artifacts[0].fetch_state = 'fetched_and_hashed'; r.artifacts[0].artifact_path = '../../escape.json'; }),
    mutationMustFail(receipt, manifest, manifestBytes, 'unknown-failure-class',
      r => { r.artifacts[0].failure_class = 'origin-definitely-down'; })
  ] : [];

  const audit = {
    benchmark_id: 'MYC-GS1-ARTIFACT-INTAKE-001',
    result: result.complete ? 'NormativeArtifactIntakeComplete' : 'NormativeArtifactIntakeIncomplete',
    receipt_state: receipt.receipt_state,
    claim_ceiling: receipt.claim_ceiling,
    artifact_count: receipt.artifacts.length,
    artifacts_fetched_and_hashed: receipt.artifacts.filter(x => x.fetch_state === 'fetched_and_hashed').length,
    artifacts_not_fetched: receipt.artifacts.filter(x => x.fetch_state === 'not_fetched').length,
    artifacts_fetch_failed: receipt.artifacts.filter(x => x.fetch_state === 'fetch_failed').length,
    source_manifest_sha256: sha256(manifestBytes),
    receipt_sha256: sha256(receiptBytes),
    gate_passed: result.complete,
    self_test_executed: selfTest,
    mutation_cases: mutations.length,
    mutation_cases_rejected: mutations.filter(x => x.rejected).length,
    mutations,
    nonclaims: [
      'This verifier validates the intake receipt and any locally captured bytes; it does not access the network.',
      'A complete intake receipt is not the same as successful JSON Schema or SHACL validation.',
      'The official GS1 normative validators and a real adapter benchmark must produce separate receipts.'
    ]
  };
  process.stdout.write(JSON.stringify(audit, null, 2) + '\n');
  if (requireComplete && !result.complete) process.exitCode = 2;
}
main();
