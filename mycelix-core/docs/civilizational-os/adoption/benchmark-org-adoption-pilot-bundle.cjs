#!/usr/bin/env node
'use strict';

// Reproducible local scale harness for the organization-adoption cross-invariant
// verifier. This measures the verifier process only; JSON Schema instance validation
// is deliberately a separate lane, and no vendor implementation is executed.

const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const crypto = require('node:crypto');
const { spawnSync } = require('node:child_process');
const { performance } = require('node:perf_hooks');

const root = __dirname;
const verifierPath = path.join(root, 'verify-org-adoption-pilot-bundle.cjs');
const schemaPath = path.join(root, 'org-adoption-pilot-bundle-v1.schema.json');

function fail(message) {
  process.stderr.write(`FAIL: ${message}\n`);
  process.exit(1);
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
function readOptional(filename) {
  try { return fs.readFileSync(filename, 'utf8').trim(); } catch { return null; }
}
function parseArgs(argv) {
  const result = { runs: 5, sizes: [1000, 10000, 50000], out: null };
  for (let i = 0; i < argv.length; i++) {
    if (argv[i] === '--runs') {
      result.runs = Number(argv[++i]);
    } else if (argv[i] === '--sizes') {
      result.sizes = String(argv[++i]).split(',').map(Number);
    } else if (argv[i] === '--out') {
      result.out = argv[++i];
    } else {
      fail(`unknown argument: ${argv[i]}`);
    }
  }
  if (!Number.isInteger(result.runs) || result.runs < 1 || result.runs > 25) {
    fail('--runs must be an integer from 1 through 25');
  }
  if (!Array.isArray(result.sizes) || result.sizes.length < 1 ||
      result.sizes.some(n => !Number.isInteger(n) || n < 4 || n > 50000)) {
    fail('--sizes must be comma-separated integers in range 4..50000');
  }
  return result;
}
function makeBundle(size, verifierSha) {
  const sourceId = 'synthetic-fixture:gs1-epcis-shaped';
  const startMillis = Date.parse('2026-10-21T09:00:00Z');
  const eventTypes = ['scan_observation', 'shipping', 'receiving', 'acceptance'];
  const events = Array.from({ length: size }, (_, index) => {
    const occurred = new Date(startMillis + index * 1000).toISOString();
    const eventType = eventTypes[index % eventTypes.length];
    return {
      event_id: `evt-${String(index + 1).padStart(6, '0')}`,
      sequence: index + 1,
      event_type: eventType,
      subject_id: `lot-${String(Math.floor(index / 100) + 1).padStart(5, '0')}`,
      occurred_at_utc: occurred,
      recorded_at_utc: new Date(startMillis + index * 1000 + 2000).toISOString(),
      source_id: sourceId,
      disposition: eventType === 'scan_observation' ? 'observed' :
        eventType === 'shipping' ? 'asserted' : 'accepted_by_receiver'
    };
  });
  const entry = {
    source_id: sourceId,
    source_kind: 'synthetic_fixture',
    source_revision: `scale-fixture-${size}`,
    content_sha256: sha256(canonical(events)),
    rights_disposition: 'synthetic_no_external_rights_required',
    record_ids: events.map(event => event.event_id)
  };
  const manifestBody = {
    data_classification: 'synthetic',
    rights_disposition: 'synthetic_no_external_rights_required',
    source_entries: [{ ...entry, record_ids: [...entry.record_ids].sort() }],
    events: [...events].sort((left, right) =>
      (left.sequence - right.sequence) || left.event_id.localeCompare(right.event_id))
  };
  const manifestSha = sha256(canonical(manifestBody));
  const refs = [events[0].event_id, events[Math.floor(size / 3)].event_id,
    events[Math.floor(2 * size / 3)].event_id, events[size - 1].event_id];
  const windowEnd = new Date(startMillis + (size - 1) * 1000 + 3000).toISOString();

  const bundle = {
    schema_version: '1.0.0',
    bundle_id: `supply-chain-scale-${size}`,
    profile_id: 'tracelink',
    pilot_id: 'epcis-shaped-scale-001',
    created_at_utc: '2026-10-21T18:30:00Z',
    relationship_status: 'not_recorded',
    relationship_claim_guard: 'No partnership, customer relationship, endorsement, or integration is implied by this bundle.',
    data_classification: 'synthetic',
    effects_mode: 'disabled',
    claim_ceiling: 'SyntheticReferenceOnly',
    nonclaims: [
      'Synthetic scale fixture only; it does not establish a vendor integration or relationship.',
      'Recorded events do not independently establish physical delivery, product authenticity, or regulatory compliance.'
    ],
    comparison: {
      baseline: {
        implementation_id: 'manual-reference-normalizer',
        version: 'scale-fixture-v1',
        implementation_sha256: '2222222222222222222222222222222222222222222222222222222222222222',
        input_manifest_sha256: manifestSha,
        resource_budget_sha256: 'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa',
        authority_budget_sha256: 'bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb'
      },
      candidate: {
        implementation_id: 'mycelix-loss-aware-event-projection',
        version: 'scale-fixture-v1',
        implementation_sha256: '3333333333333333333333333333333333333333333333333333333333333333',
        input_manifest_sha256: manifestSha,
        resource_budget_sha256: 'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa',
        authority_budget_sha256: 'bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb'
      },
      measurement_profile_id: 'supply-chain-event-preservation-v1',
      hardware_profile_id: 'benchmark-harness-records-runtime-below',
      window_start_utc: events[0].occurred_at_utc,
      window_end_utc: windowEnd
    },
    input_manifest: {
      data_classification: 'synthetic',
      rights_disposition: 'synthetic_no_external_rights_required',
      source_entries: [entry],
      events,
      manifest_sha256: manifestSha
    },
    evidence_claims: [
      {
        claim_id: 'claim-event-lineage',
        claim_type: 'event_lineage_preserved',
        disposition: 'supported_within_fixture',
        evidence_refs: refs,
        explanation: 'The synthetic event fixture preserves declared event identifiers and source lineage.'
      },
      {
        claim_id: 'claim-physical-delivery',
        claim_type: 'physical_delivery_verified',
        disposition: 'indeterminate',
        evidence_refs: [events[0].event_id, events[Math.min(2, size - 1)].event_id],
        explanation: 'Synthetic scans and receiving assertions do not establish independently measured physical delivery.'
      },
      {
        claim_id: 'claim-regulatory-compliance',
        claim_type: 'regulatory_compliance',
        disposition: 'unsupported',
        evidence_refs: [events[size - 1].event_id],
        explanation: 'No jurisdictional compliance profile or evidence exists in this synthetic fixture.'
      }
    ],
    metrics: [
      { metric_id: 'lineage-preservation', unit: 'percent', comparison_rule: 'candidate_gte_baseline', baseline_value: 100, candidate_value: 100, disposition: 'pass' },
      { metric_id: 'duplicate-replay-controls', unit: 'count', comparison_rule: 'candidate_gt_baseline', baseline_value: 0, candidate_value: 1, disposition: 'pass' },
      { metric_id: 'unsupported-claims-promoted', unit: 'count', comparison_rule: 'candidate_zero', baseline_value: 0, candidate_value: 0, disposition: 'pass' }
    ],
    negative_controls: [
      { control_id: 'nc-duplicate-replay', attack: 'Replay an already accepted source event using the same identifier.', expected_disposition: 'rejected_duplicate', observed_disposition: 'rejected_duplicate', passed: true },
      { control_id: 'nc-wrong-subject', attack: 'Replace an event subject with a different lot identifier.', expected_disposition: 'rejected_subject_mismatch', observed_disposition: 'rejected_subject_mismatch', passed: true },
      { control_id: 'nc-scan-promotion', attack: 'Present a scan observation alone as proof of physical delivery.', expected_disposition: 'claim_indeterminate', observed_disposition: 'claim_indeterminate', passed: true },
      { control_id: 'nc-source-digest', attack: 'Change a source record while keeping its committed digest.', expected_disposition: 'rejected_digest_mismatch', observed_disposition: 'rejected_digest_mismatch', passed: true },
      { control_id: 'nc-rights-gap', attack: 'Remove the required rights disposition from an external source.', expected_disposition: 'rejected_missing_rights', observed_disposition: 'rejected_missing_rights', passed: true }
    ],
    export_test: { performed: true, expected_sha256: manifestSha, observed_sha256: manifestSha, round_trip_matches: true },
    rollback_test: { performed: true, result: 'rollback_completed', retained_external_effects: false },
    result: {
      disposition: 'ReferenceFixtureConforms',
      evaluator_id: 'verify-org-adoption-pilot-bundle.cjs',
      evaluator_sha256: verifierSha,
      bundle_scope: 'synthetic_reference_fixture_only'
    }
  };
  return { bundle, manifestSha };
}

function percentile(sorted, fraction) {
  const index = Math.max(0, Math.min(sorted.length - 1, Math.ceil(fraction * sorted.length) - 1));
  return sorted[index];
}

function main() {
  const options = parseArgs(process.argv.slice(2));
  const schemaText = fs.readFileSync(schemaPath, 'utf8');
  const schema = JSON.parse(schemaText);
  if (schema.$schema !== 'https://json-schema.org/draft/2020-12/schema') {
    fail('schema must declare JSON Schema Draft 2020-12');
  }
  const schemaSha = sha256(schemaText);
  const verifierBytes = fs.readFileSync(verifierPath);
  const verifierSha = sha256(verifierBytes);
  const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'mycelix-adoption-bench-'));
  const results = [];
  try {
    for (const size of options.sizes) {
      const { bundle, manifestSha } = makeBundle(size, verifierSha);
      const fixturePath = path.join(tmp, `scale-${size}.json`);
      const fixtureBytes = Buffer.from(JSON.stringify(bundle, null, 2) + '\n');
      fs.writeFileSync(fixturePath, fixtureBytes);
      const samples = [];
      for (let run = 0; run < options.runs; run++) {
        const started = performance.now();
        const child = spawnSync(process.execPath, [verifierPath, fixturePath, schemaPath], {
          encoding: 'utf8',
          maxBuffer: 2 * 1024 * 1024
        });
        const elapsedMs = performance.now() - started;
        if (child.error || child.status !== 0) {
          fail(`verifier failed for ${size} events: ${child.error?.message || child.stderr || child.stdout}`);
        }
        let verification;
        try { verification = JSON.parse(child.stdout); } catch (error) {
          fail(`verifier returned invalid JSON receipt: ${error.message}`);
        }
        if (verification.qualification_result !== 'ReferenceFixtureConforms' ||
            verification.mode !== 'cross-invariant-only' ||
            verification.mutation_tests_executed !== false ||
            verification.input_manifest_sha256 !== manifestSha) {
          fail(`unexpected verifier receipt for ${size} events`);
        }
        samples.push(Number(elapsedMs.toFixed(2)));
      }
      const sorted = [...samples].sort((a, b) => a - b);
      results.push({
        event_count: size,
        bundle_bytes: fixtureBytes.length,
        input_manifest_sha256: manifestSha,
        bundle_sha256: sha256(fixtureBytes),
        runs: samples.length,
        samples_ms: samples,
        median_ms: Number(percentile(sorted, 0.5).toFixed(2)),
        max_of_n_ms: Number(sorted[sorted.length - 1].toFixed(2)),
        schema_validation_included: false,
        result: 'ReferenceFixtureConforms'
      });
    }
  } finally {
    fs.rmSync(tmp, { recursive: true, force: true });
  }

  const cpus = os.cpus();
  const cpuMax = readOptional('/sys/fs/cgroup/cpu.max');
  const memoryMax = readOptional('/sys/fs/cgroup/memory.max');
  const report = {
    benchmark_id: 'MYC-ORG-VERIFY-SCALE-001',
    schema_version: '1.0.0',
    generated_at_utc: new Date().toISOString(),
    claim_ceiling: 'LocalSyntheticPerformanceBaseline',
    mode: 'cross-invariant-only',
    self_test_mode: 'not_included',
    schema_validation_included: false,
    runtime: {
      node: process.version,
      platform: process.platform,
      architecture: process.arch,
      os_release: os.release(),
      cpu_model: cpus[0]?.model || 'unknown',
      visible_cpu_count: cpus.length,
      cgroup_cpu_max: cpuMax,
      cgroup_memory_max_bytes: memoryMax,
      hardware_profile_id: 'recorded-at-runtime'
    },
    artifacts: {
      verifier_path: path.relative(root, verifierPath),
      verifier_sha256: verifierSha,
      schema_path: path.relative(root, schemaPath),
      schema_sha256: schemaSha,
      schema_role: 'identity-and-version marker only during this performance test; no schema instance validation was performed'
    },
    runs_per_size: options.runs,
    measurements: results,
    nonclaims: [
      'This is a single-host synthetic microbenchmark, not a comparison against a vendor implementation or SOTA.',
      'JSON Schema instance validation is not included in measured timings.',
      'The fixture does not execute the named baseline or candidate implementation; it measures only bundle-verifier overhead.',
      'The reported median and max-of-N are not statistically reliable p95 claims.',
      'No physical-delivery, product-authenticity, regulatory-compliance, or real-world efficacy claim is established.'
    ]
  };
  const output = JSON.stringify(report, null, 2) + '\n';
  if (options.out) fs.writeFileSync(path.resolve(options.out), output);
  process.stdout.write(output);
}

main();
