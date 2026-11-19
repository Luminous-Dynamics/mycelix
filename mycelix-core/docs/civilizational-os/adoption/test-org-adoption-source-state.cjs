#!/usr/bin/env node
'use strict';

// Narrow conformance test for organization-profile source-evidence states.
// It checks source-state semantics; the main catalog verifier remains
// responsible for the complete profile contract and exact manifest/file hash checks.

const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');

const root = __dirname;
const catalogPath = path.join(root, 'org-adoption-profiles-v1.json');
const catalog = JSON.parse(fs.readFileSync(catalogPath, 'utf8'));
const sha256 = bytes => crypto.createHash('sha256').update(bytes).digest('hex');
const SHA = /^[0-9a-f]{64}$/;
const STATES = new Set([
  'reference_links_only',
  'version_pinned_manifest_attached',
  'content_snapshot_manifest_attached',
  'mixed_manifest_attached'
]);
const CAPTURE_STATES = new Set(['exact_revision_pinned', 'content_bytes_captured']);
const RIGHTS_STATES = new Set(['permitted_public_use', 'authorized_private_use', 'metadata_only']);

function insideRoot(relative) {
  if (typeof relative !== 'string' || relative.length === 0 || path.isAbsolute(relative)) return null;
  const base = path.resolve(root);
  const target = path.resolve(base, relative);
  return target.startsWith(base + path.sep) ? target : null;
}

function sourceStateErrors(profile, manifest, { verifyFiles = false } = {}) {
  const errors = [];
  const add = (condition, message) => { if (!condition) errors.push(message); };
  add(profile && typeof profile === 'object', 'profile must be an object');
  if (!profile || typeof profile !== 'object') return errors;
  add(STATES.has(profile.source_evidence_state), 'unknown source_evidence_state');
  if (!STATES.has(profile.source_evidence_state)) return errors;

  if (profile.source_evidence_state === 'reference_links_only') {
    add(profile.source_snapshot_manifest_path === null, 'reference-only state must not name a manifest path');
    add(profile.source_snapshot_manifest_sha256 === null, 'reference-only state must not claim a manifest digest');
    add(profile.source_snapshot_required_before_pilot === true, 'reference-only state must keep the source gate open');
    add(manifest === null, 'reference-only state cannot be supplied an attached manifest');
    return errors;
  }

  add(typeof profile.source_snapshot_manifest_path === 'string' &&
      /^source-snapshots\/[a-z0-9][a-z0-9-]+-v[1-9][0-9]*\.json$/.test(profile.source_snapshot_manifest_path),
    'attached state must name a versioned source manifest');
  add(typeof profile.source_snapshot_manifest_sha256 === 'string' && SHA.test(profile.source_snapshot_manifest_sha256),
    'attached state must declare a SHA-256 manifest digest');
  const shouldRequireContent = profile.source_evidence_state !== 'content_snapshot_manifest_attached';
  add(profile.source_snapshot_required_before_pilot === shouldRequireContent,
    'pre-pilot source gate does not match evidence state');
  add(manifest && manifest.schema_version === '1.0.0', 'source manifest missing or wrong schema version');
  add(manifest && manifest.claim_ceiling === 'SourceSnapshotOnly', 'source manifest claim ceiling mismatch');
  add(manifest && manifest.profile_id === profile.profile_id, 'source manifest profile_id mismatch');
  if (!manifest || !Array.isArray(manifest.sources)) {
    errors.push('source manifest must have a sources array');
    return errors;
  }

  const urls = manifest.sources.map(source => source.source_url).sort();
  add(urls.length === profile.source_urls.length, 'manifest source count differs from profile URL count');
  add(new Set(urls).size === urls.length, 'manifest source URLs contain duplicates');
  add(JSON.stringify(urls) === JSON.stringify([...profile.source_urls].sort()),
    'manifest source URL set differs from the profile');

  const captureStates = new Set();
  for (const [index, source] of manifest.sources.entries()) {
    const where = 'source[' + index + ']';
    add(CAPTURE_STATES.has(source.capture_state), where + ' has unsupported capture_state');
    captureStates.add(source.capture_state);
    add(RIGHTS_STATES.has(source.rights_disposition), where + ' has unresolved/blocked rights disposition');
    if (source.capture_state === 'exact_revision_pinned') {
      add(typeof source.source_revision === 'string' && source.source_revision.trim().length > 0,
        where + ' revision pin has no source_revision');
    } else if (source.capture_state === 'content_bytes_captured') {
      add(typeof source.content_artifact_path === 'string' &&
          /^source-artifacts\/[a-z0-9][a-z0-9._-]{1,180}$/.test(source.content_artifact_path),
        where + ' content-captured state requires a safe artifact path');
      add(typeof source.content_sha256 === 'string' && SHA.test(source.content_sha256),
        where + ' content-captured state requires a SHA-256 digest');
      if (verifyFiles) {
        const artifactPath = insideRoot(source.content_artifact_path);
        add(!!artifactPath && fs.existsSync(artifactPath), where + ' captured artifact missing or outside the adoption directory');
        if (artifactPath && fs.existsSync(artifactPath)) {
          const bytes = fs.readFileSync(artifactPath);
          add(sha256(bytes) === source.content_sha256, where + ' captured artifact digest mismatch');
        }
      }
    }
  }

  if (profile.source_evidence_state === 'version_pinned_manifest_attached') {
    add(captureStates.size === 1 && captureStates.has('exact_revision_pinned'),
      'version-pinned manifest must contain only exact revision pins');
  } else if (profile.source_evidence_state === 'content_snapshot_manifest_attached') {
    add(captureStates.size === 1 && captureStates.has('content_bytes_captured'),
      'content-snapshot manifest must contain only captured payloads');
  } else if (profile.source_evidence_state === 'mixed_manifest_attached') {
    add(captureStates.has('exact_revision_pinned') && captureStates.has('content_bytes_captured'),
      'mixed manifest must contain both revision pins and captured payloads');
  }
  return errors;
}

function expectedRejected(name, profile, manifest) {
  const errors = sourceStateErrors(profile, manifest);
  if (errors.length === 0) throw new Error('mutation was not rejected: ' + name);
  return { name, rejected: true, invariant_errors: errors.length };
}

function main() {
  const byId = new Map(catalog.profiles.map(profile => [profile.profile_id, profile]));
  if (byId.size !== catalog.profiles.length) throw new Error('duplicate profile_id in catalog');
  let checked = 0;
  for (const profile of catalog.profiles) {
    if (profile.source_evidence_state === 'reference_links_only') {
      const errors = sourceStateErrors(profile, null);
      if (errors.length) throw new Error(profile.profile_id + ': ' + errors.join('; '));
      checked++;
      continue;
    }
    const relative = profile.source_snapshot_manifest_path;
    const manifestPath = insideRoot(relative);
    if (!manifestPath || !fs.existsSync(manifestPath)) throw new Error(profile.profile_id + ': source manifest missing/unsafe');
    const bytes = fs.readFileSync(manifestPath);
    if (sha256(bytes) !== profile.source_snapshot_manifest_sha256) {
      throw new Error(profile.profile_id + ': exact-byte source manifest digest mismatch');
    }
    const manifest = JSON.parse(bytes.toString('utf8'));
    const errors = sourceStateErrors(profile, manifest, { verifyFiles: true });
    if (errors.length) throw new Error(profile.profile_id + ': ' + errors.join('; '));
    checked++;
  }

  const gs1 = byId.get('gs1-epcis-cbv');
  const gs1Manifest = JSON.parse(fs.readFileSync(insideRoot(gs1.source_snapshot_manifest_path), 'utf8'));
  const mutations = [
    ['reference-only-claims-manifest', p => { p.source_evidence_state = 'reference_links_only'; }],
    ['version-pin-clears-content-gate', p => { p.source_snapshot_required_before_pilot = false; }],
    ['version-pin-with-content-captured-source', (p, m) => {
      m.sources[0].capture_state = 'content_bytes_captured';
      m.sources[0].content_artifact_path = 'source-artifacts/not-captured.json';
      m.sources[0].content_sha256 = 'a'.repeat(64);
    }],
    ['content-snapshot-with-only-revision-pins', p => {
      p.source_evidence_state = 'content_snapshot_manifest_attached';
      p.source_snapshot_required_before_pilot = false;
    }],
    ['content-snapshot-keeps-gate-open', p => {
      p.source_evidence_state = 'content_snapshot_manifest_attached';
      p.source_snapshot_required_before_pilot = true;
    }],
    ['mixed-state-with-only-revision-pins', p => {
      p.source_evidence_state = 'mixed_manifest_attached';
      p.source_snapshot_required_before_pilot = true;
    }],
    ['mixed-state-clears-content-gate', (p, m) => {
      p.source_evidence_state = 'mixed_manifest_attached';
      p.source_snapshot_required_before_pilot = false;
      m.sources[0].capture_state = 'content_bytes_captured';
      m.sources[0].content_artifact_path = 'source-artifacts/not-captured.json';
      m.sources[0].content_sha256 = 'a'.repeat(64);
    }],
    ['missing-source-url-coverage', (p, m) => { m.sources.pop(); }],
    ['duplicate-source-url', (p, m) => { m.sources[1].source_url = m.sources[0].source_url; }],
    ['malformed-state-label', p => { p.source_evidence_state = 'snapshot_manifest_attached'; }]
  ];
  const mutationResults = mutations.map(([name, mutate]) => {
    const profile = JSON.parse(JSON.stringify(gs1));
    const manifest = JSON.parse(JSON.stringify(gs1Manifest));
    mutate(profile, manifest);
    return expectedRejected(name, profile, manifest);
  });
  if (mutationResults.length !== 10 || mutationResults.some(result => !result.rejected)) {
    throw new Error('source-state mutation suite did not fully reject');
  }

  const stateCounts = catalog.profiles.reduce((counts, profile) => {
    counts[profile.source_evidence_state] = (counts[profile.source_evidence_state] || 0) + 1;
    return counts;
  }, {});
  process.stdout.write(JSON.stringify({
    test_id: 'MYC-ORG-SOURCE-STATE-001',
    result: 'PASS',
    profiles_checked: checked,
    profile_count: catalog.profiles.length,
    state_counts: stateCounts,
    profiles_requiring_source_content_before_pilot: catalog.profiles.filter(profile =>
      profile.source_snapshot_required_before_pilot).length,
    mutation_cases: mutationResults.length,
    mutation_cases_rejected: mutationResults.filter(result => result.rejected).length,
    test_scope: 'Source-evidence state semantics and attached-manifest URL/state consistency; not external source truth or official JSON Schema engine.',
    nonclaims: [
      'A version-pinned reference is not the same as captured source bytes.',
      'A passing source-state test does not establish integration readiness or real-world efficacy.',
      'It does not validate the complete organization profile schema with a standards-compliant external JSON Schema engine.'
    ]
  }, null, 2) + '\n');
}
main();
