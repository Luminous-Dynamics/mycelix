#!/usr/bin/env node
'use strict';

// Zero-dependency verifier for the organization-adoption catalog.
// JSON Schema remains the interoperable structural contract; this check adds
// cross-row invariants (notably uniqueness of profile_id) that JSON Schema
// cannot express portably without a custom extension.

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
function sha256(bytes) {
  return crypto.createHash('sha256').update(bytes).digest('hex');
}
function isDate(value) {
  if (typeof value !== 'string' || !/^\d{4}-\d{2}-\d{2}$/.test(value)) return false;
  const parsed = new Date(`${value}T00:00:00Z`);
  return !Number.isNaN(parsed.valueOf()) && parsed.toISOString().slice(0, 10) === value;
}

const root = __dirname;
const jsonPath = process.argv[2] || path.join(root, 'org-adoption-profiles-v1.json');
const schemaPath = process.argv[3] || path.join(root, 'org-adoption-profiles-v1.schema.json');
const jsonBytes = fs.readFileSync(jsonPath);
const schemaBytes = fs.readFileSync(schemaPath);
const catalog = readJson(jsonPath);
const schema = readJson(schemaPath);

requireCondition(schema.$schema === 'https://json-schema.org/draft/2020-12/schema',
  'schema must declare JSON Schema Draft 2020-12');
requireCondition(catalog.schema_version === '1.0.0', 'unsupported catalog schema_version');
requireCondition(catalog.registry_id === 'MYC-ORG-PORTFOLIO-001', 'unexpected registry_id');
requireCondition(catalog.claim_ceiling === 'TargetMappingOnly', 'claim ceiling must remain TargetMappingOnly');
requireCondition(catalog.relationship_status_default === 'not_recorded', 'default relationship status must remain not_recorded');
requireCondition(isDate(catalog.snapshot_date), 'catalog snapshot_date must be an ISO calendar date');
requireCondition(Array.isArray(catalog.profiles) && catalog.profiles.length >= 10 && catalog.profiles.length <= 100,
  'profile count must be between 10 and 100');

const required = [
  'profile_id', 'display_name', 'profile_kind', 'adoption_domain',
  'technical_priority', 'readiness', 'source_urls', 'snapshot_date',
  'relationship_status', 'relationship_claim_guard', 'gap_requires_issue',
  'mycelix_owner_issues', 'pilot_scope', 'acceptance_metrics',
  'negative_controls', 'deployment_assumptions', 'off_ramp', 'nonclaims',
  'source_evidence_state', 'source_snapshot_manifest_sha256', 'source_snapshot_manifest_path', 'source_snapshot_required_before_pilot',
];
const profileKinds = new Set([
  'organization', 'protocol_project', 'platform_ecosystem', 'consortium',
  'standards_body', 'identifier_registry',
]);
const priorities = new Set(['P0', 'P1', 'P2']);
const readiness = new Set([
  'existing_work_to_reuse', 'standards_reference_only', 'gap_requires_issue',
]);
const ids = new Set();
const domains = new Set();

for (const [index, profile] of catalog.profiles.entries()) {
  const label = `profiles[${index}]`;
  requireCondition(profile && typeof profile === 'object' && !Array.isArray(profile), `${label} must be an object`);
  for (const key of required) requireCondition(Object.hasOwn(profile, key), `${label} missing ${key}`);
  for (const key of Object.keys(profile)) requireCondition(required.includes(key), `${label} has unexpected field ${key}`);

  requireCondition(typeof profile.profile_id === 'string' && /^[a-z0-9]+(?:-[a-z0-9]+)*$/.test(profile.profile_id),
    `${label}.profile_id is malformed`);
  requireCondition(!ids.has(profile.profile_id), `duplicate profile_id: ${profile.profile_id}`);
  ids.add(profile.profile_id);

  requireCondition(typeof profile.display_name === 'string' && profile.display_name.trim().length >= 2,
    `${label}.display_name is empty`);
  requireCondition(profileKinds.has(profile.profile_kind), `${label}.profile_kind is invalid`);
  requireCondition(typeof profile.adoption_domain === 'string' && profile.adoption_domain.trim().length >= 3,
    `${label}.adoption_domain is invalid`);
  domains.add(profile.adoption_domain);
  requireCondition(priorities.has(profile.technical_priority), `${label}.technical_priority is invalid`);
  requireCondition(readiness.has(profile.readiness), `${label}.readiness is invalid`);
  requireCondition(isDate(profile.snapshot_date), `${label}.snapshot_date is invalid`);
  requireCondition(profile.relationship_status === 'not_recorded', `${label} must not imply relationship status`);
  requireCondition(profile.relationship_claim_guard === 'No partnership, customer relationship, endorsement, or integration is implied by this catalog.',
    `${label} relationship guard changed or missing`);
  requireCondition(typeof profile.gap_requires_issue === 'boolean', `${label}.gap_requires_issue must be boolean`);
  requireCondition(profile.gap_requires_issue === (profile.readiness === 'gap_requires_issue'),
    `${label} gap_requires_issue must agree with readiness`);
  const sourceEvidenceStates = [
    'reference_links_only',
    'version_pinned_manifest_attached',
    'content_snapshot_manifest_attached',
    'mixed_manifest_attached'
  ];
  requireCondition(sourceEvidenceStates.includes(profile.source_evidence_state),
    `${label} source_evidence_state is invalid`);
  if (profile.source_evidence_state === 'reference_links_only') {
    requireCondition(profile.source_snapshot_manifest_sha256 === null &&
      profile.source_snapshot_manifest_path === null &&
      profile.source_snapshot_required_before_pilot === true,
      `${label} reference-only links cannot claim a manifest or clear the pre-pilot content gate`);
  } else {
    requireCondition(hasSha(profile.source_snapshot_manifest_sha256),
      `${label} attached source manifest must have a SHA-256 digest`);
    requireCondition(typeof profile.source_snapshot_manifest_path === 'string' &&
      /^source-snapshots\/[a-z0-9][a-z0-9-]+-v[1-9][0-9]*\.json$/.test(profile.source_snapshot_manifest_path),
      `${label} attached source manifest must point to a versioned manifest`);
    const contentComplete = profile.source_evidence_state === 'content_snapshot_manifest_attached';
    requireCondition(profile.source_snapshot_required_before_pilot === !contentComplete,
      `${label} pre-pilot content gate must remain open until every source payload is captured and hashed`);
  }

  requireCondition(Array.isArray(profile.source_urls) && profile.source_urls.length >= 1,
    `${label}.source_urls must not be empty`);
  for (const source of profile.source_urls) {
    let url;
    try { url = new URL(source); } catch { fail(`${label} has invalid source URL: ${source}`); }
    requireCondition(url.protocol === 'https:', `${label} source URL must use HTTPS`);
  }

  if (profile.source_evidence_state !== 'reference_links_only') {
    const snapshotPath = path.resolve(root, profile.source_snapshot_manifest_path);
    requireCondition(snapshotPath.startsWith(path.resolve(root) + path.sep),
      `${label} source snapshot path escapes adoption directory`);
    requireCondition(fs.existsSync(snapshotPath), `${label} attached source snapshot manifest is missing`);
    const snapshotBytes = fs.readFileSync(snapshotPath);
    requireCondition(sha256(snapshotBytes) === profile.source_snapshot_manifest_sha256,
      `${label} source snapshot manifest digest mismatch`);
    const manifest = readJson(snapshotPath);
    requireCondition(manifest.schema_version === '1.0.0' &&
      manifest.claim_ceiling === 'SourceSnapshotOnly' &&
      manifest.profile_id === profile.profile_id,
      `${label} source snapshot manifest identity or claim ceiling mismatch`);
    requireCondition(Array.isArray(manifest.sources) &&
      manifest.sources.length === profile.source_urls.length,
      `${label} source snapshot does not cover every declared source URL`);
    const manifestUrls = manifest.sources.map(source => source.source_url).sort();
    requireCondition(new Set(manifestUrls).size === manifestUrls.length,
      `${label} source snapshot contains duplicate source URLs`);
    requireCondition(JSON.stringify(manifestUrls) === JSON.stringify([...profile.source_urls].sort()),
      `${label} source snapshot URL set differs from profile source_urls`);
    for (const sourceRecord of manifest.sources) {
      requireCondition(['content_bytes_captured', 'exact_revision_pinned'].includes(sourceRecord.capture_state),
        `${label} source ${sourceRecord.source_url} is not snapshot-qualified`);
      requireCondition(['permitted_public_use', 'authorized_private_use', 'metadata_only'].includes(sourceRecord.rights_disposition),
        `${label} source ${sourceRecord.source_url} has unresolved or blocked rights status`);
      if (sourceRecord.capture_state === 'content_bytes_captured') {
        requireCondition(typeof sourceRecord.content_artifact_path === 'string' &&
          /^source-artifacts\/[a-z0-9][a-z0-9._-]{1,180}$/.test(sourceRecord.content_artifact_path),
          `${label} captured source lacks a safe local artifact path`);
        const artifactPath = path.resolve(root, sourceRecord.content_artifact_path);
        requireCondition(artifactPath.startsWith(path.resolve(root) + path.sep) && fs.existsSync(artifactPath),
          `${label} source content artifact is missing or escapes adoption directory`);
        requireCondition(hasSha(sourceRecord.content_sha256) &&
          sha256(fs.readFileSync(artifactPath)) === sourceRecord.content_sha256,
          `${label} source content artifact digest mismatch`);
      } else {
        requireCondition(typeof sourceRecord.source_revision === 'string' && sourceRecord.source_revision.length > 0,
          `${label} revision-pinned source lacks source_revision`);
      }
    }
    const captureStates = new Set(manifest.sources.map(sourceRecord => sourceRecord.capture_state));
    if (profile.source_evidence_state === 'version_pinned_manifest_attached') {
      requireCondition(captureStates.size === 1 && captureStates.has('exact_revision_pinned'),
        `${label} version-pinned state requires every source to be revision-pinned and no content-captured sources`);
      requireCondition(profile.source_snapshot_required_before_pilot === true,
        `${label} version-pinned references do not clear the pre-pilot source-content gate`);
    } else if (profile.source_evidence_state === 'content_snapshot_manifest_attached') {
      requireCondition(captureStates.size === 1 && captureStates.has('content_bytes_captured'),
        `${label} content-snapshot state requires every source payload to be locally captured and hashed`);
      requireCondition(profile.source_snapshot_required_before_pilot === false,
        `${label} content snapshot should clear the source-content gate`);
    } else if (profile.source_evidence_state === 'mixed_manifest_attached') {
      requireCondition(captureStates.has('exact_revision_pinned') && captureStates.has('content_bytes_captured'),
        `${label} mixed manifest state requires at least one reference pin and one captured payload`);
      requireCondition(profile.source_snapshot_required_before_pilot === true,
        `${label} mixed manifest must keep the pre-pilot source-content gate open`);
    }
  }

  requireCondition(Array.isArray(profile.mycelix_owner_issues) && profile.mycelix_owner_issues.length >= 1,
    `${label} must name an owner issue or declare a gap`);
  for (const owner of profile.mycelix_owner_issues) {
    requireCondition(/^mycelix#\d{1,6}$/.test(owner.issue_id), `${label} has malformed issue id`);
    requireCondition(typeof owner.role === 'string' && owner.role.trim().length >= 3, `${label} has empty owner role`);
  }
  requireCondition(!profile.gap_requires_issue || profile.readiness === 'gap_requires_issue',
    `${label} has an issue gap without explicit readiness`);

  for (const key of ['acceptance_metrics', 'negative_controls', 'deployment_assumptions', 'nonclaims']) {
    requireCondition(Array.isArray(profile[key]) && profile[key].length >= (key === 'deployment_assumptions' ? 1 : 2),
      `${label}.${key} is underspecified`);
    requireCondition(profile[key].every(item => typeof item === 'string' && item.trim().length >= 10),
      `${label}.${key} contains an empty item`);
  }
  requireCondition(typeof profile.pilot_scope === 'string' && profile.pilot_scope.trim().length >= 40,
    `${label}.pilot_scope is too short`);
  requireCondition(typeof profile.off_ramp === 'string' && profile.off_ramp.trim().length >= 25,
    `${label}.off_ramp is too short`);
}

requireCondition(ids.size === catalog.profiles.length, 'profile_id values are not unique');
requireCondition(domains.size >= 6, 'catalog must cover at least six adoption domains');
requireCondition(catalog.profiles.some(p => p.profile_id === 'common-planet-creditism'),
  'Common Planet profile missing');
requireCondition(catalog.profiles.some(p => p.profile_id === 'integral-collective-specifications'),
  'Integral Collective profile missing');
requireCondition(catalog.profiles.every(p => p.relationship_status === 'not_recorded'),
  'catalog must not act as an outreach/status tracker');

const receipt = {
  verifier: 'verify-org-adoption-profiles.cjs',
  result: 'PASS',
  profile_count: catalog.profiles.length,
  unique_profile_ids: ids.size,
  adoption_domain_count: domains.size,
  source_reference_only_count: catalog.profiles.filter(p => p.source_evidence_state === 'reference_links_only').length,
  source_version_pinned_manifest_count: catalog.profiles.filter(p => p.source_evidence_state === 'version_pinned_manifest_attached').length,
  source_content_snapshot_manifest_count: catalog.profiles.filter(p => p.source_evidence_state === 'content_snapshot_manifest_attached').length,
  source_mixed_manifest_count: catalog.profiles.filter(p => p.source_evidence_state === 'mixed_manifest_attached').length,
  source_snapshot_manifest_digests: catalog.profiles.filter(p => p.source_snapshot_manifest_sha256)
    .map(p => ({ profile_id: p.profile_id, manifest_sha256: p.source_snapshot_manifest_sha256 })),
  pre_pilot_snapshot_required_count: catalog.profiles.filter(p => p.source_snapshot_required_before_pilot).length,
  explicit_gap_count: catalog.profiles.filter(p => p.gap_requires_issue).length,
  snapshot_date: catalog.snapshot_date,
  catalog_sha256: sha256(jsonBytes),
  schema_sha256: sha256(schemaBytes),
  claim_ceiling: catalog.claim_ceiling,
  nonclaims: [
    'This verifier checks the catalog contract and cross-row invariants only.',
    'It does not validate external organizations, source truth, integration readiness, or real-world efficacy.',
    'Formal JSON Schema validation remains a separate check against the versioned schema.'
  ]
};
process.stdout.write(JSON.stringify(receipt, null, 2) + '\n');
