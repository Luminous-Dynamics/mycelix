#!/usr/bin/env node
'use strict';

// Cross-invariant evaluator for the version-pinned GS1 EPCIS/CBV minimal
// mapping fixture. This is NOT a general Draft 2020-12 JSON Schema validator,
// nor an EPCIS certification tool. Schema-instance validation and validation
// against the complete normative GS1 artefacts are separate evaluation lanes.

const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');

const root = __dirname;
const fixtureDefault = path.join(root, 'gs1-epcis-mapping-fixture-v1.json');
const schemaDefault = path.join(root, 'gs1-epcis-mapping-fixture-v1.schema.json');
const manifestDefault = path.join(root, 'source-snapshots/gs1-epcis-cbv-v1.json');

function sha256(bytes) {
  return crypto.createHash('sha256').update(bytes).digest('hex');
}
function fail(message) {
  process.stderr.write('FAIL: ' + message + '\n');
  process.exit(1);
}
function addError(errors, condition, message) {
  if (!condition) errors.push(message);
}
function readJson(filename) {
  try {
    return JSON.parse(fs.readFileSync(filename, 'utf8'));
  } catch (error) {
    fail('cannot parse ' + filename + ': ' + error.message);
  }
}
function hasSha(value) {
  return typeof value === 'string' && /^[0-9a-f]{64}$/.test(value);
}
function isUtcMilliseconds(value) {
  return typeof value === 'string' &&
    /^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3}Z$/.test(value) &&
    Number.isFinite(Date.parse(value));
}
function isUtcOffset(value) {
  return typeof value === 'string' &&
    /^[+-](?:(?:0[0-9]|1[0-3]):[0-5][0-9]|14:00)$/.test(value);
}

const STEP_CLAIM = Object.freeze({
  shipping: 'shipment_process_asserted',
  arriving: 'arrival_observed',
  receiving: 'receiving_event_recorded',
  accepting: 'possession_or_ownership_change_asserted'
});
const REQUIRED_NON_EQUIVALENCES = Object.freeze([
  'physical_delivery_verified',
  'product_authenticity_verified',
  'regulatory_compliance',
  'legal_title_verified'
]);
const EXPECTED_VERSIONS = Object.freeze({
  'GS1-EPCIS': '2.0.1',
  'GS1-CBV': '2.0.0'
});
const EXPECTED_MANIFEST_PATH = 'source-snapshots/gs1-epcis-cbv-v1.json';
const EXPECTED_MANIFEST_SHA = 'acb4eee1c6b5ec625047f6857e466408716bb4b09b911029faac303c990cb647';
const EXPECTED_SOURCES = Object.freeze({
  'GS1-EPCIS': {
    url: 'https://ref.gs1.org/standards/epcis/2.0.1/',
    revision: 'EPCIS 2.0.1; published 2025-07-01',
    source_id: 'gs1-epcis-2-0-1'
  },
  'GS1-CBV': {
    url: 'https://ref.gs1.org/standards/cbv/2.0.0/',
    revision: 'CBV 2.0.0; published 2022-06-22',
    source_id: 'gs1-cbv-2-0-0'
  }
});
const EXPECTED_ARTIFACTS = Object.freeze([
  {
    url: 'https://ref.gs1.org/standards/epcis/2.0.0/epcis-context.jsonld',
    source_id: 'gs1-epcis-2-0-0-context',
    revision: 'EPCIS 2.0.0 JSON-LD context explicitly referenced by EPCIS 2.0.1'
  },
  {
    url: 'https://ref.gs1.org/standards/epcis/2.0.0/epcis-json-schema.json',
    source_id: 'gs1-epcis-2-0-0-json-schema',
    revision: 'EPCIS 2.0.0 normative validation artifact explicitly referenced by EPCIS 2.0.1'
  },
  {
    url: 'https://ref.gs1.org/standards/epcis/2.0.0/epcis-shacl.ttl',
    source_id: 'gs1-epcis-2-0-0-shacl',
    revision: 'EPCIS 2.0.0 normative validation artifact explicitly referenced by EPCIS 2.0.1'
  },
  {
    url: 'https://ref.gs1.org/standards/epcis/2.0.0/cbv-ontology.jsonld',
    source_id: 'gs1-cbv-2-0-0-ontology-jsonld',
    revision: 'CBV 2.0.0 JSON-LD ontology associated with the EPCIS/CBV 2.0.0 artifact set'
  }
]);

function validateFixture(fixture, schema, manifest) {
  const errors = [];
  addError(errors, fixture && typeof fixture === 'object' && !Array.isArray(fixture),
    'fixture must be a JSON object');
  if (!fixture || typeof fixture !== 'object' || Array.isArray(fixture)) return errors.concat('fixture invalid');

  addError(errors, fixture.schema_version === '1.0.0', 'unsupported schema version');
  addError(errors, fixture.evaluation_state === 'contract_fixture_only',
    'example must remain a contract fixture, not a measured evaluation');
  addError(errors, fixture.data_classification === 'synthetic', 'fixture must remain synthetic');
  addError(errors, fixture.claim_ceiling === 'PinnedStandardMappingOnly',
    'claim ceiling must remain PinnedStandardMappingOnly');
  addError(errors, fixture.fixture_scope === 'minimal_epcis_document_with_isolated_object_event_cases',
    'fixture scope must state this is a minimal EPCISDocument with isolated cases');
  addError(errors, fixture.projection_policy_id === 'mycelix-epcis-loss-aware-v1',
    'unexpected projection policy');
  addError(errors, Array.isArray(fixture.nonclaims) && fixture.nonclaims.length >= 3,
    'explicit nonclaims are required');

  addError(errors, schema && schema.$schema === 'https://json-schema.org/draft/2020-12/schema',
    'schema must declare JSON Schema Draft 2020-12');
  addError(errors, schema && schema.properties && schema.properties.fixture_scope &&
    schema.properties.fixture_scope.const === fixture.fixture_scope,
    'fixture scope does not match the checked schema contract');
  addError(errors, schema && schema.$defs && schema.$defs.objectEvent &&
    JSON.stringify(schema.$defs.objectEvent.properties.bizStep.enum) === JSON.stringify(Object.keys(STEP_CLAIM)),
    'schema bizStep enum must be bare EPCIS JSON CBV values');
  addError(errors, schema && schema.$defs && schema.$defs.objectEvent &&
    JSON.stringify(schema.$defs.objectEvent.properties.disposition.enum) === JSON.stringify(['in_transit', 'in_progress', 'active']),
    'schema disposition enum must be bare EPCIS JSON CBV values');
  addError(errors, schema && schema.properties && schema.properties.epcis_document &&
    schema.properties.epcis_document.$ref === '#/$defs/epcisDocument',
    'fixture schema must include an EPCISDocument envelope');
  addError(errors, schema && schema.$defs && schema.$defs.epcisDocument &&
    schema.$defs.epcisDocument.properties['@context'] &&
    schema.$defs.epcisDocument.properties['@context'].items.const ===
      'https://ref.gs1.org/standards/epcis/2.0.0/epcis-context.jsonld',
    'schema must pin the versioned EPCIS JSON-LD context');

  // Verify the manifest bytes and the exact source-reference bindings used by
  // the fixture. The manifest is a revision pin, not a copy of standard bytes.
  addError(errors, manifest && manifest.claim_ceiling === 'SourceSnapshotOnly',
    'GS1 source snapshot manifest missing or has unexpected claim ceiling');
  addError(errors, manifest && manifest.profile_id === 'gs1-epcis-cbv',
    'GS1 source snapshot manifest profile id mismatch');
  addError(errors, manifest && manifest.manifest_id === 'gs1-epcis-cbv-v1',
    'GS1 source snapshot manifest id mismatch');
  addError(errors, Array.isArray(manifest && manifest.sources) &&
    manifest.sources.length === Object.keys(EXPECTED_SOURCES).length + EXPECTED_ARTIFACTS.length,
    'GS1 source manifest must bind both standards and all selected normative artefacts');
  const manifestSources = manifest && Array.isArray(manifest.sources) ? manifest.sources : [];
  const sourcesByUrl = new Map(manifestSources.map(source => [source.source_url, source]));
  addError(errors, sourcesByUrl.size === manifestSources.length, 'source manifest contains duplicate source URLs');
  const expectedSourceUrls = [
    ...Object.values(EXPECTED_SOURCES).map(source => source.url),
    ...EXPECTED_ARTIFACTS.map(source => source.url)
  ];
  addError(errors, expectedSourceUrls.length === sourcesByUrl.size &&
    expectedSourceUrls.every(url => sourcesByUrl.has(url)),
    'source manifest URL set differs from the version-pinned standard/artifact set');
  for (const [standardId, expected] of Object.entries(EXPECTED_SOURCES)) {
    const source = sourcesByUrl.get(expected.url);
    addError(errors, !!source, 'source manifest missing standard URL for ' + standardId);
    if (!source) continue;
    addError(errors, source.source_id === expected.source_id, 'source id mismatch for ' + standardId);
    addError(errors, source.source_revision === expected.revision, 'source revision mismatch for ' + standardId);
    addError(errors, source.capture_state === 'exact_revision_pinned', 'source must be an immutable version/revision pin');
    addError(errors, source.integrity_method === 'immutable_source_revision', 'source pin integrity method mismatch');
    addError(errors, source.rights_disposition === 'metadata_only', 'normative artefact rights must not be inferred');
  }
  for (const expected of EXPECTED_ARTIFACTS) {
    const source = sourcesByUrl.get(expected.url);
    addError(errors, !!source, 'source manifest missing normative artefact ' + expected.url);
    if (!source) continue;
    addError(errors, source.source_id === expected.source_id, 'source id mismatch for artefact ' + expected.source_id);
    addError(errors, source.source_revision === expected.revision, 'source revision mismatch for artefact ' + expected.source_id);
    addError(errors, source.source_kind === 'official_standard', 'artefact must be classified as an official standard resource');
    addError(errors, source.capture_state === 'exact_revision_pinned', 'artefact must use a version-pinned URL');
    addError(errors, source.integrity_method === 'immutable_source_revision', 'artefact integrity method mismatch');
    addError(errors, source.rights_disposition === 'metadata_only', 'normative artefact rights must not be inferred');
  }
  const actualManifestSha = sha256(Buffer.from(JSON.stringify(manifest, null, 2) + '\n'));
  addError(errors, actualManifestSha === EXPECTED_MANIFEST_SHA,
    'source manifest content digest differs from the pinned bytes');
  addError(errors, fixture.standard_bindings && fixture.standard_bindings.length === 2,
    'exactly two standard bindings are required');
  const bindingIds = new Set();
  for (const binding of fixture.standard_bindings || []) {
    if (!binding || typeof binding !== 'object') {
      errors.push('standard binding must be an object');
      continue;
    }
    addError(errors, !bindingIds.has(binding.standard_id), 'duplicate standard binding: ' + binding.standard_id);
    bindingIds.add(binding.standard_id);
    addError(errors, EXPECTED_VERSIONS[binding.standard_id] === binding.version,
      'wrong version for standard ' + binding.standard_id);
    addError(errors, binding.source_snapshot_manifest_path === EXPECTED_MANIFEST_PATH,
      'standard binding points to unexpected source manifest');
    addError(errors, binding.source_snapshot_manifest_sha256 === EXPECTED_MANIFEST_SHA,
      'standard binding has wrong source manifest digest');
    const expectedSource = EXPECTED_SOURCES[binding.standard_id];
    addError(errors, !!expectedSource && sourcesByUrl.has(expectedSource.url),
      'binding has no source manifest entry for ' + binding.standard_id);
  }
  addError(errors, bindingIds.has('GS1-EPCIS') && bindingIds.has('GS1-CBV'),
    'both EPCIS and CBV standard bindings are required');
  addError(errors, actualManifestSha.length === 64, 'manifest serialization check failed');

  const epcisDocument = fixture.epcis_document;
  addError(errors, epcisDocument && epcisDocument.type === 'EPCISDocument',
    'document type must be EPCISDocument');
  addError(errors, epcisDocument && epcisDocument.schemaVersion === '2.0',
    'EPCIS schemaVersion must be 2.0');
  addError(errors, epcisDocument && Array.isArray(epcisDocument['@context']) &&
    epcisDocument['@context'].length === 1 &&
    epcisDocument['@context'][0] === 'https://ref.gs1.org/standards/epcis/2.0.0/epcis-context.jsonld',
    'document must use the pinned EPCIS 2.0.1 JSON-LD context');
  addError(errors, epcisDocument && isUtcMilliseconds(epcisDocument.creationDate),
    'document creationDate must be UTC with millisecond precision');
  const eventSteps = new Set();
  const eventById = new Map();
  const epcs = new Set();
  const orderedTimes = [];
  const events = epcisDocument && epcisDocument.epcisBody &&
    Array.isArray(epcisDocument.epcisBody.eventList) ? epcisDocument.epcisBody.eventList : [];
  addError(errors, !!(epcisDocument && epcisDocument.epcisBody &&
    Array.isArray(epcisDocument.epcisBody.eventList)), 'EPCISDocument epcisBody.eventList is missing');
  addError(errors, events.length === 4, 'this bounded fixture must contain exactly four isolated event cases');
  for (const [index, event] of events.entries()) {
    const label = 'events[' + index + ']';
    if (!event || typeof event !== 'object') {
      errors.push(label + ' must be an object');
      continue;
    }
    addError(errors, event.type === 'ObjectEvent', label + ' type must be ObjectEvent');
    addError(errors, typeof event.eventID === 'string' && /^urn:uuid:[0-9a-fA-F-]{36}$/.test(event.eventID),
      label + ' eventID must be a UUID URN');
    addError(errors, !eventById.has(event.eventID), label + ' duplicate eventID');
    eventById.set(event.eventID, event);
    addError(errors, isUtcMilliseconds(event.eventTime), label + ' eventTime must be UTC with millisecond precision');
    addError(errors, isUtcOffset(event.eventTimeZoneOffset), label + ' eventTimeZoneOffset is invalid');
    if (isUtcMilliseconds(event.eventTime)) orderedTimes.push(Date.parse(event.eventTime));
    addError(errors, event.action === 'OBSERVE', label + ' must use OBSERVE in this minimal fixture');
    addError(errors, Object.hasOwn(STEP_CLAIM, event.bizStep), label + ' must use a bare supported CBV bizStep');
    if (typeof event.bizStep === 'string') {
      addError(errors, !event.bizStep.startsWith('https://'), label + ' must not expand CBV URI in EPCIS JSON notation');
      addError(errors, !eventSteps.has(event.bizStep), label + ' duplicates a fixture case bizStep');
      eventSteps.add(event.bizStep);
    }
    addError(errors, ['in_transit', 'in_progress', 'active'].includes(event.disposition),
      label + ' must use a bare CBV disposition');
    addError(errors, typeof event.disposition === 'string' && !event.disposition.startsWith('https://'),
      label + ' must not expand CBV disposition URI in EPCIS JSON notation');
    addError(errors, Array.isArray(event.epcList) && event.epcList.length === 1,
      label + ' must have exactly one EPC in this isolated fixture');
    if (Array.isArray(event.epcList) && event.epcList.length === 1) {
      const epc = event.epcList[0];
      addError(errors, typeof epc === 'string' && /^urn:epc:id:sgtin:\d+(?:\.\d+){2}$/.test(epc),
        label + ' EPC must use SGTIN instance syntax');
      addError(errors, !epcs.has(epc), label + ' shares an EPC with another isolated case');
      epcs.add(epc);
    }
    for (const field of ['readPoint', 'bizLocation']) {
      addError(errors, event[field] && typeof event[field].id === 'string' &&
        /^urn:epc:id:sgln:\d+(?:\.\d+){2}$/.test(event[field].id),
        label + ' ' + field + ' must identify an SGLN location');
    }
    addError(errors, event.disposition === ({
      shipping: 'in_transit',
      arriving: 'in_progress',
      receiving: 'active',
      accepting: 'active'
    })[event.bizStep], label + ' disposition/business-step fixture pairing changed');
  }
  addError(errors, eventSteps.size === 4 && Object.keys(STEP_CLAIM).every(step => eventSteps.has(step)),
    'shipping, arriving, receiving, and accepting must remain distinct cases');
  addError(errors, orderedTimes.length === events.length &&
    orderedTimes.every((value, index) => index === 0 || orderedTimes[index - 1] <= value),
    'fixture event timestamps must be nondecreasing');

  const mappings = Array.isArray(fixture.mappings) ? fixture.mappings : [];
  addError(errors, mappings.length === events.length, 'each event must have exactly one mapping');
  const mappingById = new Map();
  for (const [index, mapping] of mappings.entries()) {
    const label = 'mappings[' + index + ']';
    if (!mapping || typeof mapping !== 'object') {
      errors.push(label + ' must be an object');
      continue;
    }
    addError(errors, !mappingById.has(mapping.source_event_id),
      label + ' duplicate mapping for source event');
    mappingById.set(mapping.source_event_id, mapping);
    const sourceEvent = eventById.get(mapping.source_event_id);
    addError(errors, !!sourceEvent, label + ' references an unknown event');
    if (!sourceEvent) continue;
    addError(errors, mapping.source_biz_step === sourceEvent.bizStep,
      label + ' business step differs from source event');
    addError(errors, mapping.mycelix_claim_type === STEP_CLAIM[sourceEvent.bizStep],
      label + ' claim type promotes, collapses, or mislabels source semantics');
    addError(errors, mapping.qualification_disposition === 'source_assertion_only',
      label + ' cannot upgrade a source assertion to independent verification');
    addError(errors, Array.isArray(mapping.non_equivalences),
      label + ' must declare non-equivalence claims');
    const missing = REQUIRED_NON_EQUIVALENCES.filter(name =>
      !Array.isArray(mapping.non_equivalences) || !mapping.non_equivalences.includes(name));
    addError(errors, missing.length === 0,
      label + ' fails to preserve minimum non-equivalence warnings: ' + missing.join(', '));
  }
  for (const eventId of eventById.keys()) {
    addError(errors, mappingById.has(eventId), 'event has no mapping: ' + eventId);
  }

  // This fixture is deliberately a source-shape/semantic mapping contract only.
  // It does not prove that an event was captured, that a physical delivery
  // occurred, or that a product, owner, or regulatory claim is true.
  return errors;
}

function assertMutationRejected(mutator, label, fixture, schema, manifest) {
  const changed = JSON.parse(JSON.stringify(fixture));
  mutator(changed);
  const errors = validateFixture(changed, schema, manifest);
  if (errors.length === 0) throw new Error('mutation not rejected: ' + label);
  return { mutation: label, rejected: true, invariant_failures: errors.length };
}

function main() {
  const args = process.argv.slice(2);
  const selfTest = args.includes('--self-test');
  const positional = args.filter(arg => arg !== '--self-test');
  const fixturePath = positional[0] || fixtureDefault;
  const schemaPath = positional[1] || schemaDefault;
  const manifestPath = positional[2] || manifestDefault;
  const fixtureBytes = fs.readFileSync(fixturePath);
  const schemaBytes = fs.readFileSync(schemaPath);
  const manifestBytes = fs.readFileSync(manifestPath);
  const fixture = JSON.parse(fixtureBytes.toString('utf8'));
  const schema = JSON.parse(schemaBytes.toString('utf8'));
  const manifest = JSON.parse(manifestBytes.toString('utf8'));
  const errors = validateFixture(fixture, schema, manifest);
  if (errors.length) fail(errors.join('\n'));

  // Bind the fixture to exact manifest bytes, rather than the reserialized form.
  const manifestSha = sha256(manifestBytes);
  for (const binding of fixture.standard_bindings) {
    if (binding.source_snapshot_manifest_sha256 !== manifestSha) {
      fail('fixture standard binding digest does not match exact source-manifest bytes');
    }
  }

  const mutations = selfTest ? [
    assertMutationRejected(b => { b.epcis_document.epcisBody.eventList[0].bizStep = 'https://ref.gs1.org/cbv/BizStep-shipping'; }, 'expanded-bizstep-uri', fixture, schema, manifest),
    assertMutationRejected(b => { b.epcis_document.epcisBody.eventList[0].disposition = 'https://ref.gs1.org/cbv/Disp-in_transit'; }, 'expanded-disposition-uri', fixture, schema, manifest),
    assertMutationRejected(b => { b.mappings[0].mycelix_claim_type = 'possession_or_ownership_change_asserted'; }, 'shipping-promoted-to-ownership-change', fixture, schema, manifest),
    assertMutationRejected(b => { b.mappings[0].non_equivalences = b.mappings[0].non_equivalences.filter(x => x !== 'physical_delivery_verified'); }, 'physical-delivery-non-equivalence-removed', fixture, schema, manifest),
    assertMutationRejected(b => { b.epcis_document.epcisBody.eventList[1].eventID = b.epcis_document.epcisBody.eventList[0].eventID; }, 'duplicate-event-id', fixture, schema, manifest),
    assertMutationRejected(b => { b.mappings[1].source_event_id = b.mappings[0].source_event_id; }, 'duplicate-mapping-target', fixture, schema, manifest),
    assertMutationRejected(b => { b.mappings[1].source_biz_step = 'receiving'; }, 'mapping-source-step-mismatch', fixture, schema, manifest),
    assertMutationRejected(b => { b.mappings.pop(); }, 'missing-event-mapping', fixture, schema, manifest),
    assertMutationRejected(b => { b.epcis_document.epcisBody.eventList[1].epcList[0] = b.epcis_document.epcisBody.eventList[0].epcList[0]; }, 'two-cases-share-epc', fixture, schema, manifest),
    assertMutationRejected(b => { b.standard_bindings[0].version = '2.0.0'; }, 'epcis-version-downgrade', fixture, schema, manifest),
    assertMutationRejected(b => { b.epcis_document.epcisBody.eventList[0].eventTime = '2026-10-21T08:00:00+02:00'; }, 'non-utc-event-time', fixture, schema, manifest),
    assertMutationRejected(b => { b.standard_bindings[1].source_snapshot_manifest_sha256 = '0000000000000000000000000000000000000000000000000000000000000000'; }, 'source-manifest-digest-tamper', fixture, schema, manifest)
  ] : [];

  const receipt = {
    benchmark_id: 'MYC-GS1-EPCIS-LOSS-001',
    result: 'ClaimSeparationFixtureConforms',
    fixture_state: fixture.evaluation_state,
    claim_ceiling: fixture.claim_ceiling,
    fixture_scope: fixture.fixture_scope,
    event_count: events.length,
    mapping_count: fixture.mappings.length,
    distinct_biz_steps: [...new Set(events.map(event => event.bizStep))].sort(),
    all_events_mapped_exactly_once: events.every(event =>
      fixture.mappings.filter(mapping => mapping.source_event_id === event.eventID).length === 1),
    all_mappings_source_assertion_only: fixture.mappings.every(mapping =>
      mapping.qualification_disposition === 'source_assertion_only'),
    non_equivalence_floor_met: fixture.mappings.every(mapping =>
      REQUIRED_NON_EQUIVALENCES.every(name => mapping.non_equivalences.includes(name))),
    standards: fixture.standard_bindings.map(binding => ({
      id: binding.standard_id,
      version: binding.version,
      source_manifest_path: binding.source_snapshot_manifest_path,
      source_manifest_sha256: binding.source_snapshot_manifest_sha256
    })),
    fixture_sha256: sha256(fixtureBytes),
    schema_sha256: sha256(schemaBytes),
    source_manifest_sha256: manifestSha,
    verifier_sha256: sha256(fs.readFileSync(__filename)),
    self_test_executed: selfTest,
    mutation_cases: mutations.length,
    mutation_cases_rejected: mutations.filter(item => item.rejected).length,
    schema_instance_validation: 'not_performed_by_this_verifier',
    normative_gs1_artifact_validation: 'not_performed_by_this_verifier',
    nonclaims: [
      'This receipt covers fixture-level cross-invariants, not full EPCIS/CBV conformance.',
      'The verifier does not run a vendor adapter or prove that a real-world event occurred.',
      'No physical-delivery, product-authenticity, legal-title, regulatory-compliance, or SOTA claim is established.',
      'JSON Schema instance validation by a conforming Draft 2020-12 validator is a separate evaluation lane.'
    ]
  };
  process.stdout.write(JSON.stringify(receipt, null, 2) + '\n');
}

main();
