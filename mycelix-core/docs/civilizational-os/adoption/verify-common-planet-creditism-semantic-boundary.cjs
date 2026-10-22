#!/usr/bin/env node
'use strict';

// Bounded, zero-dependency evaluator for a synthetic Common Planet Creditism
// semantic contract. This is a reference witness, not a production allocator,
// currency ledger, contribution oracle, or financial-accounting implementation.

const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');

const root = __dirname;
const corpusPath = path.join(root, 'common-planet-creditism-semantic-boundary-v1.json');
const schemaPath = path.join(root, 'common-planet-creditism-semantic-boundary-v1.schema.json');
const sha256 = bytes => crypto.createHash('sha256').update(bytes).digest('hex');
const DIGEST = /^[0-9a-f]{64}$/;
const UUID_URN = /^urn:uuid:[0-9a-fA-F-]{36}$/;
const REQUIRED_INVARIANTS = [
  'buyer credit deletion != seller financial payment',
  'consumer settlement != contribution recognition',
  'contribution recognition != moral worth',
  'credit balance != wealth',
  'credit issuance != productive capacity',
  'allocation authorization != resource reservation != physical delivery',
  'unknown sna mapping != ordinary financial instrument',
  'stored personal credit != transferable wealth',
  'external conventional liability != domestic personal credit offset',
  'evaluator identity != verification authority',
  'event frequency != contribution value',
  'latent outcome != observed evidence',
  'future negative evidence != current qualified observation'
];

function equalJson];

function equalJson(a, b) {
  return JSON.stringify(a) === JSON.stringify(b);
}
function isRecord(value) {
  return value !== null && typeof value === 'object' && !Array.isArray(value);
}
function diffPaths(left, right, prefix = '') {
  if (equalJson(left, right)) return [];
  if (isRecord(left) && isRecord(right)) {
    const keys = new Set([...Object.keys(left), ...Object.keys(right)]);
    const differences = [];
    for (const key of keys) {
      const path = prefix ? prefix + '.' + key : key;
      const hasLeft = Object.hasOwn(left, key);
      const hasRight = Object.hasOwn(right, key);
      if (!hasLeft || !hasRight) {
        differences.push(path);
      } else {
        differences.push(...diffPaths(left[key], right[key], path));
      }
    }
    return differences;
  }
  return [prefix || '
function fail(message) {
  process.stderr.write('FAIL: ' + message + '\n');
  process.exit(1);
}
function readJson(filename) {
  try {
    return JSON.parse(fs.readFileSync(filename, 'utf8'));
  } catch (error) {
    fail('cannot read ' + filename + ': ' + error.message);
  }
}
function result(decision, disposition, effects, prohibitedInferences) {
  return { decision, disposition, effects, prohibited_inferences: prohibitedInferences };
}
function positiveNumber(value) {
  return typeof value === 'number' && Number.isFinite(value) && value > 0;
}
function uniqueNonEmptyStrings(values) {
  return Array.isArray(values) &&
    values.length > 0 &&
    values.every(value => typeof value === 'string' && value.trim().length > 0) &&
    new Set(values).size === values.length;
}

const SUPPORTED_OPERATIONS = Object.freeze([
  'issue_personal_credit',
  'delete_personal_credit',
  'retain_personal_credit_balance',
  'recognize_contribution',
  'transfer_personal_credit',
  'lend_personal_credit',
  'collateralize_personal_credit',
  'accrue_interest',
  'invest_for_return',
  'inherit_concentrated_personal_credit',
  'allocate_community_credit',
  'recognize_bonus',
  'exchange_pair',
  'project_to_sna',
  'grant_housing_right'
]);

const FORBIDDEN_PERSONAL_CREDIT_OPERATIONS = Object.freeze({
  transfer_personal_credit: ['recipient_balance_change', 'gift_or_transfer_authority'],
  lend_personal_credit: ['loan_claim', 'borrower_debt'],
  collateralize_personal_credit: ['security_interest', 'recoverable_collateral'],
  accrue_interest: ['interest_income', 'yield_claim'],
  invest_for_return: ['investment_asset', 'autonomous_return'],
  inherit_concentrated_personal_credit: ['inherited_concentrated_power', 'transferable_wealth']
});

function evaluate(input) {
  if (!input || typeof input !== 'object' || Array.isArray(input) ||
      typeof input.operation !== 'string') {
    return result('Indeterminate', 'Unknown', [], ['unsupported_operation']);
  }

  const op = input.operation;
  if (Object.hasOwn(FORBIDDEN_PERSONAL_CREDIT_OPERATIONS, op)) {
    if (input.instrument !== 'PersonalCredit') {
      return result('Indeterminate', 'Unknown', [], ['instrument_semantics_unqualified']);
    }
    return result('Reject', 'Unsupported', [], FORBIDDEN_PERSONAL_CREDIT_OPERATIONS[op]);
  }

  switch (op) {
    case 'issue_personal_credit': {
      if (input.instrument !== 'PersonalCredit' ||
          input.recipient_scope !== 'person' ||
          !positiveNumber(input.amount)) {
        return result('Reject', 'Unsupported', [], ['invalid_issuance_scope']);
      }
      if (input.reason === 'baseline') {
        return result('Allow', 'ProfileMapped', ['personal_credit_balance_increase'],
          ['ordinary_financial_asset', 'seller_payment', 'debt_claim']);
      }
      return result('Indeterminate', 'Unknown', [],
        ['unqualified_contribution_issuance', 'ordinary_financial_asset']);
    }

    case 'delete_personal_credit': {
      if (input.instrument !== 'PersonalCredit' ||
          input.reason !== 'consumer_purchase' ||
          !positiveNumber(input.amount) ||
          typeof input.seller_payment_observed !== 'boolean') {
        return result('Reject', 'Unsupported', [], ['invalid_deletion_scope']);
      }
      return result('Allow', 'ProfileMapped', ['buyer_personal_credit_balance_decrease'],
        ['seller_payment', 'contribution_recognition', 'producer_financial_asset']);
    }

    case 'retain_personal_credit_balance': {
      if (input.instrument !== 'PersonalCredit' ||
          input.reason !== 'future_use' ||
          !positiveNumber(input.amount)) {
        return result('Reject', 'Unsupported', [], ['invalid_retained_balance_scope']);
      }
      return result('Allow', 'ProfileMapped', ['personal_credit_balance_retained'],
        ['transferability', 'ordinary_financial_asset', 'wealth_equivalence']);
    }

    case 'recognize_contribution': {
      if (Array.isArray(input.evidence_refs) &&
          new Set(input.evidence_refs).size !== input.evidence_refs.length) {
        return result('Reject', 'Unsupported', [], ['duplicate_contribution_issuance']);
      }
      if (input.outcome_laundering_flag === true) {
        return result('Reject', 'Unsupported', [], ['outcome_laundering_detected']);
      }
      if (input.verifier_relationship === 'colluding') {
        return result('Reject', 'Unsupported', [], ['verifier_collusion_not_independent']);
      }
      if (input.verifier_panel_state === 'captured') {
        return result('PendingAppeal', 'Unknown', [],
          ['contribution_issuance_before_review', 'verifier_panel_review_required']);
      }
      if (input.dispute_state === 'disputed') {
        const prohibited = ['contribution_issuance_before_review', 'moral_worth_score'];
        if (typeof input.appeal_route !== 'string' || input.appeal_route.trim().length === 0) {
          prohibited.push('appeal_route_suppression');
        }
        return result('PendingAppeal', 'Unknown', [], prohibited);
      }
      if (input.identity_uniqueness_state === 'unresolved') {
        return result('Indeterminate', 'Unknown', [],
          ['identity_uniqueness_unresolved', 'contribution_credit_issuance']);
      }
      if (input.task_decomposition_state === 'unreviewed') {
        return result('Indeterminate', 'Unknown', [],
          ['task_decomposition_unreviewed', 'contribution_credit_issuance']);
      }
      if (input.metric_profile === 'event_frequency_only' &&
          (!Array.isArray(input.quality_evidence_refs) || input.quality_evidence_refs.length === 0)) {
        return result('Indeterminate', 'Unknown', [],
          ['event_frequency_as_contribution_value', 'contribution_credit_issuance']);
      }
      if (input.activity_source === 'model_generated' && input.independently_observed !== true) {
        return result('Indeterminate', 'Unknown', [],
          ['model_generated_activity_unverified', 'contribution_credit_issuance']);
      }
      if (input.derived_only_from_transaction_settlement === true ||
          !positiveNumber(input.amount) ||
          !DIGEST.test(input.criteria_digest || '') ||
          !uniqueNonEmptyStrings(input.evidence_refs) ||
          typeof input.verifier_authority_scope !== 'string' ||
          input.verifier_authority_scope.trim().length === 0 ||
          input.corroboration_state !== 'qualified' ||
          input.dispute_state !== 'none') {
        return result('Indeterminate', 'Unknown', [],
          ['contribution_credit_issuance', 'moral_worth_score']);
      }
      if (input.evidence_privacy_mode === 'opaque_commitment') {
        if (input.personal_payload_disclosed !== false) {
          return result('Reject', 'Unsupported', [], ['unnecessary_personal_payload_disclosure']);
        }
        return result('Allow', 'ProfileMapped',
          ['bounded_completion_fact_recorded', 'personal_credit_issuance_allowed'],
          ['surveillance_requirement', 'moral_worth_score']);
      }
      return result('Allow', 'ProfileMapped',
        ['contribution_recognition_recorded', 'personal_credit_issuance_allowed'],
        ['moral_worth_score', 'causal_impact_without_evidence']);
    }

    case 'allocate_community_credit': {
      if (input.instrument !== 'CommunityCredit' ||
          input.account_scope !== 'community_project' ||
          !positiveNumber(input.requested_amount)) {
        return result('Reject', 'Unsupported', [], ['invalid_community_allocation_scope']);
      }
      if (input.capacity_state !== 'known' ||
          typeof input.available_capacity !== 'number' ||
          !Number.isFinite(input.available_capacity) ||
          input.available_capacity < 0 ||
          input.authorization_state !== 'authorized') {
        return result('Indeterminate', 'Unknown', [],
          ['unlimited_capacity', 'physical_delivery']);
      }
      if (input.requested_amount > input.available_capacity) {
        return result('Reject', 'Unsupported', [],
          ['capacity_override', 'physical_delivery']);
      }
      return result('Allow', 'ProfileMapped', ['restricted_community_credit_allocation_recorded'],
        ['personal_credit_balance_increase', 'physical_delivery', 'project_completion']);
    }

    case 'recognize_bonus': {
      if (input.dispute_state === 'disputed') {
        return result('PendingAppeal', 'Unknown', [], ['bonus_issuance_before_review']);
      }
      if (input.same_outcome_already_counted_in_contribution_pay === true) {
        return result('Reject', 'Unsupported', [], ['double_counted_recognition']);
      }
      if (!positiveNumber(input.amount) ||
          !DIGEST.test(input.criteria_digest || '') ||
          typeof input.outcome_key !== 'string' ||
          input.outcome_key.trim().length === 0 ||
          !uniqueNonEmptyStrings(input.outcome_evidence_refs) ||
          typeof input.verifier_authority_scope !== 'string' ||
          input.verifier_authority_scope.trim().length === 0 ||
          input.dispute_state !== 'none') {
        return result('Indeterminate', 'Unknown', [],
          ['bonus_issuance_without_qualified_outcome']);
      }
      return result('Allow', 'ProfileMapped', ['bonus_recognition_recorded'],
        ['automatic_contribution_pay', 'moral_worth_score']);
    }

    case 'exchange_pair': {
      const buyer = input.buyer_deletion;
      const seller = input.seller_recredit;
      if (input.instrument !== 'PersonalCredit' ||
          !buyer || !seller ||
          !UUID_URN.test(buyer.event_id || '') ||
          !UUID_URN.test(seller.event_id || '') ||
          buyer.event_id === seller.event_id ||
          !positiveNumber(buyer.amount) ||
          !positiveNumber(seller.amount) ||
          buyer.amount !== seller.amount) {
        return result('Reject', 'Unsupported', [], ['ambiguous_exchange_accounting']);
      }
      return result('Allow', 'ProfileMapped',
        ['buyer_personal_credit_deletion_recorded', 'seller_personal_credit_recredit_recorded'],
        ['ordinary_seller_financial_asset', 'contribution_recognition_from_deletion']);
    }

    case 'project_to_sna': {
      const effects = ['native_instrument_semantics_preserved'];
      const externalLiabilities = Array.isArray(input.conventional_external_liabilities) &&
        input.conventional_external_liabilities.length > 0;
      if (externalLiabilities) effects.push('external_liability_kept_separate');
      if (input.mapping_disposition === 'Unknown' ||
          input.mapping_disposition === 'Unsupported' ||
          input.semantic_equivalence_established !== true) {
        const prohibited = ['deposit_equivalence', 'financial_asset_coercion'];
        if (externalLiabilities && input.fx_conversion_enabled !== true) {
          prohibited.unshift('direct_offset_of_foreign_liability');
        }
        return result('Indeterminate', 'Unknown', effects, prohibited);
      }
      return result('Indeterminate', 'Unknown', effects,
        ['unsupported_sna_equivalence']);
    }

    case 'grant_housing_right': {
      if (input.capacity_state !== 'known' ||
          !Number.isInteger(input.available_dwelling_count) ||
          input.available_dwelling_count < 0) {
        return result('Indeterminate', 'Unknown', [],
          ['housing_entitlement_from_credit_balance', 'unlimited_housing_capacity']);
      }
      if (input.available_dwelling_count === 0 ||
          input.eligibility_verified !== true ||
          input.stewardship_agreement !== true) {
        return result('Reject', 'Unsupported', [],
          ['housing_entitlement_without_resource_and_agreement']);
      }
      return result('Allow', 'ProfileMapped', ['residence_right_recorded'],
        ['housing_as_monetary_balance', 'unbounded_resource_claim']);
    }

    default:
      return result('Indeterminate', 'Unknown', [], ['unsupported_operation']);
  }
}

function corpusErrors(corpus) {
  const errors = [];
  const add = (condition, message) => { if (!condition) errors.push(message); };
  add(corpus && corpus.schema_version === '1.0.0', 'unsupported corpus schema_version');
  add(corpus && corpus.benchmark_id === 'CP-CREDITISM-SEMANTIC-BOUNDARY-001', 'benchmark id mismatch');
  add(corpus && corpus.profile_id === 'common-planet-creditism', 'profile id mismatch');
  add(corpus && corpus.evaluation_state === 'contract_fixture_only', 'fixture must remain contract-only');
  add(corpus && corpus.claim_ceiling === 'ReferenceModelOnly', 'claim ceiling must remain ReferenceModelOnly');
  add(corpus && corpus.data_classification === 'synthetic', 'corpus must remain synthetic');
  add(corpus && corpus.case_count === 32, 'declared case_count must remain 32');
  add(corpus && Array.isArray(corpus.cases) && corpus.cases.length === corpus.case_count,
    'case_count differs from cases array length');
  add(corpus && Array.isArray(corpus.metamorphic_properties) &&
      corpus.metamorphic_properties.length >= 9 && corpus.metamorphic_properties.length <= 15,
    'metamorphic property suite must contain 9..15 invariance checks');
  const owners = new Set(corpus.related_owners || []);
  add(owners.has('mycelix#4572') && owners.has('mycelix#4573') &&
      owners.has('mycelix#4771') && owners.has('mycelix#5366') &&
      owners.has('symthaea#7031'), 'corpus must name existing finance, contribution, capacity, water-loop, and Creditism owners');
  const invariants = corpus.source_semantics?.separation_invariants || [];
  for (const invariant of REQUIRED_INVARIANTS) {
    add(invariants.includes(invariant), 'required semantic separation missing: ' + invariant);
  }

  const ids = new Set();
  for (const [index, item] of (corpus.cases || []).entries()) {
    const label = 'cases[' + index + ']';
    add(item && typeof item.id === 'string' && /^cp-[a-z0-9]+(?:-[a-z0-9]+)*$/.test(item.id),
      label + ' has invalid id');
    add(!ids.has(item && item.id), label + ' duplicate id');
    if (item) ids.add(item.id);
    add(item && item.input && typeof item.input.operation === 'string',
      label + ' has invalid input operation');
    add(item && item.input && SUPPORTED_OPERATIONS.includes(item.input.operation),
      label + ' uses an operation outside the versioned profile contract');
    add(item && item.expected && ['Allow', 'Reject', 'Indeterminate', 'PendingAppeal'].includes(item.expected.decision),
      label + ' has invalid expected decision');
    if (item && item.expected) {
      const observed = evaluate(item.input);
      add(equalJson(observed, item.expected),
        label + ' expected output differs from evaluator output for ' + item.id);
    }
  }

  const metamorphicIds = new Set();
  for (const [index, property] of (corpus.metamorphic_properties || []).entries()) {
    const label = 'metamorphic_properties[' + index + ']';
    add(property && typeof property.property_id === 'string',
      label + ' has no property_id');
    add(!metamorphicIds.has(property && property.property_id),
      label + ' duplicate property_id');
    if (property) metamorphicIds.add(property.property_id);
    add(property && ids.has(property.case_id),
      label + ' references an unknown base case');
    add(property && property.expected_relation === 'evaluation_outputs_equal',
      label + ' has an unsupported relation');
    add(property && Array.isArray(property.input_variants) && property.input_variants.length === 2,
      label + ' must have exactly two input variants');
    add(property && Array.isArray(property.varied_fields) && property.varied_fields.length > 0,
      label + ' must declare at least one varied input field');
    if (property && Array.isArray(property.input_variants) && property.input_variants.length === 2) {
      for (const [variantIndex, variant] of property.input_variants.entries()) {
        add(variant && typeof variant === 'object' && !Array.isArray(variant),
          label + ' input variant ' + variantIndex + ' must be an object');
        add(variant && SUPPORTED_OPERATIONS.includes(variant.operation),
          label + ' input variant ' + variantIndex + ' uses an unsupported operation');
      }
      const actualDiffs = diffPaths(property.input_variants[0], property.input_variants[1]).sort();
      const declaredDiffs = Array.isArray(property.varied_fields) ?
        [...property.varied_fields].sort() : [];
      add(actualDiffs.length > 0,
        label + ' input variants must actually differ');
      add(equalJson(actualDiffs, declaredDiffs),
        label + ' changed input fields do not exactly match varied_fields');
      const [left, right] = property.input_variants.map(evaluate);
      add(equalJson(left, right),
        label + ' violates its claimed non-interference/equivalence property');
    }
  }
  return errors;
}

function mustTransition(name, before, mutate, expectedDecision) {
  const changed = JSON.parse(JSON.stringify(before));
  mutate(changed);
  const output = evaluate(changed);
  if (output.decision !== expectedDecision) {
    throw new Error(name + ': expected ' + expectedDecision + ', got ' + output.decision);
  }
  return { name, passed: true, decision: output.decision };
}

function runMutationTests(corpus) {
  const find = id => {
    const item = corpus.cases.find(entry => entry.id === id);
    if (!item) throw new Error('missing mutation base case: ' + id);
    return JSON.parse(JSON.stringify(item.input));
  };
  const results = [
    mustTransition('personal-credit-transfer-denied', find('cp-pc-baseline-issuance'),
      input => { input.operation = 'transfer_personal_credit'; input.instrument = 'PersonalCredit'; }, 'Reject'),
    mustTransition('settlement-only-contribution-indeterminate', find('cp-qualified-contribution'),
      input => { input.derived_only_from_transaction_settlement = true; }, 'Indeterminate'),
    mustTransition('missing-contribution-evidence-indeterminate', find('cp-qualified-contribution'),
      input => { input.evidence_refs = []; }, 'Indeterminate'),
    mustTransition('duplicate-contribution-evidence-rejected', find('cp-qualified-contribution'),
      input => { input.evidence_refs = [input.evidence_refs[0], input.evidence_refs[0]]; }, 'Reject'),
    mustTransition('dispute-enters-appeal', find('cp-qualified-contribution'),
      input => { input.dispute_state = 'disputed'; }, 'PendingAppeal'),
    mustTransition('unknown-capacity-indeterminate', find('cp-community-allocation-within-capacity'),
      input => { input.capacity_state = 'unknown'; input.available_capacity = null; }, 'Indeterminate'),
    mustTransition('over-capacity-rejected', find('cp-community-allocation-within-capacity'),
      input => { input.requested_amount = input.available_capacity + 1; }, 'Reject'),
    mustTransition('bonus-double-count-rejected', find('cp-bonus-qualified-outcome'),
      input => { input.same_outcome_already_counted_in_contribution_pay = true; }, 'Reject'),
    mustTransition('duplicate-exchange-event-rejected', find('cp-exchange-events-separate'),
      input => { input.seller_recredit.event_id = input.buyer_deletion.event_id; }, 'Reject'),
    mustTransition('housing-without-known-capacity-indeterminate', find('cp-housing-needs-capacity'),
      input => { input.capacity_state = 'unknown'; }, 'Indeterminate'),
    mustTransition('unknown-operation-fails-closed', find('cp-pc-baseline-issuance'),
      input => { input.operation = 'invent_new_financial_asset'; }, 'Indeterminate'),
    mustTransition('retained-personal-credit-is-not-transferable', find('cp-pc-retained-for-future-use'),
      input => { input.operation = 'transfer_personal_credit'; input.instrument = 'PersonalCredit'; }, 'Reject'),
    mustTransition('verifier-collusion-rejected', find('cp-qualified-contribution'),
      input => { input.verifier_relationship = 'colluding'; }, 'Reject'),
    mustTransition('sybil-identity-indeterminate', find('cp-qualified-contribution'),
      input => { input.identity_uniqueness_state = 'unresolved'; }, 'Indeterminate'),
    mustTransition('task-decomposition-indeterminate', find('cp-qualified-contribution'),
      input => { input.task_decomposition_state = 'unreviewed'; }, 'Indeterminate'),
    mustTransition('frequency-only-metric-indeterminate', find('cp-qualified-contribution'),
      input => { input.metric_profile = 'event_frequency_only'; input.quality_evidence_refs = []; }, 'Indeterminate'),
    mustTransition('outcome-laundering-rejected', find('cp-qualified-contribution'),
      input => { input.outcome_laundering_flag = true; }, 'Reject'),
    mustTransition('captured-verifier-panel-pending', find('cp-qualified-contribution'),
      input => { input.verifier_panel_state = 'captured'; }, 'PendingAppeal'),
    mustTransition('model-generated-activity-indeterminate', find('cp-qualified-contribution'),
      input => { input.activity_source = 'model_generated'; input.independently_observed = false; }, 'Indeterminate'),
    mustTransition('missing-appeal-route-stays-pending', find('cp-disputed-contribution-paused'),
      input => { input.appeal_route = null; }, 'PendingAppeal')
  ];
  const tamperedExpected = JSON.parse(JSON.stringify(corpus));
  tamperedExpected.cases[0].expected.decision = 'Reject';
  const expectedMutationRejected = corpusErrors(tamperedExpected).length > 0;
  if (!expectedMutationRejected) throw new Error('tampered expected output was not rejected');
  results.push({ name: 'tampered-known-answer-rejected', passed: true, decision: 'RejectedByCorpusVerifier' });

  const missingInvariant = JSON.parse(JSON.stringify(corpus));
  missingInvariant.source_semantics.separation_invariants =
    missingInvariant.source_semantics.separation_invariants.filter(x => x !== 'consumer settlement != contribution recognition');
  const invariantMutationRejected = corpusErrors(missingInvariant).length > 0;
  if (!invariantMutationRejected) throw new Error('missing required semantic invariant was not rejected');
  results.push({ name: 'missing-required-invariant-rejected', passed: true, decision: 'RejectedByCorpusVerifier' });

  const tamperedMetamorphic = JSON.parse(JSON.stringify(corpus));
  tamperedMetamorphic.metamorphic_properties[0].input_variants[1].operation = 'transfer_personal_credit';
  const metamorphicMutationRejected = corpusErrors(tamperedMetamorphic).length > 0;
  if (!metamorphicMutationRejected) throw new Error('broken metamorphic relation was not rejected');
  results.push({ name: 'tampered-metamorphic-relation-rejected', passed: true, decision: 'RejectedByCorpusVerifier' });

  const tamperedFields = JSON.parse(JSON.stringify(corpus));
  tamperedFields.metamorphic_properties[0].varied_fields = ['buyer_event_id', 'amount'];
  const variedFieldsMutationRejected = corpusErrors(tamperedFields).length > 0;
  if (!variedFieldsMutationRejected) throw new Error('bad varied_fields declaration was not rejected');
  results.push({ name: 'tampered-varied-fields-rejected', passed: true, decision: 'RejectedByCorpusVerifier' });
  return results;
}

function main() {
  const selfTest = process.argv.includes('--self-test');
  const corpusBytes = fs.readFileSync(corpusPath);
  const schemaBytes = fs.readFileSync(schemaPath);
  const corpus = JSON.parse(corpusBytes.toString('utf8'));
  const schema = JSON.parse(schemaBytes.toString('utf8'));
  const errors = corpusErrors(corpus);
  if (errors.length) fail(errors.join('\n'));
  if (schema.$schema !== 'https://json-schema.org/draft/2020-12/schema') {
    fail('schema must declare Draft 2020-12');
  }
  if (!equalJson(schema.properties.cases.items.$ref, '#/$defs/case')) {
    fail('schema case reference changed');
  }
  if (schema.properties.case_count.const !== 32 ||
      schema.properties.cases.minItems !== 32 ||
      schema.properties.cases.maxItems !== 32 ||
      schema.properties.metamorphic_properties.minItems !== 9) {
    fail('schema corpus/metamorphic count contract changed');
  }
  if (!equalJson(schema.$defs.case.properties.input.properties.operation.enum, SUPPORTED_OPERATIONS)) {
    fail('schema operation enum differs from the evaluator supported-operation contract');
  }
  if (!equalJson(schema.properties.metamorphic_properties.items.$ref, '#/$defs/metamorphicProperty')) {
    fail('schema metamorphic property reference changed');
  }
  if (schema.$defs.metamorphicProperty.properties.expected_relation.const !== 'evaluation_outputs_equal') {
    fail('schema metamorphic relation must remain equality-based');
  }
  if (!schema.$defs.metamorphicProperty.required.includes('varied_fields') ||
      schema.$defs.metamorphicProperty.properties.varied_fields.type !== 'array') {
    fail('schema must require declared metamorphic varied_fields');
  }

  const mutations = selfTest ? runMutationTests(corpus) : [];
  const receipt = {
    benchmark_id: corpus.benchmark_id,
    result: 'CreditismSemanticBoundaryConforms',
    evaluation_state: corpus.evaluation_state,
    claim_ceiling: corpus.claim_ceiling,
    corpus_sha256: sha256(corpusBytes),
    schema_sha256: sha256(schemaBytes),
    case_count: corpus.cases.length,
    known_answer_cases_passed: corpus.cases.length,
    metamorphic_property_count: corpus.metamorphic_properties.length,
    metamorphic_properties_passed: corpus.metamorphic_properties.filter(property =>
      equalJson(evaluate(property.input_variants[0]), evaluate(property.input_variants[1]))).length,
    decisions: corpus.cases.reduce((counts, item) => {
      counts[item.expected.decision] = (counts[item.expected.decision] || 0) + 1;
      return counts;
    }, {}),
    owner_refs: corpus.related_owners,
    self_test_executed: selfTest,
    mutation_cases: mutations.length,
    mutation_cases_passed: mutations.filter(item => item.passed).length,
    nonclaims: [
      'This evaluates a synthetic reference policy, not production Common Planet or Mycelix settlement.',
      'A profile-mapped result is not evidence that contribution occurred or that an outcome was caused.',
      'Unknown and disputed inputs intentionally fail closed; a future pilot needs an appealable, qualified evidence process.',
      'No SNA equivalence, seller payment, physical delivery, moral value, economic superiority, or SOTA claim is established.'
    ]
  };
  process.stdout.write(JSON.stringify(receipt, null, 2) + '\n');
}

main();
];
}
function fail(message) {
  process.stderr.write('FAIL: ' + message + '\n');
  process.exit(1);
}
function readJson(filename) {
  try {
    return JSON.parse(fs.readFileSync(filename, 'utf8'));
  } catch (error) {
    fail('cannot read ' + filename + ': ' + error.message);
  }
}
function result(decision, disposition, effects, prohibitedInferences) {
  return { decision, disposition, effects, prohibited_inferences: prohibitedInferences };
}
function positiveNumber(value) {
  return typeof value === 'number' && Number.isFinite(value) && value > 0;
}
function uniqueNonEmptyStrings(values) {
  return Array.isArray(values) &&
    values.length > 0 &&
    values.every(value => typeof value === 'string' && value.trim().length > 0) &&
    new Set(values).size === values.length;
}

const SUPPORTED_OPERATIONS = Object.freeze([
  'issue_personal_credit',
  'delete_personal_credit',
  'retain_personal_credit_balance',
  'recognize_contribution',
  'transfer_personal_credit',
  'lend_personal_credit',
  'collateralize_personal_credit',
  'accrue_interest',
  'invest_for_return',
  'inherit_concentrated_personal_credit',
  'allocate_community_credit',
  'recognize_bonus',
  'exchange_pair',
  'project_to_sna',
  'grant_housing_right'
]);

const FORBIDDEN_PERSONAL_CREDIT_OPERATIONS = Object.freeze({
  transfer_personal_credit: ['recipient_balance_change', 'gift_or_transfer_authority'],
  lend_personal_credit: ['loan_claim', 'borrower_debt'],
  collateralize_personal_credit: ['security_interest', 'recoverable_collateral'],
  accrue_interest: ['interest_income', 'yield_claim'],
  invest_for_return: ['investment_asset', 'autonomous_return'],
  inherit_concentrated_personal_credit: ['inherited_concentrated_power', 'transferable_wealth']
});

function evaluate(input) {
  if (!input || typeof input !== 'object' || Array.isArray(input) ||
      typeof input.operation !== 'string') {
    return result('Indeterminate', 'Unknown', [], ['unsupported_operation']);
  }

  const op = input.operation;
  if (Object.hasOwn(FORBIDDEN_PERSONAL_CREDIT_OPERATIONS, op)) {
    if (input.instrument !== 'PersonalCredit') {
      return result('Indeterminate', 'Unknown', [], ['instrument_semantics_unqualified']);
    }
    return result('Reject', 'Unsupported', [], FORBIDDEN_PERSONAL_CREDIT_OPERATIONS[op]);
  }

  switch (op) {
    case 'issue_personal_credit': {
      if (input.instrument !== 'PersonalCredit' ||
          input.recipient_scope !== 'person' ||
          !positiveNumber(input.amount)) {
        return result('Reject', 'Unsupported', [], ['invalid_issuance_scope']);
      }
      if (input.reason === 'baseline') {
        return result('Allow', 'ProfileMapped', ['personal_credit_balance_increase'],
          ['ordinary_financial_asset', 'seller_payment', 'debt_claim']);
      }
      return result('Indeterminate', 'Unknown', [],
        ['unqualified_contribution_issuance', 'ordinary_financial_asset']);
    }

    case 'delete_personal_credit': {
      if (input.instrument !== 'PersonalCredit' ||
          input.reason !== 'consumer_purchase' ||
          !positiveNumber(input.amount) ||
          typeof input.seller_payment_observed !== 'boolean') {
        return result('Reject', 'Unsupported', [], ['invalid_deletion_scope']);
      }
      return result('Allow', 'ProfileMapped', ['buyer_personal_credit_balance_decrease'],
        ['seller_payment', 'contribution_recognition', 'producer_financial_asset']);
    }

    case 'retain_personal_credit_balance': {
      if (input.instrument !== 'PersonalCredit' ||
          input.reason !== 'future_use' ||
          !positiveNumber(input.amount)) {
        return result('Reject', 'Unsupported', [], ['invalid_retained_balance_scope']);
      }
      return result('Allow', 'ProfileMapped', ['personal_credit_balance_retained'],
        ['transferability', 'ordinary_financial_asset', 'wealth_equivalence']);
    }

    case 'recognize_contribution': {
      if (Array.isArray(input.evidence_refs) &&
          new Set(input.evidence_refs).size !== input.evidence_refs.length) {
        return result('Reject', 'Unsupported', [], ['duplicate_contribution_issuance']);
      }
      if (input.outcome_laundering_flag === true) {
        return result('Reject', 'Unsupported', [], ['outcome_laundering_detected']);
      }
      if (input.verifier_relationship === 'colluding') {
        return result('Reject', 'Unsupported', [], ['verifier_collusion_not_independent']);
      }
      if (input.verifier_panel_state === 'captured') {
        return result('PendingAppeal', 'Unknown', [],
          ['contribution_issuance_before_review', 'verifier_panel_review_required']);
      }
      if (input.dispute_state === 'disputed') {
        const prohibited = ['contribution_issuance_before_review', 'moral_worth_score'];
        if (typeof input.appeal_route !== 'string' || input.appeal_route.trim().length === 0) {
          prohibited.push('appeal_route_suppression');
        }
        return result('PendingAppeal', 'Unknown', [], prohibited);
      }
      if (input.identity_uniqueness_state === 'unresolved') {
        return result('Indeterminate', 'Unknown', [],
          ['identity_uniqueness_unresolved', 'contribution_credit_issuance']);
      }
      if (input.task_decomposition_state === 'unreviewed') {
        return result('Indeterminate', 'Unknown', [],
          ['task_decomposition_unreviewed', 'contribution_credit_issuance']);
      }
      if (input.metric_profile === 'event_frequency_only' &&
          (!Array.isArray(input.quality_evidence_refs) || input.quality_evidence_refs.length === 0)) {
        return result('Indeterminate', 'Unknown', [],
          ['event_frequency_as_contribution_value', 'contribution_credit_issuance']);
      }
      if (input.activity_source === 'model_generated' && input.independently_observed !== true) {
        return result('Indeterminate', 'Unknown', [],
          ['model_generated_activity_unverified', 'contribution_credit_issuance']);
      }
      if (input.derived_only_from_transaction_settlement === true ||
          !positiveNumber(input.amount) ||
          !DIGEST.test(input.criteria_digest || '') ||
          !uniqueNonEmptyStrings(input.evidence_refs) ||
          typeof input.verifier_authority_scope !== 'string' ||
          input.verifier_authority_scope.trim().length === 0 ||
          input.corroboration_state !== 'qualified' ||
          input.dispute_state !== 'none') {
        return result('Indeterminate', 'Unknown', [],
          ['contribution_credit_issuance', 'moral_worth_score']);
      }
      if (input.evidence_privacy_mode === 'opaque_commitment') {
        if (input.personal_payload_disclosed !== false) {
          return result('Reject', 'Unsupported', [], ['unnecessary_personal_payload_disclosure']);
        }
        return result('Allow', 'ProfileMapped',
          ['bounded_completion_fact_recorded', 'personal_credit_issuance_allowed'],
          ['surveillance_requirement', 'moral_worth_score']);
      }
      return result('Allow', 'ProfileMapped',
        ['contribution_recognition_recorded', 'personal_credit_issuance_allowed'],
        ['moral_worth_score', 'causal_impact_without_evidence']);
    }

    case 'allocate_community_credit': {
      if (input.instrument !== 'CommunityCredit' ||
          input.account_scope !== 'community_project' ||
          !positiveNumber(input.requested_amount)) {
        return result('Reject', 'Unsupported', [], ['invalid_community_allocation_scope']);
      }
      if (input.capacity_state !== 'known' ||
          typeof input.available_capacity !== 'number' ||
          !Number.isFinite(input.available_capacity) ||
          input.available_capacity < 0 ||
          input.authorization_state !== 'authorized') {
        return result('Indeterminate', 'Unknown', [],
          ['unlimited_capacity', 'physical_delivery']);
      }
      if (input.requested_amount > input.available_capacity) {
        return result('Reject', 'Unsupported', [],
          ['capacity_override', 'physical_delivery']);
      }
      return result('Allow', 'ProfileMapped', ['restricted_community_credit_allocation_recorded'],
        ['personal_credit_balance_increase', 'physical_delivery', 'project_completion']);
    }

    case 'recognize_bonus': {
      if (input.dispute_state === 'disputed') {
        return result('PendingAppeal', 'Unknown', [], ['bonus_issuance_before_review']);
      }
      if (input.same_outcome_already_counted_in_contribution_pay === true) {
        return result('Reject', 'Unsupported', [], ['double_counted_recognition']);
      }
      if (!positiveNumber(input.amount) ||
          !DIGEST.test(input.criteria_digest || '') ||
          typeof input.outcome_key !== 'string' ||
          input.outcome_key.trim().length === 0 ||
          !uniqueNonEmptyStrings(input.outcome_evidence_refs) ||
          typeof input.verifier_authority_scope !== 'string' ||
          input.verifier_authority_scope.trim().length === 0 ||
          input.dispute_state !== 'none') {
        return result('Indeterminate', 'Unknown', [],
          ['bonus_issuance_without_qualified_outcome']);
      }
      return result('Allow', 'ProfileMapped', ['bonus_recognition_recorded'],
        ['automatic_contribution_pay', 'moral_worth_score']);
    }

    case 'exchange_pair': {
      const buyer = input.buyer_deletion;
      const seller = input.seller_recredit;
      if (input.instrument !== 'PersonalCredit' ||
          !buyer || !seller ||
          !UUID_URN.test(buyer.event_id || '') ||
          !UUID_URN.test(seller.event_id || '') ||
          buyer.event_id === seller.event_id ||
          !positiveNumber(buyer.amount) ||
          !positiveNumber(seller.amount) ||
          buyer.amount !== seller.amount) {
        return result('Reject', 'Unsupported', [], ['ambiguous_exchange_accounting']);
      }
      return result('Allow', 'ProfileMapped',
        ['buyer_personal_credit_deletion_recorded', 'seller_personal_credit_recredit_recorded'],
        ['ordinary_seller_financial_asset', 'contribution_recognition_from_deletion']);
    }

    case 'project_to_sna': {
      const effects = ['native_instrument_semantics_preserved'];
      const externalLiabilities = Array.isArray(input.conventional_external_liabilities) &&
        input.conventional_external_liabilities.length > 0;
      if (externalLiabilities) effects.push('external_liability_kept_separate');
      if (input.mapping_disposition === 'Unknown' ||
          input.mapping_disposition === 'Unsupported' ||
          input.semantic_equivalence_established !== true) {
        const prohibited = ['deposit_equivalence', 'financial_asset_coercion'];
        if (externalLiabilities && input.fx_conversion_enabled !== true) {
          prohibited.unshift('direct_offset_of_foreign_liability');
        }
        return result('Indeterminate', 'Unknown', effects, prohibited);
      }
      return result('Indeterminate', 'Unknown', effects,
        ['unsupported_sna_equivalence']);
    }

    case 'grant_housing_right': {
      if (input.capacity_state !== 'known' ||
          !Number.isInteger(input.available_dwelling_count) ||
          input.available_dwelling_count < 0) {
        return result('Indeterminate', 'Unknown', [],
          ['housing_entitlement_from_credit_balance', 'unlimited_housing_capacity']);
      }
      if (input.available_dwelling_count === 0 ||
          input.eligibility_verified !== true ||
          input.stewardship_agreement !== true) {
        return result('Reject', 'Unsupported', [],
          ['housing_entitlement_without_resource_and_agreement']);
      }
      return result('Allow', 'ProfileMapped', ['residence_right_recorded'],
        ['housing_as_monetary_balance', 'unbounded_resource_claim']);
    }

    default:
      return result('Indeterminate', 'Unknown', [], ['unsupported_operation']);
  }
}

function corpusErrors(corpus) {
  const errors = [];
  const add = (condition, message) => { if (!condition) errors.push(message); };
  add(corpus && corpus.schema_version === '1.0.0', 'unsupported corpus schema_version');
  add(corpus && corpus.benchmark_id === 'CP-CREDITISM-SEMANTIC-BOUNDARY-001', 'benchmark id mismatch');
  add(corpus && corpus.profile_id === 'common-planet-creditism', 'profile id mismatch');
  add(corpus && corpus.evaluation_state === 'contract_fixture_only', 'fixture must remain contract-only');
  add(corpus && corpus.claim_ceiling === 'ReferenceModelOnly', 'claim ceiling must remain ReferenceModelOnly');
  add(corpus && corpus.data_classification === 'synthetic', 'corpus must remain synthetic');
  add(corpus && corpus.case_count === 32, 'declared case_count must remain 32');
  add(corpus && Array.isArray(corpus.cases) && corpus.cases.length === corpus.case_count,
    'case_count differs from cases array length');
  add(corpus && Array.isArray(corpus.metamorphic_properties) &&
      corpus.metamorphic_properties.length >= 9 && corpus.metamorphic_properties.length <= 15,
    'metamorphic property suite must contain 9..15 invariance checks');
  const owners = new Set(corpus.related_owners || []);
  add(owners.has('mycelix#4572') && owners.has('mycelix#4573') &&
      owners.has('mycelix#4771') && owners.has('mycelix#5366') &&
      owners.has('symthaea#7031'), 'corpus must name existing finance, contribution, capacity, water-loop, and Creditism owners');
  const invariants = corpus.source_semantics?.separation_invariants || [];
  for (const invariant of REQUIRED_INVARIANTS) {
    add(invariants.includes(invariant), 'required semantic separation missing: ' + invariant);
  }

  const ids = new Set();
  for (const [index, item] of (corpus.cases || []).entries()) {
    const label = 'cases[' + index + ']';
    add(item && typeof item.id === 'string' && /^cp-[a-z0-9]+(?:-[a-z0-9]+)*$/.test(item.id),
      label + ' has invalid id');
    add(!ids.has(item && item.id), label + ' duplicate id');
    if (item) ids.add(item.id);
    add(item && item.input && typeof item.input.operation === 'string',
      label + ' has invalid input operation');
    add(item && item.input && SUPPORTED_OPERATIONS.includes(item.input.operation),
      label + ' uses an operation outside the versioned profile contract');
    add(item && item.expected && ['Allow', 'Reject', 'Indeterminate', 'PendingAppeal'].includes(item.expected.decision),
      label + ' has invalid expected decision');
    if (item && item.expected) {
      const observed = evaluate(item.input);
      add(equalJson(observed, item.expected),
        label + ' expected output differs from evaluator output for ' + item.id);
    }
  }

  const metamorphicIds = new Set();
  for (const [index, property] of (corpus.metamorphic_properties || []).entries()) {
    const label = 'metamorphic_properties[' + index + ']';
    add(property && typeof property.property_id === 'string',
      label + ' has no property_id');
    add(!metamorphicIds.has(property && property.property_id),
      label + ' duplicate property_id');
    if (property) metamorphicIds.add(property.property_id);
    add(property && ids.has(property.case_id),
      label + ' references an unknown base case');
    add(property && property.expected_relation === 'evaluation_outputs_equal',
      label + ' has an unsupported relation');
    add(property && Array.isArray(property.input_variants) && property.input_variants.length === 2,
      label + ' must have exactly two input variants');
    if (property && Array.isArray(property.input_variants) && property.input_variants.length === 2) {
      for (const [variantIndex, variant] of property.input_variants.entries()) {
        add(variant && typeof variant === 'object' && !Array.isArray(variant),
          label + ' input variant ' + variantIndex + ' must be an object');
        add(variant && SUPPORTED_OPERATIONS.includes(variant.operation),
          label + ' input variant ' + variantIndex + ' uses an unsupported operation');
      }
      const [left, right] = property.input_variants.map(evaluate);
      add(equalJson(left, right),
        label + ' violates its claimed non-interference/equivalence property');
    }
  }
  return errors;
}

function mustTransition(name, before, mutate, expectedDecision) {
  const changed = JSON.parse(JSON.stringify(before));
  mutate(changed);
  const output = evaluate(changed);
  if (output.decision !== expectedDecision) {
    throw new Error(name + ': expected ' + expectedDecision + ', got ' + output.decision);
  }
  return { name, passed: true, decision: output.decision };
}

function runMutationTests(corpus) {
  const find = id => {
    const item = corpus.cases.find(entry => entry.id === id);
    if (!item) throw new Error('missing mutation base case: ' + id);
    return JSON.parse(JSON.stringify(item.input));
  };
  const results = [
    mustTransition('personal-credit-transfer-denied', find('cp-pc-baseline-issuance'),
      input => { input.operation = 'transfer_personal_credit'; input.instrument = 'PersonalCredit'; }, 'Reject'),
    mustTransition('settlement-only-contribution-indeterminate', find('cp-qualified-contribution'),
      input => { input.derived_only_from_transaction_settlement = true; }, 'Indeterminate'),
    mustTransition('missing-contribution-evidence-indeterminate', find('cp-qualified-contribution'),
      input => { input.evidence_refs = []; }, 'Indeterminate'),
    mustTransition('duplicate-contribution-evidence-rejected', find('cp-qualified-contribution'),
      input => { input.evidence_refs = [input.evidence_refs[0], input.evidence_refs[0]]; }, 'Reject'),
    mustTransition('dispute-enters-appeal', find('cp-qualified-contribution'),
      input => { input.dispute_state = 'disputed'; }, 'PendingAppeal'),
    mustTransition('unknown-capacity-indeterminate', find('cp-community-allocation-within-capacity'),
      input => { input.capacity_state = 'unknown'; input.available_capacity = null; }, 'Indeterminate'),
    mustTransition('over-capacity-rejected', find('cp-community-allocation-within-capacity'),
      input => { input.requested_amount = input.available_capacity + 1; }, 'Reject'),
    mustTransition('bonus-double-count-rejected', find('cp-bonus-qualified-outcome'),
      input => { input.same_outcome_already_counted_in_contribution_pay = true; }, 'Reject'),
    mustTransition('duplicate-exchange-event-rejected', find('cp-exchange-events-separate'),
      input => { input.seller_recredit.event_id = input.buyer_deletion.event_id; }, 'Reject'),
    mustTransition('housing-without-known-capacity-indeterminate', find('cp-housing-needs-capacity'),
      input => { input.capacity_state = 'unknown'; }, 'Indeterminate'),
    mustTransition('unknown-operation-fails-closed', find('cp-pc-baseline-issuance'),
      input => { input.operation = 'invent_new_financial_asset'; }, 'Indeterminate'),
    mustTransition('retained-personal-credit-is-not-transferable', find('cp-pc-retained-for-future-use'),
      input => { input.operation = 'transfer_personal_credit'; input.instrument = 'PersonalCredit'; }, 'Reject'),
    mustTransition('verifier-collusion-rejected', find('cp-qualified-contribution'),
      input => { input.verifier_relationship = 'colluding'; }, 'Reject'),
    mustTransition('sybil-identity-indeterminate', find('cp-qualified-contribution'),
      input => { input.identity_uniqueness_state = 'unresolved'; }, 'Indeterminate'),
    mustTransition('task-decomposition-indeterminate', find('cp-qualified-contribution'),
      input => { input.task_decomposition_state = 'unreviewed'; }, 'Indeterminate'),
    mustTransition('frequency-only-metric-indeterminate', find('cp-qualified-contribution'),
      input => { input.metric_profile = 'event_frequency_only'; input.quality_evidence_refs = []; }, 'Indeterminate'),
    mustTransition('outcome-laundering-rejected', find('cp-qualified-contribution'),
      input => { input.outcome_laundering_flag = true; }, 'Reject'),
    mustTransition('captured-verifier-panel-pending', find('cp-qualified-contribution'),
      input => { input.verifier_panel_state = 'captured'; }, 'PendingAppeal'),
    mustTransition('model-generated-activity-indeterminate', find('cp-qualified-contribution'),
      input => { input.activity_source = 'model_generated'; input.independently_observed = false; }, 'Indeterminate'),
    mustTransition('missing-appeal-route-stays-pending', find('cp-disputed-contribution-paused'),
      input => { input.appeal_route = null; }, 'PendingAppeal')
  ];
  const tamperedExpected = JSON.parse(JSON.stringify(corpus));
  tamperedExpected.cases[0].expected.decision = 'Reject';
  const expectedMutationRejected = corpusErrors(tamperedExpected).length > 0;
  if (!expectedMutationRejected) throw new Error('tampered expected output was not rejected');
  results.push({ name: 'tampered-known-answer-rejected', passed: true, decision: 'RejectedByCorpusVerifier' });

  const missingInvariant = JSON.parse(JSON.stringify(corpus));
  missingInvariant.source_semantics.separation_invariants =
    missingInvariant.source_semantics.separation_invariants.filter(x => x !== 'consumer settlement != contribution recognition');
  const invariantMutationRejected = corpusErrors(missingInvariant).length > 0;
  if (!invariantMutationRejected) throw new Error('missing required semantic invariant was not rejected');
  results.push({ name: 'missing-required-invariant-rejected', passed: true, decision: 'RejectedByCorpusVerifier' });

  const tamperedMetamorphic = JSON.parse(JSON.stringify(corpus));
  tamperedMetamorphic.metamorphic_properties[0].input_variants[1].operation = 'transfer_personal_credit';
  const metamorphicMutationRejected = corpusErrors(tamperedMetamorphic).length > 0;
  if (!metamorphicMutationRejected) throw new Error('broken metamorphic relation was not rejected');
  results.push({ name: 'tampered-metamorphic-relation-rejected', passed: true, decision: 'RejectedByCorpusVerifier' });
  return results;
}

function main() {
  const selfTest = process.argv.includes('--self-test');
  const corpusBytes = fs.readFileSync(corpusPath);
  const schemaBytes = fs.readFileSync(schemaPath);
  const corpus = JSON.parse(corpusBytes.toString('utf8'));
  const schema = JSON.parse(schemaBytes.toString('utf8'));
  const errors = corpusErrors(corpus);
  if (errors.length) fail(errors.join('\n'));
  if (schema.$schema !== 'https://json-schema.org/draft/2020-12/schema') {
    fail('schema must declare Draft 2020-12');
  }
  if (!equalJson(schema.properties.cases.items.$ref, '#/$defs/case')) {
    fail('schema case reference changed');
  }
  if (schema.properties.case_count.const !== 32 ||
      schema.properties.cases.minItems !== 32 ||
      schema.properties.cases.maxItems !== 32 ||
      schema.properties.metamorphic_properties.minItems !== 9) {
    fail('schema corpus/metamorphic count contract changed');
  }
  if (!equalJson(schema.$defs.case.properties.input.properties.operation.enum, SUPPORTED_OPERATIONS)) {
    fail('schema operation enum differs from the evaluator supported-operation contract');
  }
  if (!equalJson(schema.properties.metamorphic_properties.items.$ref, '#/$defs/metamorphicProperty')) {
    fail('schema metamorphic property reference changed');
  }
  if (schema.$defs.metamorphicProperty.properties.expected_relation.const !== 'evaluation_outputs_equal') {
    fail('schema metamorphic relation must remain equality-based');
  }

  const mutations = selfTest ? runMutationTests(corpus) : [];
  const receipt = {
    benchmark_id: corpus.benchmark_id,
    result: 'CreditismSemanticBoundaryConforms',
    evaluation_state: corpus.evaluation_state,
    claim_ceiling: corpus.claim_ceiling,
    corpus_sha256: sha256(corpusBytes),
    schema_sha256: sha256(schemaBytes),
    case_count: corpus.cases.length,
    known_answer_cases_passed: corpus.cases.length,
    metamorphic_property_count: corpus.metamorphic_properties.length,
    metamorphic_properties_passed: corpus.metamorphic_properties.filter(property =>
      equalJson(evaluate(property.input_variants[0]), evaluate(property.input_variants[1]))).length,
    decisions: corpus.cases.reduce((counts, item) => {
      counts[item.expected.decision] = (counts[item.expected.decision] || 0) + 1;
      return counts;
    }, {}),
    owner_refs: corpus.related_owners,
    self_test_executed: selfTest,
    mutation_cases: mutations.length,
    mutation_cases_passed: mutations.filter(item => item.passed).length,
    nonclaims: [
      'This evaluates a synthetic reference policy, not production Common Planet or Mycelix settlement.',
      'A profile-mapped result is not evidence that contribution occurred or that an outcome was caused.',
      'Unknown and disputed inputs intentionally fail closed; a future pilot needs an appealable, qualified evidence process.',
      'No SNA equivalence, seller payment, physical delivery, moral value, economic superiority, or SOTA claim is established.'
    ]
  };
  process.stdout.write(JSON.stringify(receipt, null, 2) + '\n');
}

main();
