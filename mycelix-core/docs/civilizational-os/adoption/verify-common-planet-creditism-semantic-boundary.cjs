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
  'unknown sna mapping != ordinary financial instrument'
];

function equalJson(a, b) {
  return JSON.stringify(a) === JSON.stringify(b);
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

    case 'recognize_contribution': {
      if (Array.isArray(input.evidence_refs) &&
          new Set(input.evidence_refs).size !== input.evidence_refs.length) {
        return result('Reject', 'Unsupported', [], ['duplicate_contribution_issuance']);
      }
      if (input.dispute_state === 'disputed') {
        return result('PendingAppeal', 'Unknown', [],
          ['contribution_issuance_before_review', 'moral_worth_score']);
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
      if (input.mapping_disposition === 'Unknown' ||
          input.mapping_disposition === 'Unsupported' ||
          input.semantic_equivalence_established !== true) {
        return result('Indeterminate', 'Unknown', ['native_instrument_semantics_preserved'],
          ['deposit_equivalence', 'financial_asset_coercion']);
      }
      return result('Indeterminate', 'Unknown', ['native_instrument_semantics_preserved'],
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
  add(corpus && corpus.case_count === 21, 'declared case_count must remain 21');
  add(corpus && Array.isArray(corpus.cases) && corpus.cases.length === corpus.case_count,
    'case_count differs from cases array length');
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
    add(item && item.expected && ['Allow', 'Reject', 'Indeterminate', 'PendingAppeal'].includes(item.expected.decision),
      label + ' has invalid expected decision');
    if (item && item.expected) {
      const observed = evaluate(item.input);
      add(equalJson(observed, item.expected),
        label + ' expected output differs from evaluator output for ' + item.id);
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
      input => { input.operation = 'invent_new_financial_asset'; }, 'Indeterminate')
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
