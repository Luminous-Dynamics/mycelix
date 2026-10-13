// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

import test from 'node:test';
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { mkdtemp, readFile, rm, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { dirname, resolve } from 'node:path';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { compareCashPlanToActuals, projectCashRunway } from '../src/index.js';
import { verifyActualsVarianceBundle } from '../src/verify-actuals-bundle.mjs';

const testDir = dirname(fileURLToPath(import.meta.url));
const verifierPath = resolve(testDir, '../src/verify-actuals-bundle.mjs');

function fixture() {
  const model = {
    currency: { code: 'ZAR', minorUnitExponent: 2 },
    sellingPricePerUnitMinor: 100,
    variableCostPerUnitMinor: 60,
    fixedMonthlyCostsMinor: 10,
    plannedUnitsPerMonth: 10,
    setupCostsMinor: 0,
    initialWorkingCapitalBufferMinor: 0,
    availableCashForStartupMinor: 5000,
    costCoverage: { level: 'complete_for_declared_scope', omittedCategories: [] }
  };
  const monthlyPlan = [
    { salesUnits: 10, salesCashReceiptsMinor: 1000, operatingCashOutflowsMinor: 400, otherCashInflowsMinor: 0, otherCashOutflowsMinor: 0, investingCashInflowsMinor: 0, investingCashOutflowsMinor: 0, financingCashInflowsMinor: 0, financingCashOutflowsMinor: 0 },
    { salesUnits: 10, salesCashReceiptsMinor: 2000, operatingCashOutflowsMinor: 500, otherCashInflowsMinor: 0, otherCashOutflowsMinor: 0, investingCashInflowsMinor: 0, investingCashOutflowsMinor: 0, financingCashInflowsMinor: 0, financingCashOutflowsMinor: 0 }
  ];
  const actualsSource = {
    schemaVersion: 1,
    kind: 'business-launchpad-actuals-source',
    currency: { code: 'ZAR', minorUnitExponent: 2 },
    source: {
      type: 'bank_statement_export',
      label: 'Synthetic test bank statement export',
      evidenceReference: 'Synthetic fixture reference only',
      coverageStartDate: '2026-01-01',
      coverageEndDate: '2026-02-28',
      completeMonthlyCoverage: true,
      openingCashBalanceMinor: 5000,
      closingCashBalanceMinor: 7050
    },
    transactions: [
      { transactionId: 'tx-001', date: '2026-01-10', currencyCode: 'ZAR', category: 'sales_receipt', amountMinor: 900, sourceReference: 'statement line 1' },
      { transactionId: 'tx-002', date: '2026-01-14', currencyCode: 'ZAR', category: 'operating_outflow', amountMinor: 450, sourceReference: 'statement line 2' },
      { transactionId: 'tx-003', date: '2026-02-10', currencyCode: 'ZAR', category: 'sales_receipt', amountMinor: 2200, sourceReference: 'statement line 3' },
      { transactionId: 'tx-004', date: '2026-02-14', currencyCode: 'ZAR', category: 'operating_outflow', amountMinor: 600, sourceReference: 'statement line 4' }
    ]
  };
  const sourceArtifactRawText = JSON.stringify(actualsSource, null, 2) + '\n';
  const sourceArtifactSha256 = createHash('sha256').update(Buffer.from(sourceArtifactRawText, 'utf8')).digest('hex');
  const cashRunway = projectCashRunway({ model, monthlyPlan });
  assert.equal(cashRunway.ok, true);
  const comparison = compareCashPlanToActuals({
    model, monthlyPlan, forecastStartMonth: '2026-01', actualsSource, sourceSha256: sourceArtifactSha256
  });
  assert.equal(comparison.ok, true);
  return {
    schemaVersion: 1,
    kind: 'business-launchpad-actuals-variance-report-bundle',
    forecastStartMonth: '2026-01',
    sourceArtifactSha256,
    sourceArtifactRawText,
    scenarioInput: model,
    cashRunwayPlan: monthlyPlan,
    cashRunwayProjection: cashRunway.report,
    comparison: comparison.report
  };
}

test('re-hashes source bytes and reproduces projection and variance report', () => {
  const bundle = fixture();
  const result = verifyActualsVarianceBundle(bundle);
  assert.equal(result.ok, true);
  assert.equal(result.verification.sourceSha256, bundle.sourceArtifactSha256);
  assert.equal(result.verification.sourceAuthenticity, 'not_independently_verified');
  assert.equal(result.verification.transactionCount, 4);
  assert.equal(result.verification.periodCount, 2);
  assert.equal(result.verification.reconciliation, 'passed');
  assert.equal(result.verification.financialModelImplementation, 'shared-business-launchpad-core');
});

test('fails closed when embedded source text changes after fingerprinting', () => {
  const bundle = fixture();
  bundle.sourceArtifactRawText = bundle.sourceArtifactRawText.replace('"amountMinor": 900', '"amountMinor": 901');
  const result = verifyActualsVarianceBundle(bundle);
  assert.equal(result.ok, false);
  assert.equal(result.error.code, 'source_digest_mismatch');
});

test('fails closed when the cached projection or variance report has been altered', () => {
  const projectionTamper = fixture();
  projectionTamper.cashRunwayProjection.periods[0].closingCashMinor += 1;
  assert.equal(verifyActualsVarianceBundle(projectionTamper).error.code, 'cash_runway_projection_mismatch');

  const reportTamper = fixture();
  reportTamper.comparison.periods[0].categories[0].actualMinor += 1;
  assert.equal(verifyActualsVarianceBundle(reportTamper).error.code, 'variance_report_mismatch');
});

test('CLI emits a small receipt for valid bundles and nonzero exit for a tampered source', async () => {
  const directory = await mkdtemp(resolve(tmpdir(), 'business-launchpad-verifier-'));
  try {
    const bundlePath = resolve(directory, 'bundle.json');
    await writeFile(bundlePath, JSON.stringify(fixture(), null, 2) + '\n', 'utf8');
    const good = spawnSync(process.execPath, [verifierPath, bundlePath], { encoding: 'utf8', timeout: 10_000 });
    assert.equal(good.status, 0, good.stderr);
    const receipt = JSON.parse(good.stdout);
    assert.equal(receipt.status, 'verified');
    assert.equal(receipt.sourceAuthenticity, 'not_independently_verified');

    const bundle = JSON.parse(await readFile(bundlePath, 'utf8'));
    bundle.sourceArtifactRawText = bundle.sourceArtifactRawText.replace('"amountMinor": 900', '"amountMinor": 901');
    await writeFile(bundlePath, JSON.stringify(bundle, null, 2) + '\n', 'utf8');
    const bad = spawnSync(process.execPath, [verifierPath, bundlePath], { encoding: 'utf8', timeout: 10_000 });
    assert.equal(bad.status, 1);
    assert.match(bad.stderr, /source_digest_mismatch/);
  } finally {
    await rm(directory, { recursive: true, force: true });
  }
});
