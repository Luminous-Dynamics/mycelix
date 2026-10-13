// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Verifies an exported actuals bundle's source-byte fingerprint and deterministically
// replays the bundled cash projection and variance calculation. This is not an
// independent implementation of the financial model and cannot authenticate a source.

import { createHash } from 'node:crypto';
import { readFile } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { compareCashPlanToActuals, projectCashRunway } from './index.js';

const MAX_BUNDLE_BYTES = 20_000_000;
const MAX_SOURCE_BYTES = 5_000_000;
const SHA256_HEX = /^[a-f0-9]{64}$/;

function isRecord(value) {
  return value !== null && typeof value === 'object' && !Array.isArray(value);
}

function failure(code, message) {
  return { ok: false, error: { code, message } };
}

function canonical(value) {
  if (Array.isArray(value)) return value.map(canonical);
  if (isRecord(value)) {
    return Object.fromEntries(
      Object.keys(value).sort().map(key => [key, canonical(value[key])])
    );
  }
  return value;
}

function sameJson(left, right) {
  return JSON.stringify(canonical(left)) === JSON.stringify(canonical(right));
}

/**
 * Re-hash the exact embedded source text and replay the calculation.
 *
 * Returns a small verification receipt, never the source text or financial data.
 * This proves consistency with the bundled calculator implementation, not that the
 * source came from a bank/ledger or that the source itself is complete or truthful.
 */
export function verifyActualsVarianceBundle(bundle) {
  try {
    if (!isRecord(bundle) || bundle.schemaVersion !== 1 ||
        bundle.kind !== 'business-launchpad-actuals-variance-report-bundle') {
      return failure('invalid_bundle_schema', 'Expected a Business Launchpad actuals variance bundle, schema version 1.');
    }
    if (typeof bundle.sourceArtifactRawText !== 'string') {
      return failure('missing_source_text', 'Bundle does not contain the original UTF-8 source text.');
    }
    if (bundle.sourceArtifactRawText.charCodeAt(0) === 0xfeff) {
      return failure('source_bom_unsupported', 'The imported source contains a UTF-8 BOM, which this workflow does not accept.');
    }
    const sourceBytes = Buffer.from(bundle.sourceArtifactRawText, 'utf8');
    if (sourceBytes.byteLength > MAX_SOURCE_BYTES) {
      return failure('source_text_too_large', 'Embedded source text exceeds the 5 MB import limit.');
    }
    const roundTrippedText = new TextDecoder('utf-8', { fatal: true }).decode(sourceBytes);
    if (roundTrippedText !== bundle.sourceArtifactRawText) {
      return failure('source_text_encoding_invalid', 'Embedded source text cannot be represented losslessly as UTF-8.');
    }
    if (typeof bundle.sourceArtifactSha256 !== 'string' || !SHA256_HEX.test(bundle.sourceArtifactSha256)) {
      return failure('invalid_source_digest', 'Source digest must be a lowercase 64-character SHA-256 value.');
    }

    const actualDigest = createHash('sha256').update(sourceBytes).digest('hex');
    if (actualDigest !== bundle.sourceArtifactSha256) {
      return failure('source_digest_mismatch', 'The embedded source text no longer matches its recorded SHA-256 digest.');
    }

    let actualsSource;
    try {
      actualsSource = JSON.parse(bundle.sourceArtifactRawText);
    } catch {
      return failure('source_json_invalid', 'Embedded source text is not valid JSON.');
    }
    if (!isRecord(bundle.scenarioInput) || !Array.isArray(bundle.cashRunwayPlan)) {
      return failure('missing_recalculation_inputs', 'Bundle is missing its model or monthly cash plan.');
    }

    const projection = projectCashRunway({
      model: bundle.scenarioInput,
      monthlyPlan: bundle.cashRunwayPlan
    });
    if (!projection.ok) {
      return failure('cash_runway_recalculation_failed', 'Bundled model and cash plan cannot reproduce a valid projection: ' + projection.error.code + '.');
    }
    if (!sameJson(projection.report, bundle.cashRunwayProjection)) {
      return failure('cash_runway_projection_mismatch', 'The bundled cash projection differs from the projection recomputed by this calculator version.');
    }

    const comparison = compareCashPlanToActuals({
      model: bundle.scenarioInput,
      monthlyPlan: bundle.cashRunwayPlan,
      forecastStartMonth: bundle.forecastStartMonth,
      actualsSource,
      sourceSha256: actualDigest
    });
    if (!comparison.ok) {
      return failure('actuals_recalculation_failed', 'Embedded source cannot reproduce a valid variance report: ' + comparison.error.code + '.');
    }
    if (!sameJson(comparison.report, bundle.comparison)) {
      return failure('variance_report_mismatch', 'The bundled variance report differs from the report recomputed by this calculator version.');
    }

    return {
      ok: true,
      verification: {
        kind: 'business-launchpad-actuals-bundle-verification',
        method: 'source-sha256-and-deterministic-recomputation',
        sourceSha256: actualDigest,
        sourceAuthenticity: 'not_independently_verified',
        financialModelImplementation: 'shared-business-launchpad-core',
        currency: comparison.report.currency.code,
        forecastStartMonth: comparison.report.forecastStartMonth,
        coverageStartDate: comparison.report.source.coverageStartDate,
        coverageEndDate: comparison.report.source.coverageEndDate,
        transactionCount: comparison.report.source.transactionCount,
        periodCount: comparison.report.periods.length,
        reconciliation: comparison.report.reconciliation.status,
        limitations: [
          'The verifier uses the same Business Launchpad core implementation as the producer; it is not an independent financial-model implementation or formal proof.',
          'SHA-256 checks byte consistency only. It does not authenticate the bank, ledger, operator, source completeness, or transaction classification.',
          'This is a cash-basis plan comparison, not accounting, tax, audit, underwriting, or regulatory assurance.'
        ]
      }
    };
  } catch {
    return failure('verification_failed', 'Bundle verification failed safely; no verification receipt was produced.');
  }
}

async function main() {
  if (process.argv.length !== 3 || !process.argv[2]) {
    process.stderr.write('Usage: node src/verify-actuals-bundle.mjs <business-launchpad-actuals-variance-v1.json>\n');
    process.exitCode = 2;
    return;
  }

  let bytes;
  try {
    bytes = await readFile(resolve(process.argv[2]));
  } catch {
    process.stderr.write('FAIL [bundle_read_failed] Could not read the specified bundle file.\n');
    process.exitCode = 1;
    return;
  }
  if (bytes.byteLength > MAX_BUNDLE_BYTES) {
    process.stderr.write('FAIL [bundle_file_too_large] Bundle exceeds the 20 MB verifier limit.\n');
    process.exitCode = 1;
    return;
  }

  let text;
  try {
    text = new TextDecoder('utf-8', { fatal: true }).decode(bytes);
  } catch {
    process.stderr.write('FAIL [bundle_utf8_invalid] Bundle must be valid UTF-8.\n');
    process.exitCode = 1;
    return;
  }
  if (text.charCodeAt(0) === 0xfeff) {
    process.stderr.write('FAIL [bundle_bom_unsupported] Bundle must be UTF-8 without a BOM.\n');
    process.exitCode = 1;
    return;
  }

  let bundle;
  try {
    bundle = JSON.parse(text);
  } catch {
    process.stderr.write('FAIL [bundle_json_invalid] Bundle is not valid JSON.\n');
    process.exitCode = 1;
    return;
  }

  const result = verifyActualsVarianceBundle(bundle);
  if (!result.ok) {
    process.stderr.write('FAIL [' + result.error.code + '] ' + result.error.message + '\n');
    process.exitCode = 1;
    return;
  }
  process.stdout.write(JSON.stringify({ status: 'verified', ...result.verification }, null, 2) + '\n');
}

if (process.argv[1] && pathToFileURL(resolve(process.argv[1])).href === import.meta.url) {
  await main();
}
