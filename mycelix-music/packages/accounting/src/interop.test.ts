import { describe, expect, it } from 'vitest';
import {
  exportCisacCrdReportingProjection,
  exportDdexRoyaltyReportingProjection,
  type RoyaltyReportingLine,
} from './interop.js';
import { money } from './money.js';

const source = {
  statementId: 'statement:s92',
  statementCommitmentRoot: 'statement-root-92',
  generatedAt: '2026-10-08T12:00:00Z',
} as const;

const lines: RoyaltyReportingLine[] = [
  {
    obligationId: 'obligation:o2',
    workReference: 'isrc:work-2',
    beneficiaryReference: 'creator:alice',
    amount: money(200n),
    usageReference: 'usage:u2',
    territory: 'ZA',
  },
  {
    obligationId: 'obligation:o1',
    workReference: 'isrc:work-1',
    beneficiaryReference: 'creator:alice',
    amount: money(127n),
    usageReference: 'usage:u1',
    territory: 'US',
  },
];

describe('royalty reporting projections', () => {
  it('exports a deterministic DDEX projection with exact Mycelix provenance', () => {
    const projection = exportDdexRoyaltyReportingProjection({
      profileVersion: 'configured-ddex-profile',
      source,
      lines,
    });
    expect(projection.authority).toBe('reporting_projection_only');
    expect(projection.source.statementCommitmentRoot).toBe('statement-root-92');
    expect(projection.lines.map(line => line.obligationReference)).toEqual([
      'obligation:o1',
      'obligation:o2',
    ]);
    expect(projection.lines[0]?.royaltyAmount.amountMinor).toBe('127');
  });

  it('exports a CRD projection without granting the report ledger authority', () => {
    const projection = exportCisacCrdReportingProjection({
      profileVersion: 'configured-crd-profile',
      source,
      lines,
    });
    expect(projection.format).toBe('CISAC_CRD');
    expect(projection.authority).toBe('reporting_projection_only');
    expect(projection.lines[0]?.distributionReference).toBe('obligation:o1');
  });

  it('rejects duplicate obligation lines rather than double counting', () => {
    expect(() => exportDdexRoyaltyReportingProjection({
      profileVersion: 'configured-ddex-profile',
      source,
      lines: [lines[0]!, lines[0]!],
    })).toThrow(/duplicate obligation/);
  });
});
