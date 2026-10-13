import { describe, expect, it } from 'vitest';
import {
  assertCisacCrdReportingProjection,
  assertDdexRoyaltyReportingProjection,
  exportCisacCrdReportingProjection,
  exportDdexRoyaltyReportingProjection,
  REPORTING_STANDARD_CONFORMANCE,
  ROYALTY_REPORTING_MAPPING_VERSION,
  type RoyaltyReportingLine,
} from './interop.js';
import { money } from './money.js';

const root = (character: string) => character.repeat(64);

const source = {
  statementId: 'statement:s92',
  statementCommitmentRoot: root('a'),
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
    expect(projection.mappingVersion).toBe(ROYALTY_REPORTING_MAPPING_VERSION);
    expect(projection.standardConformance).toBe(REPORTING_STANDARD_CONFORMANCE);
    expect(projection.source.statementCommitmentRoot).toBe(root('a'));
    expect(projection.source.generatedAt).toBe('2026-10-08T12:00:00.000Z');
    expect(projection.projectionRoot).toMatch(/^[0-9a-f]{64}$/);
    assertDdexRoyaltyReportingProjection(projection);
    expect(exportDdexRoyaltyReportingProjection({
      profileVersion: 'configured-ddex-profile',
      source,
      lines: [...lines].reverse(),
    }).projectionRoot).toBe(projection.projectionRoot);
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
    expect(projection.standardConformance).toBe('mapping_only_not_schema_conformant');
    expect(projection.lines[0]?.distributionReference).toBe('obligation:o1');
    assertCisacCrdReportingProjection(projection);
  });

  it('rejects duplicate normalized obligation IDs rather than double counting', () => {
    expect(() => exportDdexRoyaltyReportingProjection({
      profileVersion: 'configured-ddex-profile',
      source,
      lines: [
        { ...lines[0]!, obligationId: ' obligation:o2 ' },
        { ...lines[0]!, obligationId: 'obligation:o2' },
      ],
    })).toThrow(/duplicate obligation/);
  });

  it('rejects a non-digest statement source root', () => {
    expect(() => exportDdexRoyaltyReportingProjection({
      profileVersion: 'configured-ddex-profile',
      source: { ...source, statementCommitmentRoot: 'statement-root-92' },
      lines,
    })).toThrow(/lowercase SHA-256 digest/);
  });

  it('rejects post-export edits using the projection commitment', () => {
    const projection = exportDdexRoyaltyReportingProjection({
      profileVersion: 'configured-ddex-profile',
      source,
      lines,
    });
    const altered = {
      ...projection,
      lines: projection.lines.map((line, index) => index === 0
        ? { ...line, royaltyAmount: { ...line.royaltyAmount, amountMinor: '999999' } }
        : line),
    };
    expect(() => assertDdexRoyaltyReportingProjection(altered)).toThrow(/root does not match/);
  });

  it('returns detached, frozen output and canonicalizes identifiers/territories', () => {
    const inputLines: RoyaltyReportingLine[] = [{
      obligationId: ' obligation:o9 ',
      workReference: ' work:9 ',
      beneficiaryReference: ' creator:bob ',
      amount: money(1n, 'usd'),
      usageReference: ' usage:9 ',
      territory: 'za',
    }];
    const projection = exportDdexRoyaltyReportingProjection({
      profileVersion: ' configured-profile ',
      source: { ...source, statementId: ' statement:s92 ', generatedAt: '2026-10-08T14:00:00+02:00' },
      lines: inputLines,
    });
    inputLines[0] = { ...inputLines[0]!, amount: money(900n, 'USD') };
    expect(projection.profileVersion).toBe('configured-profile');
    expect(projection.source.statementId).toBe('statement:s92');
    expect(projection.source.generatedAt).toBe('2026-10-08T12:00:00.000Z');
    expect(projection.lines[0]?.obligationReference).toBe('obligation:o9');
    expect(projection.lines[0]?.workReference).toBe('work:9');
    expect(projection.lines[0]?.territory).toBe('ZA');
    expect(projection.lines[0]?.royaltyAmount.amountMinor).toBe('1');
    expect(Object.isFrozen(projection.lines)).toBe(true);
    expect(Object.isFrozen(projection.lines[0])).toBe(true);
    assertDdexRoyaltyReportingProjection(projection);
  });

  it('rejects malformed territories and missing currency units', () => {
    expect(() => exportDdexRoyaltyReportingProjection({
      profileVersion: 'configured-ddex-profile',
      source,
      lines: [{ ...lines[0]!, territory: 'USA' }],
    })).toThrow(/two-letter uppercase code/);
    expect(() => exportDdexRoyaltyReportingProjection({
      profileVersion: 'configured-ddex-profile',
      source,
      lines: [{ ...lines[0]!, amount: money(1n, '   ') }],
    })).toThrow(/reporting currency must be non-empty/);
  });
});
