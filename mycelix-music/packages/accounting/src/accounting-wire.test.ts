import { describe, expect, it } from 'vitest';
import {
  accountingWireDigest,
  accountingWireDigestFromText,
  deserializeAccountingWire,
  serializeAccountingWire,
} from './accounting-wire.js';
import {
  assertEconomicAuditCapsule,
  createEconomicAuditCapsule,
  economicAuditCapsuleDigest,
  type EconomicAuditCapsuleV2,
} from './audit.js';
import { money } from './money.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';
import { StatementKind } from './statements.js';

function digest(seed: string): string {
  if (!/^[0-9a-f]$/.test(seed)) throw new Error('test digest seed must be one hex character');
  return seed.repeat(64);
}

const obligation = createRoyaltyObligationAuthority({
  id: 'obligation:wire:1',
  beneficiaryId: 'creator:wire',
  amount: money(12345678901234567890n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:wire:1',
    rightsResolutionRef: 'rights:wire:1',
    economicTermsRef: 'terms:wire:1',
  },
});
const epoch: SettlementEpoch = {
  id: 'epoch:wire',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(100n, 'USD'),
};
const capsule = createEconomicAuditCapsule({
  statementInput: {
    statementId: 'statement:wire',
    kind: StatementKind.Periodic,
    beneficiaryId: obligation.beneficiaryId,
    period: { startInclusive: '2026-09-01T00:00:00Z', endExclusive: '2026-10-01T00:00:00Z' },
    asOf: '2026-10-02T00:00:00Z',
    completeness: {
      kind: 'complete',
      through: {
        usageObservedThrough: '2026-10-01T00:00:00Z',
        rightsResolvedThrough: '2026-10-01T00:00:00Z',
        settlementsObservedThrough: '2026-10-02T00:00:00Z',
      },
    },
    settlementEpoch: epoch,
    obligations: [obligation],
    eligibilityObservations: [{
      id: 'eligibility:wire:1',
      obligationId: obligation.id,
      code: SettlementEligibilityCode.Eligible,
      sourceRef: 'eligibility-source:wire:1',
      observedAt: '2026-09-30T00:00:00Z',
    }],
  },
  usageCommitment: digest('1'),
  repertoireSnapshot: digest('2'),
  rightsPolicy: digest('3'),
  economicTerms: digest('4'),
  rightsResolutionRoot: digest('5'),
  nettingRoot: digest('6'),
  settlementPlanRoot: digest('7'),
  paymentReceiptRoot: digest('8'),
  generatedBy: { id: 'mycelix-music-royalty-compiler', version: '2.0.0', buildDigest: digest('9') },
});

const goldenVectors = [
  {
    name: 'null',
    value: null,
    text: '{"format":"mycelix-accounting-wire","version":1,"value":["null"]}',
    digest: 'a0b055b121a7e855f6eb22fec26eec04b847653c9eed355f6f9bbb2c7b3a0c26',
  },
  {
    name: 'arbitrary precision integer',
    value: 123456789012345678901234567890n,
    text: '{"format":"mycelix-accounting-wire","version":1,"value":["integer","123456789012345678901234567890"]}',
    digest: 'df6a69677ee7d5d063ba77f773859806ab97a4507f85a6ae710311aa8fd31e71',
  },
  {
    name: 'negative zero binary64',
    value: -0,
    text: '{"format":"mycelix-accounting-wire","version":1,"value":["number","8000000000000000"]}',
    digest: '71f20fe7a02b0319cfbfa4f569e148863db49dc6cbd8b195ecf87616d1bc9a9a',
  },
  {
    name: 'one-and-a-half binary64',
    value: 1.5,
    text: '{"format":"mycelix-accounting-wire","version":1,"value":["number","3ff8000000000000"]}',
    digest: '56c9bfcd0e7109fdca8031f0274e278d2eba02cf6a795e6a89007553951e5894',
  },
  {
    name: 'sorted unicode object',
    value: { z: 3n, a: 'é' },
    text: '{"format":"mycelix-accounting-wire","version":1,"value":["object",[["a",["string","é"]],["z",["integer","3"]]]]}',
    digest: '52cee7923ac978c5bdd8c0d1d1b25b6c324b4a2845ae96615381793d4d0c7909',
  },
  {
    name: 'UTF-8 key order differs from JavaScript UTF-16 sort',
    value: { '\uE000': 'bmp', '𐀀': 'astral' },
    text: '{"format":"mycelix-accounting-wire","version":1,"value":["object",[["",["string","bmp"]],["𐀀",["string","astral"]]]]}',
    digest: 'dba4eb609b3466d6b42b49fa360390559fc755955012855453a31c7c365c556f',
  },
] as const;

describe('canonical accounting wire v1', () => {
  it('serializes bigint-bearing EconomicAuditCapsule v2 as ordinary JSON and preserves authority digests', () => {
    expect(() => JSON.stringify(capsule)).toThrow(/BigInt|bigint/);
    const text = serializeAccountingWire(capsule);
    expect(() => JSON.parse(text)).not.toThrow();
    const decoded = deserializeAccountingWire(text) as EconomicAuditCapsuleV2;
    expect(typeof decoded.statement.gross.amountMinor).toBe('bigint');
    expect(decoded.statement.gross.amountMinor).toBe(capsule.statement.gross.amountMinor);
    expect(() => assertEconomicAuditCapsule(decoded)).not.toThrow();
    expect(economicAuditCapsuleDigest(decoded)).toBe(economicAuditCapsuleDigest(capsule));
    expect(accountingWireDigestFromText(text)).toBe(accountingWireDigest(capsule));
  });

  it('matches cross-language golden bytes and wire digests', () => {
    for (const vector of goldenVectors) {
      expect(serializeAccountingWire(vector.value), vector.name).toBe(vector.text);
      expect(accountingWireDigestFromText(vector.text), vector.name).toBe(vector.digest);
      expect(accountingWireDigest(vector.value), vector.name).toBe(vector.digest);
    }
  });

  it('is deterministic across object insertion order', () => {
    const left = { z: 3n, a: 'value', nested: { y: true, x: -0 } };
    const right = { nested: { x: -0, y: true }, a: 'value', z: 3n };
    expect(serializeAccountingWire(left)).toBe(serializeAccountingWire(right));
    expect(accountingWireDigest(left)).toBe(accountingWireDigest(right));
  });

  it('round-trips arbitrary-size signed integers and IEEE-754 numbers without precision loss', () => {
    const value = {
      negative: -999999999999999999999999999999999999n,
      positive: 999999999999999999999999999999999999n,
      negativeZero: -0,
      one: 1,
      oneAndHalf: 1.5,
      small: Number.MIN_VALUE,
      large: Number.MAX_VALUE,
    };
    const text = serializeAccountingWire(value);
    expect(text).toContain('["number","8000000000000000"]');
    expect(text).toContain('["number","3ff0000000000000"]');
    expect(text).toContain('["number","3ff8000000000000"]');
    const decoded = deserializeAccountingWire(text) as typeof value;
    expect(decoded.negative).toBe(value.negative);
    expect(decoded.positive).toBe(value.positive);
    expect(Object.is(decoded.negativeZero, -0)).toBe(true);
    expect(decoded.one).toBe(1);
    expect(decoded.oneAndHalf).toBe(1.5);
    expect(decoded.small).toBe(Number.MIN_VALUE);
    expect(decoded.large).toBe(Number.MAX_VALUE);
  });

  it('rejects noncanonical integer text instead of normalizing it', () => {
    const malformed = JSON.stringify({
      format: 'mycelix-accounting-wire',
      version: 1,
      value: ['integer', '01'],
    });
    expect(() => deserializeAccountingWire(malformed)).toThrow(/canonical decimal text/);
  });

  it('rejects noncanonical or non-finite binary64 encodings', () => {
    const uppercase = JSON.stringify({
      format: 'mycelix-accounting-wire',
      version: 1,
      value: ['number', '3FF0000000000000'],
    });
    expect(() => deserializeAccountingWire(uppercase)).toThrow(/binary64 hex/);

    const short = JSON.stringify({
      format: 'mycelix-accounting-wire',
      version: 1,
      value: ['number', '3ff00000'],
    });
    expect(() => deserializeAccountingWire(short)).toThrow(/binary64 hex/);

    const positiveInfinity = JSON.stringify({
      format: 'mycelix-accounting-wire',
      version: 1,
      value: ['number', '7ff0000000000000'],
    });
    expect(() => deserializeAccountingWire(positiveInfinity)).toThrow(/canonical finite IEEE-754/);

    const nan = JSON.stringify({
      format: 'mycelix-accounting-wire',
      version: 1,
      value: ['number', '7ff8000000000000'],
    });
    expect(() => deserializeAccountingWire(nan)).toThrow(/canonical finite IEEE-754/);
  });

  it('rejects unsorted or duplicate object keys in wire nodes', () => {
    const unsorted = JSON.stringify({
      format: 'mycelix-accounting-wire',
      version: 1,
      value: ['object', [['b', ['string', '2']], ['a', ['string', '1']]]],
    });
    expect(() => deserializeAccountingWire(unsorted)).toThrow(/strictly sorted and unique/);

    const duplicate = JSON.stringify({
      format: 'mycelix-accounting-wire',
      version: 1,
      value: ['object', [['a', ['string', '1']], ['a', ['string', '2']]]],
    });
    expect(() => deserializeAccountingWire(duplicate)).toThrow(/strictly sorted and unique/);
  });

  it('rejects JavaScript UTF-16 key order when it differs from canonical UTF-8 byte order', () => {
    const wrongOrder = JSON.stringify({
      format: 'mycelix-accounting-wire',
      version: 1,
      value: ['object', [['𐀀', ['string', 'astral']], ['\uE000', ['string', 'bmp']]]],
    });
    expect(() => deserializeAccountingWire(wrongOrder)).toThrow(/sorted and unique by UTF-8 bytes/);
  });

  it('rejects lone UTF-16 surrogates in string values and object keys', () => {
    expect(() => serializeAccountingWire('\uD800')).toThrow(/lone UTF-16 surrogates/);
    expect(() => serializeAccountingWire({ ['\uDC00']: 'value' })).toThrow(/lone UTF-16 surrogates/);

    const malformedString = '{"format":"mycelix-accounting-wire","version":1,"value":["string","\\ud800"]}';
    expect(() => deserializeAccountingWire(malformedString)).toThrow(/lone UTF-16 surrogates/);

    const malformedKey = '{"format":"mycelix-accounting-wire","version":1,"value":["object",[["\\udc00",["string","value"]]]]}';
    expect(() => deserializeAccountingWire(malformedKey)).toThrow(/lone UTF-16 surrogates/);
  });

  it('rejects noncanonical JSON bytes and undefined values', () => {
    const canonical = serializeAccountingWire({ value: 1n });
    expect(() => deserializeAccountingWire(` ${canonical}`)).toThrow(/canonical JSON/);
    expect(() => deserializeAccountingWire(canonical.replace(',', ', '))).toThrow(/canonical serialized form/);
    expect(() => serializeAccountingWire({ value: undefined })).toThrow(/forbid undefined/);
  });

  it('rejects non-finite numbers and non-plain objects', () => {
    expect(() => serializeAccountingWire(Number.POSITIVE_INFINITY)).toThrow(/must be finite/);
    expect(() => serializeAccountingWire(new Date('2026-09-11T00:00:00Z'))).toThrow(/plain records/);
  });
});
