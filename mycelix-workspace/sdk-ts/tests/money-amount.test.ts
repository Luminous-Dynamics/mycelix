// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import {
  checkedAddMoneyAmountV1,
  isPositiveMoneyAmountV1,
  requireMoneyAmountProfileV1,
  validateMoneyAmountV1,
} from '../src/clients/finance/money-amount.js';

type GoldenVector = {
  id: string;
  profile?: string;
  minor_units?: string;
  parsed_u64?: string;
  positive?: boolean;
  expected?: string;
  expected_error?: string;
  left_profile?: string;
  left?: string;
  right_profile?: string;
  right?: string;
};

type GoldenCorpus = {
  positive_vectors: GoldenVector[];
  negative_vectors: GoldenVector[];
  arithmetic_vectors: GoldenVector[];
  exact_json_roundtrip: string;
};

const golden = JSON.parse(
  readFileSync(
    new URL('../../../mycelix-finance/docs/money-amount-golden-v1.json', import.meta.url),
    'utf8',
  ),
) as GoldenCorpus;

describe('MoneyAmountV1 shared golden corpus', () => {
  it('parses every positive vector without numeric coercion', () => {
    for (const vector of golden.positive_vectors) {
      const result = validateMoneyAmountV1({
        instrument_profile_id: vector.profile,
        minor_units: vector.minor_units,
      });

      expect(result, vector.id).toEqual({
        ok: true,
        value: {
          instrument_profile_id: vector.profile,
          minor_units: vector.minor_units,
        },
      });
      if (!result.ok) continue;
      expect(result.value.minor_units, vector.id).toBe(vector.parsed_u64);
      expect(isPositiveMoneyAmountV1(result.value), vector.id).toEqual({
        ok: true,
        value: vector.positive,
      });
    }
  });

  it('rejects every negative vector with its stable error code', () => {
    for (const vector of golden.negative_vectors) {
      const result = validateMoneyAmountV1({
        instrument_profile_id: vector.profile,
        minor_units: vector.minor_units,
      });

      expect(result.ok, vector.id).toBe(false);
      if (result.ok) continue;
      expect(result.error, vector.id).toBe(vector.expected);
    }
  });

  it('executes every arithmetic vector with exact decimal-string addition', () => {
    for (const vector of golden.arithmetic_vectors) {
      const result = checkedAddMoneyAmountV1(
        {
          instrument_profile_id: vector.left_profile,
          minor_units: vector.left,
        },
        {
          instrument_profile_id: vector.right_profile,
          minor_units: vector.right,
        },
      );

      if (vector.expected !== undefined) {
        expect(result, vector.id).toEqual({
          ok: true,
          value: {
            instrument_profile_id: vector.left_profile,
            minor_units: vector.expected,
          },
        });
      } else {
        expect(result.ok, vector.id).toBe(false);
        if (result.ok) continue;
        expect(result.error, vector.id).toBe(vector.expected_error);
      }
    }
  });

  it('preserves the maximum u64 wire value through JSON serialization', () => {
    const amount = {
      instrument_profile_id: 'mycelix.finance.instrument.sap.micro.v1',
      minor_units: '18446744073709551615',
    };
    expect(JSON.stringify(amount)).toBe(golden.exact_json_roundtrip);
    expect(validateMoneyAmountV1(JSON.parse(golden.exact_json_roundtrip))).toEqual({
      ok: true,
      value: amount,
    });
  });

  it('rejects object shape changes and profile substitution', () => {
    expect(
      validateMoneyAmountV1({
        instrument_profile_id: 'mycelix.finance.instrument.sap.micro.v1',
        minor_units: '1',
        amount: 1,
      }),
    ).toEqual({ ok: false, error: 'InvalidEnvelope' });

    expect(
      requireMoneyAmountProfileV1(
        {
          instrument_profile_id: 'mycelix.finance.instrument.tend.v1',
          minor_units: '1',
        },
        'mycelix.finance.instrument.sap.micro.v1',
      ),
    ).toEqual({ ok: false, error: 'ProfileMismatch' });
  });
});
