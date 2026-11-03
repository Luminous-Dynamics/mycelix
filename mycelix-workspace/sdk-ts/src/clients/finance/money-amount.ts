// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

import {
  SAP_MICRO_MONEY_PROFILE_V1,
  type MoneyAmountV1,
} from './types.js';

const MAX_U64_MINOR_UNITS = '18446744073709551615';
const CANONICAL_PROFILE_ID = /^[a-z0-9]+([._-][a-z0-9]+)*$/;
const CANONICAL_MINOR_UNITS = /^(0|[1-9][0-9]*)$/;

export type MoneyAmountV1ErrorCode =
  | 'InvalidEnvelope'
  | 'InvalidProfileId'
  | 'InvalidMinorUnits'
  | 'MinorUnitsOverflow'
  | 'ProfileMismatch'
  | 'AdditionOverflow';

export type MoneyAmountV1Result =
  | { ok: true; value: MoneyAmountV1 }
  | { ok: false; error: MoneyAmountV1ErrorCode };

export type MoneyAmountV1BooleanResult =
  | { ok: true; value: boolean }
  | { ok: false; error: MoneyAmountV1ErrorCode };

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null && !Array.isArray(value);
}

function compareCanonicalUnsignedIntegers(left: string, right: string): number {
  if (left.length !== right.length) {
    return left.length < right.length ? -1 : 1;
  }
  if (left === right) return 0;
  return left < right ? -1 : 1;
}

/**
 * Validate the exact JSON envelope without coercion.
 *
 * This checks the current u64 wire range and canonical syntax. The caller
 * must still resolve/admit the instrument profile before interpreting units.
 */
export function validateMoneyAmountV1(input: unknown): MoneyAmountV1Result {
  if (!isRecord(input)) {
    return { ok: false, error: 'InvalidEnvelope' };
  }
  const keys = Object.keys(input);
  if (
    keys.length !== 2 ||
    !Object.hasOwn(input, 'instrument_profile_id') ||
    !Object.hasOwn(input, 'minor_units')
  ) {
    return { ok: false, error: 'InvalidEnvelope' };
  }

  const profile = input.instrument_profile_id;
  const minorUnits = input.minor_units;
  if (
    typeof profile !== 'string' ||
    profile.length === 0 ||
    profile.length > 128 ||
    !CANONICAL_PROFILE_ID.test(profile)
  ) {
    return { ok: false, error: 'InvalidProfileId' };
  }
  if (typeof minorUnits !== 'string' || !CANONICAL_MINOR_UNITS.test(minorUnits)) {
    return { ok: false, error: 'InvalidMinorUnits' };
  }
  if (
    minorUnits.length > MAX_U64_MINOR_UNITS.length ||
    (minorUnits.length === MAX_U64_MINOR_UNITS.length &&
      compareCanonicalUnsignedIntegers(minorUnits, MAX_U64_MINOR_UNITS) > 0)
  ) {
    return { ok: false, error: 'MinorUnitsOverflow' };
  }

  return {
    ok: true,
    value: {
      instrument_profile_id: profile,
      minor_units: minorUnits,
    },
  };
}

/** Check an admitted profile without changing the amount or its wire spelling. */
export function requireMoneyAmountProfileV1(
  input: unknown,
  expectedProfileId: string,
): MoneyAmountV1Result {
  const result = validateMoneyAmountV1(input);
  if (!result.ok) return result;
  if (result.value.instrument_profile_id !== expectedProfileId) {
    return { ok: false, error: 'ProfileMismatch' };
  }
  return result;
}

/**
 * Add two exact decimal-string amounts using digit arithmetic only.
 *
 * This intentionally avoids Number() and BigInt so the result is portable
 * across JSON, browsers and older JS runtimes. The values must have the same
 * instrument profile and the sum must fit the v1 u64 range.
 */
export function checkedAddMoneyAmountV1(
  leftInput: unknown,
  rightInput: unknown,
): MoneyAmountV1Result {
  const left = validateMoneyAmountV1(leftInput);
  if (!left.ok) return left;
  const right = validateMoneyAmountV1(rightInput);
  if (!right.ok) return right;
  if (left.value.instrument_profile_id !== right.value.instrument_profile_id) {
    return { ok: false, error: 'ProfileMismatch' };
  }

  let leftIndex = left.value.minor_units.length - 1;
  let rightIndex = right.value.minor_units.length - 1;
  let carry = 0;
  let output = '';

  while (leftIndex >= 0 || rightIndex >= 0 || carry > 0) {
    const leftDigit = leftIndex >= 0 ? left.value.minor_units.charCodeAt(leftIndex) - 48 : 0;
    const rightDigit = rightIndex >= 0 ? right.value.minor_units.charCodeAt(rightIndex) - 48 : 0;
    const sum = leftDigit + rightDigit + carry;
    output = String(sum % 10) + output;
    carry = Math.floor(sum / 10);
    leftIndex -= 1;
    rightIndex -= 1;
  }

  if (compareCanonicalUnsignedIntegers(output, MAX_U64_MINOR_UNITS) > 0) {
    return { ok: false, error: 'AdditionOverflow' };
  }
  return {
    ok: true,
    value: {
      instrument_profile_id: left.value.instrument_profile_id,
      minor_units: output,
    },
  };
}

/** Return whether a valid amount is greater than zero. */
export function isPositiveMoneyAmountV1(input: unknown): MoneyAmountV1BooleanResult {
  const result = validateMoneyAmountV1(input);
  if (!result.ok) return result;
  // Canonical unsigned integers are positive iff they are not the single digit "0".
  return { ok: true, value: result.value.minor_units !== '0' };
}

/** Construct an SAP micro-unit envelope; this does not imply external settlement. */
export function sapMicroMoneyAmountV1(minorUnits: string): MoneyAmountV1Result {
  return requireMoneyAmountProfileV1(
    {
      instrument_profile_id: SAP_MICRO_MONEY_PROFILE_V1,
      minor_units: minorUnits,
    },
    SAP_MICRO_MONEY_PROFILE_V1,
  );
}
