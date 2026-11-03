// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

/**
 * Exact MoneyAmountV1 wire helpers.
 *
 * All atomic/minor-unit values stay canonical decimal strings. These helpers
 * intentionally avoid Number() and BigInt so the SDK behaves consistently in
 * browser runtimes and never loses precision above the JavaScript safe-integer range.
 */

export const SAP_MICRO_V1_PROFILE_ID = 'mycelix.finance.instrument.sap.micro.v1';
export const TEND_V1_PROFILE_ID = 'mycelix.finance.instrument.tend.v1';
export const MONEY_AMOUNT_V1_MAX_MINOR_UNITS = '18446744073709551615';

export interface MoneyAmountV1 {
  readonly instrument_profile_id: string;
  readonly minor_units: string;
}

export interface MoneyAmountProfileV1 {
  readonly instrument_profile_id: typeof SAP_MICRO_V1_PROFILE_ID;
  readonly instrument_code: 'SAP';
  readonly atomic_unit: 'micro-SAP';
  readonly decimal_scale: 6;
}

export type MoneyAmountV1Error =
  | 'InvalidEnvelope'
  | 'InvalidProfileId'
  | 'InvalidMinorUnits'
  | 'NonCanonicalMinorUnits'
  | 'OutOfRange'
  | 'ProfileMismatch'
  | 'Overflow'
  | 'Underflow'
  | 'UnsupportedProfile';

export type MoneyAmountV1Result<T> =
  | { readonly ok: true; readonly value: T }
  | { readonly ok: false; readonly error: MoneyAmountV1Error };

export const SAP_MICRO_PROFILE_V1: MoneyAmountProfileV1 = Object.freeze({
  instrument_profile_id: SAP_MICRO_V1_PROFILE_ID,
  instrument_code: 'SAP',
  atomic_unit: 'micro-SAP',
  decimal_scale: 6,
});

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null && !Array.isArray(value);
}

function isValidProfileId(value: unknown): value is string {
  return (
    typeof value === 'string' &&
    value.length > 0 &&
    value.length <= 128 &&
    /^[a-z0-9][a-z0-9.:-]*$/.test(value)
  );
}

/** Compare canonical non-negative decimal strings without numeric coercion. */
function compareDecimalStrings(left: string, right: string): -1 | 0 | 1 {
  if (left.length < right.length) return -1;
  if (left.length > right.length) return 1;
  if (left < right) return -1;
  if (left > right) return 1;
  return 0;
}

function parseCanonicalU64(value: unknown): MoneyAmountV1Result<string> {
  if (typeof value !== 'string' || value.length === 0 || !/^[0-9]+$/.test(value)) {
    return { ok: false, error: 'InvalidMinorUnits' };
  }
  if (value.length > 1 && value.startsWith('0')) {
    return { ok: false, error: 'NonCanonicalMinorUnits' };
  }
  if (compareDecimalStrings(value, MONEY_AMOUNT_V1_MAX_MINOR_UNITS) > 0) {
    return { ok: false, error: 'OutOfRange' };
  }
  return { ok: true, value };
}

function addDecimalStrings(left: string, right: string): string {
  let leftIndex = left.length - 1;
  let rightIndex = right.length - 1;
  let carry = 0;
  let output = '';

  while (leftIndex >= 0 || rightIndex >= 0 || carry > 0) {
    const leftDigit = leftIndex >= 0 ? left.charCodeAt(leftIndex) - 48 : 0;
    const rightDigit = rightIndex >= 0 ? right.charCodeAt(rightIndex) - 48 : 0;
    const sum = leftDigit + rightDigit + carry;
    const digit = sum % 10;
    carry = Math.floor(sum / 10);
    output = String.fromCharCode(48 + digit) + output;
    leftIndex -= 1;
    rightIndex -= 1;
  }

  return output;
}

export function resolveMoneyAmountProfileV1(
  instrumentProfileId: string,
): MoneyAmountV1Result<MoneyAmountProfileV1> {
  if (instrumentProfileId === SAP_MICRO_V1_PROFILE_ID) {
    return { ok: true, value: SAP_MICRO_PROFILE_V1 };
  }
  return { ok: false, error: 'UnsupportedProfile' };
}

export function validateMoneyAmountV1(input: unknown): MoneyAmountV1Result<MoneyAmountV1> {
  if (!isRecord(input)) return { ok: false, error: 'InvalidEnvelope' };
  const keys = Object.keys(input).sort();
  if (
    keys.length !== 2 ||
    keys[0] !== 'instrument_profile_id' ||
    keys[1] !== 'minor_units'
  ) {
    return { ok: false, error: 'InvalidEnvelope' };
  }
  if (!isValidProfileId(input.instrument_profile_id)) {
    return { ok: false, error: 'InvalidProfileId' };
  }
  const minor = parseCanonicalU64(input.minor_units);
  if (!minor.ok) return minor;
  return {
    ok: true,
    value: {
      instrument_profile_id: input.instrument_profile_id,
      minor_units: minor.value,
    },
  };
}

export function isPositiveMoneyAmountV1(
  input: unknown,
): MoneyAmountV1Result<boolean> {
  const amount = validateMoneyAmountV1(input);
  if (!amount.ok) return amount;
  return { ok: true, value: amount.value.minor_units !== '0' };
}

export function requireMoneyAmountProfileV1(
  input: unknown,
  expectedProfileId: string,
): MoneyAmountV1Result<MoneyAmountV1> {
  const amount = validateMoneyAmountV1(input);
  if (!amount.ok) return amount;
  if (amount.value.instrument_profile_id !== expectedProfileId) {
    return { ok: false, error: 'ProfileMismatch' };
  }
  const profile = resolveMoneyAmountProfileV1(expectedProfileId);
  if (!profile.ok) return profile;
  return amount;
}

function subtractDecimalStrings(left: string, right: string): string {
  let leftIndex = left.length - 1;
  let rightIndex = right.length - 1;
  let borrow = 0;
  let output = '';

  while (leftIndex >= 0) {
    const leftDigit = left.charCodeAt(leftIndex) - 48 - borrow;
    const rightDigit = rightIndex >= 0 ? right.charCodeAt(rightIndex) - 48 : 0;
    const difference = leftDigit - rightDigit;
    if (difference < 0) {
      output = String.fromCharCode(48 + difference + 10) + output;
      borrow = 1;
    } else {
      output = String.fromCharCode(48 + difference) + output;
      borrow = 0;
    }
    leftIndex -= 1;
    rightIndex -= 1;
  }

  while (output.length > 1 && output.startsWith('0')) {
    output = output.slice(1);
  }
  return output;
}

export function checkedAddMoneyAmountV1(
  leftInput: unknown,
  rightInput: unknown,
): MoneyAmountV1Result<MoneyAmountV1> {
  const left = validateMoneyAmountV1(leftInput);
  if (!left.ok) return left;
  const right = validateMoneyAmountV1(rightInput);
  if (!right.ok) return right;
  if (left.value.instrument_profile_id !== right.value.instrument_profile_id) {
    return { ok: false, error: 'ProfileMismatch' };
  }
  const profile = resolveMoneyAmountProfileV1(left.value.instrument_profile_id);
  if (!profile.ok) return profile;

  const sum = addDecimalStrings(left.value.minor_units, right.value.minor_units);
  if (compareDecimalStrings(sum, MONEY_AMOUNT_V1_MAX_MINOR_UNITS) > 0) {
    return { ok: false, error: 'Overflow' };
  }
  return {
    ok: true,
    value: {
      instrument_profile_id: left.value.instrument_profile_id,
      minor_units: sum,
    },
  };
}

export function checkedSubMoneyAmountV1(
  leftInput: unknown,
  rightInput: unknown,
): MoneyAmountV1Result<MoneyAmountV1> {
  const left = validateMoneyAmountV1(leftInput);
  if (!left.ok) return left;
  const right = validateMoneyAmountV1(rightInput);
  if (!right.ok) return right;
  if (left.value.instrument_profile_id !== right.value.instrument_profile_id) {
    return { ok: false, error: 'ProfileMismatch' };
  }
  const profile = resolveMoneyAmountProfileV1(left.value.instrument_profile_id);
  if (!profile.ok) return profile;
  if (compareDecimalStrings(left.value.minor_units, right.value.minor_units) < 0) {
    return { ok: false, error: 'Underflow' };
  }
  return {
    ok: true,
    value: {
      instrument_profile_id: left.value.instrument_profile_id,
      minor_units: subtractDecimalStrings(left.value.minor_units, right.value.minor_units),
    },
  };
}
