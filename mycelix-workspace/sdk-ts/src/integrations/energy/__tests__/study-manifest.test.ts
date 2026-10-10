// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import goldenFixture from '../__fixtures__/cloud-evening-outage.json';
import {
  digestEnergyStudyManifestV1,
  EnergyStudyManifestError,
  validateEnergyStudyManifestV1,
  verifyEnergyStudyArtifactsV1,
  type EnergyStudyArtifactBytesV1,
  type EnergyStudyManifestV1,
} from '../study-manifest.js';

const fixture = goldenFixture as unknown as EnergyStudyManifestV1;
const RUST_GOLDEN_DIGEST =
  '896b4579a632dac32bafd3b2347f19f5da5a74f7173c14cf818b00a8e5c484da';

function copyFixture(): EnergyStudyManifestV1 {
  return JSON.parse(JSON.stringify(fixture)) as EnergyStudyManifestV1;
}

function artifactBytes(name: string): Uint8Array {
  return new Uint8Array(readFileSync(new URL(`../__fixtures__/${name}`, import.meta.url)));
}

function fixtureArtifacts(): EnergyStudyArtifactBytesV1 {
  return {
    datasets: {
      'load-profile': artifactBytes('load-profile.csv'),
      'renewable-generation': artifactBytes('renewable-generation.csv'),
      'grid-availability': artifactBytes('grid-availability.csv'),
    },
    asset_registry: artifactBytes('asset-registry.json'),
    constraints: artifactBytes('constraints.json'),
    model_configuration: artifactBytes('model-config.json'),
    policy_configuration: artifactBytes('policy-config.json'),
  };
}

describe('EnergyStudyManifestV1', () => {
  it('matches the frozen Rust canonical digest', async () => {
    expect(validateEnergyStudyManifestV1(fixture)).toEqual(fixture);
    await expect(digestEnergyStudyManifestV1(fixture)).resolves.toBe(RUST_GOLDEN_DIGEST);
  });

  it('canonicalizes dataset order without mutating the caller object', async () => {
    const reversed = copyFixture();
    const originalIds = reversed.datasets.map((dataset) => dataset.dataset_id);
    reversed.datasets.reverse();

    await expect(digestEnergyStudyManifestV1(reversed)).resolves.toBe(RUST_GOLDEN_DIGEST);
    expect(reversed.datasets.map((dataset) => dataset.dataset_id)).toEqual(
      [...originalIds].reverse()
    );
  });

  it('changes identity when a bound input or configuration changes', async () => {
    const changedInput = copyFixture();
    changedInput.datasets[0].content_sha256 = '9'.repeat(64);
    await expect(digestEnergyStudyManifestV1(changedInput)).resolves.not.toBe(
      RUST_GOLDEN_DIGEST
    );

    const changedModel = copyFixture();
    changedModel.model.configuration_sha256 = '8'.repeat(64);
    await expect(digestEnergyStudyManifestV1(changedModel)).resolves.not.toBe(
      RUST_GOLDEN_DIGEST
    );

    const changedPolicy = copyFixture();
    changedPolicy.policy.git_revision = '3'.repeat(40);
    await expect(digestEnergyStudyManifestV1(changedPolicy)).resolves.not.toBe(
      RUST_GOLDEN_DIGEST
    );

    const changedConstraints = copyFixture();
    changedConstraints.constraints_sha256 = '7'.repeat(64);
    await expect(digestEnergyStudyManifestV1(changedConstraints)).resolves.not.toBe(
      RUST_GOLDEN_DIGEST
    );
  });

  it('rejects duplicate dataset identities and malformed hashes', () => {
    const duplicate = copyFixture();
    duplicate.datasets.push({ ...duplicate.datasets[0] });
    expect(() => validateEnergyStudyManifestV1(duplicate)).toThrow(
      EnergyStudyManifestError
    );
    expect(() => validateEnergyStudyManifestV1(duplicate)).toThrow('duplicate dataset_id');

    const malformed = copyFixture();
    malformed.datasets[0].content_sha256 = 'ABC';
    expect(() => validateEnergyStudyManifestV1(malformed)).toThrow(
      'lowercase 64-character SHA-256'
    );
  });

  it('rejects unknown fields, unpinned revisions, and unsafe timestamps', () => {
    const withUnknownField = {
      ...copyFixture(),
      self_approved: true,
    };
    expect(() => validateEnergyStudyManifestV1(withUnknownField)).toThrow(
      'must contain exactly'
    );

    const unpinned = copyFixture();
    unpinned.model.git_revision = 'main';
    expect(() => validateEnergyStudyManifestV1(unpinned)).toThrow(
      'exact lowercase 40- or 64-character Git revision'
    );

    const unsafeTime = copyFixture();
    unsafeTime.start_unix_seconds = Number.MAX_SAFE_INTEGER;
    expect(() => validateEnergyStudyManifestV1(unsafeTime)).toThrow(
      'exact cross-language integer range'
    );
  });

  it('verifies the exact bytes of every referenced data and configuration artifact', async () => {
    const receipt = await verifyEnergyStudyArtifactsV1(fixture, fixtureArtifacts());
    expect(receipt.schema_version).toBe(1);
    expect(receipt.manifest_sha256).toBe(RUST_GOLDEN_DIGEST);
    expect(receipt.verified_dataset_count).toBe(3);
    expect(receipt.verified_configuration_count).toBe(4);
    expect(receipt.artifact_set_sha256).toBe(
      '74a2bb57ea351ab2f0d68502443d27a971db33293fa4c11e12a45d0afa3b30e2'
    );
  });

  it('fails closed on missing, extra, or content-mismatched artifacts', async () => {
    const complete = fixtureArtifacts();
    const missing = {
      ...complete,
      datasets: {
        'renewable-generation': complete.datasets['renewable-generation'],
        'grid-availability': complete.datasets['grid-availability'],
      },
    };
    await expect(verifyEnergyStudyArtifactsV1(fixture, missing)).rejects.toMatchObject({
      code: 'ARTIFACT_SET_MISMATCH',
    });

    const extra = fixtureArtifacts();
    (extra.datasets as Record<string, Uint8Array>)['unreferenced-extra'] = new Uint8Array([1]);
    await expect(verifyEnergyStudyArtifactsV1(fixture, extra)).rejects.toMatchObject({
      code: 'ARTIFACT_SET_MISMATCH',
    });

    const tampered = fixtureArtifacts();
    (tampered.datasets as Record<string, Uint8Array>)['load-profile'] =
      new TextEncoder().encode('tampered input');
    await expect(verifyEnergyStudyArtifactsV1(fixture, tampered)).rejects.toMatchObject({
      code: 'ARTIFACT_HASH_MISMATCH',
    });

    const badConfiguration = fixtureArtifacts();
    badConfiguration.policy_configuration = new TextEncoder().encode('different policy');
    await expect(verifyEnergyStudyArtifactsV1(fixture, badConfiguration)).rejects.toMatchObject({
      code: 'ARTIFACT_HASH_MISMATCH',
    });
  });

  it('rejects non-portable study durations', () => {
    const oversized = copyFixture();
    oversized.interval_seconds = 0xffff_ffff;
    oversized.interval_count = 0xffff_ffff;
    expect(() => validateEnergyStudyManifestV1(oversized)).toThrow(
      'exact cross-language integer range'
    );
  });
});
