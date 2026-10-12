// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Local-only scenario runner. It does not connect to a database, Holochain,
// Symthaea, a payment provider, or a regulatory source.

import { readFile } from 'node:fs/promises';
import { resolve } from 'node:path';
import { analyzeBusinessModel } from './index.js';

const inputPath = process.argv[2];

if (!inputPath) {
  console.error('Usage: node src/cli.js <business-scenario.json>');
  console.error('No scenario was loaded and no result was produced.');
  process.exitCode = 2;
} else {
  try {
    const text = await readFile(resolve(inputPath), 'utf8');
    const input = JSON.parse(text);
    const result = analyzeBusinessModel(input);

    if (!result.ok) {
      console.error(JSON.stringify(result.error, null, 2));
      process.exitCode = 1;
    } else {
      console.log(JSON.stringify(result.report, null, 2));
    }
  } catch {
    console.error('Could not read or parse the supplied scenario file. No result was produced.');
    process.exitCode = 2;
  }
}
