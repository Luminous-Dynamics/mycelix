// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

export const index = 8;
let component_cache;
export const component = async () => component_cache ??= (await import('../entries/pages/epistemic-markets/_page.svelte.js')).default;
export const imports = ["_app/immutable/nodes/8.DUmYa6cW.js","_app/immutable/chunks/CujjRMGB.js","_app/immutable/chunks/DIAPMP-w.js","_app/immutable/chunks/B4sKVuNd.js"];
export const stylesheets = ["_app/immutable/assets/8.Dpyd8UR8.css"];
export const fonts = [];
