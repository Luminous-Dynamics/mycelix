// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

export const index = 22;
let component_cache;
export const component = async () => component_cache ??= (await import('../entries/pages/welcome/_page.svelte.js')).default;
export const imports = ["_app/immutable/nodes/22.D4OqMic6.js","_app/immutable/chunks/CujjRMGB.js","_app/immutable/chunks/DIAPMP-w.js","_app/immutable/chunks/B4sKVuNd.js","_app/immutable/chunks/D2_CMSXj.js","_app/immutable/chunks/CatO2vgv.js"];
export const stylesheets = [];
export const fonts = [];
