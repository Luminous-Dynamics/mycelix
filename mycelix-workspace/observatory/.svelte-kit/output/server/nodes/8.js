

export const index = 8;
let component_cache;
export const component = async () => component_cache ??= (await import('../entries/pages/governance/_page.svelte.js')).default;
export const imports = ["_app/immutable/nodes/8.cFCeD-jn.js","_app/immutable/chunks/DMvb8rdj.js","_app/immutable/chunks/Ct2uKrlJ.js","_app/immutable/chunks/Bwzdl04Z.js","_app/immutable/chunks/CbCG2bqt.js"];
export const stylesheets = [];
export const fonts = [];
