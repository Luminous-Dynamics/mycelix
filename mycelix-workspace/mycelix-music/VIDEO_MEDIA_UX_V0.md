# Mycelix Music — Video Media UX v0

Status: **UI foundation / backend integration incomplete**

This work gives video its own discovery and watch experience rather than treating it as an enlarged music player. The current implementation is a front-end foundation, not a claim that a production video catalogue or upload pipeline exists.

## Experience principles

1. **Browse before bandwidth.** Search and filter listings; never autoplay. CID-backed poster thumbnails may be fetched from the selected gateway while browsing, so that request is disclosed in the UI. The media `src` is not attached until the viewer chooses **Load player**; native controls then require an intentional Play action.
2. **Watch with context.** The watch panel reserves space for creator attribution, topics, chapters, captions, a transcript, and a visual-description track. Missing access assets are called out rather than silently hidden.
3. **Resume privately.** Playback position is stored in the current browser's local storage at coarse intervals. It is not published to Holochain, uploaded as a timeline to a gateway, or synchronized to another device. A gateway still sees requests for the CIDs you explicitly load and can infer the corresponding media.
4. **Use only explicit routes.** Content, poster, caption, transcript, and visual-description URLs are derived from CID-addressed resources using the existing resolver. There is no hard-coded public gateway. No media or accessibility URL is constructed until browser-local delivery preferences have loaded.
5. **Fail visibly.** The watch surface distinguishes missing source metadata, missing gateway configuration, and exhausted gateway fallback. A successful HTTP response is still not proof that bytes match the content address.
6. **Keyboard and assistive-tech first.** Cards and filters are buttons with accessible names and focus styles; playback uses the native HTML video controls; chapter choices are keyboard-accessible; motion is limited and respects `prefers-reduced-motion`.
7. **Fixtures are not catalogue truth.** Synthetic content is compiled only when the existing `fixtures` feature is enabled and is labelled on screen. It has no source, poster, caption, transcript, or visual-description CID, so the fixture cannot request or masquerade as real media.

## UI scope in this increment

- New `/videos` route and primary `Watch` navigation link.
- Responsive video discovery cards, title/creator/topic search, and filters by format.
- An inline watch detail surface with native controls, chapters, accessibility resource slots, delivery-state messaging, and local resume position.
- CID-addressed source failover using the existing bounded resolver candidates.
- Explicit unavailable state in non-fixture builds because the Holochain video-listing adapter is not implemented yet.

## Deliberate non-goals

This increment does **not** implement a video publication zome, rights/territory/age policy, live streaming, HLS/CMAF manifests, adaptive bitrate selection, upload/transcoding, content-integrity verification, recommendations, comments, view analytics, or cross-device watch state. The v0 player only describes a CID-addressed resource that a browser can play as a progressive media file; adaptive streaming needs a separate versioned manifest and compatibility tests. Chapter markers and accessibility resources become authoritative only when supplied by a published catalogue record.

## Next implementation gates

1. Define a versioned video publication manifest that links content renditions, poster, captions/subtitles with BCP 47 language tags, transcript, visual description, chapters, credits/provenance, duration, and a rights/availability policy. Do not overload the audio `Song` schema.
2. Wire a read-only authorized Holochain query; keep test fixtures explicitly separate from production results.
3. Implement the actual create/publish flow only after rights and metadata validation are defined. Publication status must not imply that media bytes are pinned or playable.
4. Test progressive-video seeking and range requests through each supported gateway. Later qualify HLS/CMAF and adaptive bitrate separately from MP4/WebM progressive playback.
5. Add visual regression and keyboard/screen-reader review at desktop, tablet, and narrow mobile widths.

## Accessibility and platform references

- [W3C Web Accessibility Initiative: Making Audio and Video Media Accessible](https://www.w3.org/WAI/media/av/) — captions, transcripts, visual descriptions, and accessible players.
- [W3C Media Accessibility User Requirements](https://www.w3.org/TR/media-accessibility-reqs/) — requirements for synchronized text, transcripts, keyboard control, chapters, and descriptions.
- [MDN: HTML video element](https://developer.mozilla.org/en-US/docs/Web/HTML/Reference/Elements/video) and [MDN: `track` element](https://developer.mozilla.org/en-US/docs/Web/HTML/Reference/Elements/track) — browser-native playback and WebVTT cues.

The references inform interface requirements; they do not replace conformance testing against a published accessibility checklist.
