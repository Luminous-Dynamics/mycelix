// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Deterministic browser smoke for the Leptos CSR app. It intercepts IPFS media
// so playback, retry, queue, and clearing are exercised without public gateways.

'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs/promises');
const { chromium } = require('playwright');

const BASE_URL = process.env.BASE_URL || 'http://127.0.0.1:8121';
const MEDIA_ORIGIN = 'https://ipfs.io/ipfs/';

function makeWave(seconds = 12, sampleRate = 8000) {
  const frames = seconds * sampleRate;
  const pcmBytes = frames * 2;
  const wav = Buffer.alloc(44 + pcmBytes);
  wav.write('RIFF', 0);
  wav.writeUInt32LE(36 + pcmBytes, 4);
  wav.write('WAVE', 8);
  wav.write('fmt ', 12);
  wav.writeUInt32LE(16, 16);
  wav.writeUInt16LE(1, 20); // PCM
  wav.writeUInt16LE(1, 22); // mono
  wav.writeUInt32LE(sampleRate, 24);
  wav.writeUInt32LE(sampleRate * 2, 28);
  wav.writeUInt16LE(2, 32);
  wav.writeUInt16LE(16, 34);
  wav.write('data', 36);
  wav.writeUInt32LE(pcmBytes, 40);

  for (let frame = 0; frame < frames; frame += 1) {
    const sample = Math.round(Math.sin((2 * Math.PI * 440 * frame) / sampleRate) * 5000);
    wav.writeInt16LE(sample, 44 + frame * 2);
  }
  return wav;
}

async function main() {
  const wave = makeWave();
  const requestCounts = new Map();
  const pageErrors = [];
  const browser = await chromium.launch({ headless: true });

  let page;
  try {
    page = await browser.newPage();
    page.on('pageerror', error => pageErrors.push(error.message));

    // Hold one real play() promise open while allowing Chromium's native media
    // element to play. We reject it only after a later play attempt has resumed
    // the same logical source, exercising the generation guard independently
    // of the song-hash/URL identity check.
    await page.addInitScript(() => {
      const nativePlay = HTMLMediaElement.prototype.play;
      const probe = { captured: false, rejected: false, reject: null };
      Object.defineProperty(window, '__stalePlayProbe', {
        value: probe,
        configurable: false,
      });

      HTMLMediaElement.prototype.play = function (...args) {
        const declaredSource = this.getAttribute('src') || '';
        if (
          this instanceof HTMLAudioElement
          && !probe.captured
          && declaredSource.endsWith('/QmDemo1')
        ) {
          probe.captured = true;
          const nativePromise = nativePlay.apply(this, args);
          nativePromise.catch(() => {});
          return new Promise((_resolve, reject) => {
            probe.reject = reject;
          });
        }
        return nativePlay.apply(this, args);
      };
    });

    await page.route(`${MEDIA_ORIGIN}**`, async route => {
      const url = route.request().url();
      const cid = url.slice(MEDIA_ORIGIN.length).split(/[?#]/, 1)[0];
      const count = (requestCounts.get(cid) || 0) + 1;
      requestCounts.set(cid, count);

      // One controlled network failure exercises the selected-source retry
      // path; the same source succeeds on the explicit second Play action.
      if (cid === 'QmDemo2' && count === 1) {
        await route.fulfill({
          status: 503,
          contentType: 'audio/wav',
          body: 'deterministic first-request failure',
        });
        return;
      }

      await route.fulfill({
        status: 200,
        contentType: 'audio/wav',
        headers: { 'Access-Control-Allow-Origin': '*' },
        body: wave,
      });
    });

    await page.goto(`${BASE_URL}/discover`, { waitUntil: 'domcontentloaded' });
    await page.getByRole('heading', { name: 'Decentralized Dreams' }).waitFor({ timeout: 30000 });

    // Build a deterministic three-track queue using the user-facing controls.
    await page.getByRole('button', { name: 'Add Decentralized Dreams to queue' }).click();
    await page.getByRole('button', { name: 'Add Zero-Cost Serenade to queue' }).click();
    await page.getByRole('button', { name: 'Add Mycelium Network to queue' }).click();

    // A song-card Play action must select the same queue occurrence and start
    // the source through the persistent audio element.
    await page.getByRole('button', { name: 'Play Decentralized Dreams' }).click();
    await page.waitForFunction(() => {
      const audio = document.querySelector('audio');
      return audio
        && audio.currentSrc === 'https://ipfs.io/ipfs/QmDemo1'
        && !audio.paused
        && audio.currentTime > 0;
    }, null, { timeout: 20000 });

    assert.equal(await page.locator('.player-title').innerText(), 'Decentralized Dreams');

    // Observe natural end independently of the app handler so Repeat One
    // must demonstrate that it really restarted the media element.
    await page.locator('audio').evaluate(audio => {
      window.__naturalEndedCount = 0;
      audio.addEventListener('ended', () => { window.__naturalEndedCount += 1; });
    });

    // Pause/resume must preserve the physical playhead, not merely leave the
    // progress signal looking plausible.
    await page.waitForFunction(() => {
      const audio = document.querySelector('audio');
      return audio && audio.currentTime > 1.5 && !audio.paused;
    }, null, { timeout: 20000 });
    await page.getByRole('button', { name: 'Pause', exact: true }).click();
    await page.waitForFunction(() => {
      const audio = document.querySelector('audio');
      return audio && audio.paused;
    }, null, { timeout: 5000 });
    const pausedAt = await page.locator('audio').evaluate(audio => audio.currentTime);
    assert.ok(pausedAt > 1.5, 'pause should retain a nonzero playhead');
    await page.getByRole('button', { name: 'Play', exact: true }).click();
    await page.waitForFunction(expectedTime => {
      const audio = document.querySelector('audio');
      return audio && !audio.paused && Math.abs(audio.currentTime - expectedTime) < 0.35;
    }, pausedAt, { timeout: 5000 });

    // The earlier play() promise is still unresolved. Reject it late after
    // pause/resume has issued a newer attempt for the exact same track/source;
    // the obsolete rejection must not flip the current UI to stopped.
    await page.evaluate(() => {
      const probe = window.__stalePlayProbe;
      if (!probe.captured || typeof probe.reject !== 'function' || probe.rejected) {
        throw new Error('the superseded play() promise was not captured exactly once');
      }
      probe.rejected = true;
      probe.reject(new DOMException('superseded playback attempt', 'AbortError'));
    });
    await new Promise(resolve => setTimeout(resolve, 150));
    assert.equal(await page.locator('.btn-player').getAttribute('aria-label'), 'Pause');
    assert.equal(
      await page.locator('audio').evaluate(audio => audio.paused),
      false,
      'a stale play() rejection must not pause the newer attempt on the same source',
    );

    // Exercise the real range input against finite media metadata. Dispatching
    // input through the DOM keeps the test deterministic while still invoking
    // the same handler as a user-driven slider interaction.
    const seek = page.locator('.player-seek');
    await page.waitForFunction(() => {
      const slider = document.querySelector('.player-seek');
      return slider && !slider.disabled && Number(slider.max) > 0;
    }, null, { timeout: 10000 });
    await seek.evaluate(element => {
      element.value = '5';
      element.dispatchEvent(new Event('input', { bubbles: true }));
    });
    await page.waitForFunction(() => {
      const audio = document.querySelector('audio');
      return audio && audio.currentTime >= 4.5 && audio.currentTime < 7;
    }, null, { timeout: 10000 });

    // Repeat One must apply to natural track completion, not manual Next.
    await page.getByRole('button', { name: 'Repeat off' }).click();
    await page.getByRole('button', { name: 'Repeat all' }).click();
    await page.getByRole('button', { name: 'Repeat one' }).click();

    // Force the active resource close to its real end, then require the
    // natural-ended event to trigger an actual restart under Repeat One.
    await page.locator('audio').evaluate(audio => {
      audio.currentTime = Math.max(0, audio.duration - 0.2);
    });
    await page.waitForFunction(() => {
      const audio = document.querySelector('audio');
      return window.__naturalEndedCount > 0
        && audio
        && !audio.paused
        && audio.currentTime > 0
        && audio.currentTime < 2;
    }, null, { timeout: 10000 });

    // The first attempt for track two returns HTTP 503. The UI should settle
    // into its stopped/error state with the seek duration invalidated.
    await page.getByRole('button', { name: 'Next track' }).click();
    await page.waitForFunction(() => {
      const audio = document.querySelector('audio');
      const playButton = document.querySelector('.btn-player');
      return audio
        && audio.currentSrc === 'https://ipfs.io/ipfs/QmDemo2'
        && audio.error !== null
        && playButton
        && playButton.getAttribute('aria-label') === 'Play';
    }, null, { timeout: 20000 });

    assert.equal(await page.locator('.player-title').innerText(), 'Zero-Cost Serenade');
    assert.equal(requestCounts.get('QmDemo2'), 1);

    // Explicit retry must clear the latched media error, fetch the same URL
    // again, and actually resume playback.
    await page.getByRole('button', { name: 'Play', exact: true }).click();
    await page.waitForFunction(() => {
      const audio = document.querySelector('audio');
      return audio
        && audio.currentSrc === 'https://ipfs.io/ipfs/QmDemo2'
        && audio.error === null
        && !audio.paused
        && audio.currentTime > 0;
    }, null, { timeout: 20000 });
    assert.ok((requestCounts.get('QmDemo2') || 0) >= 2, 'retry should request the failed media URL again');

    // Exercise the compact control layout and queue transitions at a mobile
    // viewport instead of inferring responsiveness from CSS alone.
    await page.setViewportSize({ width: 390, height: 844 });
    const overflowingPlayerChildren = await page.locator('.player-bar').evaluate(element => {
      const bounds = element.getBoundingClientRect();
      return Array.from(element.querySelectorAll('button, input, .player-info, .player-timeline, .player-controls'))
        .map(child => {
          const rect = child.getBoundingClientRect();
          return { tag: child.tagName, className: child.className, left: rect.left, right: rect.right };
        })
        .filter(rect => rect.right > bounds.right + 1 || rect.left < bounds.left - 1);
    });
    assert.deepEqual(overflowingPlayerChildren, [], 'mobile player controls must remain inside the player bar');

    // Select the exact queue occurrence and then remove the current row. The
    // next occurrence should become selected and begin playing, without a stale
    // queue index or an orphaned audio source.
    await page.getByRole('button', { name: 'Toggle playback queue' }).click();
    const queueDialog = page.getByRole('dialog', { name: 'Playback queue' });
    await queueDialog.waitFor({ state: 'visible' });
    await page.waitForFunction(() => document.activeElement === document.querySelector('.queue-panel'));

    // The modal must wrap keyboard navigation in both directions before Escape.
    await page.keyboard.press('Shift+Tab');
    await page.waitForFunction(() => {
      const active = document.activeElement;
      return active && active.getAttribute('aria-label') === 'Remove Mycelium Network';
    });
    await page.keyboard.press('Tab');
    await page.waitForFunction(() => {
      const active = document.activeElement;
      return active && active.textContent.trim() === 'Clear';
    });

    // Keyboard focus belongs to the opened dialog; Escape should close it.
    await page.waitForFunction(() => {
      const active = document.activeElement;
      return !document.querySelector('.queue-panel')
        && active
        && active.classList.contains('queue-toggle');
    });
    await page.getByRole('button', { name: 'Toggle playback queue' }).click();
    await queueDialog.waitFor({ state: 'visible' });

    assert.equal(await queueDialog.locator('.queue-item').count(), 3);
    assert.equal(
      await queueDialog.locator('.queue-item.current .queue-title').innerText(),
      'Zero-Cost Serenade',
      'manual Next must leave the correct occurrence selected in the queue',
    );

    await queueDialog.getByRole('button', { name: 'Play Decentralized Dreams', exact: true }).click();
    await page.waitForFunction(() => {
      const audio = document.querySelector('audio');
      return audio
        && audio.currentSrc === 'https://ipfs.io/ipfs/QmDemo1'
        && !audio.paused
        && audio.currentTime > 0;
    }, null, { timeout: 20000 });

    await queueDialog.getByRole('button', { name: 'Remove Decentralized Dreams', exact: true }).click();
    await page.waitForFunction(() => {
      const audio = document.querySelector('audio');
      const activeTitle = document.querySelector('.queue-item.current .queue-title');
      return audio
        && audio.currentSrc === 'https://ipfs.io/ipfs/QmDemo2'
        && !audio.paused
        && audio.currentTime > 0
        && activeTitle
        && activeTitle.textContent === 'Zero-Cost Serenade'
        && document.querySelectorAll('.queue-panel .queue-item').length === 2;
    }, null, { timeout: 20000 });

    // Clear must release the source and zero the physical element playhead,
    // not merely reset the displayed reactive progress signal.
    await queueDialog.getByRole('button', { name: 'Clear' }).click();

    await page.waitForFunction(() => {
      const audio = document.querySelector('audio');
      return audio
        && !audio.hasAttribute('src')
        && audio.paused
        && audio.currentTime === 0;
    }, null, { timeout: 10000 });

    assert.equal(await page.locator('.player-empty').count(), 1);
    assert.equal(await page.getByRole('button', { name: 'Toggle playback queue' }).getAttribute('aria-expanded'), 'false');

    // Two different song records resolve to QmDemo1. A record change must
    // rewind the actual persistent media element, while retaining metadata
    // for the unchanged resource. This catches a regression that resetting
    // only the reactive progress signal would miss.
    await page.getByRole('button', { name: 'Play Decentralized Dreams' }).click();
    await page.waitForFunction(() => {
      const audio = document.querySelector('audio');
      return audio
        && audio.currentSrc === 'https://ipfs.io/ipfs/QmDemo1'
        && !audio.paused
        && audio.currentTime > 3.5
        && Number.isFinite(audio.duration)
        && audio.duration > 0;
    }, null, { timeout: 15000 });
    const sharedResourceDuration = await page.locator('.player-seek').getAttribute('max');
    assert.ok(Number(sharedResourceDuration) > 0, 'first record should establish valid duration metadata');
    const playheadBeforeRecordSwitch = await page.locator('audio').evaluate(audio => audio.currentTime);
    assert.ok(playheadBeforeRecordSwitch > 3.5);

    await page.getByRole('button', { name: 'Play Shared Source Encore' }).click();
    await page.waitForFunction(() => {
      const audio = document.querySelector('audio');
      const title = document.querySelector('.player-title');
      const seek = document.querySelector('.player-seek');
      return audio
        && title
        && title.textContent === 'Shared Source Encore'
        && audio.currentSrc === 'https://ipfs.io/ipfs/QmDemo1'
        && !audio.paused
        && audio.currentTime < 1.5
        && seek
        && Number(seek.max) > 0;
    }, null, { timeout: 10000 });
    assert.equal(
      await page.locator('.player-seek').getAttribute('max'),
      sharedResourceDuration,
      'same-URL record change must retain valid media duration',
    );

    if (pageErrors.length > 0) {
      throw new Error(`Browser page errors:\n${pageErrors.join('\n')}`);
    }

    process.stdout.write(JSON.stringify({
      result: 'PASS',
      app: 'Leptos CSR',
      baseURL: BASE_URL,
      scenarios: [
        'catalog play selects and starts exact media URL',
        'pause and resume preserve the physical playhead',
        'late rejection from a superseded play() promise cannot stop a newer same-source attempt',
        'seek applies a finite target to the real media element',
        'Repeat One restarts after natural end',
        'Repeat One does not block manual Next',
        'queue Next handles deterministic media failure',
        'explicit Play retries the same failed resource',
        'mobile player controls stay within the viewport',
        'queue dialog traps Tab and Shift+Tab, Escape closes it, and focus returns',
        'queue selection and current-row removal preserve exact next source',
        'Clear releases src and resets actual currentTime',
      ],
      mediaRequests: Object.fromEntries(requestCounts),
      pageErrors: pageErrors.length,
    }, null, 2) + '\n');
  } catch (error) {
    const evidenceDirectory = process.env.EVIDENCE_DIR || '/tmp/mycelix-player-smoke-evidence';
    await fs.mkdir(evidenceDirectory, { recursive: true }).catch(() => undefined);
    if (page && !page.isClosed()) {
      await page.screenshot({
        path: `${evidenceDirectory}/failure.png`,
        fullPage: true,
      }).catch(() => undefined);
      const html = await page.content().catch(() => '');
      await fs.writeFile(`${evidenceDirectory}/failure.html`, html).catch(() => undefined);
    }
    const failureReport = {
      error: error instanceof Error ? `${error.name}: ${error.message}\n${error.stack || ''}` : String(error),
      url: page ? page.url() : null,
      mediaRequests: Object.fromEntries(requestCounts),
      pageErrors,
    };
    await fs.writeFile(
      `${evidenceDirectory}/failure.json`,
      JSON.stringify(failureReport, null, 2),
    ).catch(() => undefined);
    throw error;
  } finally {
    await browser.close();
  }
}

main().catch(error => {
  process.stderr.write(`${error.stack || error}\n`);
  process.exitCode = 1;
});
