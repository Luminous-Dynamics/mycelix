// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Email Import/Export Service for Mycelix Mail
 *
 * Supports standard email formats:
 * - MBOX (mailbox format)
 * - EML (single email)
 * - Batch import/export
 * - Progress tracking
 * - Validation and sanitization
 */

import type { ActionHash, AgentPubKey } from '@holochain/client';
import type { MycelixMailClient } from '../index';
import type { DecryptedEmail, EmailAttachment } from '../types';

// ==================== TYPES ====================

export interface ImportOptions {
  /** Skip duplicate detection */
  skipDuplicates?: boolean;
  /** Target folder for imports */
  targetFolder?: ActionHash;
  /** Labels to apply */
  labels?: string[];
  /** Progress callback */
  onProgress?: (progress: ImportProgress) => void;
  /** Validate sender addresses */
  validateSenders?: boolean;
  /** Maximum attachment size (bytes) */
  maxAttachmentSize?: number;
}

export interface ExportOptions {
  /** Export format */
  format: 'mbox' | 'eml' | 'json';
  /** Include attachments */
  includeAttachments?: boolean;
  /** Compress output */
  compress?: boolean;
  /** Progress callback */
  onProgress?: (progress: ExportProgress) => void;
}

export interface ImportProgress {
  total: number;
  processed: number;
  imported: number;
  skipped: number;
  failed: number;
  currentEmail?: string;
  errors: ImportError[];
}

export interface ExportProgress {
  total: number;
  processed: number;
  currentEmail?: string;
}

export interface ImportResult {
  success: boolean;
  imported: number;
  skipped: number;
  failed: number;
  errors: ImportError[];
  duration: number;
  importedHashes: ActionHash[];
}

export interface ExportResult {
  success: boolean;
  exported: number;
  data: Uint8Array | string;
  format: string;
  duration: number;
}

export interface ImportError {
  index: number;
  subject?: string;
  error: string;
  recoverable: boolean;
}

export interface ParsedEmail {
  messageId?: string;
  from: string;
  to: string[];
  cc: string[];
  bcc: string[];
  subject: string;
  date: Date;
  body: string;
  htmlBody?: string;
  headers: Map<string, string>;
  attachments: ParsedAttachment[];
  inReplyTo?: string;
  references: string[];
}

export interface ParsedAttachment {
  filename: string;
  mimeType: string;
  content: Uint8Array;
  contentId?: string;
  size: number;
}

// ==================== IMPORT/EXPORT SERVICE ====================

export class ImportExportService {
  private textEncoder = new TextEncoder();
  private textDecoder = new TextDecoder();

  constructor(private client: MycelixMailClient) {}

  // ==================== MBOX IMPORT ====================

  /**
   * Import emails from MBOX format
   */
  async importMbox(
    mboxData: Uint8Array | string,
    options: ImportOptions = {}
  ): Promise<ImportResult> {
    const startTime = Date.now();
    const content = typeof mboxData === 'string' ? mboxData : this.textDecoder.decode(mboxData);

    const emails = this.parseMbox(content);
    return this.importParsedEmails(emails, options, startTime);
  }

  /**
   * Parse MBOX format into individual emails
   */
  private parseMbox(content: string): ParsedEmail[] {
    const emails: ParsedEmail[] = [];
    const mboxRegex = /^From .+$/gm;
    const parts = content.split(mboxRegex).filter(Boolean);

    for (const part of parts) {
      try {
        const email = this.parseEmailContent(part.trim());
        if (email) {
          emails.push(email);
        }
      } catch (e) {
        // Skip malformed emails in MBOX
        console.warn('Skipping malformed email in MBOX:', e);
      }
    }

    return emails;
  }

  // ==================== EML IMPORT ====================

  /**
   * Import a single EML file
   */
  async importEml(
    emlData: Uint8Array | string,
    options: ImportOptions = {}
  ): Promise<ImportResult> {
    const startTime = Date.now();
    const content = typeof emlData === 'string' ? emlData : this.textDecoder.decode(emlData);

    const email = this.parseEmailContent(content);
    if (!email) {
      return {
        success: false,
        imported: 0,
        skipped: 0,
        failed: 1,
        errors: [{ index: 0, error: 'Failed to parse EML content', recoverable: false }],
        duration: Date.now() - startTime,
        importedHashes: [],
      };
    }

    return this.importParsedEmails([email], options, startTime);
  }

  /**
   * Import multiple EML files
   */
  async importEmls(
    emlFiles: Array<{ name: string; data: Uint8Array | string }>,
    options: ImportOptions = {}
  ): Promise<ImportResult> {
    const startTime = Date.now();
    const emails: ParsedEmail[] = [];
    const parseErrors: ImportError[] = [];

    for (let i = 0; i < emlFiles.length; i++) {
      const file = emlFiles[i];
      try {
        const content = typeof file.data === 'string'
          ? file.data
          : this.textDecoder.decode(file.data);
        const email = this.parseEmailContent(content);
        if (email) {
          emails.push(email);
        } else {
          parseErrors.push({
            index: i,
            subject: file.name,
            error: 'Failed to parse EML',
            recoverable: false,
          });
        }
      } catch (e) {
        parseErrors.push({
          index: i,
          subject: file.name,
          error: String(e),
          recoverable: false,
        });
      }
    }

    const result = await this.importParsedEmails(emails, options, startTime);
    result.errors = [...parseErrors, ...result.errors];
    result.failed += parseErrors.length;

    return result;
  }

  // ==================== EMAIL PARSING ====================

  /**
   * Parse raw email content (RFC 5322 format)
   */
  private parseEmailContent(content: string): ParsedEmail | null {
    const lines = content.split(/\r?\n/);
    const headers = new Map<string, string>();
    let bodyStart = 0;

    // Parse headers
    let currentHeader = '';
    let currentValue = '';

    for (let i = 0; i < lines.length; i++) {
      const line = lines[i];

      // Empty line marks end of headers
      if (line === '') {
        if (currentHeader) {
          headers.set(currentHeader.toLowerCase(), currentValue.trim());
        }
        bodyStart = i + 1;
        break;
      }

      // Continuation line (starts with whitespace)
      if (/^\s/.test(line)) {
        currentValue += ' ' + line.trim();
        continue;
      }

      // New header
      if (currentHeader) {
        headers.set(currentHeader.toLowerCase(), currentValue.trim());
      }

      const colonIndex = line.indexOf(':');
      if (colonIndex > 0) {
        currentHeader = line.substring(0, colonIndex);
        currentValue = line.substring(colonIndex + 1);
      }
    }

    // Parse body
    const bodyLines = lines.slice(bodyStart);
    const rawBody = bodyLines.join('\n');

    // Extract body based on content type
    const contentType = headers.get('content-type') || 'text/plain';
    const { body, htmlBody, attachments } = this.parseBody(rawBody, contentType);

    // Parse addresses
    const from = this.parseAddress(headers.get('from') || '');
    const to = this.parseAddressList(headers.get('to') || '');
    const cc = this.parseAddressList(headers.get('cc') || '');
    const bcc = this.parseAddressList(headers.get('bcc') || '');

    // Parse date
    const dateStr = headers.get('date') || '';
    const date = this.parseDate(dateStr);

    // Parse references
    const inReplyTo = headers.get('in-reply-to')?.replace(/[<>]/g, '');
    const referencesStr = headers.get('references') || '';
    const references = referencesStr
      .split(/\s+/)
      .filter(Boolean)
      .map((r) => r.replace(/[<>]/g, ''));

    return {
      messageId: headers.get('message-id')?.replace(/[<>]/g, ''),
      from,
      to,
      cc,
      bcc,
      subject: this.decodeHeader(headers.get('subject') || '(No Subject)'),
      date,
      body,
      htmlBody,
      headers,
      attachments,
      inReplyTo,
      references,
    };
  }

  /**
   * Parse email body handling MIME multipart
   */
  private parseBody(
    rawBody: string,
    contentType: string
  ): { body: string; htmlBody?: string; attachments: ParsedAttachment[] } {
    const attachments: ParsedAttachment[] = [];
    let body = '';
    let htmlBody: string | undefined;

    // Check for multipart
    const boundaryMatch = contentType.match(/boundary="?([^";\s]+)"?/i);

    if (boundaryMatch) {
      const boundary = boundaryMatch[1];
      const parts = rawBody.split(new RegExp(`--${this.escapeRegex(boundary)}`));

      for (const part of parts) {
        if (part.trim() === '' || part.trim() === '--') continue;

        const partLines = part.split(/\r?\n/);
        const partHeaders = new Map<string, string>();
        let partBodyStart = 0;

        // Parse part headers
        for (let i = 0; i < partLines.length; i++) {
          if (partLines[i] === '') {
            partBodyStart = i + 1;
            break;
          }
          const colonIdx = partLines[i].indexOf(':');
          if (colonIdx > 0) {
            const key = partLines[i].substring(0, colonIdx).toLowerCase();
            const value = partLines[i].substring(colonIdx + 1).trim();
            partHeaders.set(key, value);
          }
        }

        const partBody = partLines.slice(partBodyStart).join('\n');
        const partContentType = partHeaders.get('content-type') || 'text/plain';
        const disposition = partHeaders.get('content-disposition') || '';
        const encoding = partHeaders.get('content-transfer-encoding') || '';

        // Decode content
        const decodedBody = this.decodeBody(partBody, encoding);

        // Check if it's an attachment
        if (disposition.includes('attachment') || disposition.includes('inline')) {
          const filenameMatch = disposition.match(/filename="?([^";\n]+)"?/i);
          const filename = filenameMatch ? filenameMatch[1] : 'attachment';
          const mimeType = partContentType.split(';')[0].trim();

          attachments.push({
            filename,
            mimeType,
            content: this.textEncoder.encode(decodedBody),
            contentId: partHeaders.get('content-id')?.replace(/[<>]/g, ''),
            size: decodedBody.length,
          });
        } else if (partContentType.includes('text/html')) {
          htmlBody = decodedBody;
        } else if (partContentType.includes('text/plain')) {
          body = decodedBody;
        } else if (partContentType.includes('multipart/')) {
          // Nested multipart - recurse
          const nested = this.parseBody(partBody, partContentType);
          if (!body && nested.body) body = nested.body;
          if (!htmlBody && nested.htmlBody) htmlBody = nested.htmlBody;
          attachments.push(...nested.attachments);
        }
      }
    } else {
      // Simple body
      const encoding = contentType.match(/charset="?([^";\s]+)"?/i)?.[1] || 'utf-8';
      body = rawBody;
    }

    // Fallback to HTML if no plain text
    if (!body && htmlBody) {
      body = this.stripHtml(htmlBody);
    }

    return { body, htmlBody, attachments };
  }

  /**
   * Decode body based on transfer encoding
   */
  private decodeBody(body: string, encoding: string): string {
    const enc = encoding.toLowerCase();

    if (enc === 'base64') {
      try {
        return atob(body.replace(/\s/g, ''));
      } catch {
        return body;
      }
    }

    if (enc === 'quoted-printable') {
      return body
        .replace(/=\r?\n/g, '')
        .replace(/=([0-9A-F]{2})/gi, (_, hex) =>
          String.fromCharCode(parseInt(hex, 16))
        );
    }

    return body;
  }

  /**
   * Decode RFC 2047 encoded header
   */
  private decodeHeader(header: string): string {
    return header.replace(
      /=\?([^?]+)\?([BQ])\?([^?]+)\?=/gi,
      (_, charset, encoding, text) => {
        if (encoding.toUpperCase() === 'B') {
          try {
            return atob(text);
          } catch {
            return text;
          }
        } else if (encoding.toUpperCase() === 'Q') {
          return text.replace(/_/g, ' ').replace(/=([0-9A-F]{2})/gi, (__, hex) =>
            String.fromCharCode(parseInt(hex, 16))
          );
        }
        return text;
      }
    );
  }

  /**
   * Parse email address from header
   */
  private parseAddress(header: string): string {
    // Handle "Name <email>" format
    const angleMatch = header.match(/<([^>]+)>/);
    if (angleMatch) {
      return angleMatch[1].trim();
    }
    return header.trim();
  }

  /**
   * Parse address list
   */
  private parseAddressList(header: string): string[] {
    if (!header) return [];

    // Split by comma, but respect quoted strings
    const addresses: string[] = [];
    let current = '';
    let inQuotes = false;
    let depth = 0;

    for (const char of header) {
      if (char === '"') {
        inQuotes = !inQuotes;
      } else if (char === '<') {
        depth++;
      } else if (char === '>') {
        depth--;
      } else if (char === ',' && !inQuotes && depth === 0) {
        const addr = this.parseAddress(current);
        if (addr) addresses.push(addr);
        current = '';
        continue;
      }
      current += char;
    }

    const lastAddr = this.parseAddress(current);
    if (lastAddr) addresses.push(lastAddr);

    return addresses;
  }

  /**
   * Parse date header
   */
  private parseDate(dateStr: string): Date {
    if (!dateStr) return new Date();

    try {
      // Try standard parsing first
      const date = new Date(dateStr);
      if (!isNaN(date.getTime())) {
        return date;
      }

      // Handle common formats
      // "Tue, 15 Jan 2024 10:30:00 +0000"
      const rfc2822Match = dateStr.match(
        /(\d{1,2})\s+(\w{3})\s+(\d{4})\s+(\d{2}):(\d{2}):(\d{2})/
      );
      if (rfc2822Match) {
        const [, day, month, year, hour, min, sec] = rfc2822Match;
        const months: Record<string, number> = {
          Jan: 0, Feb: 1, Mar: 2, Apr: 3, May: 4, Jun: 5,
          Jul: 6, Aug: 7, Sep: 8, Oct: 9, Nov: 10, Dec: 11,
        };
        return new Date(
          parseInt(year),
          months[month] ?? 0,
          parseInt(day),
          parseInt(hour),
          parseInt(min),
          parseInt(sec)
        );
      }

      return new Date();
    } catch {
      return new Date();
    }
  }

  /**
   * Strip HTML tags
   */
  private stripHtml(html: string): string {
    return html
      .replace(/<style[^>]*>[\s\S]*?<\/style>/gi, '')
      .replace(/<script[^>]*>[\s\S]*?<\/script>/gi, '')
      .replace(/<[^>]+>/g, '')
      .replace(/&nbsp;/g, ' ')
      .replace(/&lt;/g, '<')
      .replace(/&gt;/g, '>')
      .replace(/&amp;/g, '&')
      .replace(/\s+/g, ' ')
      .trim();
  }

  // ==================== IMPORT EXECUTION ====================

  /**
   * Import parsed emails into Mycelix
   */
  private async importParsedEmails(
    emails: ParsedEmail[],
    options: ImportOptions,
    startTime: number
  ): Promise<ImportResult> {
    const errors: ImportError[] = [];
    const importedHashes: ActionHash[] = [];
    let imported = 0;
    let skipped = 0;

    for (let i = 0; i < emails.length; i++) {
      const email = emails[i];

      try {
        // Report progress
        if (options.onProgress) {
          options.onProgress({
            total: emails.length,
            processed: i,
            imported,
            skipped,
            failed: errors.length,
            currentEmail: email.subject,
            errors,
          });
        }

        // Check for duplicates by message ID
        if (!options.skipDuplicates && email.messageId) {
          const existing = await this.findByMessageId(email.messageId);
          if (existing) {
            skipped++;
            continue;
          }
        }

        // Validate attachment sizes
        if (options.maxAttachmentSize) {
          const oversized = email.attachments.filter(
            (a) => a.size > options.maxAttachmentSize!
          );
          if (oversized.length > 0) {
            errors.push({
              index: i,
              subject: email.subject,
              error: `Attachments exceed size limit: ${oversized.map((a) => a.filename).join(', ')}`,
              recoverable: true,
            });
            // Continue without attachments
            email.attachments = email.attachments.filter(
              (a) => a.size <= options.maxAttachmentSize!
            );
          }
        }

        // Convert to Mycelix format and store
        const hash = await this.storeImportedEmail(email, options);
        importedHashes.push(hash);
        imported++;

      } catch (e) {
        errors.push({
          index: i,
          subject: email.subject,
          error: String(e),
          recoverable: false,
        });
      }
    }

    // Final progress report
    if (options.onProgress) {
      options.onProgress({
        total: emails.length,
        processed: emails.length,
        imported,
        skipped,
        failed: errors.length,
        errors,
      });
    }

    return {
      success: errors.length === 0,
      imported,
      skipped,
      failed: errors.length,
      errors,
      duration: Date.now() - startTime,
      importedHashes,
    };
  }

  /**
   * Check if email with message ID already exists
   */
  private async findByMessageId(messageId: string): Promise<ActionHash | null> {
    try {
      const results = await this.client.messages.searchEmails(`message-id:${messageId}`);
      return results.length > 0 ? results[0].hash : null;
    } catch {
      return null;
    }
  }

  /**
   * Store imported email in Holochain
   */
  private async storeImportedEmail(
    email: ParsedEmail,
    options: ImportOptions
  ): Promise<ActionHash> {
    // Convert attachments
    const attachments: EmailAttachment[] = email.attachments.map((a) => ({
      filename: a.filename,
      mime_type: a.mimeType,
      data: a.content,
      size: a.size,
    }));

    // Create email draft (imported emails go to a special imported state)
    const hash = await this.client.messages.createDraft({
      to: email.to,
      cc: email.cc,
      bcc: email.bcc,
      subject: email.subject,
      body: email.body,
      html_body: email.htmlBody,
      attachments,
      in_reply_to: email.inReplyTo,
      references: email.references,
      imported_from: email.from,
      imported_date: email.date.toISOString(),
      message_id: email.messageId,
    });

    // Apply labels if specified
    if (options.labels?.length) {
      for (const label of options.labels) {
        await this.client.messages.addLabel(hash, label);
      }
    }

    // Move to folder if specified
    if (options.targetFolder) {
      await this.client.messages.moveToFolder(hash, options.targetFolder);
    }

    // Mark as imported (special state)
    await this.client.messages.markAsImported(hash);

    return hash;
  }

  // ==================== MBOX EXPORT ====================

  /**
   * Export emails to MBOX format
   */
  async exportToMbox(
    emailHashes: ActionHash[],
    options: ExportOptions
  ): Promise<ExportResult> {
    const startTime = Date.now();
    const mboxParts: string[] = [];

    for (let i = 0; i < emailHashes.length; i++) {
      const hash = emailHashes[i];

      if (options.onProgress) {
        options.onProgress({
          total: emailHashes.length,
          processed: i,
        });
      }

      try {
        const email = await this.client.messages.getEmail(hash);
        if (email) {
          const mboxEntry = this.emailToMbox(email, options.includeAttachments ?? true);
          mboxParts.push(mboxEntry);
        }
      } catch (e) {
        console.warn(`Failed to export email ${hash}:`, e);
      }
    }

    const content = mboxParts.join('\n');
    const data = options.compress
      ? await this.compress(content)
      : this.textEncoder.encode(content);

    return {
      success: true,
      exported: mboxParts.length,
      data,
      format: 'mbox',
      duration: Date.now() - startTime,
    };
  }

  /**
   * Convert email to MBOX entry
   */
  private emailToMbox(email: DecryptedEmail, includeAttachments: boolean): string {
    const lines: string[] = [];
    const timestamp = new Date(email.timestamp);

    // MBOX "From " line
    const fromLine = `From ${email.sender} ${timestamp.toUTCString()}`;
    lines.push(fromLine);

    // Headers
    lines.push(`From: ${email.sender}`);
    lines.push(`To: ${email.recipients.to.join(', ')}`);
    if (email.recipients.cc?.length) {
      lines.push(`Cc: ${email.recipients.cc.join(', ')}`);
    }
    lines.push(`Subject: ${email.subject}`);
    lines.push(`Date: ${timestamp.toUTCString()}`);
    if (email.message_id) {
      lines.push(`Message-ID: <${email.message_id}>`);
    }
    if (email.in_reply_to) {
      lines.push(`In-Reply-To: <${email.in_reply_to}>`);
    }
    if (email.references?.length) {
      lines.push(`References: ${email.references.map((r) => `<${r}>`).join(' ')}`);
    }

    // Handle multipart
    if ((email.html_body || (includeAttachments && email.attachments?.length))) {
      const boundary = this.generateBoundary();
      lines.push(`MIME-Version: 1.0`);
      lines.push(`Content-Type: multipart/mixed; boundary="${boundary}"`);
      lines.push('');

      // Plain text part
      lines.push(`--${boundary}`);
      lines.push('Content-Type: text/plain; charset=utf-8');
      lines.push('Content-Transfer-Encoding: quoted-printable');
      lines.push('');
      lines.push(this.encodeQuotedPrintable(email.body));

      // HTML part
      if (email.html_body) {
        lines.push(`--${boundary}`);
        lines.push('Content-Type: text/html; charset=utf-8');
        lines.push('Content-Transfer-Encoding: quoted-printable');
        lines.push('');
        lines.push(this.encodeQuotedPrintable(email.html_body));
      }

      // Attachments
      if (includeAttachments && email.attachments?.length) {
        for (const attachment of email.attachments) {
          lines.push(`--${boundary}`);
          lines.push(`Content-Type: ${attachment.mime_type}; name="${attachment.filename}"`);
          lines.push(`Content-Disposition: attachment; filename="${attachment.filename}"`);
          lines.push('Content-Transfer-Encoding: base64');
          lines.push('');
          lines.push(this.encodeBase64(attachment.data));
        }
      }

      lines.push(`--${boundary}--`);
    } else {
      lines.push('Content-Type: text/plain; charset=utf-8');
      lines.push('');
      // Escape "From " at start of lines in body
      lines.push(email.body.replace(/^From /gm, '>From '));
    }

    lines.push('');
    return lines.join('\n');
  }

  // ==================== EML EXPORT ====================

  /**
   * Export single email to EML format
   */
  async exportToEml(
    emailHash: ActionHash,
    options: ExportOptions
  ): Promise<ExportResult> {
    const startTime = Date.now();

    const email = await this.client.messages.getEmail(emailHash);
    if (!email) {
      return {
        success: false,
        exported: 0,
        data: '',
        format: 'eml',
        duration: Date.now() - startTime,
      };
    }

    const emlContent = this.emailToEml(email, options.includeAttachments ?? true);
    const data = options.compress
      ? await this.compress(emlContent)
      : this.textEncoder.encode(emlContent);

    return {
      success: true,
      exported: 1,
      data,
      format: 'eml',
      duration: Date.now() - startTime,
    };
  }

  /**
   * Export multiple emails to individual EML files (returns zip)
   */
  async exportToEmls(
    emailHashes: ActionHash[],
    options: ExportOptions
  ): Promise<ExportResult & { files: Map<string, Uint8Array> }> {
    const startTime = Date.now();
    const files = new Map<string, Uint8Array>();

    for (let i = 0; i < emailHashes.length; i++) {
      const hash = emailHashes[i];

      if (options.onProgress) {
        options.onProgress({
          total: emailHashes.length,
          processed: i,
        });
      }

      try {
        const email = await this.client.messages.getEmail(hash);
        if (email) {
          const emlContent = this.emailToEml(email, options.includeAttachments ?? true);
          const filename = this.sanitizeFilename(email.subject) + '.eml';
          files.set(filename, this.textEncoder.encode(emlContent));
        }
      } catch (e) {
        console.warn(`Failed to export email ${hash}:`, e);
      }
    }

    return {
      success: true,
      exported: files.size,
      data: new Uint8Array(0), // Actual files in the map
      format: 'eml',
      duration: Date.now() - startTime,
      files,
    };
  }

  /**
   * Convert email to EML format
   */
  private emailToEml(email: DecryptedEmail, includeAttachments: boolean): string {
    const lines: string[] = [];
    const timestamp = new Date(email.timestamp);

    // Headers
    lines.push(`From: ${email.sender}`);
    lines.push(`To: ${email.recipients.to.join(', ')}`);
    if (email.recipients.cc?.length) {
      lines.push(`Cc: ${email.recipients.cc.join(', ')}`);
    }
    lines.push(`Subject: ${this.encodeHeaderIfNeeded(email.subject)}`);
    lines.push(`Date: ${timestamp.toUTCString()}`);
    lines.push(`MIME-Version: 1.0`);

    if (email.message_id) {
      lines.push(`Message-ID: <${email.message_id}>`);
    }
    if (email.in_reply_to) {
      lines.push(`In-Reply-To: <${email.in_reply_to}>`);
    }
    if (email.references?.length) {
      lines.push(`References: ${email.references.map((r) => `<${r}>`).join(' ')}`);
    }

    // Mycelix-specific headers
    lines.push(`X-Mycelix-Hash: ${email.hash}`);
    lines.push(`X-Mycelix-State: ${email.state}`);

    // Body
    if (email.html_body || (includeAttachments && email.attachments?.length)) {
      const boundary = this.generateBoundary();
      lines.push(`Content-Type: multipart/mixed; boundary="${boundary}"`);
      lines.push('');

      // Text part
      lines.push(`--${boundary}`);
      lines.push('Content-Type: text/plain; charset=utf-8');
      lines.push('Content-Transfer-Encoding: quoted-printable');
      lines.push('');
      lines.push(this.encodeQuotedPrintable(email.body));

      // HTML part
      if (email.html_body) {
        lines.push(`--${boundary}`);
        lines.push('Content-Type: text/html; charset=utf-8');
        lines.push('Content-Transfer-Encoding: quoted-printable');
        lines.push('');
        lines.push(this.encodeQuotedPrintable(email.html_body));
      }

      // Attachments
      if (includeAttachments && email.attachments?.length) {
        for (const attachment of email.attachments) {
          lines.push(`--${boundary}`);
          lines.push(`Content-Type: ${attachment.mime_type}; name="${attachment.filename}"`);
          lines.push(`Content-Disposition: attachment; filename="${attachment.filename}"`);
          lines.push('Content-Transfer-Encoding: base64');
          lines.push('');
          lines.push(this.encodeBase64(attachment.data));
        }
      }

      lines.push(`--${boundary}--`);
    } else {
      lines.push('Content-Type: text/plain; charset=utf-8');
      lines.push('Content-Transfer-Encoding: quoted-printable');
      lines.push('');
      lines.push(this.encodeQuotedPrintable(email.body));
    }

    return lines.join('\r\n');
  }

  // ==================== JSON EXPORT ====================

  /**
   * Export emails to JSON format
   */
  async exportToJson(
    emailHashes: ActionHash[],
    options: ExportOptions
  ): Promise<ExportResult> {
    const startTime = Date.now();
    const emails: DecryptedEmail[] = [];

    for (let i = 0; i < emailHashes.length; i++) {
      const hash = emailHashes[i];

      if (options.onProgress) {
        options.onProgress({
          total: emailHashes.length,
          processed: i,
        });
      }

      try {
        const email = await this.client.messages.getEmail(hash);
        if (email) {
          // Optionally strip attachments
          if (!options.includeAttachments) {
            email.attachments = [];
          }
          emails.push(email);
        }
      } catch (e) {
        console.warn(`Failed to export email ${hash}:`, e);
      }
    }

    const jsonContent = JSON.stringify(
      {
        version: '1.0',
        exported_at: new Date().toISOString(),
        count: emails.length,
        emails,
      },
      null,
      2
    );

    const data = options.compress
      ? await this.compress(jsonContent)
      : this.textEncoder.encode(jsonContent);

    return {
      success: true,
      exported: emails.length,
      data,
      format: 'json',
      duration: Date.now() - startTime,
    };
  }

  // ==================== UTILITIES ====================

  private escapeRegex(str: string): string {
    return str.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
  }

  private generateBoundary(): string {
    const chars = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789';
    let boundary = '----=_Part_';
    for (let i = 0; i < 24; i++) {
      boundary += chars.charAt(Math.floor(Math.random() * chars.length));
    }
    return boundary;
  }

  private encodeQuotedPrintable(str: string): string {
    return str
      .split('')
      .map((char) => {
        const code = char.charCodeAt(0);
        if (code >= 32 && code <= 126 && char !== '=') {
          return char;
        }
        return '=' + code.toString(16).toUpperCase().padStart(2, '0');
      })
      .join('')
      .replace(/(.{75})/g, '$1=\r\n');
  }

  private encodeBase64(data: Uint8Array): string {
    const binary = Array.from(data)
      .map((b) => String.fromCharCode(b))
      .join('');
    const base64 = btoa(binary);
    // Split into 76-char lines
    return base64.match(/.{1,76}/g)?.join('\r\n') ?? base64;
  }

  private encodeHeaderIfNeeded(str: string): string {
    // Check if encoding needed (non-ASCII characters)
    if (/[^\x20-\x7E]/.test(str)) {
      return `=?UTF-8?B?${btoa(unescape(encodeURIComponent(str)))}?=`;
    }
    return str;
  }

  private sanitizeFilename(name: string): string {
    return name
      .replace(/[<>:"/\\|?*]/g, '_')
      .replace(/\s+/g, '_')
      .substring(0, 100);
  }

  private async compress(data: string): Promise<Uint8Array> {
    // Use CompressionStream if available (modern browsers)
    if (typeof CompressionStream !== 'undefined') {
      const stream = new Blob([data])
        .stream()
        .pipeThrough(new CompressionStream('gzip'));
      const response = new Response(stream);
      const buffer = await response.arrayBuffer();
      return new Uint8Array(buffer);
    }
    // Fallback: return uncompressed
    return this.textEncoder.encode(data);
  }
}

/**
 * Create import/export service for a client
 */
export function createImportExportService(client: MycelixMailClient): ImportExportService {
  return new ImportExportService(client);
}

export default ImportExportService;
