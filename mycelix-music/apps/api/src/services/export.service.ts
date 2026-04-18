// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Data Export Service
 *
 * CSV/JSON export of analytics, play history, and earnings.
 * Supports streaming for large datasets.
 */

import { Pool } from 'pg';
import { Readable, Transform } from 'stream';
import { createGzip } from 'zlib';
import { getLogger } from '../logging';

const logger = getLogger();

/**
 * Export format
 */
export type ExportFormat = 'csv' | 'json' | 'ndjson';

/**
 * Export options
 */
export interface ExportOptions {
  format: ExportFormat;
  compress?: boolean;
  dateFrom?: Date;
  dateTo?: Date;
  limit?: number;
}

/**
 * Export job status
 */
export interface ExportJob {
  id: string;
  type: string;
  status: 'pending' | 'processing' | 'completed' | 'failed';
  progress: number;
  totalRows?: number;
  filePath?: string;
  error?: string;
  createdAt: Date;
  completedAt?: Date;
}

/**
 * CSV Transformer
 */
class CSVTransformer extends Transform {
  private headers: string[] | null = null;
  private isFirstChunk = true;

  constructor() {
    super({ objectMode: true });
  }

  _transform(row: Record<string, unknown>, encoding: string, callback: Function): void {
    try {
      if (!this.headers) {
        this.headers = Object.keys(row);
        // Emit headers
        this.push(this.headers.map(h => this.escapeCSV(h)).join(',') + '\n');
      }

      // Emit row
      const values = this.headers.map(h => this.escapeCSV(String(row[h] ?? '')));
      this.push(values.join(',') + '\n');

      callback();
    } catch (error) {
      callback(error);
    }
  }

  private escapeCSV(value: string): string {
    if (value.includes(',') || value.includes('"') || value.includes('\n')) {
      return `"${value.replace(/"/g, '""')}"`;
    }
    return value;
  }
}

/**
 * JSON Array Transformer
 */
class JSONArrayTransformer extends Transform {
  private isFirst = true;

  constructor() {
    super({ objectMode: true });
    this.push('[\n');
  }

  _transform(row: Record<string, unknown>, encoding: string, callback: Function): void {
    try {
      if (!this.isFirst) {
        this.push(',\n');
      }
      this.isFirst = false;
      this.push('  ' + JSON.stringify(row));
      callback();
    } catch (error) {
      callback(error);
    }
  }

  _flush(callback: Function): void {
    this.push('\n]');
    callback();
  }
}

/**
 * NDJSON Transformer
 */
class NDJSONTransformer extends Transform {
  constructor() {
    super({ objectMode: true });
  }

  _transform(row: Record<string, unknown>, encoding: string, callback: Function): void {
    try {
      this.push(JSON.stringify(row) + '\n');
      callback();
    } catch (error) {
      callback(error);
    }
  }
}

/**
 * Data Export Service
 */
export class ExportService {
  constructor(private pool: Pool) {}

  /**
   * Export play history for a wallet
   */
  async exportPlayHistory(
    walletAddress: string,
    options: ExportOptions
  ): Promise<Readable> {
    const { format, dateFrom, dateTo, limit } = options;

    let query = `
      SELECT
        p.id,
        p.song_id,
        s.title as song_title,
        s.artist_name,
        s.genre,
        p.duration,
        p.played_at,
        p.source
      FROM plays p
      JOIN songs s ON p.song_id = s.id
      WHERE p.wallet_address = $1
    `;

    const params: unknown[] = [walletAddress.toLowerCase()];

    if (dateFrom) {
      params.push(dateFrom);
      query += ` AND p.played_at >= $${params.length}`;
    }

    if (dateTo) {
      params.push(dateTo);
      query += ` AND p.played_at <= $${params.length}`;
    }

    query += ` ORDER BY p.played_at DESC`;

    if (limit) {
      params.push(limit);
      query += ` LIMIT $${params.length}`;
    }

    return this.streamQuery(query, params, format, options.compress);
  }

  /**
   * Export artist analytics
   */
  async exportArtistAnalytics(
    artistAddress: string,
    options: ExportOptions
  ): Promise<Readable> {
    const { format, dateFrom, dateTo } = options;

    let query = `
      SELECT
        date_trunc('day', p.played_at) as date,
        s.id as song_id,
        s.title as song_title,
        COUNT(p.id) as play_count,
        COUNT(DISTINCT p.wallet_address) as unique_listeners,
        SUM(p.duration) as total_duration_seconds
      FROM plays p
      JOIN songs s ON p.song_id = s.id
      WHERE s.artist_address = $1
    `;

    const params: unknown[] = [artistAddress.toLowerCase()];

    if (dateFrom) {
      params.push(dateFrom);
      query += ` AND p.played_at >= $${params.length}`;
    }

    if (dateTo) {
      params.push(dateTo);
      query += ` AND p.played_at <= $${params.length}`;
    }

    query += `
      GROUP BY date_trunc('day', p.played_at), s.id, s.title
      ORDER BY date DESC, play_count DESC
    `;

    return this.streamQuery(query, params, format, options.compress);
  }

  /**
   * Export song catalog for an artist
   */
  async exportSongCatalog(
    artistAddress: string,
    options: ExportOptions
  ): Promise<Readable> {
    const { format } = options;

    const query = `
      SELECT
        s.id,
        s.title,
        s.genre,
        s.duration,
        s.ipfs_hash,
        s.play_count,
        s.created_at,
        s.updated_at,
        s.metadata
      FROM songs s
      WHERE s.artist_address = $1
      AND s.deleted_at IS NULL
      ORDER BY s.created_at DESC
    `;

    return this.streamQuery(query, [artistAddress.toLowerCase()], format, options.compress);
  }

  /**
   * Export earnings report (if earnings tracking is implemented)
   */
  async exportEarningsReport(
    artistAddress: string,
    options: ExportOptions
  ): Promise<Readable> {
    const { format, dateFrom, dateTo } = options;

    // This would query an earnings table if implemented
    const query = `
      SELECT
        date_trunc('day', p.played_at) as date,
        s.id as song_id,
        s.title,
        COUNT(p.id) as plays,
        COUNT(p.id) * 0.001 as estimated_earnings_eth
      FROM plays p
      JOIN songs s ON p.song_id = s.id
      WHERE s.artist_address = $1
      ${dateFrom ? `AND p.played_at >= $2` : ''}
      ${dateTo ? `AND p.played_at <= $${dateFrom ? 3 : 2}` : ''}
      GROUP BY date_trunc('day', p.played_at), s.id, s.title
      ORDER BY date DESC
    `;

    const params: unknown[] = [artistAddress.toLowerCase()];
    if (dateFrom) params.push(dateFrom);
    if (dateTo) params.push(dateTo);

    return this.streamQuery(query, params, format, options.compress);
  }

  /**
   * Export global analytics (admin only)
   */
  async exportGlobalAnalytics(options: ExportOptions): Promise<Readable> {
    const { format, dateFrom, dateTo } = options;

    let query = `
      SELECT
        date_trunc('day', played_at) as date,
        COUNT(*) as total_plays,
        COUNT(DISTINCT wallet_address) as unique_listeners,
        COUNT(DISTINCT song_id) as unique_songs,
        SUM(duration) as total_duration_seconds
      FROM plays
      WHERE 1=1
    `;

    const params: unknown[] = [];

    if (dateFrom) {
      params.push(dateFrom);
      query += ` AND played_at >= $${params.length}`;
    }

    if (dateTo) {
      params.push(dateTo);
      query += ` AND played_at <= $${params.length}`;
    }

    query += `
      GROUP BY date_trunc('day', played_at)
      ORDER BY date DESC
    `;

    return this.streamQuery(query, params, format, options.compress);
  }

  /**
   * Stream query results with formatting
   */
  private async streamQuery(
    query: string,
    params: unknown[],
    format: ExportFormat,
    compress?: boolean
  ): Promise<Readable> {
    // Get a client for streaming
    const client = await this.pool.connect();

    try {
      // Use cursor for memory-efficient streaming
      const cursor = client.query(new (require('pg').Cursor)(query, params));

      // Create object-mode readable stream from cursor
      const sourceStream = new Readable({
        objectMode: true,
        read(size) {
          cursor.read(100, (err: Error | null, rows: any[]) => {
            if (err) {
              this.destroy(err);
              return;
            }

            if (rows.length === 0) {
              this.push(null);
              cursor.close(() => client.release());
              return;
            }

            for (const row of rows) {
              this.push(row);
            }
          });
        },
      });

      // Apply format transformer
      let transformer: Transform;
      switch (format) {
        case 'csv':
          transformer = new CSVTransformer();
          break;
        case 'json':
          transformer = new JSONArrayTransformer();
          break;
        case 'ndjson':
        default:
          transformer = new NDJSONTransformer();
      }

      let outputStream: Readable = sourceStream.pipe(transformer);

      // Apply compression if requested
      if (compress) {
        outputStream = outputStream.pipe(createGzip());
      }

      return outputStream;
    } catch (error) {
      client.release();
      throw error;
    }
  }

  /**
   * Get content type for format
   */
  static getContentType(format: ExportFormat, compressed: boolean): string {
    if (compressed) {
      return 'application/gzip';
    }

    switch (format) {
      case 'csv':
        return 'text/csv; charset=utf-8';
      case 'json':
        return 'application/json; charset=utf-8';
      case 'ndjson':
        return 'application/x-ndjson; charset=utf-8';
      default:
        return 'application/octet-stream';
    }
  }

  /**
   * Get file extension for format
   */
  static getExtension(format: ExportFormat, compressed: boolean): string {
    const ext = format === 'ndjson' ? 'ndjson' : format;
    return compressed ? `${ext}.gz` : ext;
  }

  /**
   * Create export for download
   */
  async createExportDownload(
    type: 'plays' | 'analytics' | 'catalog' | 'earnings',
    identifier: string,
    options: ExportOptions
  ): Promise<{
    stream: Readable;
    contentType: string;
    filename: string;
  }> {
    let stream: Readable;
    const timestamp = new Date().toISOString().split('T')[0];

    switch (type) {
      case 'plays':
        stream = await this.exportPlayHistory(identifier, options);
        break;
      case 'analytics':
        stream = await this.exportArtistAnalytics(identifier, options);
        break;
      case 'catalog':
        stream = await this.exportSongCatalog(identifier, options);
        break;
      case 'earnings':
        stream = await this.exportEarningsReport(identifier, options);
        break;
      default:
        throw new Error(`Unknown export type: ${type}`);
    }

    const ext = ExportService.getExtension(options.format, options.compress || false);
    const filename = `mycelix-${type}-${timestamp}.${ext}`;
    const contentType = ExportService.getContentType(options.format, options.compress || false);

    logger.info('Export created', { type, identifier, format: options.format });

    return { stream, contentType, filename };
  }
}

export default ExportService;
