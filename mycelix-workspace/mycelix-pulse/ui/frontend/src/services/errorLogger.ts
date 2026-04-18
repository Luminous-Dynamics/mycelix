// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
import { config } from '@/config/env';

export interface ErrorContext {
  userId?: string;
  userEmail?: string;
  action?: string;
  component?: string;
  metadata?: Record<string, any>;
}

export interface LoggedError {
  message: string;
  stack?: string;
  timestamp: Date;
  context?: ErrorContext;
  severity: 'low' | 'medium' | 'high' | 'critical';
  type: 'error' | 'warning' | 'info';
}

class ErrorLogger {
  private errors: LoggedError[] = [];
  private maxStoredErrors = 100;

  /**
   * Log an error with context
   */
  logError(
    error: Error | string,
    context?: ErrorContext,
    severity: LoggedError['severity'] = 'medium'
  ): void {
    const loggedError: LoggedError = {
      message: typeof error === 'string' ? error : error.message,
      stack: typeof error === 'string' ? undefined : error.stack,
      timestamp: new Date(),
      context,
      severity,
      type: 'error',
    };

    this.storeError(loggedError);
    this.outputError(loggedError);

    // In production, you could send to an error tracking service
    if (config.isProduction) {
      this.sendToErrorTracker(loggedError);
    }
  }

  /**
   * Log a warning
   */
  logWarning(message: string, context?: ErrorContext): void {
    const loggedError: LoggedError = {
      message,
      timestamp: new Date(),
      context,
      severity: 'low',
      type: 'warning',
    };

    this.storeError(loggedError);

    if (config.enableDebug) {
      console.warn('⚠️ [Warning]', message, context);
    }
  }

  /**
   * Log informational message
   */
  logInfo(message: string, context?: ErrorContext): void {
    const loggedError: LoggedError = {
      message,
      timestamp: new Date(),
      context,
      severity: 'low',
      type: 'info',
    };

    this.storeError(loggedError);

    if (config.enableDebug) {
      console.info('ℹ️ [Info]', message, context);
    }
  }

  /**
   * Store error in memory (useful for debugging)
   */
  private storeError(error: LoggedError): void {
    this.errors.unshift(error);

    // Keep only the most recent errors
    if (this.errors.length > this.maxStoredErrors) {
      this.errors = this.errors.slice(0, this.maxStoredErrors);
    }
  }

  /**
   * Output error to console based on environment
   */
  private outputError(error: LoggedError): void {
    const prefix = this.getSeverityPrefix(error.severity);
    const output = {
      message: error.message,
      timestamp: error.timestamp.toISOString(),
      severity: error.severity,
      context: error.context,
      stack: error.stack,
    };

    if (config.isDevelopment || config.enableDebug) {
      console.error(`${prefix} [Error]`, output);
    }
  }

  /**
   * Send error to tracking service (Sentry, LogRocket, etc.)
   */
  private sendToErrorTracker(error: LoggedError): void {
    // Placeholder for integration with error tracking service
    // Example: Sentry.captureException(error)

    // For now, just log that we would send it
    if (config.enableDebug) {
      console.log('📤 Would send to error tracker:', error);
    }

    // You could implement an API call to your backend error logging endpoint
    // fetch('/api/errors', {
    //   method: 'POST',
    //   headers: { 'Content-Type': 'application/json' },
    //   body: JSON.stringify(error),
    // }).catch(console.error);
  }

  /**
   * Get all stored errors
   */
  getErrors(): LoggedError[] {
    return [...this.errors];
  }

  /**
   * Get errors filtered by severity
   */
  getErrorsBySeverity(severity: LoggedError['severity']): LoggedError[] {
    return this.errors.filter((error) => error.severity === severity);
  }

  /**
   * Get errors filtered by type
   */
  getErrorsByType(type: LoggedError['type']): LoggedError[] {
    return this.errors.filter((error) => error.type === type);
  }

  /**
   * Clear all stored errors
   */
  clearErrors(): void {
    this.errors = [];
  }

  /**
   * Get severity prefix for console output
   */
  private getSeverityPrefix(severity: LoggedError['severity']): string {
    switch (severity) {
      case 'low':
        return '🟢';
      case 'medium':
        return '🟡';
      case 'high':
        return '🟠';
      case 'critical':
        return '🔴';
      default:
        return '⚪';
    }
  }

  /**
   * Export errors as JSON for debugging
   */
  exportErrors(): string {
    return JSON.stringify(this.errors, null, 2);
  }
}

// Singleton instance
export const errorLogger = new ErrorLogger();

// Global error handler
if (typeof window !== 'undefined') {
  window.addEventListener('error', (event) => {
    errorLogger.logError(event.error || event.message, {
      action: 'global_error_handler',
      metadata: {
        filename: event.filename,
        lineno: event.lineno,
        colno: event.colno,
      },
    }, 'high');
  });

  window.addEventListener('unhandledrejection', (event) => {
    errorLogger.logError(
      event.reason instanceof Error ? event.reason : String(event.reason),
      {
        action: 'unhandled_promise_rejection',
      },
      'high'
    );
  });
}
