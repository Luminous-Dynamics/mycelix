// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Email Processor
 *
 * Handles sending various types of emails.
 */

import { Job } from 'bullmq';
import nodemailer from 'nodemailer';
import { config } from '../config';
import { createLogger } from '../logger';

const logger = createLogger('email-processor');

// Email job types
export interface WelcomeEmailJob {
  type: 'welcome';
  to: string;
  name: string;
}

export interface PasswordResetJob {
  type: 'password-reset';
  to: string;
  resetToken: string;
  expiresAt: string;
}

export interface NotificationEmailJob {
  type: 'notification';
  to: string;
  subject: string;
  body: string;
}

export interface NewFollowerEmailJob {
  type: 'new-follower';
  to: string;
  artistName: string;
  followerName: string;
  followerAvatar?: string;
}

export interface RoyaltyPaymentEmailJob {
  type: 'royalty-payment';
  to: string;
  artistName: string;
  amount: string;
  currency: string;
  txHash: string;
}

export type EmailJob =
  | WelcomeEmailJob
  | PasswordResetJob
  | NotificationEmailJob
  | NewFollowerEmailJob
  | RoyaltyPaymentEmailJob;

// Create transporter
const transporter = nodemailer.createTransport({
  host: config.email.host,
  port: config.email.port,
  secure: config.email.port === 465,
  auth: config.email.user ? {
    user: config.email.user,
    pass: config.email.password,
  } : undefined,
});

export async function emailProcessor(job: Job<EmailJob>): Promise<void> {
  const { data } = job;

  logger.info({ type: data.type, to: data.to }, 'Processing email job');

  const email = buildEmail(data);

  try {
    await transporter.sendMail({
      from: config.email.from,
      ...email,
    });

    logger.info({ type: data.type, to: data.to }, 'Email sent successfully');
  } catch (error) {
    logger.error({ type: data.type, to: data.to, error }, 'Failed to send email');
    throw error; // Rethrow to trigger retry
  }
}

function buildEmail(data: EmailJob): { to: string; subject: string; html: string } {
  switch (data.type) {
    case 'welcome':
      return {
        to: data.to,
        subject: 'Welcome to Mycelix!',
        html: `
          <h1>Welcome to Mycelix, ${data.name}!</h1>
          <p>We're excited to have you join our decentralized music platform.</p>
          <p>Start exploring music, follow your favorite artists, and discover new sounds.</p>
          <a href="https://mycelix.io/explore" style="background: #8b5cf6; color: white; padding: 12px 24px; text-decoration: none; border-radius: 8px;">
            Start Exploring
          </a>
        `,
      };

    case 'password-reset':
      return {
        to: data.to,
        subject: 'Reset Your Password - Mycelix',
        html: `
          <h1>Password Reset Request</h1>
          <p>Click the link below to reset your password:</p>
          <a href="https://mycelix.io/reset-password?token=${data.resetToken}" style="background: #8b5cf6; color: white; padding: 12px 24px; text-decoration: none; border-radius: 8px;">
            Reset Password
          </a>
          <p>This link expires at ${new Date(data.expiresAt).toLocaleString()}.</p>
          <p>If you didn't request this, please ignore this email.</p>
        `,
      };

    case 'notification':
      return {
        to: data.to,
        subject: data.subject,
        html: `
          <div style="font-family: sans-serif;">
            ${data.body}
          </div>
        `,
      };

    case 'new-follower':
      return {
        to: data.to,
        subject: `${data.followerName} started following you on Mycelix`,
        html: `
          <h1>You have a new follower!</h1>
          <p><strong>${data.followerName}</strong> is now following ${data.artistName}.</p>
          <a href="https://mycelix.io/profile/${data.followerName}" style="background: #8b5cf6; color: white; padding: 12px 24px; text-decoration: none; border-radius: 8px;">
            View Profile
          </a>
        `,
      };

    case 'royalty-payment':
      return {
        to: data.to,
        subject: `Royalty Payment Received - ${data.amount} ${data.currency}`,
        html: `
          <h1>Royalty Payment Received</h1>
          <p>Great news, ${data.artistName}! You've received a royalty payment.</p>
          <div style="background: #1f2937; padding: 20px; border-radius: 8px; margin: 20px 0;">
            <p style="margin: 0; font-size: 24px; font-weight: bold;">${data.amount} ${data.currency}</p>
          </div>
          <p>Transaction: <a href="https://etherscan.io/tx/${data.txHash}">${data.txHash.slice(0, 20)}...</a></p>
          <a href="https://mycelix.io/dashboard/earnings" style="background: #8b5cf6; color: white; padding: 12px 24px; text-decoration: none; border-radius: 8px;">
            View Earnings Dashboard
          </a>
        `,
      };

    default:
      throw new Error(`Unknown email type: ${(data as any).type}`);
  }
}
