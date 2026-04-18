// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
import Imap from 'imap';
import { simpleParser, ParsedMail } from 'mailparser';
import nodemailer from 'nodemailer';
import { prisma } from '../utils/prisma';
import { decrypt, encrypt } from '../utils/encryption';
import { AppError } from '../middleware/errorHandler';

export class EmailService {
  private createImapConnection(accountId: string, credentials: any): Promise<Imap> {
    return new Promise(async (resolve, reject) => {
      const imap = new Imap({
        user: credentials.imapUser,
        password: decrypt(credentials.imapPassword),
        host: credentials.imapHost,
        port: credentials.imapPort,
        tls: credentials.imapSecure,
        tlsOptions: { rejectUnauthorized: false },
      });

      imap.once('ready', () => resolve(imap));
      imap.once('error', (err) => reject(err));
      imap.connect();
    });
  }

  async fetchEmails(userId: string, accountId: string, folderPath: string = 'INBOX', limit: number = 50) {
    const account = await prisma.emailAccount.findFirst({
      where: { id: accountId, userId },
    });

    if (!account) {
      throw new AppError('Email account not found', 404);
    }

    const imap = await this.createImapConnection(accountId, account);

    return new Promise<any[]>((resolve, reject) => {
      imap.openBox(folderPath, false, (err, box) => {
        if (err) {
          imap.end();
          return reject(err);
        }

        const totalMessages = box.messages.total;
        if (totalMessages === 0) {
          imap.end();
          return resolve([]);
        }

        const start = Math.max(1, totalMessages - limit + 1);
        const end = totalMessages;

        const fetch = imap.seq.fetch(`${start}:${end}`, {
          bodies: '',
          struct: true,
        });

        const emails: any[] = [];

        fetch.on('message', (msg, seqno) => {
          let buffer = '';

          msg.on('body', (stream) => {
            stream.on('data', (chunk) => {
              buffer += chunk.toString('utf8');
            });
          });

          msg.once('end', async () => {
            try {
              const parsed = await simpleParser(buffer);
              emails.push(this.parsedMailToEmail(parsed, accountId, userId));
            } catch (error) {
              console.error('Error parsing email:', error);
            }
          });
        });

        fetch.once('error', (err) => {
          imap.end();
          reject(err);
        });

        fetch.once('end', () => {
          imap.end();
          resolve(emails);
        });
      });
    });
  }

  private parsedMailToEmail(parsed: ParsedMail, accountId: string, userId: string) {
    return {
      messageId: parsed.messageId || `${Date.now()}@mycelix`,
      subject: parsed.subject || '(No Subject)',
      from: parsed.from?.value[0] || { name: '', address: '' },
      to: parsed.to?.value || [],
      cc: parsed.cc?.value || [],
      bcc: parsed.bcc?.value || [],
      replyTo: parsed.replyTo?.value || [],
      bodyText: parsed.text || '',
      bodyHtml: parsed.html || '',
      date: parsed.date || new Date(),
      size: parsed.text?.length || 0,
      attachments: (parsed.attachments || []).map((att) => ({
        filename: att.filename || 'unnamed',
        contentType: att.contentType,
        size: att.size,
        contentId: att.contentId,
      })),
    };
  }

  async sendEmail(
    userId: string,
    accountId: string,
    to: string[],
    subject: string,
    body: string,
    cc?: string[],
    bcc?: string[],
    attachments?: any[]
  ) {
    const account = await prisma.emailAccount.findFirst({
      where: { id: accountId, userId },
    });

    if (!account) {
      throw new AppError('Email account not found', 404);
    }

    const transporter = nodemailer.createTransport({
      host: account.smtpHost,
      port: account.smtpPort,
      secure: account.smtpSecure,
      auth: {
        user: account.smtpUser,
        pass: decrypt(account.smtpPassword),
      },
    });

    const mailOptions = {
      from: account.email,
      to: to.join(', '),
      cc: cc?.join(', '),
      bcc: bcc?.join(', '),
      subject,
      html: body,
      attachments,
    };

    const info = await transporter.sendMail(mailOptions);

    // Save to sent folder
    const sentFolder = await prisma.folder.findFirst({
      where: {
        emailAccountId: accountId,
        type: 'SENT',
      },
    });

    if (sentFolder) {
      await prisma.email.create({
        data: {
          userId,
          emailAccountId: accountId,
          folderId: sentFolder.id,
          messageId: info.messageId,
          subject,
          from: { name: '', address: account.email },
          to: to.map((email) => ({ name: '', address: email })),
          cc: cc ? cc.map((email) => ({ name: '', address: email })) : [],
          bodyHtml: body,
          bodyText: body.replace(/<[^>]*>/g, ''),
          date: new Date(),
          isRead: true,
        },
      });
    }

    return info;
  }

  async syncFolder(userId: string, accountId: string, folderPath: string) {
    const emails = await this.fetchEmails(userId, accountId, folderPath);

    const folder = await prisma.folder.findFirst({
      where: {
        emailAccountId: accountId,
        path: folderPath,
      },
    });

    if (!folder) {
      throw new AppError('Folder not found', 404);
    }

    // Store emails in database
    for (const emailData of emails) {
      await prisma.email.upsert({
        where: { messageId: emailData.messageId },
        update: emailData,
        create: {
          ...emailData,
          userId,
          emailAccountId: accountId,
          folderId: folder.id,
        },
      });
    }

    // Update folder counts
    await prisma.folder.update({
      where: { id: folder.id },
      data: {
        totalCount: emails.length,
        unreadCount: emails.filter((e) => !e.isRead).length,
      },
    });

    return { synced: emails.length };
  }
}

export const emailService = new EmailService();
