import crypto from 'crypto';
import { readConfig } from './routes/config.js';

const VALID_CONTROL_TOKENS = new Set<string>();

export function issueControlToken(password: string): string | null {
  try {
    const config = readConfig();
    const controlSection = (config as any).control || {};
    const expected = controlSection.password;

    if (!expected || typeof expected !== 'string') {
      console.warn('[control-auth] control.password is not set in config.toml');
      return null;
    }

    if (password !== expected) {
      return null;
    }

    const token = crypto.randomBytes(24).toString('hex');
    VALID_CONTROL_TOKENS.add(token);
    return token;
  } catch (error) {
    console.error('[control-auth] Failed to read control password from config:', error);
    return null;
  }
}

export function validateControlToken(token?: string): boolean {
  if (!token) return false;
  return VALID_CONTROL_TOKENS.has(token);
}

