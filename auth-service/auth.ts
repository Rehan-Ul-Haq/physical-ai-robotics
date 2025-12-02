/**
 * Better-Auth Server Configuration
 *
 * Implements T009-T016 from tasks.md:
 * - PostgreSQL adapter using Neon database
 * - Email/password authentication
 * - GitHub OAuth provider
 * - Google OAuth provider
 * - CORS configuration for localhost:3000
 * - Session cookie settings
 *
 * Based on research.md patterns.
 */
import { betterAuth } from "better-auth";
import { Pool } from "pg";
import nodemailer from "nodemailer";
import dotenv from "dotenv";

// Load environment variables
dotenv.config();

// Initialize Gmail SMTP transporter
const transporter = nodemailer.createTransport({
  service: "gmail",
  auth: {
    user: process.env.GMAIL_USER,
    pass: process.env.GMAIL_APP_PASSWORD, // Use App Password, not regular password
  },
});

const emailFrom = process.env.GMAIL_USER || "noreply@example.com";
const appName = "RoboAI";

// Validate required environment variables
const requiredEnvVars = [
  "DATABASE_URL",
  "BETTER_AUTH_SECRET",
];

for (const envVar of requiredEnvVars) {
  if (!process.env[envVar]) {
    throw new Error(`Missing required environment variable: ${envVar}`);
  }
}

// PostgreSQL connection pool (T010)
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: process.env.NODE_ENV === "production" ? { rejectUnauthorized: false } : undefined,
});

/**
 * Better-Auth instance configuration
 */
export const auth = betterAuth({
  // Database configuration (T010)
  database: pool,

  // Base URL for callbacks
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:8002",

  // Trusted origins for CORS and CSRF protection
  trustedOrigins: [
    "http://localhost:3000",
    "http://localhost:3001",
    "http://127.0.0.1:3000",
    process.env.FRONTEND_URL || "",
  ].filter(Boolean),

  // Secret for signing tokens and cookies
  secret: process.env.BETTER_AUTH_SECRET,

  // Email/Password authentication (T011)
  emailAndPassword: {
    enabled: true,
    // Require email verification before sign-in (T050)
    requireEmailVerification: true,
    // Password requirements
    minPasswordLength: 8,
    // Send verification email on sign-up
    sendVerificationOnSignUp: true,
    // Verification token expiration (24 hours) (T031)
    verificationTokenExpiresIn: 60 * 60 * 24, // 24 hours in seconds
    // Password reset token expiration (1 hour) (T103)
    resetPasswordTokenExpiresIn: 60 * 60, // 1 hour in seconds
    // Don't auto sign-in until email is verified
    autoSignIn: false,
  },

  // OAuth Social Providers (T012, T013)
  socialProviders: {
    // GitHub OAuth (T012)
    github: {
      clientId: process.env.GITHUB_CLIENT_ID || "",
      clientSecret: process.env.GITHUB_CLIENT_SECRET || "",
      // Callback URL: http://localhost:8002/api/auth/callback/github
    },
    // Google OAuth (T013)
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID || "",
      clientSecret: process.env.GOOGLE_CLIENT_SECRET || "",
      // Callback URL: http://localhost:8002/api/auth/callback/google
    },
  },

  // Session configuration (T114, T115, T119)
  session: {
    // Cookie cache to reduce database queries
    cookieCache: {
      enabled: true,
      maxAge: 5 * 60, // 5 minutes cache
    },
    // Session expiration (7 days for "Remember me", session cookie otherwise) (T115)
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    // Update session on each request if close to expiration
    updateAge: 60 * 60 * 24, // 1 day
    // Session expires after 7 days of inactivity (T119)
    freshAge: 0, // Session is always fresh if not expired
  },

  // Cookie configuration (T114)
  cookies: {
    // Session cookie settings
    sessionCookie: {
      name: "roboai_session",
      options: {
        httpOnly: true,
        secure: process.env.NODE_ENV === "production",
        sameSite: "lax" as const,
        path: "/",
      },
    },
  },

  // Account configuration
  account: {
    // Allow linking multiple OAuth accounts to same user
    accountLinking: {
      enabled: true,
      // Link accounts when email matches
      trustedProviders: ["github", "google"],
    },
  },

  // User configuration
  user: {
    // Additional fields to store
    additionalFields: {
      // User's display name
      name: {
        type: "string",
        required: false,
      },
    },
  },

  // Email sending configuration using Gmail SMTP
  emailVerification: {
    sendVerificationEmail: async ({ user, url, token }: { user: { email: string; name?: string }; url: string; token: string }) => {
      console.log(`[Email] Sending verification email to ${user.email}`);
      console.log(`[Email] Original verification URL: ${url}`);
      
      if (!process.env.GMAIL_USER || !process.env.GMAIL_APP_PASSWORD) {
        console.log(`[Email] Gmail not configured. Verification URL: ${url}`);
        return;
      }

      // The URL from Better-Auth points to the auth server
      // We'll keep it as-is since Better-Auth handles the verification
      // After verification, user will be redirected to the frontend
      const verificationUrl = url;

      try {
        await transporter.sendMail({
          from: `"${appName}" <${emailFrom}>`,
          to: user.email,
          subject: `Verify your ${appName} account`,
          html: `
            <div style="font-family: Arial, sans-serif; max-width: 600px; margin: 0 auto;">
              <h2>Welcome to ${appName}!</h2>
              <p>Hi ${user.name || "there"},</p>
              <p>Please verify your email address by clicking the button below:</p>
              <a href="${verificationUrl}" style="display: inline-block; background-color: #4F46E5; color: white; padding: 12px 24px; text-decoration: none; border-radius: 6px; margin: 16px 0;">
                Verify Email
              </a>
              <p>Or copy and paste this link in your browser:</p>
              <p style="color: #666; word-break: break-all;">${verificationUrl}</p>
              <p>This link will expire in 24 hours.</p>
              <hr style="border: none; border-top: 1px solid #eee; margin: 24px 0;">
              <p style="color: #999; font-size: 12px;">If you didn't create an account, you can ignore this email.</p>
            </div>
          `,
        });
        console.log(`[Email] Verification email sent to ${user.email}`);
      } catch (error) {
        console.error(`[Email] Failed to send verification email:`, error);
      }
    },
    sendResetPassword: async ({ user, url, token }: { user: { email: string; name?: string }; url: string; token: string }) => {
      console.log(`[Email] Sending password reset email to ${user.email}`);
      
      if (!process.env.GMAIL_USER || !process.env.GMAIL_APP_PASSWORD) {
        console.log(`[Email] Gmail not configured. Reset URL: ${url}`);
        return;
      }

      try {
        await transporter.sendMail({
          from: `"${appName}" <${emailFrom}>`,
          to: user.email,
          subject: `Reset your ${appName} password`,
          html: `
            <div style="font-family: Arial, sans-serif; max-width: 600px; margin: 0 auto;">
              <h2>Password Reset Request</h2>
              <p>Hi ${user.name || "there"},</p>
              <p>We received a request to reset your password. Click the button below to create a new password:</p>
              <a href="${url}" style="display: inline-block; background-color: #4F46E5; color: white; padding: 12px 24px; text-decoration: none; border-radius: 6px; margin: 16px 0;">
                Reset Password
              </a>
              <p>Or copy and paste this link in your browser:</p>
              <p style="color: #666; word-break: break-all;">${url}</p>
              <p>This link will expire in 1 hour.</p>
              <hr style="border: none; border-top: 1px solid #eee; margin: 24px 0;">
              <p style="color: #999; font-size: 12px;">If you didn't request a password reset, you can ignore this email. Your password will remain unchanged.</p>
            </div>
          `,
        });
        console.log(`[Email] Password reset email sent to ${user.email}`);
      } catch (error) {
        console.error(`[Email] Failed to send password reset email:`, error);
      }
    },
  },

  // Advanced options
  advanced: {
    // Generate secure session tokens
    generateId: () => crypto.randomUUID(),
    // Cross-origin support
    crossSubDomainCookies: {
      enabled: false,
    },
  },

  // Logging and error handling (T128)
  logger: {
    level: process.env.NODE_ENV === "production" ? "error" : "debug",
    // Log auth events
    log: (level, message, ...args) => {
      const timestamp = new Date().toISOString();
      const prefix = `[${timestamp}] [Auth] [${level.toUpperCase()}]`;
      
      if (level === "error") {
        console.error(prefix, message, ...args);
      } else if (level === "warn") {
        console.warn(prefix, message, ...args);
      } else if (level === "info") {
        console.info(prefix, message, ...args);
      } else {
        console.log(prefix, message, ...args);
      }
    },
  },

  // Hooks for OAuth and session events (T061-T063, T067-T068)
  onRequest: async (request: Request) => {
    // Log all auth requests in development
    if (process.env.NODE_ENV !== "production") {
      console.log(`[Auth] Request: ${request.method} ${request.url}`);
    }
    return request;
  },
});

export default auth;
