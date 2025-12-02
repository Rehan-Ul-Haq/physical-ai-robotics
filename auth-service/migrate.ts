/**
 * Database Migration Script
 *
 * Implements T015 from tasks.md:
 * - Run Better-Auth migrations to create auth tables
 * - Creates: users, sessions, accounts, verification tables
 *
 * Run with: npm run migrate
 */
import { Pool } from "pg";
import dotenv from "dotenv";

// Load environment variables
dotenv.config();

if (!process.env.DATABASE_URL) {
  console.error("ERROR: DATABASE_URL environment variable is required");
  process.exit(1);
}

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: process.env.NODE_ENV === "production" ? { rejectUnauthorized: false } : undefined,
});

/**
 * Better-Auth required tables schema
 * Based on Better-Auth documentation
 */
const migrations = [
  // Users table
  `CREATE TABLE IF NOT EXISTS "user" (
    "id" TEXT PRIMARY KEY,
    "name" TEXT,
    "email" TEXT NOT NULL UNIQUE,
    "emailVerified" BOOLEAN NOT NULL DEFAULT FALSE,
    "image" TEXT,
    "createdAt" TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    "updatedAt" TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
  );`,

  // Sessions table
  `CREATE TABLE IF NOT EXISTS "session" (
    "id" TEXT PRIMARY KEY,
    "expiresAt" TIMESTAMP NOT NULL,
    "token" TEXT NOT NULL UNIQUE,
    "createdAt" TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    "updatedAt" TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    "ipAddress" TEXT,
    "userAgent" TEXT,
    "userId" TEXT NOT NULL REFERENCES "user"("id") ON DELETE CASCADE
  );`,

  // Accounts table (for OAuth providers)
  `CREATE TABLE IF NOT EXISTS "account" (
    "id" TEXT PRIMARY KEY,
    "accountId" TEXT NOT NULL,
    "providerId" TEXT NOT NULL,
    "userId" TEXT NOT NULL REFERENCES "user"("id") ON DELETE CASCADE,
    "accessToken" TEXT,
    "refreshToken" TEXT,
    "idToken" TEXT,
    "accessTokenExpiresAt" TIMESTAMP,
    "refreshTokenExpiresAt" TIMESTAMP,
    "scope" TEXT,
    "password" TEXT,
    "createdAt" TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    "updatedAt" TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
  );`,

  // Verification tokens table (for email verification, password reset)
  `CREATE TABLE IF NOT EXISTS "verification" (
    "id" TEXT PRIMARY KEY,
    "identifier" TEXT NOT NULL,
    "value" TEXT NOT NULL,
    "expiresAt" TIMESTAMP NOT NULL,
    "createdAt" TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    "updatedAt" TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
  );`,

  // Indexes for performance
  `CREATE INDEX IF NOT EXISTS "session_userId_idx" ON "session"("userId");`,
  `CREATE INDEX IF NOT EXISTS "session_token_idx" ON "session"("token");`,
  `CREATE INDEX IF NOT EXISTS "account_userId_idx" ON "account"("userId");`,
  `CREATE INDEX IF NOT EXISTS "account_providerId_accountId_idx" ON "account"("providerId", "accountId");`,
  `CREATE INDEX IF NOT EXISTS "verification_identifier_idx" ON "verification"("identifier");`,
];

async function runMigrations() {
  console.log("🚀 Starting Better-Auth database migrations...\n");

  const client = await pool.connect();

  try {
    for (const migration of migrations) {
      // Extract table/index name for logging
      const match = migration.match(/(?:TABLE|INDEX)\s+(?:IF NOT EXISTS\s+)?"?(\w+)"?/i);
      const name = match ? match[1] : "unknown";

      try {
        await client.query(migration);
        console.log(`✅ Created/verified: ${name}`);
      } catch (err: any) {
        // Ignore "already exists" errors
        if (err.code === "42P07" || err.code === "42710") {
          console.log(`⏭️  Already exists: ${name}`);
        } else {
          throw err;
        }
      }
    }

    console.log("\n✨ All migrations completed successfully!");
    console.log("\nTables created:");
    console.log("  - user (users data)");
    console.log("  - session (active sessions)");
    console.log("  - account (OAuth/password accounts)");
    console.log("  - verification (email tokens)");
  } catch (err) {
    console.error("\n❌ Migration failed:", err);
    process.exit(1);
  } finally {
    client.release();
    await pool.end();
  }
}

runMigrations();
