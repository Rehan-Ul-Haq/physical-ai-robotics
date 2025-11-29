# Better-Auth Integration Context
## Physical AI & Humanoid Robotics Platform

**Version**: 1.0
**Date**: 2025-11-29
**Architecture**: Hybrid Microservices (Node.js + FastAPI)

---

## 🎯 Project Overview

This document provides comprehensive context for integrating Better-Auth into the Physical AI & Humanoid Robotics educational platform. Use this as reference material when creating specifications, ADRs, or implementation plans.

---

## 📊 Current Platform Architecture

### **Frontend Stack**
- **Framework**: Docusaurus 3.9.2 (React 19.0.0)
- **Build Tool**: Node.js >=20.0
- **Styling**: Tailwind CSS 3.4.18
- **Location**: `book-source/`
- **Dev Server**: `http://localhost:3000`
- **Features**:
  - Interactive Python execution (Pyodide)
  - Quiz components
  - PDF viewer
  - Analytics dashboard
  - Lesson content rendering

### **Backend Stack**
- **Framework**: FastAPI 0.114.1 (Python 3.11+)
- **Database**: PostgreSQL (Neon Cloud)
  - Connection: asyncpg pool
  - ORM: None (raw SQL queries)
- **Vector Store**: Qdrant Cloud
- **AI/ML**: OpenAI API (ChatKit integration)
- **Location**: `backend/`
- **API Server**: `http://localhost:8001`
- **Features**:
  - Book assistant AI agent (RAG-powered)
  - Thread management (conversation persistence)
  - ChatKit SSE streaming
  - Health checks and metrics

### **Current CORS Configuration**
```python
allow_origins=[
    "http://localhost:3000",   # Docusaurus dev server
    "http://127.0.0.1:3000",
    "http://localhost:5173",   # Vite dev server
    "http://localhost:5174",
    "http://127.0.0.1:5173",
    "http://127.0.0.1:5174",
]
```

### **Current Database Schema**
```sql
-- Existing tables (managed by app/postgres_thread_store.py)
threads (
  id TEXT PRIMARY KEY,
  title TEXT,
  metadata JSONB,
  created_at TIMESTAMP,
  updated_at TIMESTAMP
)

thread_items (
  id TEXT PRIMARY KEY,
  thread_id TEXT REFERENCES threads(id),
  type TEXT,
  role TEXT,
  content TEXT,
  sources JSONB,
  metadata JSONB,
  created_at TIMESTAMP
)
```

---

## 🏗️ Proposed Architecture: Hybrid Microservices

### **Option 1: Separate Auth Service (RECOMMENDED)**

```
┌─────────────────────────────────────────────────────────────┐
│  FRONTEND: Docusaurus (React)                               │
│  ├─ Better-Auth React Client                                │
│  ├─ Session State Management (React hooks)                  │
│  └─ Protected Routes & Components                           │
│                                                              │
│  Port: 3000                                                  │
│  Tech: React 19, TypeScript, Tailwind                       │
└──────────────┬──────────────────────────────────────────────┘
               │
      ┌────────┴─────────┐
      │                  │
      ▼                  ▼
┌─────────────┐    ┌────────────────────────────┐
│ AUTH SERVICE│    │ BACKEND: FastAPI           │
│             │    │                            │
│ Node.js     │◄───┤ ├─ JWT Validation          │
│ Better-Auth │    │ ├─ User Context Middleware │
│ TypeScript  │    │ ├─ ChatKit API             │
│             │    │ ├─ RAG Service (Qdrant)    │
│ Port: 8002  │    │ └─ Health/Metrics          │
│             │    │                            │
│             │    │ Port: 8001                 │
└──────┬──────┘    │ Tech: Python 3.11, FastAPI │
       │           └────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────┐
│  DATABASE: PostgreSQL (Neon Cloud)              │
│  ├─ Better-Auth Tables (users, sessions, etc.)  │
│  └─ App Tables (threads, thread_items)          │
│                                                  │
│  Shared by both services via connection pooling │
└─────────────────────────────────────────────────┘
```

**Communication Flow:**
1. User authenticates via Better-Auth service (Node.js)
2. Better-Auth issues JWT token (stored in httpOnly cookie)
3. Frontend includes JWT in requests to FastAPI
4. FastAPI validates JWT and extracts user context
5. Both services share PostgreSQL database

---

## 🔐 Better-Auth Capabilities & Features

### **Core Authentication Methods**
```typescript
// Email & Password
emailAndPassword: {
  enabled: true,
  requireEmailVerification: boolean,
  sendResetPasswordEmail: Function,
  sendVerificationEmail: Function,
}

// Social Providers (OAuth 2.0)
socialProviders: {
  github: { clientId, clientSecret },
  google: { clientId, clientSecret },
  discord: { clientId, clientSecret },
  facebook: { clientId, clientSecret },
  microsoft: { clientId, clientSecret },
  // 20+ providers supported
}

// Magic Link (Passwordless)
magicLink: {
  enabled: true,
  sendMagicLink: Function,
}

// Passkey (WebAuthn)
passkey: {
  enabled: true,
}
```

### **Advanced Features (Plugins)**

#### **1. Two-Factor Authentication (2FA)**
```typescript
import { twoFactor } from "better-auth/plugins";

plugins: [
  twoFactor({
    // TOTP (Time-based OTP)
    // Backup codes
    // SMS OTP (with Twilio integration)
  })
]
```

#### **2. Organization Management (Multi-Tenancy)**
```typescript
import { organization } from "better-auth/plugins";

plugins: [
  organization({
    // Team/workspace management
    // Role-based access control (RBAC)
    // Member invitations
    // Organization metadata
    allowUserToCreateOrganization: true,
    organizationLimit: 10,

    roles: {
      owner: ["create", "read", "update", "delete", "invite", "remove"],
      admin: ["read", "update", "invite"],
      member: ["read"],
    },

    teams: {
      enabled: true,
      maximumTeams: 10,
    },
  })
]
```

#### **3. Access Control (Fine-Grained Permissions)**
```typescript
import { access } from "better-auth/plugins";

plugins: [
  access({
    permissions: {
      post: ["create", "read", "update", "delete"],
      comment: ["create", "read", "delete"],
      lesson: ["read", "complete"],
      quiz: ["attempt", "review"],
      thread: ["create", "read", "delete"],
    },

    roles: {
      admin: {
        post: ["create", "read", "update", "delete"],
        comment: ["create", "read", "delete"],
        lesson: ["read", "complete"],
        quiz: ["attempt", "review"],
        thread: ["create", "read", "delete"],
      },
      student: {
        lesson: ["read", "complete"],
        quiz: ["attempt", "review"],
        thread: ["create", "read"],
        comment: ["create", "read"],
      },
      guest: {
        lesson: ["read"],
      },
    },
  })
]
```

#### **4. Multi-Session Management**
```typescript
import { multiSession } from "better-auth/plugins";

plugins: [
  multiSession()
  // Allows multiple simultaneous sessions
  // Different accounts in same browser
  // Session switching
]
```

#### **5. Admin Plugin**
```typescript
import { admin } from "better-auth/plugins";

plugins: [
  admin({
    // User management endpoints
    // Impersonation
    // Session revocation
    // Role assignment
  })
]
```

### **Session Management**
```typescript
session: {
  expiresIn: 60 * 60 * 24 * 7, // 1 week
  updateAge: 60 * 60 * 24,      // Update session every 24h
  cookieCache: {
    enabled: true,
    maxAge: 5 * 60 * 1000,       // 5 minutes
  },
}
```

### **Security Features**
```typescript
security: {
  // CSRF Protection
  csrf: {
    enabled: true,
  },

  // Rate Limiting
  rateLimit: {
    window: 60,           // 1 minute
    max: 100,             // 100 requests
  },

  // Trusted Origins
  trustedOrigins: [
    "http://localhost:3000",
    "https://yourdomain.com",
  ],
}
```

---

## 💾 Database Adapters

### **PostgreSQL Adapter Options**

#### **Option A: Prisma (Recommended for TypeScript)**
```typescript
import { prismaAdapter } from "better-auth/adapters/prisma";
import { PrismaClient } from "@prisma/client";

const prisma = new PrismaClient();

export const auth = betterAuth({
  database: prismaAdapter(prisma, {
    provider: "postgresql",
  }),
});
```

**Schema Generation:**
```bash
npx @better-auth/cli generate
npx prisma migrate dev --name init
```

**Generated Tables:**
- `users` - User accounts
- `sessions` - Active sessions
- `accounts` - OAuth provider links
- `verifications` - Email verification tokens
- `organizations` - Organizations/teams (if plugin enabled)
- `members` - Organization memberships
- `invitations` - Pending invites

#### **Option B: Drizzle ORM**
```typescript
import { drizzleAdapter } from "better-auth/adapters/drizzle";
import { drizzle } from "drizzle-orm/postgres-js";

const db = drizzle(client, { schema });

export const auth = betterAuth({
  database: drizzleAdapter(db, {
    provider: "pg",
  }),
});
```

#### **Option C: Raw PostgreSQL (pg Pool)**
```typescript
import { Pool } from "pg";

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.DATABASE_URL,
  }),
});
```

---

## 🔗 Frontend Integration (Docusaurus/React)

### **Installation**
```bash
cd book-source
npm install better-auth
```

### **Auth Client Setup**
```typescript
// src/lib/authClient.ts
import { createAuthClient } from "better-auth/react";
import { organizationClient } from "better-auth/client/plugins";
import { adminClient } from "better-auth/client/plugins";

export const authClient = createAuthClient({
  baseURL: "http://localhost:8002", // Auth service URL

  plugins: [
    organizationClient(),
    adminClient(),
  ],
});

// Type inference for session
export type Session = typeof authClient.$Infer.Session;
```

### **React Hooks**
```typescript
// Get session
const { data: session, isPending } = authClient.useSession();

// Sign in
await authClient.signIn.email({
  email: "user@example.com",
  password: "password123",
});

// Sign in with social provider
await authClient.signIn.social({ provider: "github" });

// Sign out
await authClient.signOut();

// Organization management
const { data: orgs } = authClient.organization.list();
await authClient.organization.create({ name: "My Team" });
await authClient.organization.setActive({ organizationId });
```

### **Protected Route Component**
```typescript
// src/components/Auth/ProtectedRoute.tsx
import { authClient } from "@site/src/lib/authClient";
import { useEffect } from "react";
import { useHistory } from "@docusaurus/router";

export function ProtectedRoute({ children, requiredRole = null }) {
  const { data: session, isPending } = authClient.useSession();
  const history = useHistory();

  useEffect(() => {
    if (!isPending && !session) {
      history.push("/login");
    }
  }, [session, isPending]);

  if (isPending) return <div>Loading...</div>;
  if (!session) return null;

  return <>{children}</>;
}
```

### **Navbar Integration**
```typescript
// src/theme/Navbar/index.tsx (swizzled)
import { authClient } from "@site/src/lib/authClient";

function UserMenu() {
  const { data: session } = authClient.useSession();

  if (!session) {
    return <Link to="/login">Sign In</Link>;
  }

  return (
    <div className="dropdown">
      <img src={session.user.image} alt={session.user.name} />
      <span>{session.user.name}</span>
      <button onClick={() => authClient.signOut()}>Sign Out</button>
    </div>
  );
}
```

---

## 🐍 FastAPI Integration (Backend)

### **Installation**
```bash
cd backend
uv add pyjwt python-jose[cryptography] httpx
```

### **JWT Validation Middleware**
```python
# backend/app/auth_middleware.py
from fastapi import HTTPException, Security, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from jose import jwt, JWTError
from typing import Optional
import httpx

security = HTTPBearer(auto_error=False)

class AuthMiddleware:
    def __init__(self, auth_service_url: str, jwt_secret: str):
        self.auth_service_url = auth_service_url
        self.jwt_secret = jwt_secret

    async def verify_token(
        self,
        credentials: Optional[HTTPAuthorizationCredentials] = Security(security)
    ) -> dict:
        """Verify JWT token from Better-Auth"""
        if not credentials:
            raise HTTPException(status_code=401, detail="Not authenticated")

        token = credentials.credentials

        try:
            # Decode and verify JWT
            payload = jwt.decode(
                token,
                self.jwt_secret,
                algorithms=["HS256"]
            )

            # Extract user info
            return {
                "user_id": payload.get("sub"),
                "email": payload.get("email"),
                "name": payload.get("name"),
                "role": payload.get("role"),
                "session_id": payload.get("session_id"),
            }

        except JWTError as e:
            raise HTTPException(status_code=401, detail=f"Invalid token: {str(e)}")

    async def get_current_user(
        self,
        credentials: Optional[HTTPAuthorizationCredentials] = Security(security)
    ) -> Optional[dict]:
        """Get current user (optional authentication)"""
        if not credentials:
            return None

        try:
            return await self.verify_token(credentials)
        except:
            return None

# Global instance
auth = AuthMiddleware(
    auth_service_url=settings.auth_service_url,
    jwt_secret=settings.jwt_secret,
)
```

### **Protected Endpoint Example**
```python
# backend/app/main.py
from app.auth_middleware import auth

@app.post("/assistant/chatkit")
async def chatkit_endpoint(
    request: Request,
    user: dict = Depends(auth.verify_token)  # Required auth
) -> StreamingResponse:
    """Protected endpoint - requires authentication"""
    user_id = user["user_id"]
    user_email = user["email"]

    # Associate thread with user
    context = {
        "request": request,
        "user_id": user_id,
        "user_email": user_email,
    }

    # ... existing ChatKit logic
```

### **Optional Authentication Example**
```python
@app.get("/assistant/thread/{thread_id}")
async def get_thread(
    thread_id: str,
    user: Optional[dict] = Depends(auth.get_current_user)  # Optional auth
) -> ThreadResponse:
    """
    Optional authentication:
    - Authenticated: Return full thread if user owns it
    - Anonymous: Return public threads only
    """
    if user:
        # Verify user owns this thread
        thread = await get_user_thread(thread_id, user["user_id"])
    else:
        # Return only if thread is public
        thread = await get_public_thread(thread_id)

    return thread
```

### **User Context in Thread Storage**
```python
# backend/app/postgres_thread_store.py
async def create_thread(
    self,
    thread_id: str,
    user_id: Optional[str] = None,  # New field
    metadata: dict = None,
) -> Thread:
    """Create thread with user association"""
    await self.db.execute(
        """
        INSERT INTO threads (id, user_id, title, metadata, created_at, updated_at)
        VALUES ($1, $2, $3, $4, NOW(), NOW())
        """,
        thread_id,
        user_id,
        metadata.get("title", "Untitled"),
        json.dumps(metadata),
    )
```

---

## 🌐 Auth Service Implementation (Node.js)

### **Project Structure**
```
auth-service/
├── src/
│   ├── auth.ts              # Better-Auth config
│   ├── server.ts            # Express server
│   ├── middleware/
│   │   └── cors.ts          # CORS config
│   └── utils/
│       └── email.ts         # Email sending
├── prisma/
│   └── schema.prisma        # Database schema
├── .env                     # Environment variables
├── package.json
└── tsconfig.json
```

### **Auth Configuration**
```typescript
// src/auth.ts
import { betterAuth } from "better-auth";
import { prismaAdapter } from "better-auth/adapters/prisma";
import { organization } from "better-auth/plugins";
import { access } from "better-auth/plugins";
import { twoFactor } from "better-auth/plugins";
import { admin } from "better-auth/plugins";
import { PrismaClient } from "@prisma/client";
import { sendEmail } from "./utils/email";

const prisma = new PrismaClient();

export const auth = betterAuth({
  database: prismaAdapter(prisma, {
    provider: "postgresql",
  }),

  emailAndPassword: {
    enabled: true,
    requireEmailVerification: true,
    async sendVerificationEmail({ user, url }) {
      await sendEmail({
        to: user.email,
        subject: "Verify your email",
        html: `Click here to verify: <a href="${url}">${url}</a>`,
      });
    },
    async sendResetPasswordEmail({ user, url }) {
      await sendEmail({
        to: user.email,
        subject: "Reset your password",
        html: `Click here to reset: <a href="${url}">${url}</a>`,
      });
    },
  },

  socialProviders: {
    github: {
      clientId: process.env.GITHUB_CLIENT_ID!,
      clientSecret: process.env.GITHUB_CLIENT_SECRET!,
    },
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID!,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET!,
    },
  },

  session: {
    expiresIn: 60 * 60 * 24 * 7,    // 1 week
    updateAge: 60 * 60 * 24,        // 1 day
  },

  plugins: [
    // Organization management
    organization({
      async sendInvitationEmail({ email, organization, inviter, url }) {
        await sendEmail({
          to: email,
          subject: `Join ${organization.name}`,
          html: `${inviter.user.name} invited you to join ${organization.name}. <a href="${url}">Accept</a>`,
        });
      },
      allowUserToCreateOrganization: true,
      organizationLimit: 5,
      roles: {
        owner: ["create", "read", "update", "delete", "invite", "remove"],
        admin: ["read", "update", "invite"],
        student: ["read"],
      },
    }),

    // Fine-grained permissions
    access({
      permissions: {
        lesson: ["read", "complete"],
        quiz: ["attempt", "review"],
        thread: ["create", "read", "delete"],
        comment: ["create", "read", "delete"],
      },
      roles: {
        admin: {
          lesson: ["read", "complete"],
          quiz: ["attempt", "review"],
          thread: ["create", "read", "delete"],
          comment: ["create", "read", "delete"],
        },
        student: {
          lesson: ["read", "complete"],
          quiz: ["attempt", "review"],
          thread: ["create", "read"],
          comment: ["create", "read"],
        },
      },
    }),

    // 2FA
    twoFactor(),

    // Admin tools
    admin(),
  ],

  trustedOrigins: [
    "http://localhost:3000",
    "http://localhost:5173",
  ],
});
```

### **Express Server**
```typescript
// src/server.ts
import express from "express";
import cors from "cors";
import { toNodeHandler } from "better-auth/node";
import { auth } from "./auth";

const app = express();
const PORT = process.env.PORT || 8002;

// CORS
app.use(cors({
  origin: [
    "http://localhost:3000",
    "http://localhost:5173",
  ],
  credentials: true,
}));

// Better-Auth handler
app.all("/api/auth/*", toNodeHandler(auth));

// Health check
app.get("/health", (req, res) => {
  res.json({ status: "ok", service: "auth" });
});

app.listen(PORT, () => {
  console.log(`Auth service running on http://localhost:${PORT}`);
});
```

### **Environment Variables**
```bash
# auth-service/.env
DATABASE_URL=postgresql://user:pass@neon.tech:5432/db?sslmode=require

# Better-Auth
BETTER_AUTH_SECRET=<random-secret-key>
BETTER_AUTH_URL=http://localhost:8002

# GitHub OAuth
GITHUB_CLIENT_ID=your_github_client_id
GITHUB_CLIENT_SECRET=your_github_client_secret

# Google OAuth
GOOGLE_CLIENT_ID=your_google_client_id
GOOGLE_CLIENT_SECRET=your_google_client_secret

# Email (Resend/SendGrid/etc)
EMAIL_FROM=noreply@yourdomain.com
RESEND_API_KEY=your_resend_api_key
```

---

## 📋 Use Cases for Educational Platform

### **1. User Progress Tracking**
```typescript
// Link threads to users
const createThread = async (query: string) => {
  const { data: session } = await authClient.getSession();

  const response = await fetch("/assistant/chatkit", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
      "Authorization": `Bearer ${session.token}`,
    },
    body: JSON.stringify({
      query,
      user_id: session.user.id,
    }),
  });
};
```

### **2. Role-Based Content Access**
```typescript
// Protect premium lessons
const LessonContent = ({ lessonId }) => {
  const { data: session } = authClient.useSession();

  // Check if user has access
  const hasAccess = session?.user?.role === "premium" ||
                   lesson.isFree;

  if (!hasAccess) {
    return <UpgradePrompt />;
  }

  return <LessonRenderer lessonId={lessonId} />;
};
```

### **3. Organization-Based Learning**
```typescript
// University/bootcamp organizations
const OrganizationDashboard = () => {
  const { data: orgs } = authClient.organization.list();
  const activeOrg = orgs?.find(o => o.isActive);

  return (
    <div>
      <h2>{activeOrg.name} Progress</h2>
      <StudentList organizationId={activeOrg.id} />
      <CourseProgress organizationId={activeOrg.id} />
    </div>
  );
};
```

### **4. Teacher Dashboard**
```typescript
// View student progress
const TeacherDashboard = () => {
  const { data: session } = authClient.useSession();

  if (session.user.role !== "teacher") {
    return <Redirect to="/" />;
  }

  const { data: students } = useQuery(
    () => fetch("/api/students", {
      headers: { Authorization: `Bearer ${session.token}` }
    })
  );

  return <StudentProgressTable students={students} />;
};
```

---

## 🚀 Deployment Strategy

### **Development**
```bash
# Terminal 1: Docusaurus
cd book-source && npm run start      # Port 3000

# Terminal 2: Auth Service
cd auth-service && npm run dev       # Port 8002

# Terminal 3: FastAPI Backend
cd backend && uv run uvicorn app.main:app --reload  # Port 8001
```

### **Production Options**

#### **Option A: Vercel + Railway**
- **Docusaurus**: Vercel (static site)
- **Auth Service**: Railway (Node.js)
- **FastAPI Backend**: Railway (Python)
- **PostgreSQL**: Neon (serverless)
- **Qdrant**: Qdrant Cloud

#### **Option B: AWS**
- **Docusaurus**: S3 + CloudFront
- **Auth Service**: ECS Fargate (containerized)
- **FastAPI Backend**: ECS Fargate
- **PostgreSQL**: RDS
- **Qdrant**: Self-hosted on EC2

#### **Option C: Single VPS**
- **All services**: Docker Compose on DigitalOcean/Linode
- **Nginx**: Reverse proxy
- **PostgreSQL**: Local instance
- **Qdrant**: Docker container

---

## 🔒 Security Considerations

### **1. Token Security**
- Store JWT in httpOnly cookies (not localStorage)
- Set secure flag in production
- Use SameSite=Lax for CSRF protection

### **2. Rate Limiting**
```typescript
// Better-Auth built-in
rateLimit: {
  window: 60,      // 1 minute
  max: 100,        // 100 requests per window
}
```

### **3. CORS Configuration**
```typescript
// Auth service
trustedOrigins: [
  process.env.FRONTEND_URL,
  "http://localhost:3000",  // Dev only
]

// FastAPI
allow_origins=[
  os.getenv("FRONTEND_URL"),
  os.getenv("AUTH_SERVICE_URL"),
]
```

### **4. Environment Variables**
Never commit these to version control:
- `BETTER_AUTH_SECRET`
- `GITHUB_CLIENT_SECRET`
- `GOOGLE_CLIENT_SECRET`
- `DATABASE_URL`
- `JWT_SECRET`

---

## 📊 Database Migration Plan

### **Phase 1: Add Better-Auth Tables**
```sql
-- Run: npx @better-auth/cli generate
-- Auto-generated by Better-Auth

CREATE TABLE users (
  id TEXT PRIMARY KEY,
  email TEXT UNIQUE NOT NULL,
  email_verified BOOLEAN DEFAULT false,
  name TEXT,
  image TEXT,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW()
);

CREATE TABLE sessions (
  id TEXT PRIMARY KEY,
  user_id TEXT REFERENCES users(id) ON DELETE CASCADE,
  expires_at TIMESTAMP NOT NULL,
  token TEXT UNIQUE NOT NULL,
  ip_address TEXT,
  user_agent TEXT,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW()
);

CREATE TABLE accounts (
  id TEXT PRIMARY KEY,
  user_id TEXT REFERENCES users(id) ON DELETE CASCADE,
  account_id TEXT NOT NULL,
  provider_id TEXT NOT NULL,
  access_token TEXT,
  refresh_token TEXT,
  expires_at TIMESTAMP,
  scope TEXT,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW()
);

CREATE TABLE verifications (
  id TEXT PRIMARY KEY,
  identifier TEXT NOT NULL,
  value TEXT NOT NULL,
  expires_at TIMESTAMP NOT NULL,
  created_at TIMESTAMP DEFAULT NOW()
);
```

### **Phase 2: Link Existing Tables**
```sql
-- Add user_id to threads table
ALTER TABLE threads
ADD COLUMN user_id TEXT REFERENCES users(id) ON DELETE CASCADE;

-- Create index for faster lookups
CREATE INDEX idx_threads_user_id ON threads(user_id);
```

### **Phase 3: User Data Migration**
```sql
-- If you have existing users in a different table
INSERT INTO users (id, email, name, created_at)
SELECT id, email, name, created_at
FROM legacy_users;

-- Link existing threads to users
UPDATE threads t
SET user_id = lu.new_user_id
FROM legacy_user_threads lu
WHERE t.id = lu.thread_id;
```

---

## 🎨 UI Component Examples

### **Sign-In Modal**
```typescript
// src/components/Auth/SignInModal.tsx
import { authClient } from "@site/src/lib/authClient";
import { useState } from "react";

export function SignInModal({ onClose }) {
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");

  const handleEmailSignIn = async (e) => {
    e.preventDefault();
    try {
      await authClient.signIn.email({ email, password });
      onClose();
    } catch (err) {
      setError(err.message);
    }
  };

  const handleSocialSignIn = async (provider) => {
    try {
      await authClient.signIn.social({ provider });
    } catch (err) {
      setError(err.message);
    }
  };

  return (
    <div className="modal">
      <h2>Sign In</h2>

      {/* Email/Password Form */}
      <form onSubmit={handleEmailSignIn}>
        <input
          type="email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          placeholder="Email"
        />
        <input
          type="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          placeholder="Password"
        />
        <button type="submit">Sign In</button>
      </form>

      {/* Social Sign-In */}
      <div className="social-buttons">
        <button onClick={() => handleSocialSignIn("github")}>
          GitHub
        </button>
        <button onClick={() => handleSocialSignIn("google")}>
          Google
        </button>
      </div>

      {error && <p className="error">{error}</p>}
    </div>
  );
}
```

---

## 📈 Analytics & Monitoring

### **User Engagement Tracking**
```typescript
// Track authenticated user actions
const trackUserAction = async (action: string, metadata: object) => {
  const { data: session } = await authClient.getSession();

  await fetch("/api/analytics/track", {
    method: "POST",
    headers: {
      "Authorization": `Bearer ${session.token}`,
    },
    body: JSON.stringify({
      user_id: session.user.id,
      action,
      metadata,
      timestamp: new Date().toISOString(),
    }),
  });
};

// Usage
trackUserAction("lesson_completed", {
  lessonId: "chapter-1-lesson-3",
  duration: 1200, // seconds
});
```

---

## 🧪 Testing Strategy

### **Auth Service Tests**
```typescript
// auth-service/tests/auth.test.ts
import { auth } from "../src/auth";

describe("Better-Auth Integration", () => {
  it("should create user with email/password", async () => {
    const response = await auth.api.signUpEmail({
      email: "test@example.com",
      password: "SecurePass123!",
      name: "Test User",
    });

    expect(response.user).toBeDefined();
    expect(response.user.email).toBe("test@example.com");
  });

  it("should reject weak passwords", async () => {
    await expect(
      auth.api.signUpEmail({
        email: "test@example.com",
        password: "123",
        name: "Test User",
      })
    ).rejects.toThrow();
  });
});
```

### **FastAPI Middleware Tests**
```python
# backend/tests/test_auth_middleware.py
import pytest
from app.auth_middleware import auth

@pytest.mark.asyncio
async def test_valid_jwt_token():
    token = create_test_token(user_id="user123")
    result = await auth.verify_token(token)

    assert result["user_id"] == "user123"

@pytest.mark.asyncio
async def test_invalid_jwt_token():
    with pytest.raises(HTTPException):
        await auth.verify_token("invalid_token")
```

---

## 🎯 Success Criteria

### **Phase 1: Basic Auth (Week 1)**
- [ ] Auth service deployed and running
- [ ] Email/password authentication working
- [ ] Session management functional
- [ ] JWT validation in FastAPI working
- [ ] Sign-in/sign-out UI in Docusaurus

### **Phase 2: Social Auth (Week 2)**
- [ ] GitHub OAuth working
- [ ] Google OAuth working
- [ ] User profile page
- [ ] Session persistence across refreshes

### **Phase 3: Advanced Features (Week 3+)**
- [ ] Organization management
- [ ] Role-based access control
- [ ] 2FA implementation
- [ ] Admin dashboard
- [ ] User analytics

---

## 📚 Additional Resources

### **Better-Auth Documentation**
- Official Docs: https://www.better-auth.com/docs
- GitHub: https://github.com/better-auth/better-auth
- Examples: https://github.com/better-auth/better-auth/tree/main/examples

### **Database Adapters**
- Prisma: https://www.better-auth.com/docs/adapters/prisma
- Drizzle: https://www.better-auth.com/docs/adapters/drizzle

### **Plugins**
- Organization: https://www.better-auth.com/docs/plugins/organization
- Access Control: https://www.better-auth.com/docs/plugins/access
- 2FA: https://www.better-auth.com/docs/plugins/2fa
- Admin: https://www.better-auth.com/docs/plugins/admin

---

## 💡 Recommendation

**Start with Hybrid Microservices approach (Option 1)** because:

1. ✅ **Minimal disruption** to existing FastAPI backend
2. ✅ **Best-in-class auth** using Better-Auth's battle-tested features
3. ✅ **Type-safe** TypeScript auth service
4. ✅ **Scalable** - can deploy services independently
5. ✅ **Flexible** - easy to add social providers, 2FA, organizations
6. ✅ **Shared database** - no data duplication
7. ✅ **Production-ready** - Better-Auth handles security, sessions, tokens

---

## 🤝 Next Steps

When creating your specification, consider:

1. **User roles needed**: Student, Teacher, Admin, Organization Owner?
2. **Social providers**: Which OAuth providers to prioritize?
3. **Organization features**: Do you need multi-tenancy for schools/bootcamps?
4. **Premium content**: How to gate content for paid users?
5. **Progress tracking**: What user analytics to capture?
6. **Email provider**: Resend, SendGrid, AWS SES?
7. **Deployment target**: Vercel, Railway, AWS, self-hosted?

---

**Document Version**: 1.0
**Last Updated**: 2025-11-29
**Maintained By**: Development Team
