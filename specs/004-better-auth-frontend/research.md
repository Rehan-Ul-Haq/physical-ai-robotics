# Research: Better-Auth Frontend Integration Patterns

**Feature**: 004-better-auth-frontend
**Date**: 2025-11-29
**Purpose**: Document Better-Auth integration patterns for Docusaurus React frontend

## Executive Summary

This research document consolidates best practices for implementing Better-Auth authentication in a Docusaurus 3.9.2 frontend with React 19.0.0. All technical decisions are based on Better-Auth official documentation, Docusaurus integration patterns, and industry-standard React authentication practices.

---

## 1. Better-Auth React Client Setup

### Decision: Use `better-auth/react` Package with Base URL Configuration

**Rationale**:
- Docusaurus uses React 19.0.0, requiring React-specific client (`better-auth/react`)
- Auth service runs on separate port (8002), requires explicit `baseURL` configuration
- React client provides hooks (`useSession`, `useListAccounts`) optimized for React's rendering lifecycle

**Implementation Pattern**:

```typescript
// src/lib/auth-client.ts
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.REACT_APP_AUTH_SERVICE_URL || "http://localhost:8002",

  fetchOptions: {
    onError(error) {
      if (error.error.status === 429) {
        console.error("Rate limit exceeded");
      } else if (error.error.status === 401) {
        console.error("Unauthorized - session may have expired");
      }
    },
    onSuccess(data) {
      console.log("Auth action successful:", data);
    },
  },
});

export const {
  signIn,
  signUp,
  signOut,
  useSession,
  linkSocial,
  unlinkAccount,
  listAccounts,
  changePassword,
  forgetPassword,
  resetPassword,
  sendVerificationEmail,
} = authClient;
```

**Key Configuration Options**:
- `baseURL`: Points to auth service (port 8002 in dev, production domain in prod)
- `fetchOptions.onError`: Global error handling for rate limiting, unauthorized access
- `fetchOptions.onSuccess`: Optional success callbacks for analytics/logging
- Environment variable: `REACT_APP_AUTH_SERVICE_URL` (Docusaurus uses React's env var naming)

**Alternatives Considered**:
- **Vanilla client (`better-auth/client`)**: Rejected - requires manual state management, no React hooks
- **Framework-agnostic approach**: Rejected - loses type safety and React optimizations

---

## 2. Session State Management with `useSession` Hook

### Decision: Use `useSession` Hook with Loading States and Error Handling

**Rationale**:
- Built on nanostore (reactive state library) - changes propagate immediately across components
- Returns `{ data, isPending, error, refetch }` - follows React Query patterns (familiar to developers)
- Handles session restoration automatically on page refresh
- Provides refetch function for manual session updates after profile changes

**Implementation Pattern**:

```typescript
// Component example: Protected dashboard
import { useSession } from "@/lib/auth-client";
import { redirect } from "@docusaurus/router";

export function DashboardPage() {
  const { data: session, isPending, error, refetch } = useSession();

  // Loading state - prevent flash of unauthenticated content
  if (isPending) {
    return <div>Loading session...</div>;
  }

  // Error state - network failure or corrupted session
  if (error) {
    console.error("Session error:", error);
    return <div>Failed to load session. Please refresh.</div>;
  }

  // Unauthenticated state - redirect to sign-in
  if (!session) {
    redirect("/sign-in");
    return null;
  }

  // Authenticated state - render protected content
  return (
    <div>
      <h1>Welcome, {session.user.name}!</h1>
      <p>Email: {session.user.email}</p>
      <img src={session.user.image} alt="Profile" />

      <div>
        <h2>Session Info</h2>
        <p>Session ID: {session.session.id}</p>
        <p>Expires: {new Date(session.session.expiresAt).toLocaleString()}</p>
      </div>
    </div>
  );
}
```

**Session Data Structure**:
```typescript
{
  user: {
    id: string;
    email: string;
    name: string;
    image: string | null;
    emailVerified: boolean;
    createdAt: Date;
    updatedAt: Date;
  };
  session: {
    id: string;
    userId: string;
    expiresAt: Date;
    ipAddress: string | null;
    userAgent: string | null;
  };
}
```

**Best Practices**:
- Always check `isPending` before `!session` to avoid flash of unauthenticated content
- Use `error` state for network failures vs `!session` for unauthenticated users
- Call `refetch()` after profile updates to sync UI with backend state
- Extract `useSession` logic into custom hooks for reusability (e.g., `useRequireAuth()`)

**Alternatives Considered**:
- **Manual session API calls**: Rejected - requires manual caching, no reactivity
- **Context API for session**: Rejected - Better-Auth's nanostore already provides global state

---

## 3. Protected Routes and Authorization Guards

### Decision: Component-Level Auth Checks (Not Middleware)

**Rationale**:
- Docusaurus is static site generator - no server-side middleware during build
- Next.js-style middleware adds complexity and potential bugs during hydration
- Component-level checks align with Docusaurus architecture (client-side React components)
- Better-Auth documentation recommends page/route-level checks over middleware

**Implementation Pattern**:

**Approach 1: Higher-Order Component (Reusable)**

```typescript
// src/components/Auth/RequireAuth.tsx
import { useSession } from "@/lib/auth-client";
import { useHistory } from "@docusaurus/router";
import { useEffect, ReactNode } from "react";

interface RequireAuthProps {
  children: ReactNode;
  redirectTo?: string;
  fallback?: ReactNode;
}

export function RequireAuth({
  children,
  redirectTo = "/sign-in",
  fallback = <div>Loading...</div>
}: RequireAuthProps) {
  const { data: session, isPending, error } = useSession();
  const history = useHistory();

  useEffect(() => {
    if (!isPending && !session && !error) {
      history.push(redirectTo);
    }
  }, [isPending, session, error, history, redirectTo]);

  if (isPending) {
    return fallback;
  }

  if (!session || error) {
    return null; // Will redirect via useEffect
  }

  return <>{children}</>;
}
```

**Usage**:
```typescript
// src/pages/dashboard.tsx
import { RequireAuth } from "@/components/Auth/RequireAuth";

export default function Dashboard() {
  return (
    <RequireAuth>
      <div>
        <h1>Protected Dashboard Content</h1>
      </div>
    </RequireAuth>
  );
}
```

**Approach 2: Custom Hook (Inline Usage)**

```typescript
// src/hooks/useRequireAuth.ts
import { useSession } from "@/lib/auth-client";
import { useHistory } from "@docusaurus/router";
import { useEffect } from "react";

export function useRequireAuth(redirectTo: string = "/sign-in") {
  const { data: session, isPending, error } = useSession();
  const history = useHistory();

  useEffect(() => {
    if (!isPending && !session && !error) {
      history.push(redirectTo);
    }
  }, [isPending, session, error, history, redirectTo]);

  return { session, isPending, error };
}
```

**Usage**:
```typescript
// src/pages/profile.tsx
import { useRequireAuth } from "@/hooks/useRequireAuth";

export default function ProfilePage() {
  const { session, isPending } = useRequireAuth();

  if (isPending) return <div>Loading...</div>;
  if (!session) return null;

  return <div>Profile for {session.user.name}</div>;
}
```

**Best Practices**:
- Store intended URL in session storage before redirect (for post-login navigation)
- Use Docusaurus's `useHistory` hook, not React Router's `useNavigate`
- Provide loading fallback to prevent layout shift
- Handle session expiration gracefully (show modal, not hard redirect)

**Alternatives Considered**:
- **Docusaurus middleware plugin**: Rejected - no native middleware support in Docusaurus
- **Route-level configuration**: Rejected - Docusaurus doesn't support route guards like Next.js

---

## 4. OAuth Social Provider Integration (GitHub + Google)

### Decision: Use `signIn.social()` with Provider-Specific Callbacks

**Rationale**:
- Better-Auth handles OAuth flow automatically (authorization URL, token exchange, account creation)
- Supports GitHub and Google out-of-the-box (no custom OAuth implementation needed)
- Account linking happens automatically when email matches existing user
- Callback URLs configured on auth service (port 8002), frontend only initiates flow

**Implementation Pattern**:

**GitHub OAuth**:
```typescript
// src/components/Auth/SignInModal.tsx
import { authClient } from "@/lib/auth-client";

export function SignInModal() {
  const handleGitHubSignIn = async () => {
    try {
      await authClient.signIn.social({
        provider: "github",
        callbackURL: "/dashboard", // Redirect after successful auth
      });
    } catch (error) {
      console.error("GitHub sign-in failed:", error);
      // Show error message to user
    }
  };

  return (
    <button onClick={handleGitHubSignIn} className="btn-github">
      Continue with GitHub
    </button>
  );
}
```

**Google OAuth**:
```typescript
const handleGoogleSignIn = async () => {
  try {
    await authClient.signIn.social({
      provider: "google",
      callbackURL: "/dashboard",
    });
  } catch (error) {
    console.error("Google sign-in failed:", error);
  }
};
```

**Account Linking (Existing User)**:
```typescript
// src/pages/settings.tsx
import { authClient } from "@/lib/auth-client";

export function AccountSettingsPage() {
  const handleLinkGitHub = async () => {
    try {
      await authClient.linkSocial({
        provider: "github",
        callbackURL: "/settings/accounts",
      });
      // Success - accounts now linked
    } catch (error) {
      console.error("Failed to link GitHub account:", error);
    }
  };

  return (
    <div>
      <h2>Linked Accounts</h2>
      <button onClick={handleLinkGitHub}>Link GitHub Account</button>
    </div>
  );
}
```

**Unlinking Accounts**:
```typescript
const handleUnlinkGitHub = async (accountId: string) => {
  try {
    await authClient.unlinkAccount({
      accountId: accountId, // Get from listAccounts()
    });
    // Success - account unlinked
  } catch (error) {
    console.error("Failed to unlink account:", error);
  }
};
```

**Listing Linked Accounts**:
```typescript
import { useListAccounts } from "@/lib/auth-client";

export function LinkedAccountsSection() {
  const { data: accounts, isPending } = useListAccounts();

  if (isPending) return <div>Loading accounts...</div>;

  return (
    <ul>
      {accounts?.map(account => (
        <li key={account.id}>
          {account.provider}: {account.providerId}
          <button onClick={() => handleUnlinkGitHub(account.id)}>Unlink</button>
        </li>
      ))}
    </ul>
  );
}
```

**OAuth Configuration (Auth Service)**:
```typescript
// backend/src/auth.ts (separate service on port 8002)
import { betterAuth } from "better-auth";

export const auth = betterAuth({
  socialProviders: {
    github: {
      clientId: process.env.GITHUB_CLIENT_ID,
      clientSecret: process.env.GITHUB_CLIENT_SECRET,
      // Callback URL: http://localhost:8002/api/auth/callback/github
    },
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET,
      // Callback URL: http://localhost:8002/api/auth/callback/google
    },
  },
});
```

**Best Practices**:
- Register separate OAuth apps for dev (localhost:8002) and production
- Store OAuth credentials in environment variables (never commit to git)
- Handle OAuth errors gracefully (user denial, provider failure)
- Prevent duplicate account linking (check existing providers before linking)
- Show linked account status in settings UI

**Alternatives Considered**:
- **Manual OAuth implementation**: Rejected - Better-Auth handles complexity (PKCE, state validation, token refresh)
- **OAuth proxy plugin**: Rejected - adds latency, separate apps are industry standard

---

## 5. Email/Password Authentication Flow

### Decision: Use `signUp.email()` and `signIn.email()` with Verification

**Rationale**:
- Better-Auth enforces email verification before allowing sign-in (security best practice)
- Frontend handles form validation (email format, password strength), backend validates tokens
- Verification emails sent by auth service (port 8002), frontend only triggers send
- Password reset uses same token-based flow as email verification

**Implementation Pattern**:

**Sign-Up with Email Verification**:
```typescript
// src/components/Auth/SignUpForm.tsx
import { authClient } from "@/lib/auth-client";
import { useState } from "react";

export function SignUpForm() {
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [name, setName] = useState("");
  const [error, setError] = useState("");
  const [success, setSuccess] = useState(false);

  const handleSignUp = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");

    // Client-side validation
    if (password.length < 8) {
      setError("Password must be at least 8 characters");
      return;
    }

    try {
      await authClient.signUp.email(
        {
          email,
          password,
          name,
        },
        {
          onRequest: () => {
            // Show loading state
            console.log("Signing up...");
          },
          onSuccess: () => {
            setSuccess(true);
            // Show "Check your email" message
          },
          onError: (ctx) => {
            setError(ctx.error.message || "Sign-up failed");
          },
        }
      );
    } catch (err) {
      setError("An unexpected error occurred");
    }
  };

  if (success) {
    return (
      <div>
        <h2>Check your email</h2>
        <p>We sent a verification link to {email}</p>
        <p>Click the link to verify your account and sign in.</p>
      </div>
    );
  }

  return (
    <form onSubmit={handleSignUp}>
      <input
        type="text"
        placeholder="Name"
        value={name}
        onChange={(e) => setName(e.target.value)}
        required
      />
      <input
        type="email"
        placeholder="Email"
        value={email}
        onChange={(e) => setEmail(e.target.value)}
        required
      />
      <input
        type="password"
        placeholder="Password (8+ characters)"
        value={password}
        onChange={(e) => setPassword(e.target.value)}
        required
      />
      {error && <div className="error">{error}</div>}
      <button type="submit">Sign Up</button>
    </form>
  );
}
```

**Sign-In with Verification Check**:
```typescript
// src/components/Auth/SignInForm.tsx
export function SignInForm() {
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");

  const handleSignIn = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");

    try {
      await authClient.signIn.email(
        {
          email,
          password,
        },
        {
          onRequest: () => {
            console.log("Signing in...");
          },
          onSuccess: () => {
            // Redirect to dashboard
            window.location.href = "/dashboard";
          },
          onError: (ctx) => {
            if (ctx.error.status === 403) {
              setError("Please verify your email address before signing in.");
              // Show "Resend verification email" button
            } else {
              setError(ctx.error.message || "Invalid email or password");
            }
          },
        }
      );
    } catch (err) {
      setError("An unexpected error occurred");
    }
  };

  return (
    <form onSubmit={handleSignIn}>
      <input
        type="email"
        placeholder="Email"
        value={email}
        onChange={(e) => setEmail(e.target.value)}
        required
      />
      <input
        type="password"
        placeholder="Password"
        value={password}
        onChange={(e) => setPassword(e.target.value)}
        required
      />
      {error && <div className="error">{error}</div>}
      <button type="submit">Sign In</button>
      <a href="/forgot-password">Forgot password?</a>
    </form>
  );
}
```

**Resend Verification Email**:
```typescript
const handleResendVerification = async () => {
  try {
    await authClient.sendVerificationEmail({
      email: email,
      callbackURL: "/dashboard",
    });
    alert("Verification email sent! Check your inbox.");
  } catch (error) {
    console.error("Failed to send verification email:", error);
  }
};
```

**Password Reset Flow**:
```typescript
// src/pages/forgot-password.tsx
export function ForgotPasswordPage() {
  const [email, setEmail] = useState("");
  const [sent, setSent] = useState(false);

  const handleRequestReset = async (e: React.FormEvent) => {
    e.preventDefault();

    try {
      await authClient.forgetPassword({
        email: email,
        redirectTo: "/reset-password", // Page with token input
      });
      setSent(true);
    } catch (error) {
      console.error("Failed to send reset email:", error);
    }
  };

  if (sent) {
    return (
      <div>
        <h2>Check your email</h2>
        <p>If that email exists, you'll receive a password reset link.</p>
      </div>
    );
  }

  return (
    <form onSubmit={handleRequestReset}>
      <input
        type="email"
        placeholder="Enter your email"
        value={email}
        onChange={(e) => setEmail(e.target.value)}
        required
      />
      <button type="submit">Send Reset Link</button>
    </form>
  );
}
```

**Reset Password (Token Page)**:
```typescript
// src/pages/reset-password.tsx
export function ResetPasswordPage() {
  const [newPassword, setNewPassword] = useState("");
  const [token, setToken] = useState(""); // Extract from URL query params

  const handleResetPassword = async (e: React.FormEvent) => {
    e.preventDefault();

    try {
      await authClient.resetPassword({
        newPassword: newPassword,
        token: token, // From URL: ?token=xyz
      });
      alert("Password reset successful! You can now sign in.");
      window.location.href = "/sign-in";
    } catch (error) {
      console.error("Failed to reset password:", error);
      alert("Reset link expired or invalid. Please request a new one.");
    }
  };

  return (
    <form onSubmit={handleResetPassword}>
      <input
        type="password"
        placeholder="New password (8+ characters)"
        value={newPassword}
        onChange={(e) => setNewPassword(e.target.value)}
        required
      />
      <button type="submit">Reset Password</button>
    </form>
  );
}
```

**Best Practices**:
- Validate password strength client-side (8+ chars minimum, consider complexity rules)
- Don't reveal whether email exists in database (prevent enumeration attacks)
- Show generic error "Invalid email or password" for wrong credentials
- Expire verification tokens after 24 hours, reset tokens after 1 hour
- Revoke all sessions when password is reset (security best practice)

**Alternatives Considered**:
- **Magic link authentication**: Deferred to future enhancement (spec non-goal #6)
- **Passwordless email OTP**: Rejected - requires SMS service or email OTP plugin

---

## 6. Session Cookie Management and CORS Configuration

### Decision: Use httpOnly Cookies with CORS Credentials

**Rationale**:
- Better-Auth uses httpOnly cookies (secure, XSS-protected, not accessible via JavaScript)
- Docusaurus dev server (port 3000) and auth service (port 8002) are cross-origin
- CORS must allow credentials (`credentials: true`) for cookies to work across origins
- Production uses same-origin (reverse proxy) to avoid CORS complexity

**CORS Configuration (Auth Service)**:
```typescript
// backend/src/auth.ts
import { betterAuth } from "better-auth";

export const auth = betterAuth({
  cors: {
    origin: [
      "http://localhost:3000",        // Docusaurus dev server
      "http://127.0.0.1:3000",         // Alternative localhost
      "https://yourdomain.com",        // Production domain
    ],
    credentials: true,                 // Allow cookies to be sent
  },
  session: {
    cookieCache: {
      enabled: true,                   // Cache session in signed cookie
      maxAge: 5 * 60,                  // 5 minutes (reduces DB queries)
    },
  },
});
```

**Frontend Fetch Configuration**:
```typescript
// src/lib/auth-client.ts
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.REACT_APP_AUTH_SERVICE_URL || "http://localhost:8002",

  fetchOptions: {
    credentials: "include", // Send cookies with requests
  },
});
```

**Cookie Attributes**:
- `httpOnly`: true (JavaScript cannot access, prevents XSS attacks)
- `secure`: true in production (HTTPS only)
- `sameSite`: "lax" (protects against CSRF, allows OAuth redirects)
- `path`: "/api/auth" (scoped to auth endpoints)
- `maxAge`: 7 days with "Remember me", session cookie without

**Session Persistence Logic**:
```typescript
// Sign-in with "Remember me"
await authClient.signIn.email({
  email: email,
  password: password,
  rememberMe: true, // Sets 7-day cookie
});

// Sign-in without "Remember me" (session cookie)
await authClient.signIn.email({
  email: email,
  password: password,
  rememberMe: false, // Cookie expires when browser closes
});
```

**Best Practices**:
- Use environment variables for CORS origins (different for dev/staging/prod)
- Set `SameSite=None; Secure` for cross-site cookies in production
- Use reverse proxy in production to avoid CORS (auth service on same domain)
- Enable cookie cache to reduce database queries (Better-Auth feature)

**Alternatives Considered**:
- **LocalStorage for tokens**: Rejected - vulnerable to XSS attacks
- **SessionStorage for tokens**: Rejected - loses persistence across tabs
- **OAuth proxy plugin**: Rejected - adds latency, not needed for production

---

## 7. Docusaurus Navbar Integration

### Decision: Swizzle Navbar Component to Add Auth UI

**Rationale**:
- Docusaurus doesn't support modifying navbar via plugins (requires swizzling)
- Swizzling allows full control over navbar structure while preserving Docusaurus features
- Auth state (user avatar, dropdown menu) needs to be in navbar for all pages

**Swizzle Command**:
```bash
npm run swizzle @docusaurus/theme-classic Navbar -- --eject
```

**Swizzled Navbar Component**:
```typescript
// src/theme/Navbar/index.tsx (swizzled component)
import React from "react";
import Navbar from "@theme-original/Navbar";
import { useSession } from "@/lib/auth-client";
import { UserMenu } from "@/components/Auth/UserMenu";
import { SignInButton } from "@/components/Auth/SignInButton";

export default function NavbarWrapper(props) {
  const { data: session, isPending } = useSession();

  return (
    <>
      <Navbar {...props} />
      <div className="navbar__auth">
        {isPending ? (
          <div className="navbar__auth-loading">Loading...</div>
        ) : session ? (
          <UserMenu user={session.user} />
        ) : (
          <SignInButton />
        )}
      </div>
    </>
  );
}
```

**User Menu Component**:
```typescript
// src/components/Auth/UserMenu.tsx
import { useState } from "react";
import { authClient } from "@/lib/auth-client";

interface UserMenuProps {
  user: {
    name: string;
    email: string;
    image: string | null;
  };
}

export function UserMenu({ user }: UserMenuProps) {
  const [isOpen, setIsOpen] = useState(false);

  const handleSignOut = async () => {
    await authClient.signOut();
    window.location.href = "/";
  };

  return (
    <div className="user-menu">
      <button onClick={() => setIsOpen(!isOpen)} className="user-menu__trigger">
        <img
          src={user.image || "/default-avatar.png"}
          alt={user.name}
          className="user-menu__avatar"
        />
        <span>{user.name}</span>
      </button>

      {isOpen && (
        <div className="user-menu__dropdown">
          <a href="/profile" className="user-menu__item">Profile</a>
          <a href="/settings" className="user-menu__item">Settings</a>
          <button onClick={handleSignOut} className="user-menu__item">
            Sign Out
          </button>
        </div>
      )}
    </div>
  );
}
```

**Best Practices**:
- Document swizzled components in `SWIZZLED_COMPONENTS.md` (for future upgrades)
- Test navbar after Docusaurus version upgrades (swizzled components may break)
- Use Tailwind CSS classes for consistency with existing Docusaurus theme
- Show loading skeleton during `isPending` to avoid layout shift

**Alternatives Considered**:
- **Docusaurus plugin for navbar**: Rejected - no native plugin API for navbar modifications
- **CSS injection without swizzling**: Rejected - can't add React components, only styles

---

## 8. Environment Variables and Configuration

### Decision: Use Docusaurus Environment Variables with `.env` Files

**Rationale**:
- Docusaurus supports `.env` files with `REACT_APP_` prefix (Create React App convention)
- Environment-specific configs (dev/staging/prod) managed via separate `.env` files
- Auth service URL changes between environments (localhost:8002 vs production domain)

**Environment Files**:

**`.env.development`** (local development):
```bash
REACT_APP_AUTH_SERVICE_URL=http://localhost:8002
REACT_APP_ENV=development
```

**`.env.production`** (production build):
```bash
REACT_APP_AUTH_SERVICE_URL=https://api.yourdomain.com
REACT_APP_ENV=production
```

**Usage in Code**:
```typescript
// src/lib/auth-client.ts
import { createAuthClient } from "better-auth/react";

const authServiceURL = process.env.REACT_APP_AUTH_SERVICE_URL;

if (!authServiceURL) {
  throw new Error("REACT_APP_AUTH_SERVICE_URL environment variable is required");
}

export const authClient = createAuthClient({
  baseURL: authServiceURL,
});
```

**Docusaurus Config**:
```javascript
// docusaurus.config.js
module.exports = {
  customFields: {
    authServiceURL: process.env.REACT_APP_AUTH_SERVICE_URL,
  },
};
```

**Best Practices**:
- Never commit `.env` files with secrets to git (use `.env.example` as template)
- Validate required env vars on app startup (fail fast if missing)
- Use different OAuth app credentials for dev vs prod (separate GitHub/Google apps)
- Document all environment variables in README.md

**Alternatives Considered**:
- **Runtime configuration file**: Rejected - requires additional fetch, delays app startup
- **Hardcoded URLs**: Rejected - requires rebuilds for environment changes

---

## 9. Error Handling and User Feedback

### Decision: Use Toast Notifications for Auth Errors and Success Messages

**Rationale**:
- Authentication errors need immediate user feedback (form errors, network failures)
- Toast notifications are non-blocking and don't interrupt user flow
- Better-Auth provides error codes (401, 403, 429) for specific handling

**Error Handling Pattern**:
```typescript
import { toast } from "react-toastify"; // or your preferred toast library

// Global error handler in auth client
export const authClient = createAuthClient({
  baseURL: process.env.REACT_APP_AUTH_SERVICE_URL,

  fetchOptions: {
    onError(error) {
      switch (error.error.status) {
        case 401:
          toast.error("Your session has expired. Please sign in again.");
          break;
        case 403:
          toast.error("Please verify your email address before signing in.");
          break;
        case 429:
          toast.error("Too many requests. Please try again later.");
          break;
        default:
          toast.error(error.error.message || "An unexpected error occurred");
      }
    },
    onSuccess(data) {
      // Optional success toast
      if (data.message) {
        toast.success(data.message);
      }
    },
  },
});
```

**Component-Level Error Handling**:
```typescript
// src/components/Auth/SignInForm.tsx
const handleSignIn = async (e: React.FormEvent) => {
  e.preventDefault();
  setError("");

  try {
    await authClient.signIn.email({
      email,
      password,
    }, {
      onError: (ctx) => {
        // Component-specific error handling (overrides global)
        if (ctx.error.status === 403) {
          setError("Please verify your email before signing in.");
          setShowResendButton(true); // Show UI-specific element
        } else {
          setError(ctx.error.message || "Invalid email or password");
        }
      },
    });
  } catch (err) {
    setError("Network error. Please check your connection.");
  }
};
```

**Best Practices**:
- Use global error handler for generic errors (network, server issues)
- Use component-level handlers for context-specific errors (form validation, user feedback)
- Show retry buttons for transient errors (network failures)
- Log errors to console for debugging (production: send to error tracking service)

**Alternatives Considered**:
- **Alert dialogs**: Rejected - blocking, poor UX
- **Inline error messages only**: Rejected - users may miss feedback on async actions

---

## 10. Testing Strategy for Authentication Flows

### Decision: Use React Testing Library with Mock Auth Client

**Rationale**:
- Better-Auth client is side-effect heavy (network requests, cookies)
- Mocking auth client allows testing UI logic without backend dependency
- React Testing Library promotes testing user interactions, not implementation details

**Mock Auth Client**:
```typescript
// src/lib/__mocks__/auth-client.ts
export const mockSession = {
  user: {
    id: "test-user-id",
    email: "test@example.com",
    name: "Test User",
    image: null,
    emailVerified: true,
  },
  session: {
    id: "test-session-id",
    userId: "test-user-id",
    expiresAt: new Date(Date.now() + 7 * 24 * 60 * 60 * 1000),
  },
};

export const authClient = {
  useSession: jest.fn(() => ({
    data: mockSession,
    isPending: false,
    error: null,
    refetch: jest.fn(),
  })),
  signIn: {
    email: jest.fn(),
    social: jest.fn(),
  },
  signUp: {
    email: jest.fn(),
  },
  signOut: jest.fn(),
};
```

**Test Example**:
```typescript
// src/components/Auth/__tests__/UserMenu.test.tsx
import { render, screen, fireEvent } from "@testing-library/react";
import { UserMenu } from "../UserMenu";
import { authClient } from "@/lib/auth-client";

jest.mock("@/lib/auth-client");

describe("UserMenu", () => {
  it("displays user name and avatar", () => {
    render(<UserMenu user={{ name: "John Doe", email: "john@example.com", image: null }} />);

    expect(screen.getByText("John Doe")).toBeInTheDocument();
    expect(screen.getByAltText("John Doe")).toBeInTheDocument();
  });

  it("calls signOut when sign-out button clicked", async () => {
    render(<UserMenu user={{ name: "John Doe", email: "john@example.com", image: null }} />);

    fireEvent.click(screen.getByText("John Doe")); // Open dropdown
    fireEvent.click(screen.getByText("Sign Out"));

    expect(authClient.signOut).toHaveBeenCalled();
  });
});
```

**Best Practices**:
- Mock `useSession` hook for authenticated/unauthenticated states
- Test loading states (`isPending: true`)
- Test error states (`error: { message: "..." }`)
- Use `waitFor` for async state changes (redirects, toasts)

**Alternatives Considered**:
- **End-to-end tests only**: Rejected - slow, requires running auth service
- **Testing against real auth service**: Rejected - flaky, requires test database

---

## 11. Performance Optimization

### Decision: Enable Cookie Cache and Lazy Load Auth Components

**Rationale**:
- Cookie cache reduces database queries (session cached in signed cookie for 5 minutes)
- Lazy loading auth components reduces initial bundle size (modal dialogs not always needed)
- Session validation happens client-side from cookie cache (fast)

**Cookie Cache Configuration** (Auth Service):
```typescript
// backend/src/auth.ts
export const auth = betterAuth({
  session: {
    cookieCache: {
      enabled: true,
      maxAge: 5 * 60, // 5 minutes
    },
  },
});
```

**Lazy Loading Auth Components**:
```typescript
// src/components/Auth/index.tsx
import { lazy } from "react";

export const SignInModal = lazy(() => import("./SignInModal"));
export const SignUpModal = lazy(() => import("./SignUpModal"));
export const UserMenu = lazy(() => import("./UserMenu"));
```

**Usage with Suspense**:
```typescript
// src/pages/index.tsx
import { Suspense } from "react";
import { SignInModal } from "@/components/Auth";

export default function HomePage() {
  return (
    <Suspense fallback={<div>Loading...</div>}>
      <SignInModal isOpen={showSignIn} onClose={() => setShowSignIn(false)} />
    </Suspense>
  );
}
```

**Best Practices**:
- Lazy load modals, settings pages (not navbar components)
- Prefetch auth components on hover (improve perceived performance)
- Use Suspense boundaries to prevent layout shift
- Monitor bundle size (auth client should be <50KB gzipped)

**Alternatives Considered**:
- **Preloading all auth components**: Rejected - increases initial bundle size unnecessarily
- **No cookie cache**: Rejected - requires database query on every page load

---

## Summary of Key Decisions

| Decision Area | Chosen Approach | Primary Rationale |
|--------------|-----------------|-------------------|
| **React Client** | `better-auth/react` with `baseURL` config | React hooks, type safety, optimized for React 19 |
| **Session Management** | `useSession` hook with loading/error states | Nanostore reactivity, follows React Query patterns |
| **Protected Routes** | Component-level auth checks (HOC/hooks) | Aligns with Docusaurus architecture (static site) |
| **OAuth Integration** | `signIn.social()` for GitHub/Google | Better-Auth handles OAuth complexity automatically |
| **Email/Password** | `signUp.email()` with verification flow | Industry-standard security (verified emails only) |
| **Session Cookies** | httpOnly cookies with CORS credentials | XSS-protected, cross-origin supported |
| **Navbar Integration** | Swizzle Navbar component | Only way to modify Docusaurus navbar structure |
| **Environment Config** | `REACT_APP_` env vars with `.env` files | Standard Docusaurus/Create React App pattern |
| **Error Handling** | Toast notifications with global/component handlers | Non-blocking, immediate user feedback |
| **Testing Strategy** | Mock auth client with React Testing Library | Fast, reliable, tests user interactions |
| **Performance** | Cookie cache + lazy loading | Reduces DB queries and initial bundle size |

---

## Next Steps

1. **Phase 1: Data Model**: Define authentication entities (User, Session, Account)
2. **Phase 1: API Contracts**: Document Better-Auth API endpoints used by frontend
3. **Phase 1: Quickstart**: Write developer onboarding guide for auth integration
4. **Phase 2: Tasks**: Generate actionable task list from this research and specification

---

**Research Complete**: All technical unknowns resolved. Ready for Phase 1 design artifacts.
