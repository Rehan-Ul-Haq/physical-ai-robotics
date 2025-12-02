# Feature Specification: Better-Auth Frontend Integration

**Feature Branch**: `004-better-auth-frontend`
**Created**: 2025-11-29
**Status**: Draft
**Input**: User description: "Write specs for implementing better-auth authentication in our frontend (Docusaurus). user context7 mcp server for to read better-auth docs available at https://www.better-auth.com/ . further context of our brainstorming and the existing project details is avaialble here @context\better-auth-integration-context.md . We also have to add SSO functionality provided by better-auth"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Anonymous User Sign-Up with Email (Priority: P1)

A new visitor to the Physical AI & Humanoid Robotics platform wants to create an account to track their learning progress and interact with the AI assistant. They should be able to register using their email and password, verify their email address, and immediately access their personalized dashboard.

**Why this priority**: This is the foundational authentication flow that enables user identity and all subsequent features. Without this, no personalized experience is possible.

**Independent Test**: Can be fully tested by visiting the sign-up page, entering credentials, receiving verification email, clicking verification link, and accessing the dashboard. Delivers immediate value by enabling progress tracking.

**Acceptance Scenarios**:

1. **Given** a visitor is on the homepage, **When** they click "Sign Up", **Then** they see a registration form with email, password, and name fields
2. **Given** a visitor enters valid email and password, **When** they submit the form, **Then** they receive a verification email and see a "Check your email" message
3. **Given** a user clicks the verification link in their email, **When** the link is valid, **Then** their email is verified and they are redirected to the dashboard
4. **Given** a user tries to sign in with unverified email, **When** they submit credentials, **Then** they see "Please verify your email" error message
5. **Given** a user enters an email that already exists, **When** they submit the form, **Then** they see "Email already registered" error message
6. **Given** a user enters a password shorter than 8 characters, **When** they submit the form, **Then** they see password strength requirements

---

### User Story 2 - Existing User Sign-In (Priority: P1)

A returning learner wants to access their account to continue their coursework, view their conversation history with the AI assistant, and track their progress. They should be able to sign in quickly using their email and password.

**Why this priority**: Returning users represent engaged learners. A smooth sign-in experience is critical for retention and session continuity.

**Independent Test**: Can be tested by registering a user, signing out, then signing back in with correct credentials. Success is accessing the dashboard with persisted data.

**Acceptance Scenarios**:

1. **Given** a verified user is on the sign-in page, **When** they enter correct email and password, **Then** they are redirected to the dashboard with their session active
2. **Given** a user enters incorrect password, **When** they submit the form, **Then** they see "Invalid email or password" error
3. **Given** a user enters non-existent email, **When** they submit the form, **Then** they see "Invalid email or password" error (no email enumeration)
4. **Given** a user has an active session, **When** they visit the sign-in page, **Then** they are automatically redirected to the dashboard
5. **Given** a user clicks "Remember me" checkbox, **When** they sign in, **Then** their session persists for 7 days instead of expiring on browser close

---

### User Story 3 - SSO Authentication with GitHub/Google (Priority: P1)

A user prefers to authenticate using their existing GitHub or Google account rather than creating a new password. They should be able to sign up or sign in with a single click, with their profile information automatically populated.

**Why this priority**: SSO reduces friction for new users, improves security (no password to manage), and is expected by modern web users. This is the primary differentiator of Better-Auth.

**Independent Test**: Can be tested by clicking "Continue with GitHub", completing OAuth flow, and landing on dashboard with GitHub profile data (name, avatar) populated.

**Acceptance Scenarios**:

1. **Given** a new user clicks "Continue with GitHub", **When** they authorize the app, **Then** they are redirected to the dashboard with their GitHub name and avatar displayed
2. **Given** a new user clicks "Continue with Google", **When** they authorize the app, **Then** they are redirected to the dashboard with their Google profile data
3. **Given** an existing email user clicks "Continue with GitHub" using the same email, **When** they authorize, **Then** their accounts are linked and they see both authentication methods in settings
4. **Given** a user signed up with GitHub, **When** they return and click "Continue with GitHub", **Then** they sign in without additional prompts
5. **Given** GitHub OAuth fails (user denies access), **When** they are redirected back, **Then** they see "Authentication cancelled" message with option to try again

---

### User Story 4 - Protected Content Access (Priority: P2)

A visitor browses public lesson content but wants to use interactive features like the AI assistant, quizzes, or progress tracking. They should be prompted to sign up or sign in when attempting to access protected features, then seamlessly continue their intended action after authentication.

**Why this priority**: Balancing public content discovery with authenticated features drives conversions while maintaining accessibility. This creates clear value proposition for registration.

**Independent Test**: Can be tested by clicking "Ask Assistant" as anonymous user, completing sign-in, and immediately opening chat widget with original context preserved.

**Acceptance Scenarios**:

1. **Given** an anonymous user views a lesson, **When** they click "Ask AI Assistant", **Then** they see a modal prompting sign-in with "Continue reading after signing in" message
2. **Given** a user signs in from the protected content prompt, **When** authentication succeeds, **Then** they are redirected back to the lesson with the AI assistant chat opened
3. **Given** an anonymous user starts a quiz, **When** they submit the first answer, **Then** they are prompted to sign in to save progress
4. **Given** an authenticated user accesses any protected feature, **When** they interact with it, **Then** their actions are saved to their user profile
5. **Given** a user's session expires while using a protected feature, **When** they attempt an action, **Then** they see a "Session expired" modal with one-click re-authentication

---

### User Story 5 - User Profile and Account Management (Priority: P2)

An authenticated user wants to manage their account settings, view their authentication methods, update their profile information, and sign out from all devices. They should have full control over their account security and preferences.

**Why this priority**: User autonomy and security management are essential for trust and compliance. Users need transparency about their data and authentication methods.

**Independent Test**: Can be tested by accessing profile settings, changing name/avatar, linking a second OAuth provider, and verifying changes persist across sessions.

**Acceptance Scenarios**:

1. **Given** an authenticated user clicks their avatar, **When** the dropdown opens, **Then** they see options for Profile, Settings, and Sign Out
2. **Given** a user is on the profile page, **When** they update their display name, **Then** the change is immediately reflected in the navbar
3. **Given** a user signed up with email, **When** they visit Account Settings and click "Link GitHub", **Then** they complete OAuth and see both methods listed
4. **Given** a user has multiple authentication methods, **When** they click "Unlink" on a secondary method, **Then** the account remains accessible via the primary method
5. **Given** a user clicks "Sign out from all devices", **When** they confirm, **Then** all active sessions are revoked and they are signed out

---

### User Story 6 - Password Reset Flow (Priority: P2)

A user forgot their password and needs to regain access to their account. They should be able to request a password reset via email, receive a secure reset link, and create a new password.

**Why this priority**: Account recovery is essential for user retention. Without this, users who forget passwords are permanently locked out.

**Independent Test**: Can be tested by clicking "Forgot password", entering email, receiving reset email, clicking link, setting new password, and signing in with new credentials.

**Acceptance Scenarios**:

1. **Given** a user clicks "Forgot password" on sign-in page, **When** they enter their email, **Then** they receive a password reset email with a secure token
2. **Given** a user clicks the reset link, **When** it's valid (not expired), **Then** they see a form to enter new password
3. **Given** a user enters a new password, **When** they submit the form, **Then** their password is updated and they are redirected to sign in
4. **Given** a user tries to use an expired reset link (>1 hour old), **When** they access it, **Then** they see "Link expired, request a new one" message
5. **Given** a user successfully resets their password, **When** they sign in with the new password, **Then** all other active sessions are terminated (security)

---

### User Story 7 - Session Persistence and State Management (Priority: P3)

An authenticated user navigates between lessons, refreshes the page, or closes and reopens their browser. Their authentication state should persist appropriately based on their "Remember me" choice, with seamless session restoration.

**Why this priority**: Persistent sessions improve user experience by reducing authentication friction for returning users, while respecting security preferences.

**Independent Test**: Can be tested by signing in with "Remember me", closing browser, reopening, and verifying user is still authenticated without re-entering credentials.

**Acceptance Scenarios**:

1. **Given** a user signs in without "Remember me", **When** they close the browser, **Then** their session expires (session cookie)
2. **Given** a user signs in with "Remember me", **When** they close and reopen the browser within 7 days, **Then** they remain authenticated
3. **Given** an authenticated user refreshes the page, **When** the page loads, **Then** their session state is restored without flash of unauthenticated content
4. **Given** a user has been inactive for 7 days, **When** they return, **Then** their session expires and they see "Session expired, please sign in again"
5. **Given** a user signs out explicitly, **When** they click "Sign out", **Then** their session cookie is cleared and they are redirected to the homepage

---

### Edge Cases

- **What happens when** a user's email verification token expires (>24 hours)?
  - User can request a new verification email from the sign-in page prompt

- **What happens when** OAuth provider returns an email that's already registered with email/password?
  - Accounts are automatically linked if the email matches, requiring the user to sign in with their password first to confirm identity

- **What happens when** a user has both GitHub and Google OAuth linked and tries to link the same Google account again?
  - System prevents duplicate provider linking and shows "This account is already linked" message

- **What happens when** the authentication service (port 8002) is unavailable?
  - Frontend shows "Authentication service unavailable" message with retry option, public content remains accessible

- **What happens when** a user's session cookie is manually deleted or corrupted?
  - User is treated as anonymous, redirected to sign-in when accessing protected content, no error displayed

- **What happens when** a user signs in on Device A, then signs in on Device B with "Sign out from all devices"?
  - Device A's session is immediately revoked, and they see "You've been signed out" message on next interaction

- **What happens when** a user clicks a verification or password reset link that's already been used?
  - System shows "This link has already been used" with option to request a new one

- **What happens when** network request fails during sign-in/sign-up?
  - User sees "Network error, please check your connection" message with retry button, form data is preserved

## Requirements *(mandatory)*

### Functional Requirements

**Authentication & Registration**

- **FR-001**: System MUST allow users to sign up using email and password with name field
- **FR-002**: System MUST validate email format and password strength (minimum 8 characters) before submission
- **FR-003**: System MUST send verification email upon successful registration
- **FR-004**: System MUST prevent sign-in for users with unverified email addresses
- **FR-005**: System MUST verify email addresses via unique, time-limited tokens (24-hour expiration)
- **FR-006**: System MUST allow users to request new verification emails
- **FR-007**: System MUST allow existing users to sign in using email and password
- **FR-008**: System MUST display appropriate error messages for invalid credentials without revealing whether email exists (prevent enumeration)

**SSO Integration**

- **FR-009**: System MUST support OAuth 2.0 authentication with GitHub
- **FR-010**: System MUST support OAuth 2.0 authentication with Google
- **FR-011**: System MUST automatically populate user profile (name, avatar) from OAuth provider data
- **FR-012**: System MUST handle OAuth callback redirects and error states (user denial, provider failure)
- **FR-013**: System MUST link OAuth accounts to existing email accounts when emails match
- **FR-014**: System MUST allow users to link multiple authentication methods to a single account
- **FR-015**: System MUST prevent duplicate provider linking (same GitHub account linked twice)

**Session Management**

- **FR-016**: System MUST create secure, httpOnly session cookies upon successful authentication
- **FR-017**: System MUST support "Remember me" functionality with 7-day session duration
- **FR-018**: System MUST expire sessions without "Remember me" when browser closes (session cookies)
- **FR-019**: System MUST restore user session state across page refreshes
- **FR-020**: System MUST provide session revocation ("Sign out from all devices")
- **FR-021**: System MUST automatically sign out users after 7 days of inactivity
- **FR-022**: System MUST clear session cookies upon explicit sign-out

**Password Management**

- **FR-023**: System MUST allow users to request password reset via email
- **FR-024**: System MUST send password reset emails with secure, time-limited tokens (1-hour expiration)
- **FR-025**: System MUST allow users to set new passwords via reset links
- **FR-026**: System MUST invalidate reset tokens after single use
- **FR-027**: System MUST revoke all other sessions when password is successfully reset

**Protected Content**

- **FR-028**: System MUST protect AI assistant chat feature for authenticated users only
- **FR-029**: System MUST protect quiz submission and progress tracking for authenticated users only
- **FR-030**: System MUST redirect unauthenticated users to sign-in modal when accessing protected features
- **FR-031**: System MUST preserve user's intended action and redirect back after authentication
- **FR-032**: System MUST allow anonymous users to view public lesson content
- **FR-033**: System MUST display session expiration modal when attempting actions with expired session

**User Profile & Account Management**

- **FR-034**: System MUST display user avatar and name in navigation bar when authenticated
- **FR-035**: System MUST provide account settings page showing all linked authentication methods
- **FR-036**: System MUST allow users to update their display name and profile picture
- **FR-037**: System MUST allow users to unlink secondary authentication methods (minimum one method must remain)
- **FR-038**: System MUST provide user dropdown menu with Profile, Settings, and Sign Out options

**Better-Auth Client Integration**

- **FR-039**: System MUST initialize Better-Auth React client with base URL pointing to auth service (port 8002)
- **FR-040**: System MUST use `createAuthClient` from `better-auth/react` package
- **FR-041**: System MUST expose `useSession` hook for accessing authentication state in components
- **FR-042**: System MUST handle authentication errors globally with user-friendly messages
- **FR-043**: System MUST configure CORS to allow credentials from Docusaurus dev server (localhost:3000)

**UI Components**

- **FR-044**: System MUST provide sign-in modal component with email/password form and SSO buttons
- **FR-045**: System MUST provide sign-up modal component with email, password, name fields and SSO buttons
- **FR-046**: System MUST provide password reset request form
- **FR-047**: System MUST provide password reset confirmation form (accessible via email link)
- **FR-048**: System MUST provide protected route wrapper component that shows sign-in modal for unauthenticated access
- **FR-049**: System MUST display loading states during authentication operations
- **FR-050**: System MUST display success/error feedback for all authentication actions

### Key Entities

- **User**: Represents an authenticated platform user with email, name, avatar, email verification status, and timestamps
- **Session**: Represents an active user session with session ID, user ID, expiration time, IP address, user agent, and "Remember me" flag
- **Account**: Represents an OAuth provider link with provider name (github/google), provider user ID, user ID, and access token metadata
- **Verification Token**: Represents email verification token with email, token value, expiration timestamp, and used status
- **Reset Token**: Represents password reset token with email, token value, expiration timestamp, and used status

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete sign-up flow (email entry to email verification) in under 3 minutes
- **SC-002**: Users can sign in with email/password in under 10 seconds from landing on sign-in page
- **SC-003**: Users can authenticate with GitHub or Google OAuth in under 15 seconds (including redirect)
- **SC-004**: 95% of authentication requests complete successfully without errors
- **SC-005**: Session state restoration occurs in under 500ms on page refresh (no flash of unauthenticated UI)
- **SC-006**: Protected content prompts appear within 200ms of unauthorized access attempt
- **SC-007**: Password reset flow completion rate exceeds 80% (users who request reset successfully complete it)
- **SC-008**: Zero unauthorized access to protected features (AI assistant, quizzes, progress tracking)
- **SC-009**: Authentication state persists correctly across browser sessions when "Remember me" is enabled
- **SC-010**: Error messages display within 300ms of failed authentication attempts
- **SC-011**: SSO account linking success rate exceeds 90% for users with matching emails
- **SC-012**: Session revocation ("Sign out from all devices") takes effect within 5 seconds across all active sessions

### User Experience Metrics

- **SC-013**: New user sign-up completion rate (email entry to first dashboard access) exceeds 70%
- **SC-014**: Returning user sign-in success rate on first attempt exceeds 85%
- **SC-015**: SSO authentication (GitHub/Google) accounts for at least 40% of new registrations
- **SC-016**: Password reset request-to-completion rate exceeds 60%
- **SC-017**: Users successfully navigate back to intended content after authentication interruption 95% of the time

## Assumptions *(optional)*

### Technical Assumptions

1. **Authentication Service**: A separate Node.js service running Better-Auth on port 8002 will be implemented (covered in separate specification for backend integration)
2. **Database Schema**: PostgreSQL database with Better-Auth schema (users, sessions, accounts, verifications tables) is available via the auth service
3. **Email Service**: An email delivery service (Resend, SendGrid, or AWS SES) is configured in the auth service for verification and reset emails
4. **CORS Configuration**: Auth service at port 8002 will whitelist Docusaurus dev server (localhost:3000) and production domains
5. **Session Storage**: Session cookies will be httpOnly and secure (HTTPS in production) for security
6. **OAuth Apps**: GitHub and Google OAuth applications are registered with correct redirect URIs pointing to auth service

### User Experience Assumptions

7. **Public Content**: Lesson content pages remain publicly accessible without authentication (only interactive features require sign-in)
8. **Default Session Duration**: Sessions without "Remember me" expire on browser close; "Remember me" extends to 7 days
9. **Token Expiration**: Email verification tokens expire after 24 hours; password reset tokens expire after 1 hour (industry standard)
10. **Account Linking**: When OAuth email matches existing email account, linking requires password confirmation to prevent account takeover
11. **Navigation Persistence**: After authentication, users are redirected to their originally intended destination (deep linking preserved)
12. **Profile Pictures**: OAuth providers (GitHub, Google) automatically populate user avatars; email users get default avatar or initials

### Security Assumptions

13. **Password Requirements**: Minimum 8 characters (no maximum); additional complexity requirements enforced by Better-Auth defaults
14. **Rate Limiting**: Auth service implements rate limiting for sign-in attempts (handled by Better-Auth, not frontend)
15. **CSRF Protection**: Better-Auth provides built-in CSRF protection for authentication endpoints
16. **Token Cryptography**: Better-Auth handles secure token generation and validation for email verification and password reset
17. **Session Revocation**: Signing out from one device does NOT affect other devices unless "Sign out from all devices" is explicitly clicked

### Integration Assumptions

18. **Docusaurus Version**: Using Docusaurus 3.9.2 with React 19.0.0 (as confirmed in context document)
19. **React Client Package**: `better-auth/react` package provides hooks and utilities optimized for React 19
20. **Navbar Swizzling**: Docusaurus navbar will be swizzled to integrate user authentication UI components
21. **Protected Routes**: Docusaurus page components will use custom wrapper component for authentication checks
22. **FastAPI Integration**: Backend at port 8001 will validate JWT tokens from auth service (handled in separate specification)

### Deployment Assumptions

23. **Development Environment**: Auth service (port 8002), Docusaurus (port 3000), and FastAPI (port 8001) all run on localhost during development
24. **Production Domain**: Single production domain serves frontend, with subdomains or paths for auth and API services
25. **Environment Variables**: Frontend build process injects `VITE_AUTH_SERVICE_URL` or similar for auth service base URL
26. **Build Process**: Authentication client is bundled with Docusaurus build; no runtime configuration needed in production

## Non-Goals *(optional)*

### Out of Scope for This Feature

1. **Backend Auth Service Implementation**: This specification covers only the frontend Docusaurus integration. The Node.js Better-Auth service on port 8002 is a separate feature with its own specification.

2. **FastAPI JWT Validation**: Integration between the auth service and the FastAPI backend (port 8001) for validating user sessions in API requests is handled separately.

3. **Role-Based Access Control (RBAC)**: User roles (student, teacher, admin) and permission systems are future enhancements. This feature provides authentication only.

4. **Organization/Multi-Tenancy**: Better-Auth's organization plugin for teams/schools/bootcamps is not included in MVP.

5. **Two-Factor Authentication (2FA)**: 2FA via TOTP or SMS is a future security enhancement, not part of initial launch.

6. **Magic Link Authentication**: Passwordless email authentication via magic links is deferred to future iteration.

7. **Additional OAuth Providers**: Only GitHub and Google SSO are included. Microsoft, Discord, Facebook, Apple, etc., are future additions.

8. **User Profile Pages**: Public user profiles, bio, social links, achievements are not part of this authentication feature.

9. **Account Deletion**: Self-service account deletion workflow is a future compliance feature (GDPR).

10. **Email Change Flow**: Changing registered email address with re-verification is deferred (users must contact support).

11. **Analytics Integration**: Tracking authentication events (sign-ups, logins, OAuth usage) in analytics platform is separate concern.

12. **Admin Dashboard**: Administrative UI for managing users, viewing sessions, revoking access is separate feature.

13. **Internationalization (i18n)**: Authentication UI will initially be English-only; multi-language support is future enhancement.

14. **Mobile-Specific UI**: While responsive, specialized mobile app authentication flows (deep linking, biometrics) are out of scope.

15. **Session Activity Logs**: Displaying to users a list of their active sessions with device/location info is future feature.

16. **Progressive Enhancement**: Assumes JavaScript is enabled; no-JS fallback authentication is not provided.

17. **Third-Party Widget Integration**: ChatKit widget authentication is configured separately (widget receives auth token from parent window).

## Dependencies *(optional)*

### External Dependencies

**npm Packages (Docusaurus Frontend)**

1. **better-auth** (latest): Core Better-Auth library providing React client
   - Used by: Auth client initialization, useSession hook, sign-in/sign-up methods
   - Installation: `npm install better-auth`

**OAuth Provider Configuration**

2. **GitHub OAuth Application**: Registered GitHub OAuth app with client ID and secret
   - Required for: GitHub SSO functionality (FR-009)
   - Configuration: Client ID and secret stored in auth service environment variables
   - Callback URL: `http://localhost:8002/api/auth/callback/github` (dev), `https://yourdomain.com/api/auth/callback/github` (prod)

3. **Google Cloud OAuth Application**: Registered Google OAuth 2.0 credentials
   - Required for: Google SSO functionality (FR-010)
   - Configuration: Client ID and secret stored in auth service environment variables
   - Callback URL: `http://localhost:8002/api/auth/callback/google` (dev), `https://yourdomain.com/api/auth/callback/google` (prod)

**Authentication Service**

4. **Better-Auth Service (Port 8002)**: Node.js service running Better-Auth server
   - Required for: All authentication operations, session management, token validation
   - Provides: REST API at `/api/auth/*` endpoints
   - Status: Separate feature specification (dependency)

**Email Delivery**

5. **Email Service Provider**: One of Resend, SendGrid, or AWS SES
   - Required for: Email verification (FR-003), password reset emails (FR-024)
   - Configured in: Auth service environment variables
   - Status: Handled by auth service implementation

### Internal Dependencies

**Docusaurus Infrastructure**

6. **Docusaurus 3.9.2**: Existing platform running on port 3000
   - Used by: All UI components, routing, navbar integration
   - Current state: Already deployed

7. **React 19.0.0**: React version bundled with Docusaurus
   - Used by: Better-Auth React hooks, authentication UI components
   - Current state: Already available in Docusaurus build

8. **Tailwind CSS 3.4.18**: Styling framework for authentication UI
   - Used by: Sign-in/sign-up modals, forms, buttons, error messages
   - Current state: Already configured in Docusaurus

**Database & Backend**

9. **PostgreSQL (Neon Cloud)**: Shared database for auth tables and app tables
   - Required for: User accounts, sessions, OAuth provider links
   - Tables used: `users`, `sessions`, `accounts`, `verifications`
   - Status: Auth service manages these tables via Better-Auth migrations

10. **FastAPI Backend (Port 8001)**: Existing Python backend
    - Required for: Associating AI assistant threads with authenticated users
    - Integration: FastAPI validates JWT tokens from auth service (separate spec)
    - Status: JWT validation middleware will be added in separate feature

### Configuration Dependencies

11. **Environment Variables (Docusaurus)**:
    - `VITE_AUTH_SERVICE_URL`: Base URL of auth service (default: `http://localhost:8002` for dev)
    - Required for: Initializing Better-Auth client with correct baseURL

12. **Environment Variables (Auth Service)**:
    - `GITHUB_CLIENT_ID`, `GITHUB_CLIENT_SECRET`
    - `GOOGLE_CLIENT_ID`, `GOOGLE_CLIENT_SECRET`
    - `BETTER_AUTH_SECRET`: Signing key for sessions and tokens
    - `DATABASE_URL`: PostgreSQL connection string
    - Status: Configured in auth service deployment

13. **CORS Configuration (Auth Service)**:
    - Whitelist origins: `http://localhost:3000`, `http://127.0.0.1:3000` (dev), production domain (prod)
    - Allow credentials: `true`
    - Required for: Cookie-based session management across origins

### Development Dependencies

14. **Docusaurus Swizzling**: Ability to override default Docusaurus components
    - Required for: Customizing navbar to show user authentication UI
    - Components to swizzle: `Navbar/index.tsx`, potentially `Footer/index.tsx`

15. **TypeScript Types**: Better-Auth provides TypeScript types for session, user, and client methods
    - Used by: All authentication-related components for type safety
    - Available from: `better-auth/react` package

## Technical Constraints *(optional)*

### Docusaurus Framework Constraints

1. **Server-Side Rendering (SSR) Limitations**: Docusaurus is primarily a static site generator with client-side hydration
   - **Impact**: Authentication state cannot be checked during build time; all auth checks happen client-side after page load
   - **Mitigation**: Use loading states and avoid flash of unauthenticated content (FOUC) with skeleton loaders

2. **React Router Integration**: Docusaurus uses custom router based on React Router
   - **Impact**: Navigation guards and redirects must use Docusaurus's `useHistory` and `<Redirect>` components, not standard React Router
   - **Mitigation**: Create wrapper components that integrate with Docusaurus navigation patterns

3. **Swizzling Limitations**: Overriding Docusaurus components requires swizzling, which may break on Docusaurus upgrades
   - **Impact**: Navbar customization for auth UI could need updates when upgrading Docusaurus
   - **Mitigation**: Document swizzled components and test thoroughly after Docusaurus version bumps

4. **Client-Only Components**: Better-Auth hooks (`useSession`) only work in client components
   - **Impact**: Cannot use auth hooks in Docusaurus markdown files or during SSG build
   - **Mitigation**: Wrap protected content in React components that run client-side only

### Better-Auth Integration Constraints

5. **Cookie-Based Sessions**: Better-Auth uses httpOnly cookies for session management
   - **Impact**: Requires same-origin or properly configured CORS between Docusaurus (3000) and auth service (8002) in dev
   - **Mitigation**: Configure CORS to allow credentials and use proxy in production to avoid cross-origin issues

6. **Base URL Configuration**: Better-Auth client requires absolute base URL to auth service
   - **Impact**: Cannot use relative URLs; must configure different base URLs for dev/prod environments
   - **Mitigation**: Use environment variables (`VITE_AUTH_SERVICE_URL`) injected at build time

7. **React Version Compatibility**: Better-Auth React client is optimized for React 18+
   - **Impact**: Using React 19 (bleeding edge) may encounter undocumented behaviors
   - **Mitigation**: Test all auth flows thoroughly; Better-Auth maintainers claim React 19 compatibility

8. **Token Storage**: Verification and reset tokens are managed entirely by auth service
   - **Impact**: Frontend cannot directly validate or decode tokens; must rely on auth service API
   - **Mitigation**: All token validation happens via API calls to auth service endpoints

### OAuth Provider Constraints

9. **GitHub OAuth Callback**: GitHub requires pre-registered redirect URIs; wildcards not supported
   - **Impact**: Must register separate OAuth apps for localhost (dev) and production domain
   - **Mitigation**: Use two GitHub OAuth apps or Better-Auth's OAuth proxy plugin for dev environments

10. **Google OAuth Callback**: Similar redirect URI restrictions as GitHub
    - **Impact**: Separate Google OAuth apps needed for dev and prod
    - **Mitigation**: Same as GitHub - use separate apps or OAuth proxy

11. **OAuth Provider Rate Limits**: GitHub and Google impose rate limits on OAuth endpoints
    - **Impact**: Excessive authentication attempts in dev/testing could trigger temporary blocks
    - **Mitigation**: Use test accounts judiciously; implement exponential backoff on OAuth errors

### Security Constraints

12. **httpOnly Cookie Limitation**: Session cookies cannot be accessed by JavaScript
    - **Impact**: Cannot manually read session token for debugging or custom integrations
    - **Mitigation**: Rely on Better-Auth's `useSession` hook; use browser DevTools to inspect cookies

13. **SameSite Cookie Policy**: Modern browsers enforce SameSite=Lax by default
    - **Impact**: Cross-origin session cookies may be blocked unless explicitly configured
    - **Mitigation**: Auth service must set SameSite=Lax (or None with Secure in prod)

14. **HTTPS Requirement in Production**: Secure cookies require HTTPS
    - **Impact**: Authentication will not work over HTTP in production
    - **Mitigation**: Ensure production deployment uses HTTPS for both frontend and auth service

### Network & Performance Constraints

15. **Cross-Origin Latency**: Requests from Docusaurus (3000) to auth service (8002) add network round-trip
    - **Impact**: Session validation adds ~50-100ms latency per request
    - **Mitigation**: Better-Auth's cookie cache feature reduces database queries; sessions cached in signed cookie

16. **Session Cookie Size**: Session data stored in cookie has ~4KB browser limit
    - **Impact**: Cannot store large objects (user preferences, progress data) in session cookie
    - **Mitigation**: Session cookie contains only user ID, session ID, expiration; fetch additional data from backend API

17. **Browser LocalStorage Unavailable**: Better-Auth uses cookies, not localStorage
    - **Impact**: Cannot use localStorage for session persistence debugging
    - **Mitigation**: Use browser DevTools Application tab to inspect cookies

### Environment Constraints

18. **Development Environment**: Must run both Docusaurus (3000) and auth service (8002) concurrently
    - **Impact**: Requires two terminal sessions or process manager (concurrently, pm2)
    - **Mitigation**: Document dev setup clearly; provide `npm run dev:all` script

19. **Build-Time Environment Variables**: Docusaurus injects env vars at build time, not runtime
    - **Impact**: Cannot change auth service URL after build without rebuilding
    - **Mitigation**: Use different builds for dev/staging/prod or implement runtime config

20. **No Server-Side Auth Validation**: Docusaurus static build cannot validate sessions server-side
    - **Impact**: Protected content is hidden via client-side checks, but HTML is still sent to browser
    - **Mitigation**: Sensitive data must be fetched client-side after auth check, never embedded in static HTML

## Open Questions *(optional)*

[NEEDS CLARIFICATION: Email Service Provider Selection]

**Question**: Which email service provider should be used for verification and password reset emails?

| Option | Pros | Cons |
|--------|------|------|
| **Resend** | Modern API, generous free tier (100 emails/day), excellent deliverability, simple setup | Newer service (less mature), limited to transactional emails |
| **SendGrid** | Mature provider, 100 emails/day free tier, battle-tested deliverability, extensive docs | More complex API, requires domain verification for production |
| **AWS SES** | Lowest cost at scale ($0.10/1000 emails), high reliability, integrates with AWS ecosystem | Requires AWS account, more complex setup, starts in sandbox mode |
| **Custom SMTP** | No external dependency, full control | Deliverability issues (likely spam folder), requires mail server management |

**Recommendation**: Default to **Resend** for simplicity and modern DX unless AWS infrastructure already exists (then use SES).

---

[NEEDS CLARIFICATION: OAuth App Registration Strategy]

**Question**: Should we use separate OAuth apps for development and production, or use Better-Auth's OAuth proxy plugin?

| Option | Pros | Cons |
|--------|------|------|
| **Separate Apps** | Simple, standard approach, clear separation of environments | Requires managing 4 OAuth apps (GitHub dev/prod, Google dev/prod), callback URL changes across environments |
| **OAuth Proxy Plugin** | Single production OAuth app for all environments, simplified dev setup | Adds dependency on Better-Auth plugin, proxy adds latency, less common approach |
| **Shared OAuth App** | Only 2 OAuth apps (GitHub, Google), simplest setup | Security risk (dev and prod share credentials), requires wildcard callback URLs (not supported by all providers) |

**Recommendation**: Default to **Separate Apps** for security and simplicity unless development team strongly prefers OAuth proxy for dev convenience.

---

[NEEDS CLARIFICATION: Default Authentication Method Preference]

**Question**: On the sign-in/sign-up modal, should we emphasize SSO (GitHub/Google) over email/password, or present them equally?

| Option | Pros | Cons |
|--------|------|------|
| **SSO Primary (large buttons, top position)** | Reduces friction, better security (no passwords to manage), faster sign-ups, modern UX | May confuse users without GitHub/Google accounts, slight bias toward OAuth providers |
| **Equal Prominence (SSO and email/password same size)** | Neutral approach, accommodates all user preferences | Doesn't guide users toward better security option (SSO) |
| **Email/Password Primary** | Familiar for traditional users, clear privacy (no third-party involvement) | Slower sign-up flow, password management burden, less modern UX |

**Recommendation**: Default to **SSO Primary** - position GitHub/Google buttons above email form with "Or continue with email" separator, aligning with modern authentication UX patterns.

---

## Related Features *(optional)*

### Prerequisite Features (Must Exist Before This Feature)

1. **Better-Auth Service Implementation** (Feature #TBD)
   - **Status**: Needs specification
   - **Relationship**: This feature depends on the auth service being deployed on port 8002
   - **Required Functionality**:
     - Node.js service running Better-Auth with PostgreSQL adapter
     - Email/password authentication enabled
     - GitHub and Google OAuth providers configured
     - Email verification and password reset flows
     - Session management with cookie support
     - CORS configured for Docusaurus origin
   - **Blocking**: Cannot test or deploy frontend auth without backend service

2. **PostgreSQL Database with Better-Auth Schema** (Existing)
   - **Status**: Available (Neon Cloud)
   - **Relationship**: Auth service uses database; frontend never directly accesses it
   - **Required Tables**: `users`, `sessions`, `accounts`, `verifications`
   - **Blocking**: Auth service requires database to store user accounts

### Dependent Features (Depend on This Feature)

3. **FastAPI JWT Validation Middleware** (Feature #TBD)
   - **Status**: Future specification
   - **Relationship**: FastAPI backend (port 8001) must validate session tokens from auth service
   - **Required by**: Associating AI assistant threads with authenticated users, progress tracking
   - **Use Case**: When frontend sends authenticated request to FastAPI, backend validates JWT from auth cookie

4. **User Progress Tracking** (Feature #TBD)
   - **Status**: Future enhancement
   - **Relationship**: Requires user ID from authentication session to track lesson completion, quiz scores
   - **Depends on**: User authentication (this feature), backend user context (FastAPI JWT middleware)

5. **AI Assistant Thread Persistence** (Enhancement to Existing Feature)
   - **Status**: Existing feature needs enhancement
   - **Relationship**: ChatKit threads should be associated with authenticated user ID instead of anonymous sessions
   - **Depends on**: User authentication session providing user ID to ChatKit API calls

6. **Role-Based Access Control (RBAC)** (Future Feature)
   - **Status**: Future specification
   - **Relationship**: Builds on authentication to add roles (student, teacher, admin) and permissions
   - **Depends on**: User authentication (this feature), Better-Auth access control plugin

### Related Features (Shared Components or Context)

7. **Docusaurus Navbar Customization** (Existing)
   - **Status**: Existing component that will be modified
   - **Relationship**: This feature swizzles navbar to add user authentication UI
   - **Impact**: Navbar shows sign-in button (anonymous) or user avatar/dropdown (authenticated)

8. **Email Service Configuration** (Infrastructure Feature)
   - **Status**: Needs setup (one-time configuration)
   - **Relationship**: Auth service requires email provider for verification and reset emails
   - **Shared by**: Future features like notifications, announcements, newsletters

9. **Environment Configuration Management** (DevOps Feature)
   - **Status**: Existing infrastructure
   - **Relationship**: This feature requires environment variables for auth service URL, OAuth credentials
   - **Shared by**: All features requiring environment-specific configuration

10. **Analytics Event Tracking** (Future Feature)
    - **Status**: Future specification
    - **Relationship**: Would track authentication events (sign-ups, logins, OAuth usage)
    - **Depends on**: Authentication state (this feature), analytics SDK integration

## Acceptance Tests *(optional)*

### Test Suite 1: Email/Password Registration Flow

**Test Case 1.1: Successful Sign-Up with Email Verification**
- **Given**: User visits homepage as anonymous visitor
- **When**: User clicks "Sign Up", enters valid email, password (8+ chars), and name, then submits form
- **Then**:
  - Success message displays: "Check your email to verify your account"
  - User receives verification email within 1 minute
  - Email contains verification link with valid token
  - User is NOT signed in until email verified

**Test Case 1.2: Email Verification Link Success**
- **Given**: User receives verification email with valid token
- **When**: User clicks verification link
- **Then**:
  - User is redirected to dashboard
  - User's `email_verified` field is set to `true` in database
  - Session cookie is created (user is signed in)
  - Navbar displays user avatar and name

**Test Case 1.3: Duplicate Email Rejection**
- **Given**: User with email `test@example.com` already exists
- **When**: New user tries to sign up with `test@example.com`
- **Then**: Error message displays: "Email already registered"

**Test Case 1.4: Password Strength Validation**
- **Given**: User is on sign-up form
- **When**: User enters password "1234567" (7 chars)
- **Then**: Error displays: "Password must be at least 8 characters"

---

### Test Suite 2: Email/Password Sign-In Flow

**Test Case 2.1: Successful Sign-In with Verified Email**
- **Given**: User has verified account with email `user@example.com` and password `SecurePass123`
- **When**: User enters correct credentials and clicks "Sign In"
- **Then**:
  - User is redirected to dashboard
  - Session cookie is created
  - Navbar shows user avatar and name
  - `useSession` hook returns user data

**Test Case 2.2: Sign-In Rejected for Unverified Email**
- **Given**: User registered but never verified email
- **When**: User enters correct email and password
- **Then**:
  - Error displays: "Please verify your email address"
  - User remains signed out
  - Option to "Resend verification email" is shown

**Test Case 2.3: Invalid Credentials Error**
- **Given**: User enters email `user@example.com` with wrong password
- **When**: User submits sign-in form
- **Then**: Error displays: "Invalid email or password"

**Test Case 2.4: Non-Existent Email Error**
- **Given**: User enters email that doesn't exist in database
- **When**: User submits sign-in form
- **Then**: Error displays: "Invalid email or password" (same as wrong password to prevent enumeration)

---

### Test Suite 3: SSO Authentication (GitHub)

**Test Case 3.1: New User Sign-Up with GitHub**
- **Given**: User has never signed up, has GitHub account
- **When**: User clicks "Continue with GitHub" and authorizes the app
- **Then**:
  - User is redirected to dashboard
  - User account created with GitHub email, name, and avatar
  - Account marked as email verified (GitHub email assumed verified)
  - Session cookie created

**Test Case 3.2: Existing User Sign-In with GitHub**
- **Given**: User previously signed up with GitHub
- **When**: User clicks "Continue with GitHub" and authorizes
- **Then**:
  - User is signed in to existing account
  - Session cookie created
  - No duplicate user created

**Test Case 3.3: Account Linking - GitHub to Existing Email Account**
- **Given**: User signed up with `user@example.com` via email/password
- **When**: User signs in, goes to Settings, clicks "Link GitHub" using same email
- **Then**:
  - GitHub account linked to existing user
  - User can now sign in with either email/password OR GitHub
  - Settings page shows both authentication methods

**Test Case 3.4: OAuth Error Handling - User Denies Access**
- **Given**: User clicks "Continue with GitHub"
- **When**: User denies authorization on GitHub's page
- **Then**:
  - User redirected back to sign-in page
  - Error message: "GitHub authentication was cancelled"
  - User can try again

---

### Test Suite 4: SSO Authentication (Google)

**Test Case 4.1: New User Sign-Up with Google**
- **Given**: User has never signed up, has Google account
- **When**: User clicks "Continue with Google" and authorizes
- **Then**:
  - User redirected to dashboard
  - Account created with Google email, name, avatar
  - Email marked as verified
  - Session cookie created

**Test Case 4.2: Account Linking - Google to GitHub Account**
- **Given**: User originally signed up with GitHub
- **When**: User goes to Settings, clicks "Link Google" using same email
- **Then**:
  - Google account linked
  - User can sign in with GitHub OR Google
  - Settings shows both providers

---

### Test Suite 5: Password Reset Flow

**Test Case 5.1: Password Reset Request**
- **Given**: User forgot password
- **When**: User clicks "Forgot password", enters email, submits
- **Then**:
  - Success message: "If that email exists, you'll receive a reset link"
  - User receives password reset email within 1 minute
  - Email contains reset link with valid token (1-hour expiration)

**Test Case 5.2: Password Reset Link Success**
- **Given**: User receives reset email with valid token
- **When**: User clicks reset link, enters new password (8+ chars), submits
- **Then**:
  - Password updated in database
  - All other sessions revoked (security)
  - User redirected to sign-in page
  - User can sign in with new password

**Test Case 5.3: Expired Reset Link**
- **Given**: User received reset link 2 hours ago (expired)
- **When**: User clicks reset link
- **Then**:
  - Error message: "This link has expired. Request a new one."
  - Link to request new reset email

**Test Case 5.4: Already-Used Reset Link**
- **Given**: User successfully reset password using a link
- **When**: User clicks same reset link again
- **Then**:
  - Error message: "This link has already been used."
  - Link to request new reset email if needed

---

### Test Suite 6: Protected Content Access

**Test Case 6.1: Anonymous User Accessing AI Assistant**
- **Given**: User is NOT signed in
- **When**: User clicks "Ask AI Assistant" button on lesson page
- **Then**:
  - AI Assistant widget does NOT open
  - Sign-in modal appears with message: "Sign in to use the AI Assistant"
  - After signing in, AI Assistant opens automatically

**Test Case 6.2: Anonymous User Starting Quiz**
- **Given**: User is NOT signed in
- **When**: User clicks "Start Quiz" button
- **Then**:
  - Quiz loads but first question is disabled
  - Sign-in prompt displays: "Sign in to save your quiz progress"

**Test Case 6.3: Authenticated User Accessing Protected Content**
- **Given**: User is signed in
- **When**: User clicks "Ask AI Assistant" or "Start Quiz"
- **Then**: Feature works immediately without prompts

**Test Case 6.4: Session Expiration During Interaction**
- **Given**: User is signed in but session expires (7 days passed)
- **When**: User attempts to submit quiz answer or send AI message
- **Then**:
  - Modal displays: "Your session has expired. Please sign in again."
  - After re-authentication, user's action is completed (quiz submission succeeds)

---

### Test Suite 7: Session Management

**Test Case 7.1: Session Persistence with "Remember Me"**
- **Given**: User signs in with "Remember me" checked
- **When**: User closes browser and reopens within 7 days
- **Then**:
  - User is still signed in
  - Dashboard accessible without re-authentication
  - `useSession` returns valid session

**Test Case 7.2: Session Expiration Without "Remember Me"**
- **Given**: User signs in WITHOUT "Remember me" checked
- **When**: User closes browser
- **Then**:
  - Session cookie deleted (session cookie type)
  - User must sign in again when returning

**Test Case 7.3: Sign Out from All Devices**
- **Given**: User signed in on Device A and Device B
- **When**: User clicks "Sign out from all devices" on Device A
- **Then**:
  - Device A signed out immediately
  - Device B's session revoked (user signed out within 5 seconds)
  - All session tokens invalidated in database

**Test Case 7.4: Session Restoration on Page Refresh**
- **Given**: User is signed in with active session
- **When**: User refreshes page
- **Then**:
  - User remains signed in
  - No flash of unauthenticated UI (loading state shows)
  - `useSession` hook returns data within 500ms

---

### Test Suite 8: User Profile Management

**Test Case 8.1: Update Display Name**
- **Given**: User is signed in as "John Doe"
- **When**: User goes to Profile Settings, changes name to "Jane Smith", saves
- **Then**:
  - Name updated in database
  - Navbar immediately shows "Jane Smith"
  - `useSession` hook reflects new name

**Test Case 8.2: View Linked Authentication Methods**
- **Given**: User signed up with email, then linked GitHub
- **When**: User visits Account Settings
- **Then**:
  - Both "Email/Password" and "GitHub" shown as linked methods
  - "Unlink" button available for GitHub (not for email, as it's primary)

**Test Case 8.3: Unlink Secondary OAuth Provider**
- **Given**: User has email/password AND GitHub linked
- **When**: User clicks "Unlink" on GitHub
- **Then**:
  - GitHub removed from linked methods
  - User can still sign in with email/password
  - Cannot unlink last remaining method (error: "You must have at least one sign-in method")

---

### Test Suite 9: Error Handling & Edge Cases

**Test Case 9.1: Network Error During Sign-In**
- **Given**: User is on sign-in page
- **When**: Network disconnects and user submits form
- **Then**:
  - Error message: "Network error. Please check your connection and try again."
  - Form data preserved (email field not cleared)
  - Retry button available

**Test Case 9.2: Auth Service Unavailable**
- **Given**: Auth service (port 8002) is down
- **When**: User tries to sign in
- **Then**:
  - Error message: "Authentication service unavailable. Please try again later."
  - Public content remains accessible
  - Retry button available

**Test Case 9.3: Corrupted Session Cookie**
- **Given**: User manually edits session cookie value in browser
- **When**: Page loads and checks session
- **Then**:
  - User treated as anonymous (signed out)
  - No error message displayed (graceful degradation)
  - Protected content shows sign-in prompt

**Test Case 9.4: Duplicate Provider Linking Prevention**
- **Given**: User already has GitHub linked
- **When**: User tries to link GitHub again (same account)
- **Then**: Error message: "This GitHub account is already linked"

---

### Test Suite 10: UI/UX Validation

**Test Case 10.1: Loading States During Authentication**
- **Given**: User submits sign-in form
- **When**: Request is pending
- **Then**:
  - Submit button shows spinner and is disabled
  - Form inputs are disabled
  - "Signing in..." text displayed

**Test Case 10.2: Success Feedback**
- **Given**: User successfully signs in
- **When**: Redirect to dashboard occurs
- **Then**:
  - Brief success toast: "Welcome back!" (optional but nice UX)
  - Dashboard loads with user data

**Test Case 10.3: Redirect Preservation After Sign-In**
- **Given**: Anonymous user is on `/docs/chapter-5/lesson-3` and clicks "Ask AI Assistant"
- **When**: Sign-in modal appears, user signs in
- **Then**:
  - User redirected back to `/docs/chapter-5/lesson-3`
  - AI Assistant widget automatically opens
  - User's intended action completed seamlessly

**Test Case 10.4: Auto-Redirect from Sign-In Page for Authenticated Users**
- **Given**: User is already signed in
- **When**: User navigates to `/sign-in` or `/sign-up` URL
- **Then**: User immediately redirected to `/dashboard`

---

### Performance Acceptance Tests

**Test Case P.1: Session Validation Speed**
- **Given**: User is signed in
- **When**: `useSession` hook is called on component mount
- **Then**: Session data returned within 500ms (95th percentile)

**Test Case P.2: Sign-In Response Time**
- **Given**: User enters valid credentials
- **When**: User submits sign-in form
- **Then**: Response received within 2 seconds (includes network round-trip to auth service)

**Test Case P.3: OAuth Redirect Speed**
- **Given**: User clicks "Continue with GitHub"
- **When**: OAuth redirect begins
- **Then**: Redirect to GitHub occurs within 1 second

**Test Case P.4: Protected Content Prompt Speed**
- **Given**: Anonymous user clicks protected feature
- **When**: Access check completes
- **Then**: Sign-in modal displays within 200ms
