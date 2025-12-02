---
description: "Task list for Better-Auth Frontend Integration"
---

# Tasks: Better-Auth Frontend Integration

**Input**: Design documents from `/specs/004-better-auth-frontend/`
**Prerequisites**: spec.md, research.md (plan.md empty - using research.md as source)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This is a **web application** with:
- **Frontend**: `book-source/src/` (Docusaurus React app)
- **Backend**: Separate Node.js Better-Auth service on port 8002 (out of scope - separate spec)
- **Database**: Existing PostgreSQL (Neon Cloud) - will add auth tables

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Install Better-Auth and configure project structure

- [X] T001 Install better-auth package in book-source via `npm install better-auth`
- [X] T002 [P] Create auth client directory structure: book-source/src/lib/auth/
- [X] T003 [P] Create auth components directory: book-source/src/components/Auth/
- [X] T004 [P] Create environment variable placeholder in book-source/.env.local for REACT_APP_AUTH_SERVICE_URL
- [X] T005 Update book-source/tsconfig.json with path alias @ for src/ (already configured)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core Better-Auth service and database setup that MUST be complete before ANY user story

**⚠️ CRITICAL**: This phase creates the auth backend service (separate from frontend)

### Better-Auth Service Setup (Port 8002)

- [X] T006 Create auth-service directory at repository root for Node.js auth service
- [X] T007 Initialize Node.js project in auth-service/ with npm init
- [X] T008 [P] Install Better-Auth server dependencies in auth-service: better-auth, @better-auth/cli, pg, dotenv
- [X] T009 Create auth-service/auth.ts with Better-Auth server configuration
- [X] T010 Configure PostgreSQL adapter in auth-service/auth.ts using existing Neon database
- [X] T011 [P] Configure email/password authentication in auth-service/auth.ts
- [X] T012 [P] Configure GitHub OAuth provider in auth-service/auth.ts
- [X] T013 [P] Configure Google OAuth provider in auth-service/auth.ts
- [X] T014 Create auth-service/.env file with DATABASE_URL, GITHUB_CLIENT_ID, GITHUB_CLIENT_SECRET, GOOGLE_CLIENT_ID, GOOGLE_CLIENT_SECRET, BETTER_AUTH_SECRET
- [X] T015 Run Better-Auth migrations to create users, sessions, accounts, verification tables in PostgreSQL
- [X] T016 Configure CORS in auth-service/auth.ts to allow localhost:3000 (Docusaurus dev server)
- [X] T017 Create auth-service/server.ts with Express server listening on port 8002
- [X] T018 [P] Add auth-service/package.json scripts for dev, build, start
- [X] T019 Test auth service startup on port 8002 and verify API responds at /api/auth/*

### Frontend Auth Client Setup

- [X] T020 Create book-source/src/lib/auth/auth-client.ts with createAuthClient from better-auth/react
- [X] T021 Configure auth client baseURL to point to http://localhost:8002 using REACT_APP_AUTH_SERVICE_URL
- [X] T022 [P] Export all auth client methods (signIn, signUp, signOut, useSession, etc.) from auth-client.ts
- [X] T023 [P] Add global error handling in auth-client.ts fetchOptions (429 rate limit, 401 unauthorized)

**Checkpoint**: Foundation ready - Better-Auth service running on 8002, frontend client configured

---

## Phase 3: User Story 1 - Anonymous User Sign-Up with Email (Priority: P1) 🎯 MVP

**Goal**: Enable new users to create accounts using email/password, verify email, and access dashboard

**Independent Test**: Visit sign-up page, enter credentials, receive verification email, click link, access dashboard

### Implementation for User Story 1

- [X] T024 [P] [US1] Create SignUpModal component in book-source/src/components/Auth/SignUpModal.tsx
- [X] T025 [P] [US1] Create email/password form with name, email, password fields in SignUpModal.tsx
- [X] T026 [P] [US1] Add form validation (email format, password min 8 chars) in SignUpModal.tsx
- [X] T027 [US1] Implement signUp.email() call in SignUpModal.tsx with onRequest, onSuccess, onError callbacks
- [X] T028 [US1] Add loading state during sign-up (disable form, show spinner) in SignUpModal.tsx
- [X] T029 [US1] Display "Check your email" message after successful sign-up in SignUpModal.tsx
- [X] T030 [US1] Handle error states (email exists, weak password) with user-friendly messages in SignUpModal.tsx
- [X] T031 [P] [US1] Configure email verification in auth-service/auth.ts (24-hour token expiration)
- [X] T032 [P] [US1] Setup email service provider (Gmail SMTP) in auth-service for verification emails
- [X] T033 [US1] Create email verification handler route in auth-service for /api/auth/verify-email
- [X] T034 [US1] Implement redirect to dashboard after successful email verification in auth-service
- [X] T035 [P] [US1] Add "Sign Up" button in book-source/src/components/Navbar (swizzle Navbar component)
- [X] T036 [US1] Wire Sign Up button to open SignUpModal component
- [X] T037 [P] [US1] Create Dashboard page component in book-source/src/pages/dashboard/index.tsx
- [X] T038 [US1] Display user name and email on Dashboard using useSession hook
- [X] T039 [US1] Add "Resend verification email" option for unverified users in SignUpModal.tsx

**Checkpoint**: Users can sign up with email/password, receive verification email, verify account, and access dashboard

---

## Phase 4: User Story 2 - Existing User Sign-In (Priority: P1)

**Goal**: Enable returning users to sign in with email/password

**Independent Test**: Register user, sign out, sign back in with credentials, access dashboard

### Implementation for User Story 2

- [X] T040 [P] [US2] Create SignInModal component in book-source/src/components/Auth/SignInModal.tsx
- [X] T041 [P] [US2] Create email/password form in SignInModal.tsx
- [X] T042 [US2] Implement signIn.email() call with email, password, rememberMe checkbox in SignInModal.tsx
- [X] T043 [US2] Add "Remember me" checkbox (7-day session vs session cookie) in SignInModal.tsx
- [X] T044 [US2] Handle successful sign-in redirect to dashboard in SignInModal.tsx
- [X] T045 [US2] Display error message "Invalid email or password" without email enumeration in SignInModal.tsx
- [X] T046 [US2] Add loading state during sign-in (disable form, spinner) in SignInModal.tsx
- [X] T047 [P] [US2] Check for active session and auto-redirect authenticated users from sign-in page
- [X] T048 [P] [US2] Add "Sign In" button in Navbar component
- [X] T049 [US2] Wire Sign In button to open SignInModal component
- [X] T050 [US2] Prevent sign-in for unverified email addresses with "Please verify your email" error in auth-service
- [X] T051 [US2] Add link to sign-up modal from sign-in modal ("Don't have an account? Sign up")

**Checkpoint**: Verified users can sign in with email/password and access their dashboard

---

## Phase 5: User Story 3 - SSO Authentication with GitHub/Google (Priority: P1)

**Goal**: Enable users to authenticate using GitHub or Google OAuth

**Independent Test**: Click "Continue with GitHub", complete OAuth flow, land on dashboard with GitHub profile data

### Implementation for User Story 3

- [X] T052 [P] [US3] Register GitHub OAuth application at https://github.com/settings/developers
- [X] T053 [P] [US3] Register Google OAuth application in Google Cloud Console
- [X] T054 [P] [US3] Add GitHub OAuth credentials to auth-service/.env (GITHUB_CLIENT_ID, GITHUB_CLIENT_SECRET)
- [X] T055 [P] [US3] Add Google OAuth credentials to auth-service/.env (GOOGLE_CLIENT_ID, GOOGLE_CLIENT_SECRET)
- [X] T056 [P] [US3] Create SSOButtons component in book-source/src/components/Auth/SSOButtons.tsx
- [X] T057 [P] [US3] Add "Continue with GitHub" button in SSOButtons.tsx
- [X] T058 [P] [US3] Add "Continue with Google" button in SSOButtons.tsx
- [X] T059 [US3] Implement signIn.social({ provider: "github" }) in GitHub button click handler
- [X] T060 [US3] Implement signIn.social({ provider: "google" }) in Google button click handler
- [X] T061 [US3] Handle OAuth redirect to dashboard after successful authorization in auth-service
- [X] T062 [US3] Populate user profile (name, avatar) from OAuth provider data in auth-service
- [X] T063 [US3] Mark OAuth users' emails as verified automatically in auth-service
- [X] T064 [US3] Handle OAuth errors (user denies access) with "Authentication cancelled" message in SSOButtons.tsx
- [X] T065 [P] [US3] Add SSOButtons component to SignUpModal above email form with "Or continue with email" separator
- [X] T066 [P] [US3] Add SSOButtons component to SignInModal above email form
- [X] T067 [US3] Implement account linking when OAuth email matches existing email account in auth-service
- [X] T068 [US3] Prevent duplicate provider linking (same GitHub account linked twice) in auth-service

**Checkpoint**: Users can sign up/sign in with GitHub or Google OAuth, with auto-populated profiles

---

## Phase 6: User Story 4 - Protected Content Access (Priority: P2)

**Goal**: Protect AI assistant and quiz features, prompt anonymous users to sign in

**Independent Test**: Click "Ask Assistant" as anonymous user, see sign-in prompt, authenticate, AI assistant opens

### Implementation for User Story 4

- [X] T069 [P] [US4] Create RequireAuth HOC component in book-source/src/components/Auth/RequireAuth.tsx
- [X] T070 [US4] Implement useSession check in RequireAuth with loading, error, unauthenticated states
- [X] T071 [US4] Add redirect to sign-in modal when unauthenticated user accesses protected content
- [X] T072 [US4] Preserve intended destination URL for redirect after authentication in RequireAuth
- [X] T073 [P] [US4] Create AuthPromptModal component in book-source/src/components/Auth/AuthPromptModal.tsx
- [X] T074 [US4] Display "Sign in to use the AI Assistant" message in AuthPromptModal
- [X] T075 [US4] Add sign-in and sign-up options in AuthPromptModal
- [ ] T076 [P] [US4] Wrap AI assistant chat widget trigger with RequireAuth in lesson pages
- [ ] T077 [P] [US4] Wrap quiz submission with RequireAuth check
- [ ] T078 [US4] Redirect user back to lesson and open AI assistant after successful authentication
- [ ] T079 [US4] Redirect user back to quiz after sign-in from quiz protection prompt
- [X] T080 [US4] Handle session expiration during protected feature use with "Session expired" modal
- [X] T081 [US4] Implement one-click re-authentication for expired sessions in AuthPromptModal

**Checkpoint**: AI assistant and quizzes require authentication, with seamless post-auth redirect

---

## Phase 7: User Story 5 - User Profile and Account Management (Priority: P2)

**Goal**: Enable users to manage their profile, linked accounts, and sign out

**Independent Test**: Access profile settings, change name, link OAuth provider, verify changes persist

### Implementation for User Story 5

- [X] T082 [P] [US5] Create UserMenu dropdown component in book-source/src/components/Auth/UserMenu.tsx
- [X] T083 [US5] Display user avatar and name in Navbar when authenticated using useSession
- [X] T084 [US5] Show dropdown menu with Profile, Settings, Sign Out options on avatar click
- [X] T085 [P] [US5] Create Profile page component in book-source/src/pages/profile/index.tsx
- [X] T086 [US5] Add form to update display name in Profile page
- [X] T087 [US5] Implement name update with session refetch() to sync UI in Profile page
- [X] T088 [P] [US5] Create AccountSettings page component in book-source/src/pages/settings/account.tsx
- [X] T089 [US5] Display all linked authentication methods using useListAccounts hook in AccountSettings
- [X] T090 [US5] Add "Link GitHub" button in AccountSettings for users without GitHub
- [X] T091 [US5] Add "Link Google" button in AccountSettings for users without Google
- [X] T092 [US5] Implement linkSocial({ provider: "github" }) for linking additional OAuth accounts
- [X] T093 [US5] Implement linkSocial({ provider: "google" }) for linking additional OAuth accounts
- [X] T094 [US5] Add "Unlink" button for secondary authentication methods in AccountSettings
- [X] T095 [US5] Implement unlinkAccount({ accountId }) with validation (must keep 1 method) in AccountSettings
- [X] T096 [US5] Add "Sign out" functionality in UserMenu using authClient.signOut()
- [X] T097 [US5] Add "Sign out from all devices" option in AccountSettings with session revocation
- [X] T098 [US5] Clear session cookie and redirect to homepage on sign out

**Checkpoint**: Users can manage profile, link/unlink OAuth accounts, and sign out

---

## Phase 8: User Story 6 - Password Reset Flow (Priority: P2)

**Goal**: Enable users to reset forgotten passwords via email

**Independent Test**: Click "Forgot password", enter email, receive reset email, set new password, sign in

### Implementation for User Story 6

- [X] T099 [P] [US6] Create PasswordResetRequest component in book-source/src/components/Auth/PasswordResetRequest.tsx
- [X] T100 [US6] Add email input form in PasswordResetRequest component
- [X] T101 [US6] Implement forgetPassword({ email }) call in PasswordResetRequest
- [X] T102 [US6] Display "If that email exists, you'll receive a reset link" message (no enumeration)
- [X] T103 [P] [US6] Configure password reset in auth-service with 1-hour token expiration
- [X] T104 [P] [US6] Setup password reset email template in auth-service email provider
- [X] T105 [P] [US6] Create PasswordResetConfirm component in book-source/src/pages/reset-password.tsx
- [X] T106 [US6] Add new password input form in PasswordResetConfirm
- [X] T107 [US6] Validate reset token in PasswordResetConfirm component
- [X] T108 [US6] Implement resetPassword({ newPassword, token }) in PasswordResetConfirm
- [X] T109 [US6] Handle expired reset link (>1 hour) with "Link expired" message in PasswordResetConfirm
- [X] T110 [US6] Handle already-used reset link with "Link already used" message in PasswordResetConfirm
- [X] T111 [US6] Revoke all other sessions when password is successfully reset in auth-service
- [X] T112 [US6] Redirect to sign-in page after successful password reset
- [X] T113 [US6] Add "Forgot password?" link in SignInModal to open PasswordResetRequest

**Checkpoint**: Users can reset forgotten passwords via secure email token flow

---

## Phase 9: User Story 7 - Session Persistence and State Management (Priority: P3)

**Goal**: Maintain authentication state across page refreshes and browser sessions

**Independent Test**: Sign in with "Remember me", close browser, reopen, verify still authenticated

### Implementation for User Story 7

- [X] T114 [P] [US7] Configure session cookie settings in auth-service (httpOnly, secure, sameSite: lax)
- [X] T115 [P] [US7] Implement "Remember me" logic in auth-service (7-day vs session cookie expiration)
- [X] T116 [US7] Test session persistence on page refresh using useSession hook in Dashboard
- [X] T117 [US7] Prevent flash of unauthenticated content (FOUC) using isPending state in RequireAuth
- [X] T118 [US7] Add skeleton loader during session restoration in Dashboard component
- [X] T119 [US7] Configure session expiration after 7 days of inactivity in auth-service
- [X] T120 [US7] Display "Session expired, please sign in again" modal when session expires
- [X] T121 [US7] Clear session cookie on explicit sign-out in auth-service
- [X] T122 [US7] Redirect to homepage after explicit sign-out in frontend

**Checkpoint**: Sessions persist correctly based on "Remember me" choice, with graceful expiration handling

---

## Phase 10: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T123 [P] Add Tailwind CSS styling to all auth components (SignUpModal, SignInModal, SSOButtons, etc.)
- [ ] T124 [P] Implement toast notifications for success/error feedback across all auth flows
- [X] T125 [P] Add loading skeletons for all auth-related loading states
- [X] T126 [P] Ensure all modals are responsive (mobile, tablet, desktop)
- [ ] T127 [P] Add proper focus management and keyboard navigation in auth modals
- [X] T128 [P] Implement proper error logging for auth failures in auth-service
- [ ] T129 Test all user stories end-to-end in development environment
- [X] T130 [P] Update README.md with auth service setup instructions
- [X] T131 [P] Create .env.example files for both book-source and auth-service
- [X] T132 [P] Add TypeScript types for all auth-related components
- [ ] T133 Security audit: Verify httpOnly cookies, CORS config, token expiration
- [ ] T134 Performance optimization: Test session restoration speed (<500ms requirement)
- [ ] T135 [P] Add JSDoc comments to all exported auth client methods
- [X] T136 Create development script to run both Docusaurus (3000) and auth-service (8002) concurrently
- [ ] T137 Validate all success criteria from spec.md (sign-up <3min, sign-in <10s, OAuth <15s, etc.)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-9)**: All depend on Foundational phase completion
  - US1 (Sign-up) → US2 (Sign-in) → US3 (SSO) can proceed sequentially
  - US4 (Protected content) depends on US1 or US2 or US3 (need auth working)
  - US5 (Profile) depends on US1, US2, US3 (need accounts to manage)
  - US6 (Password reset) depends on US1, US2 (email/password auth)
  - US7 (Session persistence) depends on US1, US2, US3 (need sessions to persist)
- **Polish (Phase 10)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Independent of US1 but shares auth client
- **User Story 3 (P1)**: Can start after Foundational (Phase 2) - Independent but requires OAuth setup
- **User Story 4 (P2)**: Depends on US1, US2, or US3 (needs working authentication)
- **User Story 5 (P2)**: Depends on US1, US2, US3 (needs user accounts to manage)
- **User Story 6 (P2)**: Depends on US1, US2 (email/password authentication)
- **User Story 7 (P3)**: Depends on US1, US2, US3 (needs sessions to persist)

### Within Each User Story

- Auth service configuration before frontend components
- Client components before UI integration
- Core implementation before error handling
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- Within Foundational phase:
  - Better-Auth service setup tasks (T006-T019) can be done by one developer
  - Frontend auth client setup (T020-T023) can be done by another developer in parallel
- After Foundational:
  - US1, US2, US3 can be developed in parallel by different developers
  - US4, US5, US6 can start once at least one of US1/US2/US3 is complete
- All Polish tasks marked [P] can run in parallel

---

## Parallel Example: Foundational Phase

```bash
# Backend team: Better-Auth service setup (T006-T019)
Task: "Create auth-service directory and initialize Node.js project"
Task: "Install Better-Auth dependencies"
Task: "Configure PostgreSQL adapter and run migrations"
Task: "Setup OAuth providers (GitHub, Google)"
Task: "Create Express server on port 8002"

# Frontend team (parallel): Auth client setup (T020-T023)
Task: "Create auth-client.ts with Better-Auth React client"
Task: "Configure baseURL and export auth methods"
Task: "Add global error handling"
```

---

## Implementation Strategy

### MVP First (User Stories 1, 2, 3 Only - Core Authentication)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Email sign-up)
4. Complete Phase 4: User Story 2 (Email sign-in)
5. Complete Phase 5: User Story 3 (SSO with GitHub/Google)
6. **STOP and VALIDATE**: Test all three authentication methods independently
7. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Auth service running
2. Add US1 (Email sign-up) → Test independently → Deploy (Basic auth working!)
3. Add US2 (Email sign-in) → Test independently → Deploy (Return users can access)
4. Add US3 (SSO) → Test independently → Deploy (MVP complete - all auth methods work!)
5. Add US4 (Protected content) → Test independently → Deploy (AI assistant protected)
6. Add US5 (Profile management) → Test independently → Deploy (Users can manage accounts)
7. Add US6 (Password reset) → Test independently → Deploy (Account recovery works)
8. Add US7 (Session persistence) → Test independently → Deploy (Improved UX)
9. Complete Polish phase → Production ready

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup together
2. Foundational phase:
   - **Backend developer**: Better-Auth service setup (T006-T019)
   - **Frontend developer**: Auth client setup (T020-T023)
3. Once Foundational done:
   - **Developer A**: User Story 1 (Email sign-up)
   - **Developer B**: User Story 2 (Email sign-in) + User Story 3 (SSO)
   - **Developer C**: User Story 4 (Protected content) after US1/US2 complete
4. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- **CRITICAL**: Better-Auth service (port 8002) must be running for frontend to work
- Use existing PostgreSQL database (Neon Cloud) - auth service will add new tables
- OAuth apps must be registered in GitHub/Google consoles before US3
- Environment variables must be configured in both book-source and auth-service
- Run Docusaurus (3000) and auth-service (8002) concurrently during development
