# Auth Service

Better-Auth authentication service for the Physical AI Robotics book (RoboAI).

## Features

- 📧 Email/Password authentication with verification
- 🔐 OAuth providers (GitHub, Google)
- 🔄 Password reset via email
- 🍪 Secure session management with httpOnly cookies
- 🔗 Account linking for multiple OAuth providers

## Prerequisites

- Node.js 18+
- PostgreSQL database (Neon Cloud recommended)
- Gmail account with App Password for email sending
- OAuth apps created on GitHub and Google (optional)

## Setup

### 1. Install dependencies

```bash
cd auth-service
npm install
```

### 2. Configure environment variables

```bash
cp .env.example .env
```

Edit `.env` and fill in your values:

| Variable | Description |
|----------|-------------|
| `DATABASE_URL` | PostgreSQL connection string |
| `BETTER_AUTH_SECRET` | Secret key for signing tokens (min 32 chars) |
| `GMAIL_USER` | Your Gmail address |
| `GMAIL_APP_PASSWORD` | Gmail App Password (see below) |
| `GITHUB_CLIENT_ID` | GitHub OAuth app client ID |
| `GITHUB_CLIENT_SECRET` | GitHub OAuth app secret |
| `GOOGLE_CLIENT_ID` | Google OAuth app client ID |
| `GOOGLE_CLIENT_SECRET` | Google OAuth app secret |

### 3. Gmail App Password Setup

1. Go to [Google Account Security](https://myaccount.google.com/security)
2. Enable **2-Step Verification** if not already enabled
3. Go to [App Passwords](https://myaccount.google.com/apppasswords)
4. Select **Mail** and your device
5. Copy the 16-character password to `GMAIL_APP_PASSWORD`

### 4. Run database migrations

```bash
npm run migrate
```

This creates the required tables:
- `user` - User accounts
- `session` - Active sessions
- `account` - OAuth/password credentials
- `verification` - Email verification tokens

### 5. Start the server

```bash
# Development (with hot reload)
npm run dev

# Production
npm run build
npm start
```

The server runs on `http://localhost:8002` by default.

## OAuth Setup

### GitHub OAuth

1. Go to [GitHub Developer Settings](https://github.com/settings/developers)
2. Create a new OAuth App
3. Set callback URL to: `http://localhost:8002/api/auth/callback/github`
4. Copy Client ID and Client Secret to `.env`

### Google OAuth

1. Go to [Google Cloud Console](https://console.cloud.google.com/apis/credentials)
2. Create a new OAuth 2.0 Client ID
3. Add authorized redirect URI: `http://localhost:8002/api/auth/callback/google`
4. Copy Client ID and Client Secret to `.env`

## API Endpoints

All auth endpoints are handled by Better-Auth at `/api/auth/*`:

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/auth/sign-up/email` | POST | Register with email/password |
| `/api/auth/sign-in/email` | POST | Sign in with email/password |
| `/api/auth/sign-in/social` | POST | Sign in with OAuth provider |
| `/api/auth/sign-out` | POST | Sign out current session |
| `/api/auth/session` | GET | Get current session |
| `/api/auth/verify-email` | GET | Verify email with token |
| `/api/auth/forget-password` | POST | Request password reset |
| `/api/auth/reset-password` | POST | Reset password with token |
| `/api/auth/callback/:provider` | GET | OAuth callback handler |

## Development

```bash
# Run with hot reload
npm run dev

# Generate Better-Auth types
npm run generate

# Run migrations
npm run migrate
```

## Production Deployment

1. Build the TypeScript:
   ```bash
   npm run build
   ```

2. Set environment variables on your hosting platform

3. Run migrations:
   ```bash
   npm run migrate
   ```

4. Start the server:
   ```bash
   npm start
   ```

## CORS Configuration

The server is configured to accept requests from:
- `http://localhost:3000` (development)
- Your production frontend URL

Update the CORS origins in `server.ts` for production.

## Security Notes

- Use strong `BETTER_AUTH_SECRET` (min 32 random characters)
- Never commit `.env` file to version control
- Use Gmail App Password, never your regular password
- Enable HTTPS in production
- Configure secure cookie settings for production
