# OAuth Setup Guide

This guide explains how to set up GitHub and Google OAuth for the RoboAI authentication service.

## Prerequisites

- Auth-service configured with `.env` file
- Domain for callback URLs (localhost:8002 for development)

---

## GitHub OAuth Setup (T052, T054)

### 1. Register OAuth Application

1. Go to [GitHub Developer Settings](https://github.com/settings/developers)
2. Click **"New OAuth App"**
3. Fill in the details:

| Field | Development Value |
|-------|-------------------|
| Application name | `RoboAI Development` |
| Homepage URL | `http://localhost:3000` |
| Authorization callback URL | `http://localhost:8002/api/auth/callback/github` |

4. Click **"Register application"**

### 2. Get Credentials

1. Copy the **Client ID**
2. Click **"Generate a new client secret"**
3. Copy the **Client Secret** (save it securely - shown only once!)

### 3. Add to Environment

```bash
# In auth-service/.env
GITHUB_CLIENT_ID=your_client_id_here
GITHUB_CLIENT_SECRET=your_client_secret_here
```

### Production Setup

For production, create a separate OAuth app with:
- Homepage URL: `https://yourdomain.com`
- Callback URL: `https://api.yourdomain.com/api/auth/callback/github`

---

## Google OAuth Setup (T053, T055)

### 1. Create OAuth Client

1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project or select existing
3. Navigate to **APIs & Services** → **Credentials**
4. Click **"Create Credentials"** → **"OAuth client ID"**

### 2. Configure Consent Screen (First Time Only)

1. Go to **OAuth consent screen**
2. Select **External** user type
3. Fill in required fields:
   - App name: `RoboAI`
   - User support email: your email
   - Developer contact email: your email
4. Add scopes: `email`, `profile`, `openid`
5. Add test users (for development)
6. Save and continue

### 3. Create OAuth Client ID

1. Go back to **Credentials** → **Create Credentials** → **OAuth client ID**
2. Application type: **Web application**
3. Name: `RoboAI Web Client`
4. Add **Authorized JavaScript origins**:
   - `http://localhost:3000`
   - `http://localhost:8002`
5. Add **Authorized redirect URIs**:
   - `http://localhost:8002/api/auth/callback/google`
6. Click **Create**

### 4. Get Credentials

Copy the **Client ID** and **Client Secret** from the popup.

### 5. Add to Environment

```bash
# In auth-service/.env
GOOGLE_CLIENT_ID=your_client_id_here.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=your_client_secret_here
```

### Production Setup

Add production URLs to the OAuth client:
- Authorized origins: `https://yourdomain.com`, `https://api.yourdomain.com`
- Redirect URIs: `https://api.yourdomain.com/api/auth/callback/google`

---

## Testing OAuth

### 1. Start the Services

```bash
# From repository root
npm run dev
```

This starts:
- Docusaurus on port 3000
- Auth-service on port 8002

### 2. Test OAuth Flow

1. Open `http://localhost:3000`
2. Click **Sign In** in navbar
3. Click **Continue with GitHub** or **Continue with Google**
4. Complete the OAuth flow
5. You should be redirected back to the app, authenticated

### 3. Verify Session

Check the browser's developer tools:
- **Application** → **Cookies** → `roboai_session` should be set
- **Network** → `http://localhost:8002/api/auth/session` should return user data

---

## Troubleshooting

### "Invalid redirect URI" Error

- Ensure callback URL matches exactly (including trailing slashes)
- Check that port number is correct (8002)
- Verify the URL in OAuth app settings matches `.env` config

### "Access blocked: App not verified" (Google)

- For development: Add yourself as a test user in OAuth consent screen
- For production: Submit for verification

### "OAuth scope error"

Ensure you've added required scopes in Google Cloud Console:
- `email`
- `profile`
- `openid`

### CORS Errors

Check that `FRONTEND_URL` in auth-service `.env` matches your frontend origin.

---

## Security Checklist

- [ ] Never commit OAuth secrets to version control
- [ ] Use different OAuth apps for development and production
- [ ] Rotate secrets periodically
- [ ] Enable HTTPS for production callback URLs
- [ ] Review OAuth app permissions regularly
