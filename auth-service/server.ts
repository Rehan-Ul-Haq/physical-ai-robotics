/**
 * Express Server for Better-Auth Service
 *
 * Implements T017-T019 from tasks.md:
 * - Express server listening on port 8002
 * - CORS configuration for frontend (T016)
 * - Better-Auth API handler at /api/auth/*
 *
 * Run with: npm run dev
 */
import express from "express";
import cors from "cors";
import { toNodeHandler } from "better-auth/node";
import dotenv from "dotenv";
import { auth } from "./auth.js";

// Load environment variables
dotenv.config();

const app = express();
const PORT = parseInt(process.env.PORT || "8002", 10);

// Frontend URLs for CORS (T016)
const ALLOWED_ORIGINS = [
  "http://localhost:3000",  // Docusaurus dev server
  "http://localhost:3001",  // Alternative dev port
  "http://127.0.0.1:3000",
  "https://rehan-ul-haq.github.io", // GitHub Pages production
  process.env.FRONTEND_URL, // Production frontend URL from env
].filter(Boolean) as string[];

// CORS configuration (T016)
app.use(
  cors({
    origin: (origin, callback) => {
      // Allow requests with no origin (mobile apps, Postman, etc.)
      if (!origin) {
        return callback(null, true);
      }
      
      if (ALLOWED_ORIGINS.includes(origin)) {
        return callback(null, true);
      }
      
      console.warn(`[CORS] Blocked request from origin: ${origin}`);
      return callback(new Error("Not allowed by CORS"));
    },
    credentials: true, // Allow cookies to be sent
    methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allowedHeaders: ["Content-Type", "Authorization"],
  })
);

// Parse JSON bodies
app.use(express.json());

// Health check endpoint
app.get("/health", (req, res) => {
  res.json({
    status: "ok",
    service: "auth-service",
    timestamp: new Date().toISOString(),
  });
});

// Redirect root to frontend (for post-verification redirects)
app.get("/", (req, res) => {
  const frontendUrl = process.env.FRONTEND_URL || "http://localhost:3000";
  res.redirect(`${frontendUrl}/physical-ai-robotics/?verified=true`);
});

// Better-Auth API handler
// Using all() with a regex pattern to match all /api/auth routes
const authHandler = toNodeHandler(auth);

// Handle all auth routes - both with and without trailing paths
app.all("/api/auth", authHandler);
app.all("/api/auth/*", (req, res) => {
  console.log(`[Auth] ${req.method} ${req.originalUrl}`);
  return authHandler(req, res);
});

// Verification success redirect - after Better-Auth verifies the email,
// redirect user to the frontend
app.get("/auth/verified", (req, res) => {
  const frontendUrl = process.env.FRONTEND_URL || "http://localhost:3000";
  res.redirect(`${frontendUrl}/physical-ai-robotics/?verified=true`);
});

// 404 handler
app.use((req, res) => {
  res.status(404).json({
    error: "Not Found",
    message: `Route ${req.method} ${req.path} not found`,
  });
});

// Error handler
app.use((err: Error, req: express.Request, res: express.Response, next: express.NextFunction) => {
  console.error("[Error]", err);
  res.status(500).json({
    error: "Internal Server Error",
    message: process.env.NODE_ENV === "development" ? err.message : "Something went wrong",
  });
});

// Start server
app.listen(PORT, () => {
  console.log(`
╔══════════════════════════════════════════════════════════════╗
║                    Better-Auth Service                        ║
╠══════════════════════════════════════════════════════════════╣
║  Status:    Running                                          ║
║  Port:      ${PORT}                                             ║
║  API:       http://localhost:${PORT}/api/auth/*                 ║
║  Health:    http://localhost:${PORT}/health                     ║
╠══════════════════════════════════════════════════════════════╣
║  OAuth Callbacks:                                            ║
║  - GitHub:  http://localhost:${PORT}/api/auth/callback/github   ║
║  - Google:  http://localhost:${PORT}/api/auth/callback/google   ║
╚══════════════════════════════════════════════════════════════╝
  `);
});

export default app;
