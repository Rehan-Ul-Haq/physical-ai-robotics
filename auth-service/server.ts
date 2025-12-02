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
  process.env.FRONTEND_URL, // Production frontend URL
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

// Better-Auth API handler (all routes under /api/auth/*)
// This handles: sign-up, sign-in, sign-out, OAuth callbacks, password reset, etc.
app.all("/api/auth/*", toNodeHandler(auth));

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
