/**
 * Auth module exports
 *
 * Re-exports all auth client methods and hooks for convenient imports:
 * import { useSession, signIn, signOut } from '@/lib/auth';
 */
export { authClient, signIn, signUp, signOut, useSession } from "./auth-client";

export { default } from "./auth-client";
