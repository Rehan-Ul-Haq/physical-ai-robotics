/**
 * Auth Components Module
 *
 * This module exports all authentication-related React components.
 * Import styles in your main CSS or layout component.
 *
 * @example
 * import { SignInModal, SignUpModal, UserMenu } from '@/components/Auth';
 * import '@/components/Auth/Auth.css';
 */

// Import styles
import "./Auth.css";

// Modal Components
export { SignUpModal } from "./SignUpModal";
export { SignInModal } from "./SignInModal";
export { AuthPromptModal } from "./AuthPromptModal";
export { PasswordResetRequest } from "./PasswordResetRequest";

// SSO Components
export { SSOButtons } from "./SSOButtons";

// User Menu
export { UserMenu } from "./UserMenu";

// Route Protection
export { RequireAuth, useRequireAuth, getAuthRedirectUrl } from "./RequireAuth";
