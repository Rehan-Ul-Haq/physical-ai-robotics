/**
 * Docusaurus Root Component
 *
 * This component wraps the entire site with:
 * 1. AnalyticsTracker - Tracks user interactions (page views, scroll depth, etc.)
 * 2. ChatWidget - Book Assistant AI chatbot for Q&A about book content
 *    (Only shown on /docs pages, not on homepage)
 *
 * GA4 is configured via the GA4_MEASUREMENT_ID environment variable.
 * If not set, analytics will not load.
 */

import React from 'react';
import { useLocation } from '@docusaurus/router';
import { AnalyticsTracker } from '@/components/AnalyticsTracker';
import ChatWidget from '@/components/ChatWidget';

export default function Root({ children }: { children: React.ReactNode }) {
  const location = useLocation();
  
  // Only show ChatWidget on docs pages (book content)
  const isDocsPage = location.pathname.includes('/docs');
  
  return (
    <AnalyticsTracker>
      {children}
      {isDocsPage && <ChatWidget />}
    </AnalyticsTracker>
  );
}
