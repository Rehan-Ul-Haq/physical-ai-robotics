/**
 * Docusaurus Root Component
 *
 * This component wraps the entire site with:
 * 1. PyodideProvider - Enables direct Pyodide integration for interactive Python execution
 * 2. AnalyticsTracker - Tracks user interactions (page views, scroll depth, etc.)
 * 3. ChatWidget - Book Assistant AI chatbot for Q&A about book content
 *
 * GA4 is configured via the GA4_MEASUREMENT_ID environment variable.
 * If not set, analytics will not load.
 */

import React from 'react';
import { PyodideProvider } from '@/contexts/PyodideContext';
import { AnalyticsTracker } from '@/components/AnalyticsTracker';
import ChatWidget from '@/components/ChatWidget';

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <PyodideProvider>
      <AnalyticsTracker>
        {children}
        <ChatWidget />
      </AnalyticsTracker>
    </PyodideProvider>
  );
}
