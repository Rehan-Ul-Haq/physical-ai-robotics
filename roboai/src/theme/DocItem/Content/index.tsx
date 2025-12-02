/**
 * DocItem/Content Theme Swizzle (Wrap)
 *
 * Wraps the original DocItem/Content component with LessonContent
 * to provide tabbed interface for Full Lesson and AI Summary views.
 *
 * Also implements authentication checks:
 * - README.md files (part/chapter summaries) are PUBLIC
 * - Lesson files (01-xxx.md, 02-xxx.md, etc.) require authentication
 *
 * The summary is read from global data (populated by docusaurus-summaries-plugin)
 * which scans for .summary.md files at build time.
 */

import React from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type { WrapperProps } from '@docusaurus/types';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import { usePluginData } from '@docusaurus/useGlobalData';
import LessonContent from '../../../components/LessonContent';
import { AuthGuard } from '../../../components/Auth';
import ReactMarkdown from 'react-markdown';

type Props = WrapperProps<typeof ContentType>;

interface SummariesPluginData {
  summaries: Record<string, string>;
}

export default function ContentWrapper(props: Props): React.ReactElement {
  const doc = useDoc();

  // Get summaries from global data (populated by docusaurus-summaries-plugin)
  let summaries: Record<string, string> = {};
  try {
    const pluginData = usePluginData('docusaurus-summaries-plugin') as SummariesPluginData | undefined;
    summaries = pluginData?.summaries || {};
  } catch {
    // Plugin might not be loaded yet or doesn't exist
    summaries = {};
  }

  // Build the lookup key from doc metadata
  const metadata = doc.metadata;
  const sourceDirName = metadata.sourceDirName || '';
  const slug = metadata.slug || '';
  const docId = metadata.id;

  // Determine if this is a lesson (requires auth) or a summary (public)
  // 
  // URL structure:
  // /docs/robotic-nervous-system/ - Part README (public)
  // /docs/robotic-nervous-system/introduction-to-physical-ai/ - Chapter README (public)  
  // /docs/robotic-nervous-system/introduction-to-physical-ai/paradigm-shift - LESSON (locked)
  //
  // The key insight: lessons are the deepest level (3 segments after /docs/)
  // Parts have 1 segment, Chapters have 2 segments, Lessons have 3 segments
  //
  // We use sourceDirName which contains the actual file path structure
  // Lessons are inside chapter folders: "01-robotic-nervous-system/01-introduction-to-physical-ai"
  // The slug for lessons doesn't include the folder, just the lesson name
  
  const pathSegments = slug.split('/').filter(Boolean);
  const sourceSegments = sourceDirName.split('/').filter(Boolean);
  
  // Count depth: lessons have source in chapter folder (2 segments) AND a slug segment
  // README files have sourceDirName matching their location and empty or matching slug
  // 
  // Better approach: Check if we're at lesson level by URL path depth
  // Part: /docs/part-slug (1 level) 
  // Chapter: /docs/part-slug/chapter-slug (2 levels)
  // Lesson: /docs/part-slug/chapter-slug/lesson-slug (3 levels)
  
  // Get URL path to determine depth
  const currentPath = typeof window !== 'undefined' ? window.location.pathname : '';
  const basePath = '/physical-ai-robotics/docs/';
  const docsPath = currentPath.startsWith(basePath) 
    ? currentPath.slice(basePath.length).replace(/\/$/, '')
    : '';
  const urlSegments = docsPath.split('/').filter(Boolean);
  
  // Lesson = 3 segments (part/chapter/lesson)
  const isLesson = urlSegments.length >= 3;
  
  // Also check chapter-index.md which should be public
  const isChapterIndex = slug.includes('chapter-index') || docId.includes('chapter-index');

  // Debug log in development
  if (typeof window !== 'undefined' && process.env.NODE_ENV === 'development') {
    console.log('[DocItem/Content] Doc ID:', docId);
    console.log('[DocItem/Content] Source dir:', sourceDirName);
    console.log('[DocItem/Content] Slug:', slug);
    console.log('[DocItem/Content] URL segments:', urlSegments);
    console.log('[DocItem/Content] Is lesson (3+ segments):', isLesson);
    console.log('[DocItem/Content] Available summaries:', Object.keys(summaries));
  }

  // Look up summary by doc ID (the key format matches how plugin stores them)
  const summary = summaries[docId];

  // Build the content element
  let contentElement: React.ReactElement;
  
  if (summary) {
    // Has summary - use LessonContent with tabs
    const summaryElement = <ReactMarkdown>{summary}</ReactMarkdown>;
    contentElement = (
      <LessonContent summaryElement={summaryElement}>
        <Content {...props} />
      </LessonContent>
    );
  } else {
    // No summary - just the content
    contentElement = <Content {...props} />;
  }

  // If it's a lesson (3 levels deep), wrap with AuthGuard
  if (isLesson && !isChapterIndex) {
    return (
      <AuthGuard contentType="lesson">
        {contentElement}
      </AuthGuard>
    );
  }

  // Part summaries, chapter summaries, and chapter-index are public
  return contentElement;
}
