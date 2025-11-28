import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";
import * as dotenv from "dotenv";

// Load environment variables from .env file (for local development)
// Production uses actual environment variables set in CI/CD
dotenv.config();

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

// PanaversityFS: Determine docs path based on plugin mode
// - When plugin enabled: Read from docsfs/ (fetched from MCP server)
// - When plugin disabled: Read from docs/ (local sample content)
const panaversityEnabled = process.env.PANAVERSITY_PLUGIN_ENABLED === "true";
const docsPath = panaversityEnabled ? "docsfs" : "docs";

const config: Config = {
  title: "Physical AI & Humanoid Robotics",
  tagline:
    "Embodied Intelligence from Simulation to Reality — ROS 2, Gazebo, NVIDIA Isaac",
  favicon: "img/favicon.ico",

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  // For GitHub Pages: https://<organizationName>.github.io
  url: "https://Rehan-Ul-Haq.github.io",
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: "/physical-ai-robotics/",

  // Sitemap is configured via the classic preset's sitemap option below

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: "Rehan-Ul-Haq", // Usually your GitHub org/user name.
  projectName: "physical-ai-robotics", // Usually your repo name.
  trailingSlash: false,

  onBrokenLinks: "warn",

  // Add Font Awesome for social media icons
  headTags: [
    {
      tagName: "link",
      attributes: {
        rel: "stylesheet",
        href: "https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.5.1/css/all.min.css",
        integrity:
          "sha512-DTOQO9RWCH3ppGqcWaEA1BIZOC6xxalwEsw9c2QQeAIftl+Vegovlnee1c9QX4TctnWMn13TZye+giMm8e2LwA==",
        crossorigin: "anonymous",
        referrerpolicy: "no-referrer",
      },
    },
    // Google Analytics 4 (GA4) - Configure with environment variable
    // See docs/ANALYTICS/ga4-setup.md for setup instructions
    ...(process.env.GA4_MEASUREMENT_ID
      ? [
          {
            tagName: "script",
            attributes: {
              async: "true",
              src: `https://www.googletagmanager.com/gtag/js?id=${process.env.GA4_MEASUREMENT_ID}`,
            },
          },
          {
            tagName: "script",
            attributes: {},
            innerHTML: `
          window.dataLayer = window.dataLayer || [];
          function gtag(){dataLayer.push(arguments);}
          gtag('js', new Date());
          gtag('config', '${process.env.GA4_MEASUREMENT_ID}', {
            'anonymize_ip': true,
            'allow_google_signals': false,
            'allow_ad_personalization_signals': false
          });
        `,
          },
        ]
      : []),
    // Book Assistant API URL - Configure with environment variable
    // Used by ChatWidget component to connect to the RAG-powered backend
    {
      tagName: "script",
      attributes: {},
      innerHTML: ,
    },
  ],

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },
  
  presets: [
    [
      "classic",
      {
        docs: {
          path: docsPath, // 'docs' (local) or 'docsfs' (from MCP server)
          sidebarPath: "./sidebars.ts",
          // Exclude .summary.md files from being rendered as pages
          // They are injected into lesson frontmatter by the summary injector plugin
          exclude: ["**/*.summary.md"],
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          // beforeDefaultRemarkPlugins run BEFORE Docusaurus's internal plugins
          // This is critical for modifying frontmatter via file.data.frontMatter
          beforeDefaultRemarkPlugins: [
            // Summary injection handled by docusaurus-summaries-plugin (global data approach)
          ],
          remarkPlugins: [
            // Auto-transform Python code blocks into interactive components
            [require('./plugins/remark-interactive-python'), {
              includePaths: ['/05-Python-Fundamentals/'],
              excludeMeta: ['nointeractive', 'static'],
            }],
            // Metadata-driven content enhancements (slides, etc.)
            [require('./plugins/remark-content-enhancements'), {
              enableSlides: true,
              slidesConfig: {
                defaultHeight: 700,
              },
            }],
          ],
        },
        blog: false,
        theme: {
          customCss: "./src/css/custom.css",
        },
        // Sitemap configuration for search engines
        sitemap: {
          changefreq: "weekly",
          priority: 0.5,
          filename: "sitemap.xml",
          ignorePatterns: ["**/tags/**"],
        },
      } satisfies Preset.Options,
    ],
  ],

  plugins: [
    "./plugins/docusaurus-plugin-og-image-generator",
    "./plugins/docusaurus-plugin-structured-data",
    // Summaries Plugin - Makes .summary.md content available via useGlobalData()
    [
      "./plugins/docusaurus-summaries-plugin",
      {
        docsPath: docsPath, // Use same docs path as content-docs
      },
    ],
    // PanaversityFS Plugin - Fetch content from MCP server and write to docsfs/
    // Enable via: PANAVERSITY_PLUGIN_ENABLED=true
    // Server URL: PANAVERSITY_SERVER_URL or http://localhost:8000/mcp
    // When enabled: Writes to docsfs/, Docusaurus reads from docsfs/
    // When disabled: Docusaurus reads from docs/ (local sample content)
    [
      "./plugins/docusaurus-panaversityfs-plugin",
      {
        bookId: "ai-native-dev",
        enabled: panaversityEnabled,
        serverUrl: process.env.PANAVERSITY_SERVER_URL || "http://localhost:8000/mcp",
        docsDir: "docsfs",  // Separate from docs/ to keep local sample content intact
        cleanDocsDir: true, // Clean docsfs/ before writing fresh content
      },
    ],
    function (context, options) {
      return {
        name: "custom-webpack-config",
        configureWebpack(config, isServer, utils) {
          const path = require("path");
          return {
            resolve: {
              alias: {
                "@": path.resolve(__dirname, "src"),
              },
            },
          };
        },
      };
    },
    // Webpack fix for Pyodide compatibility
    // This BannerPlugin adds a global __webpack_require__ stub to prevent runtime errors when Pyodide is loaded from CDN
    function (context, options) {
      return {
        name: "pyodide-webpack-fix",
        configureWebpack(config, isServer, utils) {
          if (isServer) return {};
          return {
            plugins: [
              new (require("webpack").BannerPlugin)({
                banner: `if (typeof __webpack_require__ === 'undefined') {
                  var __webpack_require__ = {};}`,
                raw: true,
                test: /\.js$/,
              }),
            ],
          };
        },
      };
    },
  ],

  themeConfig: {
    // Replace with your project's social card
    image: "img/book-cover-page.png",

    // Open Graph metadata for social media sharing
    metadata: [
      { property: "og:title", content: "Physical AI & Humanoid Robotics" },
      {
        property: "og:description",
        content:
          "Embodied Intelligence from Simulation to Reality — ROS 2, Gazebo, NVIDIA Isaac",
      },
      { property: "og:type", content: "website" },
      {
        property: "og:image",
        content: "https://Rehan-Ul-Haq.github.io/physical-ai-robotics/img/book-cover-page.png",
      },
      { property: "og:image:width", content: "1200" },
      { property: "og:image:height", content: "630" },
      { property: "og:url", content: "https://Rehan-Ul-Haq.github.io/physical-ai-robotics" },
      { name: "twitter:card", content: "summary_large_image" },
      { name: "twitter:title", content: "Physical AI & Humanoid Robotics" },
      {
        name: "twitter:description",
        content:
          "Embodied Intelligence from Simulation to Reality — ROS 2, Gazebo, NVIDIA Isaac",
      },
      {
        name: "twitter:image",
        content: "https://Rehan-Ul-Haq.github.io/physical-ai-robotics/img/book-cover-page.png",
      },
    ],

    colorMode: {
      respectPrefersColorScheme: true,
    },
    docs: {
      sidebar: {
        hideable: true,
      },
    },
    navbar: {
      title: "Physical AI Robotics",
      // logo: {
      //   alt: 'Panaversity Logo',
      //   src: 'img/book-cover.png',
      //   width: 32,
      //   height: 32,
      // },
      hideOnScroll: false,
      items: [
        {
          type: "docSidebar",
          sidebarId: "tutorialSidebar",
          position: "left",
          label: "Book",
        },
        {
          href: "https://github.com/panaversity/ai-native-software-development",
          label: "GitHub",
          position: "right",
        },
      ],
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Learn",
          items: [
            {
              label: "Start Your Journey",
              to: "/docs/preface-agent-native",
            },
            {
              label: "Full Curriculum",
              to: "/docs/preface-agent-native",
            },
            {
              label: "Learning Path",
              to: "/docs/preface-agent-native",
            },
          ],
        },
        {
          title: "Community",
          items: [
            {
              label: "YouTube",
              href: "https://youtube.com/@panaversity",
            },
            {
              label: "LinkedIn",
              href: "https://linkedin.com/company/panaversity",
            },
            {
              label: "Instagram",
              href: "https://instagram.com/panaversity",
            },
            {
              label: "Facebook",
              href: "https://facebook.com/panaversity",
            },
          ],
        },
        {
          title: "Resources",
          items: [
            {
              label: "GitHub Repository",
              href: "https://github.com/panaversity/ai-native-software-development",
            },
            {
              label: "AI Native Specification",
              href: "https://github.com/panaversity/ai-native-software-development/tree/main/specs",
            },
            {
              label: "Example Projects",
              href: "https://github.com/panaversity",
            },
          ],
        },
        {
          title: "About",
          items: [
            {
              label: "Panaversity",
              href: "https://panaversity.org/",
            },
            {
              label: "Our Mission",
              href: "https://panaversity.org/#about",
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} <strong>Panaversity</strong> • AI Native Software Development • Free & Open Source`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
