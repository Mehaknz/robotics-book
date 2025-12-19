// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics', // From original
  tagline: 'From Digital Code to Physical Action', // From original
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://Hira-Ali.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/hack_yt/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Hira-Ali', // Usually your GitHub org/user name.
  projectName: 'hack_yt', // Usually your repo name.
  deploymentBranch: 'gh-pages',

  onBrokenLinks: 'throw',
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'), // Use require.resolve
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Hira-Ali/hack_yt/tree/main/humanoid-robotics-book/frontend/',
        },
        blog: false, // Original blog was false
        theme: {
          customCss: require.resolve('./src/css/custom.css'), // Use require.resolve
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Humanoid Robotics', // From original
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            type: 'html',
            position: 'right',
            value: '<a href="#" class="navbar-link" onclick="document.dispatchEvent(new CustomEvent(\'openSignInModal\')); return false;">Sign In</a>',
          },
          {
            type: 'html',
            position: 'right',
            value: '<a href="#" class="navbar-link" onclick="document.dispatchEvent(new CustomEvent(\'openSignUpModal\')); return false;">Sign Up</a>',
          },
        ],
      },
      footer: {},
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
    }),
};

module.exports = config;

