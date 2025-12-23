module.exports = {
  title: 'Physical AI Book',
  tagline: 'Learn Physical AI step by step',
  url: 'https://your-site-url.com',
  baseUrl: '/',
  favicon: 'img/favicon.ico',
  organizationName: 'muhammadhamza78',
  projectName: 'physical-ai-book',
  onBrokenLinks: 'ignore',
  onBrokenMarkdownLinks: 'ignore',
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          routeBasePath: '/docs',
          sidebarPath: require.resolve('./sidebars.js'),
        },
        blog: { showReadingTime: true },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],
};
