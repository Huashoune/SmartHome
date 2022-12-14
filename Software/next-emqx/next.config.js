const withPlugins = require('next-compose-plugins')
const WindiCSSWebpackPlugin = require('windicss-webpack-plugin')

const plugins = []
if (process.env.ANALYZE === 'true') {
  plugins.push([require('@next/bundle-analyzer')({ enabled: true })])
}

const configs = withPlugins(plugins, {
  swcMinify: true,
  experimental: {
    scrollRestoration: true,
    legacyBrowsers: false,
    browsersListForSwc: true,
    newNextLinkBehavior: true,
  },
  webpack: (config, options) => {
    config.plugins.push(new WindiCSSWebpackPlugin())
    return config
  },
  eslint: {
    ignoreDuringBuilds: true,
  }
})

module.exports = configs
