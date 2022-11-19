module.exports = {
  apps: [
    {
      name: 'smart-home',
      script: 'npx next start -p 5163',
      instances: 1,
      autorestart: true,
      watch: false,
      max_memory_restart: '180M',
      env: {
        NODE_ENV: 'production',
      },
    },
  ],
}
