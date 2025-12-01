#!/usr/bin/env node

/**
 * Development server with API proxy
 * Runs Docusaurus dev server and proxies /api requests to backend
 */

const { spawn } = require('child_process');
const express = require('express');
const { createProxyMiddleware } = require('http-proxy-middleware');
const http = require('http');

const DOCUSAURUS_PORT = 3001;
const SERVER_PORT = 3000;
const BACKEND_URL = 'http://localhost:8000';

// Start Docusaurus dev server on port 3001
const docusaurus = spawn('npm', ['exec', 'docusaurus', 'start', '--', '--port', DOCUSAURUS_PORT], {
  cwd: './physical-ai-humanoid-textbook',
  stdio: 'inherit',
});

docusaurus.on('error', (err) => {
  console.error('Failed to start Docusaurus:', err);
  process.exit(1);
});

// Wait a bit for Docusaurus to start
setTimeout(() => {
  const app = express();

  // Proxy /api requests to backend
  app.use('/api', createProxyMiddleware({
    target: BACKEND_URL,
    changeOrigin: true,
    logLevel: 'info',
    onProxyReq: (proxyReq, req, res) => {
      console.log(`🔄 [API Proxy] ${req.method} ${req.url} → ${BACKEND_URL}${req.url}`);
    },
    onError: (err, req, res) => {
      console.error(`❌ [API Proxy Error] ${req.method} ${req.url}:`, err.message);
      res.status(503).json({ error: 'Backend unavailable' });
    },
  }));

  // Proxy all other requests to Docusaurus dev server
  app.use(createProxyMiddleware({
    target: `http://localhost:${DOCUSAURUS_PORT}`,
    changeOrigin: true,
    ws: true,
  }));

  const server = app.listen(SERVER_PORT, () => {
    console.log(`\n✅ Dev server running on http://localhost:${SERVER_PORT}`);
    console.log(`📚 Docusaurus on http://localhost:${DOCUSAURUS_PORT} (proxied)`);
    console.log(`📡 Backend API: ${BACKEND_URL}`);
    console.log(`🔗 All /api requests proxied to backend\n`);
  });

  server.on('error', (err) => {
    console.error('Server error:', err);
    process.exit(1);
  });
}, 3000);

process.on('exit', () => {
  docusaurus.kill();
});
