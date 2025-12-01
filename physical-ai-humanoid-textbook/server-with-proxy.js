/**
 * Express server with API proxy
 * Proxies /api/* requests to backend on localhost:8000
 */
const express = require('express');
const httpProxy = require('http-proxy');
const path = require('path');

const app = express();
const PORT = process.env.PORT || 3000;
const BACKEND_URL = process.env.BACKEND_URL || 'http://localhost:8000';

// Create proxy for API requests
const proxy = httpProxy.createProxyServer({
  target: BACKEND_URL,
  changeOrigin: true,
});

proxy.on('error', (err, req, res) => {
  console.error(`❌ [API Proxy Error] ${req.method} ${req.url}:`, err.message);
  res.writeHead(503, { 'Content-Type': 'application/json' });
  res.end(JSON.stringify({ error: 'Backend service unavailable', details: err.message }));
});

proxy.on('proxyReq', (proxyReq, req, res) => {
  console.log(`🔄 [API] ${req.method} ${req.url}`);
});

// Proxy all /api requests to the backend
app.all('/api/*', (req, res) => {
  proxy.web(req, res);
});

// Serve static files from build directory
const buildPath = path.join(__dirname, 'build');
app.use(express.static(buildPath));

// Fallback to index.html for SPA routing
app.get('*', (req, res) => {
  res.sendFile(path.join(buildPath, 'index.html'));
});

app.listen(PORT, () => {
  console.log(`\n✅ Frontend server: http://localhost:${PORT}`);
  console.log(`📡 API proxy: /api → ${BACKEND_URL}\n`);
});
