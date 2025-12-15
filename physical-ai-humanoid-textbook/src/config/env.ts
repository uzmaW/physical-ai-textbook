// Centralized environment configuration for API base URL
// Resolved at build time via webpack DefinePlugin using process.env.* mappings
// See docusaurus.config.js configureWebpack DefinePlugin for injected variables

export const API_BASE_URL =
  (process.env.REACT_APP_API_URL ||
    process.env.NEXT_PUBLIC_API_URL ||
    process.env.VITE_API_URL ||
    process.env.API_BASE_URL );
