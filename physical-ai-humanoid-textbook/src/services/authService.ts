/**
 * Authentication Service
 * Handles login, logout, register, and token management
 */

import { API_BASE_URL } from '../config/env';

const TOKEN_KEY = 'auth_token';
const USER_KEY = 'user_info';

export interface LoginCredentials {
  email: string;
  password: string;
}

export interface RegisterData {
  email: string;
  password: string;
  name: string;
}

export interface User {
  email: string;
  name: string;
  created_at: string;
}

export interface AuthResponse {
  access_token: string;
  token_type: string;
  user: User;
}

/**
 * Register a new user
 */
export async function register(data: RegisterData): Promise<AuthResponse> {
  const response = await fetch(`${API_BASE_URL}/api/auth/register`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(data),
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail || 'Registration failed');
  }

  const authData: AuthResponse = await response.json();

  // Store token and user info
  localStorage.setItem(TOKEN_KEY, authData.access_token);
  localStorage.setItem(USER_KEY, JSON.stringify(authData.user));

  return authData;
}

/**
 * Login with email and password
 */
export async function login(credentials: LoginCredentials): Promise<AuthResponse> {
  // OAuth2PasswordRequestForm expects form data
  const formData = new FormData();
  formData.append('username', credentials.email);  // OAuth2 uses 'username' field
  formData.append('password', credentials.password);

  const response = await fetch(`${API_BASE_URL}/api/auth/login`, {
    method: 'POST',
    body: formData,
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail || 'Login failed');
  }

  const authData: AuthResponse = await response.json();

  // Store token and user info
  localStorage.setItem(TOKEN_KEY, authData.access_token);
  localStorage.setItem(USER_KEY, JSON.stringify(authData.user));

  return authData;
}

/**
 * Logout user
 */
export async function logout(): Promise<void> {
  const token = getToken();

  if (token) {
    try {
      await fetch(`${API_BASE_URL}/api/auth/logout`, {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });
    } catch (error) {
      console.error('Logout request failed:', error);
    }
  }

  // Clear local storage
  localStorage.removeItem(TOKEN_KEY);
  localStorage.removeItem(USER_KEY);
}

/**
 * Get current user info
 */
export async function getCurrentUser(): Promise<User | null> {
  const token = getToken();

  if (!token) {
    return null;
  }

  try {
    const response = await fetch(`${API_BASE_URL}/api/auth/me`, {
      headers: {
        'Authorization': `Bearer ${token}`,
      },
    });

    if (!response.ok) {
      // Token expired or invalid
      logout();
      return null;
    }

    const user: User = await response.json();
    return user;
  } catch (error) {
    console.error('Failed to get current user:', error);
    return null;
  }
}

/**
 * Get stored token
 */
export function getToken(): string | null {
  if (typeof window === 'undefined') return null;
  return localStorage.getItem(TOKEN_KEY);
}

/**
 * Get stored user info
 */
export function getStoredUser(): User | null {
  if (typeof window === 'undefined') return null;

  const userJson = localStorage.getItem(USER_KEY);
  if (!userJson) return null;

  try {
    return JSON.parse(userJson);
  } catch {
    return null;
  }
}

/**
 * Check if user is authenticated
 */
export function isAuthenticated(): boolean {
  return getToken() !== null;
}

/**
 * Refresh access token
 */
export async function refreshToken(): Promise<AuthResponse> {
  const token = getToken();

  if (!token) {
    throw new Error('No token to refresh');
  }

  const response = await fetch(`${API_BASE_URL}/api/auth/refresh`, {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${token}`,
    },
  });

  if (!response.ok) {
    logout();
    throw new Error('Token refresh failed');
  }

  const authData: AuthResponse = await response.json();

  // Update stored token and user
  localStorage.setItem(TOKEN_KEY, authData.access_token);
  localStorage.setItem(USER_KEY, JSON.stringify(authData.user));

  return authData;
}
