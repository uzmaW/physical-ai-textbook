/**
 * API Service for backend communication
 * Handles all API calls to the FastAPI backend
 */

// Get API URL from environment or use default for development
// In Docusaurus, we need to check if we're in browser environment
const API_BASE_URL = typeof window !== 'undefined'
  ? (window as any).REACT_APP_API_URL || 'http://localhost:8000'
  : 'http://localhost:8000';

export interface ChatRequest {
  message: string;
  conversationId?: string;
  userEmail?: string;
  selectedText?: string;
  userLevel?: string;
  language?: string;
}

export interface Citation {
  chapter: string;
  url: string;
  relevance?: number;
}

export interface ChatResponse {
  answer: string;
  citations: Citation[];
  sourcesCount: number;
  model: string;
  conversation_id: string;
}

/**
 * Send a chat message to the RAG-powered backend
 * @param request - Chat request object
 * @param signal - Optional AbortSignal to cancel the request
 */
export async function sendChatMessage(request: ChatRequest, signal?: AbortSignal): Promise<ChatResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/chat/`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        message: request.message,
        conversation_id: request.conversationId || null,
        user_email: request.userEmail || null,
        selectedText: request.selectedText || null,
        userLevel: request.userLevel || 'intermediate',
        language: request.language || 'en',
      }),
      signal, // Pass abort signal to fetch
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `API request failed with status ${response.status}`);
    }

    const data: ChatResponse = await response.json();
    return data;
  } catch (error) {
    console.error('API Error:', error);

    // Don't show error for aborted requests
    if (error instanceof Error && error.name === 'AbortError') {
      throw error;
    }

    // Return a fallback response if API is not available
    if (error instanceof TypeError && error.message.includes('Failed to fetch')) {
      return {
        answer: `⚠️ **Backend API is not available**\n\nThe chat service is currently unavailable. This could mean:\n\n1. The backend server is not running\n2. The API URL is not configured correctly\n3. There's a network connectivity issue\n\n**For Development:**\n- Start the backend: \`cd backend && uvicorn app.main:app --reload\`\n- Check that the API is running at: ${API_BASE_URL}\n\n**For Production:**\n- Ensure the REACT_APP_API_URL environment variable is set\n- Verify the backend is deployed and accessible\n\nIn the meantime, I'm here but unable to access the textbook content to provide specific answers.`,
        citations: [],
        sourcesCount: 0,
        model: 'fallback',
      };
    }

    throw error;
  }
}

/**
 * Check if the API is available
 */
export async function checkApiHealth(): Promise<boolean> {
  try {
    const response = await fetch(`${API_BASE_URL}/health`, {
      method: 'GET',
    });
    return response.ok;
  } catch {
    return false;
  }
}

/**
 * Get the configured API base URL
 */
export function getApiBaseUrl(): string {
  return API_BASE_URL;
}
