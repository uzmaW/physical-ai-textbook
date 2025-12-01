/**
 * Chat Service for streaming responses
 * Handles conversation management and message streaming
 */

const API_BASE_URL = typeof window !== 'undefined'
  ? (window as any).REACT_APP_API_URL || 'http://localhost:8000'
  : 'http://localhost:8000';

export interface ChatMessage {
  message: string;
  conversation_id?: string;
  context_filters?: any;
  max_context_chunks?: number;
  include_citations?: boolean;
}

/**
 * Send message and get streaming response
 */
export const chatService = {
  async *sendMessage(request: ChatMessage) {
    try {
      // Create abort controller for timeout
      const controller = new AbortController();
      const timeout = setTimeout(() => controller.abort(), 30000); // 30 second timeout

      const response = await fetch(`${API_BASE_URL}/api/chat/`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: request.message,
          userLevel: 'intermediate',
          language: 'en',
        }),
        signal: controller.signal,
      });

      clearTimeout(timeout);

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();

      // Yield metadata first
      yield `__METADATA__${JSON.stringify({
        conversation_id: data.conversation_id || 'default',
        model: data.model || 'gpt-4o-mini',
      })}`;

      // Yield the answer content
      yield data.answer || 'No response';
    } catch (error) {
      console.error('Chat service error:', error);
      yield 'Error: Could not get response from backend. Please check your connection.';
    }
  },

  async getConversations() {
    try {
      const response = await fetch(`${API_BASE_URL}/api/conversations/user/anonymous`);
      if (!response.ok) return [];
      return await response.json();
    } catch {
      return [];
    }
  },

  async getConversation(conversationId: string) {
    try {
      const response = await fetch(`${API_BASE_URL}/api/conversations/${conversationId}`);
      if (!response.ok) return [];
      const data = await response.json();
      return data.messages || [];
    } catch {
      return [];
    }
  },

  async deleteConversation(conversationId: string) {
    try {
      await fetch(`${API_BASE_URL}/api/conversations/${conversationId}`, {
        method: 'DELETE',
      });
    } catch (error) {
      console.error('Delete failed:', error);
    }
  },

  newConversation() {
    // Just reset local state - no API call needed
    return 'new-' + Date.now();
  },
};
