export interface DocPage {
  id: string;
  title: string;
  content: string; // Markdown content
}

export interface DocSection {
  title: string;
  pages: DocPage[];
}

export interface ChatMessage {
  id: string;
  role: 'user' | 'model';
  text: string;
  isStreaming?: boolean;
  timestamp: number;
}

export interface ChatState {
  messages: ChatMessage[];
  isLoading: boolean;
}
