import { GoogleGenAI, Chat, GenerateContentResponse } from "@google/genai";

let chatSession: Chat | null = null;
let genAI: GoogleGenAI | null = null;

const getClient = () => {
  if (!genAI) {
    const apiKey = process.env.API_KEY;
    if (!apiKey) {
      console.error("API_KEY is missing from environment variables");
      // In a real app, handle this gracefully. For now, we return null and checks will fail.
      return null;
    }
    genAI = new GoogleGenAI({ apiKey });
  }
  return genAI;
};

export const initChatSession = (systemInstruction: string) => {
  const client = getClient();
  if (!client) return null;

  chatSession = client.chats.create({
    model: 'gemini-2.5-flash',
    config: {
      systemInstruction: systemInstruction,
    },
  });
  return chatSession;
};

export const streamChatMessage = async function* (
  message: string,
  currentContext: string
): AsyncGenerator<string, void, unknown> {
  const client = getClient();
  if (!client) {
    yield "Error: API Key not found.";
    return;
  }

  // If session doesn't exist or we want to inject context dynamically, 
  // we might just start a new chat or append context to the message.
  // For this "Book" layout, we want the chat to persist, but aware of the current page.
  // Strategy: Prepend context to the user message invisibly if it's a specific question about the page.
  // A simple robust way for a persistent sidebar is to just maintain one session.
  
  if (!chatSession) {
    initChatSession(`You are a helpful documentation assistant called 'DocuBot'. 
    You are displayed in a right-sidebar of a documentation website.
    Always be concise, elegant, and helpful. 
    Format your responses with Markdown.`);
  }

  if (!chatSession) return;

  // We append a bit of context prompt invisibly to help the model know what the user is looking at.
  const contextAwareMessage = `[User is currently viewing the page content below]
---
${currentContext.substring(0, 2000)}... (truncated)
---
[User Question]: ${message}`;

  try {
    const streamResult = await chatSession.sendMessageStream({ message: contextAwareMessage });

    for await (const chunk of streamResult) {
      const responseChunk = chunk as GenerateContentResponse;
      if (responseChunk.text) {
        yield responseChunk.text;
      }
    }
  } catch (error) {
    console.error("Gemini Chat Error:", error);
    yield "I encountered an error while processing your request. Please check your network or API key.";
  }
};
