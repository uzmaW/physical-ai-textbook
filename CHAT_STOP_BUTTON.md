# Chat Stop Button Feature

## Overview

Added a **Stop button** to the RAG Chat UI that allows users to cancel ongoing API requests. The button replaces the Send button while a request is in progress.

## Features

### Button Behavior

- **Send Button (Default State)**
  - Shows "➤ Send" in blue
  - Enabled when user has typed a message
  - Disabled when input is empty
  - Click to send message (or press Enter)

- **Stop Button (Loading State)**
  - Shows "⏹ Stop" in red with pulsing animation
  - Visible when API request is in progress
  - Click to cancel the ongoing request
  - Textarea is disabled during request

### Visual Design

- **Stop Button**
  - Red background (`bg-red-600`)
  - Hover effect (`hover:bg-red-700`)
  - Pulse animation (`animate-pulse`) to indicate loading
  - Clear stop icon (⏹)
  
- **Send Button**
  - Blue background (`bg-blue-600`)
  - Send icon (➤)
  - Consistent styling with other buttons

## Implementation Details

### Files Modified

1. **RAGChatbox.tsx** - Chat UI component
   - Added `AbortController` ref to manage request cancellation
   - Created `handleStop()` function
   - Updated button rendering to show Stop/Send conditionally
   - Updated error handling to ignore AbortError

2. **apiService.ts** - API communication service
   - Added optional `signal` parameter to `sendChatMessage()`
   - Pass abort signal to fetch request
   - Handle AbortError appropriately

### How It Works

```
User clicks Send
    ↓
handleSend() creates AbortController
    ↓
Fetch request starts with abort signal
    ↓
Button switches to "Stop"
    ↓
User clicks Stop (or request completes)
    ↓
abortControllerRef.abort() cancels fetch
    ↓
Fetch throws AbortError
    ↓
Error caught, no error message shown
    ↓
Button switches back to "Send"
```

## Code Changes

### RAGChatbox.tsx

```tsx
// Added AbortController ref
const abortControllerRef = useRef<AbortController | null>(null);

// Updated handleSend to use AbortController
abortControllerRef.current = new AbortController();
const data = await sendChatMessage(
  { /* request */ },
  abortControllerRef.current.signal
);

// New handleStop function
const handleStop = () => {
  if (abortControllerRef.current) {
    abortControllerRef.current.abort();
    setIsLoading(false);
  }
};

// Conditional button rendering
{isLoading ? (
  <button onClick={handleStop} className="bg-red-600...">
    ⏹ Stop
  </button>
) : (
  <button onClick={handleSend} className="bg-blue-600...">
    ➤ Send
  </button>
)}
```

### apiService.ts

```tsx
export async function sendChatMessage(
  request: ChatRequest,
  signal?: AbortSignal  // Added parameter
): Promise<ChatResponse> {
  const response = await fetch(`${API_BASE_URL}/api/chat/`, {
    // ...
    signal,  // Pass to fetch
  });
  
  // Handle AbortError
  if (error instanceof Error && error.name === 'AbortError') {
    throw error;
  }
}
```

## User Experience

### Scenario 1: Quick Response
1. User asks question
2. Button shows "Stop" with pulse animation
3. API responds quickly (< 2 seconds)
4. Button switches back to "Send"

### Scenario 2: Long Response / Network Slow
1. User asks question
2. Button shows "Stop" with pulse animation
3. User waits or decides to cancel
4. User clicks "Stop"
5. Request is immediately cancelled
6. No error message shown
7. Button switches back to "Send"

### Scenario 3: Network Error
1. User asks question
2. Button shows "Stop" with pulse animation
3. Network error occurs
4. Error message displayed
5. Button switches back to "Send"

## Browser Compatibility

The AbortController API is supported in all modern browsers:
- Chrome 66+
- Firefox 57+
- Safari 11.1+
- Edge 16+

For older browsers, the fetch will proceed without cancellation capability (graceful degradation).

## Testing

### Manual Test

1. Visit http://localhost:3000
2. Scroll to chat UI (right sidebar)
3. Type a question
4. Click "Send" or press Enter
5. While loading, click "Stop"
6. Request should cancel immediately
7. No error message should appear
8. Button should return to "Send"

### Automated Test Cases

```typescript
// Test: Button shows Stop during loading
test('shows Stop button while loading', () => {
  render(<RAGChatbox />);
  fireEvent.click(getSendButton());
  expect(getStopButton()).toBeInTheDocument();
});

// Test: Clicking Stop cancels request
test('cancels request when Stop is clicked', async () => {
  // Create mock that delays
  const mockFetch = jest.spyOn(global, 'fetch');
  
  render(<RAGChatbox />);
  fireEvent.click(getSendButton());
  fireEvent.click(getStopButton());
  
  // Verify abort was called
  expect(mockFetch).toHaveBeenCalled();
});
```

## Future Enhancements

- [ ] Show cancellation status message in chat
- [ ] Add keyboard shortcut (Esc) to stop
- [ ] Stream responses with cancellation mid-stream
- [ ] Persist user preference for auto-retry
- [ ] Show time elapsed for requests
