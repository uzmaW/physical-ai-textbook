# Debug Chat Width Issues

## How to Debug in Browser

1. **Open DevTools** (F12)
2. **Inspect the chat container**
3. **Check these elements**:

### Elements to Inspect

```
.theme-doc-toc-desktop
  └── div (direct child)
      └── RAGChatWidget component
          └── .chatContainer
              ├── .chatHeader
              ├── .messagesContainer
              └── .inputContainer
```

### What to Check

#### 1. Computed Width
- Right-click chat → Inspect
- Look at "Computed" tab
- Check `width` value
- Should be: `100%` or full pixel width of container

#### 2. Box Model
- In DevTools, look at the box model diagram
- Check if there's unexpected:
  - Padding
  - Margin
  - Border
  - Max-width constraint

#### 3. Parent Constraints
- Click on parent elements going up
- Check each for:
  - `max-width` values
  - `width` constraints
  - `flex` properties
  - `grid` properties

#### 4. Overriding Styles
- In "Styles" tab, look for:
  - Crossed-out styles (overridden)
  - `!important` rules
  - Inline styles
  - Higher specificity rules

## Common Issues

### Issue 1: Docusaurus Default TOC Width
**Symptom**: Chat is 300px wide
**Cause**: `.col--3` class has default width
**Fix**: Override with `width: 100% !important`

### Issue 2: Flex Shrinking
**Symptom**: Chat shrinks below content size
**Cause**: `flex-shrink: 1` on parent
**Fix**: Add `flex-shrink: 0` and `min-width: 0`

### Issue 3: Max-Width Constraint
**Symptom**: Chat stops growing at certain width
**Cause**: `max-width` on parent or container
**Fix**: Override with `max-width: none !important`

### Issue 4: Grid Column Width
**Symptom**: Chat column has fixed width
**Cause**: CSS Grid with fixed column size
**Fix**: Change to `1fr` or `auto`

## CSS Applied

### In custom.css
```css
.theme-doc-toc-desktop {
  width: 100% !important;
  max-width: none !important;
  flex: 1 1 auto !important;
}

.col--3 {
  width: 100% !important;
  max-width: none !important;
}
```

### In ChatUI.module.css
```css
.chatContainer {
  width: 100% !important;
  max-width: none !important;
  min-width: 0 !important;
}
```

## Testing Steps

1. **Clear cache**: `./clear-cache.sh`
2. **Start server**: `npm start`
3. **Open browser DevTools**
4. **Inspect chat container**
5. **Check computed width**
6. **Verify no constraints**

## Expected Result

- Chat should take full width of right column
- No horizontal scrolling
- Messages wrap properly
- No truncation

## If Still Truncated

### Check Browser Console
Look for:
- CSS errors
- Failed to load stylesheets
- Conflicting styles

### Check Network Tab
Verify:
- `custom.css` loaded
- `ChatUI.module.css` loaded
- No 404 errors

### Check Elements Tab
Find:
- `.theme-doc-toc-desktop` element
- Check its computed width
- Check parent container width
- Trace up to find constraint

### Manual Override Test
In DevTools Console, run:
```javascript
document.querySelector('.theme-doc-toc-desktop').style.width = '100%';
document.querySelector('.theme-doc-toc-desktop').style.maxWidth = 'none';
```

If this fixes it, the CSS isn't being applied correctly.

## Nuclear Option

If nothing works, add to `custom.css`:
```css
* {
  max-width: none !important;
}

.theme-doc-toc-desktop,
.theme-doc-toc-desktop * {
  width: 100% !important;
  max-width: none !important;
}
```

This will override everything (not recommended for production).
