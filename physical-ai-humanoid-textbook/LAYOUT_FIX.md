# Layout Fix - 3-Column Layout

## Problem
Chatbox was appearing below the main content instead of in the right sidebar.

## Root Cause
Docusaurus default responsive behavior was stacking columns vertically on certain screen sizes.

## Solution
Force 3-column layout with explicit flex properties.

## Layout Structure

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                   │
│  ┌──────────┐  ┌─────────────────────────┐  ┌──────────────┐   │
│  │          │  │                         │  │              │   │
│  │  Left    │  │      Main Content       │  │   Chat       │   │
│  │ Sidebar  │  │      (Flexible)         │  │  (Right)     │   │
│  │ (280px)  │  │                         │  │  (400px)     │   │
│  │          │  │                         │  │              │   │
│  └──────────┘  └─────────────────────────┘  └──────────────┘   │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

## CSS Applied

### Force Flex Layout
```css
.docPage {
  display: flex !important;
  flex-direction: row !important;
  max-width: 100vw !important;
}
```

### Column Widths

#### Left Sidebar (Fixed)
```css
.docPage > .col:first-child {
  flex: 0 0 280px !important;
  max-width: 280px !important;
}
```

#### Main Content (Flexible)
```css
.docPage > .col:nth-child(2) {
  flex: 1 1 auto !important;
  max-width: none !important;
  min-width: 0 !important;
}
```

#### Right Chat (Fixed)
```css
.docPage > .col:last-child {
  flex: 0 0 400px !important;
  max-width: 400px !important;
  min-width: 400px !important;
}
```

## Responsive Breakpoints

### Desktop (> 1400px)
- Sidebar: 280px
- Content: Flexible
- Chat: 400px

### Medium Desktop (997px - 1400px)
- Sidebar: 280px
- Content: Flexible
- Chat: 350px

### Small Desktop (997px - 1200px)
- Sidebar: 240px
- Content: Flexible
- Chat: 320px

### Mobile (< 996px)
- Stack vertically
- All columns: 100% width
- Chat appears below content

## Key Properties

### Flex Properties
- `flex: 0 0 <width>` - Don't grow, don't shrink, fixed width
- `flex: 1 1 auto` - Grow to fill space, shrink if needed
- `max-width: none` - Remove width constraints
- `min-width: 0` - Allow shrinking below content size

### Display Properties
- `display: flex !important` - Force flexbox layout
- `flex-direction: row !important` - Horizontal layout
- `!important` - Override Docusaurus defaults

## When Sidebar Collapses

```css
body.sidebar-collapsed .docPage > .col:first-child {
  display: none !important;
}
```

Layout becomes:
```
┌─────────────────────────────────────────────────────────────────┐
│  ┌─────────────────────────────────────┐  ┌──────────────┐     │
│  │                                     │  │              │     │
│  │      Main Content (Expanded)        │  │   Chat       │     │
│  │                                     │  │  (400px)     │     │
│  │                                     │  │              │     │
│  └─────────────────────────────────────┘  └──────────────┘     │
└─────────────────────────────────────────────────────────────────┘
```

## Testing

1. **Desktop (> 1400px)**
   - ✅ 3 columns visible
   - ✅ Chat on right side
   - ✅ Content in center

2. **Medium (997px - 1400px)**
   - ✅ 3 columns visible
   - ✅ Narrower chat
   - ✅ Content adjusts

3. **Mobile (< 996px)**
   - ✅ Stacked vertically
   - ✅ Chat below content
   - ✅ Full width

4. **Sidebar Collapsed**
   - ✅ Sidebar hidden
   - ✅ Content expands
   - ✅ Chat stays on right

## Files Modified

1. `src/css/custom.css` - Layout CSS
2. All changes use `!important` to override Docusaurus defaults

## Clear Cache

After applying:
```bash
cd physical-ai-humanoid-textbook
./clear-cache.sh
npm start
```

## Verify

1. Open browser
2. Check chat is on right side
3. Resize window to test responsive
4. Collapse sidebar to test layout adjustment
