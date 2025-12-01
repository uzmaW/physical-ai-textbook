# PersonalizedChapter Component - Not a Duplicate!

## What You're Seeing

The bar at the top of Week-01 is **NOT a duplicate** - it's the PersonalizedChapter feature bar.

## Visual Layout

```
┌─────────────────────────────────────────────────────────────┐
│  [Beginner]                    🎯 Personalize    ترجمہ      │  ← PersonalizedChapter Bar
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  # Foundations of Physical AI & Embodied Intelligence        │
│                                                               │
│  ## Learning Outcomes                                        │
│  ...                                                          │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

## Purpose

This bar provides:

1. **Difficulty Badge** - Shows current difficulty level
   - Beginner (Green)
   - Intermediate (Yellow)
   - Advanced (Red)

2. **Personalize Button** - Cycles through difficulty levels
   - Click to change: Beginner → Intermediate → Advanced → Beginner

3. **Translate Button** - Translates content to Urdu
   - Shows "ترجمہ" (Urdu for "Translate")
   - Click to translate chapter to Urdu
   - Click again to switch back to English

## Styling

The bar is styled to match the chat UI:
- Flat design with subtle background
- Light mode: `#f9fafb` background
- Dark mode: `#111827` background
- Buttons use soft color backgrounds (blue for personalize, green for translate)
- No shadows or borders (except bottom border)

## Why Only in Week-01?

The `<PersonalizedChapter>` component is only wrapped around Week-01 content. Other weeks don't have this feature yet.

To add it to other weeks, wrap the content:

```mdx
import {PersonalizedChapter} from '@site/src/components/PersonalizedChapter';

<PersonalizedChapter id="week-02">

# Your Chapter Content Here

</PersonalizedChapter>
```

## CSS Module

Styles are in: `src/components/PersonalizedChapter.module.css`

Key classes:
- `.chapterHeader` - The top bar
- `.difficultyBadge` - The difficulty level badge
- `.personalizeButton` - The personalize button
- `.translateButton` - The translate button

## Not a Bug!

This is a **feature**, not a duplicate or bug. It's intentionally designed to:
- Show at the top of personalized chapters
- Match the chat UI design
- Provide quick access to personalization and translation

## To Remove (if desired)

If you don't want this bar, simply remove the `<PersonalizedChapter>` wrapper from week-01.mdx:

```mdx
// Remove these lines:
<PersonalizedChapter id="week-01">
...
</PersonalizedChapter>

// Keep just the content:
# Foundations of Physical AI & Embodied Intelligence
...
```
