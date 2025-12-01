# Quiz Maker - Auto MCQ Generator

Auto-generate multiple-choice questions from textbook chapters.

## Capabilities

- Read chapter MDX content
- Extract key concepts, code, equations
- Generate MCQs with plausible distractors
- Create questions at 3 difficulty levels
- Output JSON and/or Markdown format

## Usage

`/sp.quiz-maker week-11`

## Output

JSON file with 10 MCQs:
```json
{
  "chapterId": "week-11",
  "chapterTitle": "Humanoid Kinematics & Locomotion",
  "questions": [
    {
      "id": 1,
      "difficulty": "beginner",
      "question": "What does ZMP stand for?",
      "options": [
        "Zero-Moment Point",
        "Zero-Motion Point",
        "Zonal Movement Parameter",
        "Zero-Mass Point"
      ],
      "correct": 0,
      "explanation": "ZMP (Zero-Moment Point) is a stability criterion..."
    }
  ]
}
```

Optional: Markdown quiz for embedding in chapter

## Quality

- ✅ Clear, unambiguous questions
- ✅ Plausible distractors (not obviously wrong)
- ✅ Explanations cite textbook content
- ✅ Mix of conceptual and applied questions
- ✅ Code-based questions for chapters with code
