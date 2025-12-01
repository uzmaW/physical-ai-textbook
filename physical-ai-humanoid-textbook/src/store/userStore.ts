/**
 * Zustand store for user profile and preferences
 * Manages personalization, progress tracking, and UI state
 */

import { create } from 'zustand';
import { persist } from 'zustand/middleware';

export interface UserProfile {
  id?: string;
  email?: string;
  fullName?: string;

  background: {
    hardwareExperience: 'none' | 'beginner' | 'intermediate' | 'advanced';
    softwareExperience: 'none' | 'beginner' | 'intermediate' | 'advanced';
    programmingLanguages: string[];
    roboticsPlatforms: string[];
    hasRobotAccess: boolean;
    robotType?: string;
    familiarWithROS: boolean;
    rosVersion?: string;
  };

  preferences: {
    language: 'en' | 'ur';
    difficulty_preference: 'beginner' | 'intermediate' | 'advanced';
    codeLang: 'python' | 'cpp';
    theme: 'light' | 'dark';
    fontSize: number;
  };

  progress: {
    completedChapters: number[];
    currentChapter: number;
    exerciseScores: Record<string, number>;
    timeSpentMinutes: number;
    lastAccessed?: Date;
  };
}

interface UserState {
  userProfile: UserProfile | null;
  isAuthenticated: boolean;
  setUserProfile: (profile: UserProfile) => void;
  updatePreferences: (preferences: Partial<UserProfile['preferences']>) => void;
  updateProgress: (progress: Partial<UserProfile['progress']>) => void;
  logout: () => void;
}

const defaultProfile: UserProfile = {
  background: {
    hardwareExperience: 'beginner',
    softwareExperience: 'intermediate',
    programmingLanguages: ['python'],
    roboticsPlatforms: [],
    hasRobotAccess: false,
    familiarWithROS: false,
  },
  preferences: {
    language: 'en',
    difficulty_preference: 'intermediate',
    codeLang: 'python',
    theme: 'light',
    fontSize: 16,
  },
  progress: {
    completedChapters: [],
    currentChapter: 1,
    exerciseScores: {},
    timeSpentMinutes: 0,
  },
};

export const useUserStore = create<UserState>()(
  persist(
    (set) => ({
      userProfile: defaultProfile,
      isAuthenticated: false,

      setUserProfile: (profile) =>
        set({ userProfile: profile, isAuthenticated: true }),

      updatePreferences: (preferences) =>
        set((state) => ({
          userProfile: state.userProfile
            ? {
                ...state.userProfile,
                preferences: {
                  ...state.userProfile.preferences,
                  ...preferences,
                },
              }
            : null,
        })),

      updateProgress: (progress) =>
        set((state) => ({
          userProfile: state.userProfile
            ? {
                ...state.userProfile,
                progress: {
                  ...state.userProfile.progress,
                  ...progress,
                  lastAccessed: new Date(),
                },
              }
            : null,
        })),

      logout: () =>
        set({ userProfile: null, isAuthenticated: false }),
    }),
    {
      name: 'humanoid-ai-user-storage',
    }
  )
);
