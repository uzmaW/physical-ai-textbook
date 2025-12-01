/**
 * Multilingual Controls Component
 * Language switching, translation, and audio controls for educational content
 * Uses free-tier models: Helsinki-NLP, OpenVoice, Coqui TTS
 */

import React, { useState, useEffect } from 'react';
import { 
  Languages, 
  Volume2, 
  VolumeX, 
  Play, 
  Pause, 
  RotateCcw,
  Settings,
  Globe,
  Loader2
} from 'lucide-react';
import clsx from 'clsx';
import styles from './MultilingualControls.module.css';

export interface Language {
  code: string;
  name: string;
  nativeName: string;
  rtl: boolean;
}

export interface TranslationStatus {
  isTranslated: boolean;
  isTranslating: boolean;
  quality: 'high' | 'medium' | 'low' | null;
  lastUpdated: string | null;
}

export interface AudioStatus {
  isPlaying: boolean;
  isLoading: boolean;
  currentPosition: number;
  duration: number;
  playbackRate: number;
}

interface MultilingualControlsProps {
  currentLanguage: string;
  availableLanguages: Language[];
  translationStatus: TranslationStatus;
  audioStatus: AudioStatus;
  chapterId: string;
  onLanguageChange: (languageCode: string) => Promise<void>;
  onTranslate: (targetLanguage: string) => Promise<void>;
  onAudioToggle: () => void;
  onAudioSeek: (position: number) => void;
  onPlaybackRateChange: (rate: number) => void;
  className?: string;
}

export const MultilingualControls: React.FC<MultilingualControlsProps> = ({
  currentLanguage,
  availableLanguages,
  translationStatus,
  audioStatus,
  chapterId,
  onLanguageChange,
  onTranslate,
  onAudioToggle,
  onAudioSeek,
  onPlaybackRateChange,
  className
}) => {
  const [showLanguageMenu, setShowLanguageMenu] = useState(false);
  const [showAudioControls, setShowAudioControls] = useState(false);
  const [audioError, setAudioError] = useState<string | null>(null);

  const currentLang = availableLanguages.find(lang => lang.code === currentLanguage);
  const isUrdu = currentLanguage === 'ur';

  // Handle language switching
  const handleLanguageSwitch = async (languageCode: string) => {
    if (languageCode === currentLanguage) return;
    
    try {
      await onLanguageChange(languageCode);
      setShowLanguageMenu(false);
      
      // If switching to Urdu and not translated, offer translation
      if (languageCode === 'ur' && !translationStatus.isTranslated) {
        await onTranslate('ur');
      }
    } catch (error) {
      console.error('Language switch failed:', error);
    }
  };

  // Handle manual translation trigger
  const handleTranslate = async () => {
    try {
      await onTranslate(currentLanguage);
    } catch (error) {
      console.error('Translation failed:', error);
    }
  };

  // Audio progress formatting
  const formatTime = (seconds: number): string => {
    const mins = Math.floor(seconds / 60);
    const secs = Math.floor(seconds % 60);
    return `${mins}:${secs.toString().padStart(2, '0')}`;
  };

  // Audio error handling
  const handleAudioError = (error: string) => {
    setAudioError(error);
    setTimeout(() => setAudioError(null), 5000);
  };

  useEffect(() => {
    // Clear audio error when chapter changes
    setAudioError(null);
  }, [chapterId]);

  return (
    <div className={clsx(styles.controls, className, {
      [styles.rtl]: currentLang?.rtl
    })}>
      
      {/* Language Selector */}
      <div className={styles.languageSection}>
        <button
          className={clsx(styles.controlButton, styles.languageButton, {
            [styles.active]: showLanguageMenu
          })}
          onClick={() => setShowLanguageMenu(!showLanguageMenu)}
          title={`Current: ${currentLang?.nativeName}`}
        >
          <Globe size={16} />
          <span className={styles.languageCode}>{currentLanguage.toUpperCase()}</span>
          <Languages size={14} />
        </button>

        {/* Language Menu */}
        {showLanguageMenu && (
          <div className={styles.languageMenu}>
            <div className={styles.menuHeader}>
              <span>Select Language</span>
            </div>
            {availableLanguages.map((language) => (
              <button
                key={language.code}
                className={clsx(styles.languageOption, {
                  [styles.selected]: language.code === currentLanguage
                })}
                onClick={() => handleLanguageSwitch(language.code)}
              >
                <span className={styles.languageName}>{language.name}</span>
                <span className={styles.nativeName}>{language.nativeName}</span>
                {language.code === currentLanguage && (
                  <span className={styles.checkmark}>✓</span>
                )}
              </button>
            ))}
          </div>
        )}
      </div>

      {/* Translation Status and Controls */}
      <div className={styles.translationSection}>
        {translationStatus.isTranslating ? (
          <div className={styles.translatingStatus}>
            <Loader2 size={14} className={styles.spinner} />
            <span>Translating...</span>
          </div>
        ) : (
          <>
            {/* Translation Status Indicator */}
            <div className={clsx(styles.statusIndicator, {
              [styles.translated]: translationStatus.isTranslated,
              [styles.notTranslated]: !translationStatus.isTranslated && isUrdu
            })}>
              {translationStatus.isTranslated ? (
                <>
                  <span className={styles.statusIcon}>✓</span>
                  <span className={styles.statusText}>
                    Translated ({translationStatus.quality})
                  </span>
                </>
              ) : isUrdu ? (
                <>
                  <span className={styles.statusIcon}>⚠</span>
                  <span className={styles.statusText}>Translation needed</span>
                </>
              ) : (
                <>
                  <span className={styles.statusIcon}>📝</span>
                  <span className={styles.statusText}>Original</span>
                </>
              )}
            </div>

            {/* Manual Translation Button */}
            {isUrdu && !translationStatus.isTranslated && (
              <button
                className={clsx(styles.controlButton, styles.translateButton)}
                onClick={handleTranslate}
                title="Translate this chapter to Urdu"
              >
                <RotateCcw size={14} />
                <span>Translate</span>
              </button>
            )}
          </>
        )}
      </div>

      {/* Audio Controls */}
      <div className={styles.audioSection}>
        {/* Audio Toggle Button */}
        <button
          className={clsx(styles.controlButton, styles.audioButton, {
            [styles.active]: audioStatus.isPlaying || showAudioControls
          })}
          onClick={() => {
            onAudioToggle();
            setShowAudioControls(!showAudioControls);
          }}
          disabled={audioStatus.isLoading}
          title={audioStatus.isPlaying ? 'Pause audio' : 'Play audio'}
        >
          {audioStatus.isLoading ? (
            <Loader2 size={16} className={styles.spinner} />
          ) : audioStatus.isPlaying ? (
            <Pause size={16} />
          ) : (
            <Play size={16} />
          )}
          <span>
            {audioStatus.isLoading ? 'Loading...' : 
             audioStatus.isPlaying ? 'Pause' : 'Listen'}
          </span>
        </button>

        {/* Expanded Audio Controls */}
        {(showAudioControls || audioStatus.isPlaying) && (
          <div className={styles.audioControls}>
            
            {/* Progress Bar */}
            <div className={styles.progressContainer}>
              <span className={styles.timeDisplay}>
                {formatTime(audioStatus.currentPosition)}
              </span>
              <input
                type="range"
                min={0}
                max={audioStatus.duration}
                value={audioStatus.currentPosition}
                onChange={(e) => onAudioSeek(Number(e.target.value))}
                className={styles.progressBar}
                disabled={audioStatus.isLoading}
              />
              <span className={styles.timeDisplay}>
                {formatTime(audioStatus.duration)}
              </span>
            </div>

            {/* Playback Rate Control */}
            <div className={styles.playbackControls}>
              <label className={styles.rateLabel}>Speed:</label>
              <select
                value={audioStatus.playbackRate}
                onChange={(e) => onPlaybackRateChange(Number(e.target.value))}
                className={styles.rateSelect}
              >
                <option value={0.5}>0.5x</option>
                <option value={0.75}>0.75x</option>
                <option value={1}>1x</option>
                <option value={1.25}>1.25x</option>
                <option value={1.5}>1.5x</option>
                <option value={2}>2x</option>
              </select>
            </div>

            {/* Audio Language Indicator */}
            <div className={styles.audioLanguage}>
              <Volume2 size={12} />
              <span>{currentLang?.nativeName} TTS</span>
            </div>
          </div>
        )}

        {/* Audio Error Display */}
        {audioError && (
          <div className={styles.audioError}>
            <VolumeX size={14} />
            <span>{audioError}</span>
          </div>
        )}
      </div>

      {/* Model Attribution */}
      <div className={styles.modelAttribution}>
        <Settings size={12} />
        <span className={styles.attributionText}>
          Free Models: Helsinki-NLP • OpenVoice • Coqui TTS
        </span>
      </div>
    </div>
  );
};

export default MultilingualControls;