import os
import tempfile
import pygame
from gtts import gTTS
from typing import Optional


class TTS_Core:
    """
    A class to convert text to speech and play audio.
    
    This class uses Google Text-to-Speech (gTTS) for text conversion
    and pygame for audio playback.
    """
    
    def __init__(self, language: str = 'en', slow: bool = False):
        """
        Initialize the TTS_Core class.
        
        Args:
            language (str): Language code for TTS (default: 'en' for English)
            slow (bool): Whether to speak slowly (default: False)
        """
        self.language = language
        self.slow = slow
        self.temp_dir = tempfile.gettempdir()
        
        # Initialize pygame mixer for audio playback
        pygame.mixer.init()
        
    def text_to_speech(self, text: str, filename: Optional[str] = None) -> str:
        """
        Convert text to speech and save as audio file.
        
        Args:
            text (str): The text to convert to speech
            filename (str, optional): Custom filename for the audio file
            
        Returns:
            str: Path to the generated audio file
        """
        if not text.strip():
            raise ValueError("Text cannot be empty")
            
        # Generate filename if not provided
        if filename is None:
            filename = f"tts_audio_{hash(text) % 10000}.mp3"
        
        # Ensure filename has .mp3 extension
        if not filename.endswith('.mp3'):
            filename += '.mp3'
            
        filepath = os.path.join(self.temp_dir, filename)
        
        try:
            # Convert text to speech
            tts = gTTS(text=text, lang=self.language, slow=self.slow)
            tts.save(filepath)
            return filepath
            
        except Exception as e:
            raise Exception(f"Failed to convert text to speech: {str(e)}")
    
    def play_audio(self, filepath: str, wait: bool = True) -> None:
        """
        Play audio from file.
        
        Args:
            filepath (str): Path to the audio file
            wait (bool): Whether to wait for audio to finish playing
        """
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Audio file not found: {filepath}")
            
        try:
            pygame.mixer.music.load(filepath)
            pygame.mixer.music.play()
            
            if wait:
                # Wait for audio to finish
                while pygame.mixer.music.get_busy():
                    pygame.time.wait(100)
                    
        except Exception as e:
            raise Exception(f"Failed to play audio: {str(e)}")
    
    def speak(self, text: str, filename: Optional[str] = None, 
              wait: bool = True, cleanup: bool = True) -> str:
        """
        Convert text to speech and play it immediately.
        
        Args:
            text (str): The text to convert to speech
            filename (str, optional): Custom filename for the audio file
            wait (bool): Whether to wait for audio to finish playing
            cleanup (bool): Whether to delete the audio file after playing
            
        Returns:
            str: Path to the generated audio file
        """
        # Convert text to speech
        filepath = self.text_to_speech(text, filename)
        
        # Play the audio
        self.play_audio(filepath, wait)
        
        # Cleanup if requested
        if cleanup and os.path.exists(filepath):
            try:
                os.remove(filepath)
            except:
                pass  # Ignore cleanup errors
                
        return filepath
    
    def set_language(self, language: str) -> None:
        """
        Set the language for text-to-speech conversion.
        
        Args:
            language (str): Language code (e.g., 'en', 'es', 'fr', 'de', etc.)
        """
        self.language = language
    
    def set_speed(self, slow: bool) -> None:
        """
        Set the speech speed.
        
        Args:
            slow (bool): True for slow speech, False for normal speed
        """
        self.slow = slow
    
    def stop_audio(self) -> None:
        """Stop currently playing audio."""
        pygame.mixer.music.stop()
    
    def pause_audio(self) -> None:
        """Pause currently playing audio."""
        pygame.mixer.music.pause()
    
    def unpause_audio(self) -> None:
        """Unpause currently playing audio."""
        pygame.mixer.music.unpause()
    
    def get_volume(self) -> float:
        """
        Get current audio volume.
        
        Returns:
            float: Current volume (0.0 to 1.0)
        """
        return pygame.mixer.music.get_volume()
    
    def set_volume(self, volume: float) -> None:
        """
        Set audio volume.
        
        Args:
            volume (float): Volume level (0.0 to 1.0)
        """
        volume = max(0.0, min(1.0, volume))  # Clamp between 0 and 1
        pygame.mixer.music.set_volume(volume)
    
    def cleanup(self) -> None:
        """Clean up resources."""
        pygame.mixer.quit()


# Example usage and testing
if __name__ == "__main__":
    # Create TTS instance
    tts = TTS_Core(language='en', slow=False)
    
    try:
        # Example 1: Simple text-to-speech
        print("Example 1: Converting text to speech...")
        tts.speak("Hello! This is a test of the text to speech functionality. how is your day")
        
        # Example 2: Save audio file without playing
        print("Example 2: Saving audio file...")
        filepath = tts.text_to_speech("This audio was saved to a file.", "test_audio.mp3")
        print(f"Audio saved to: {filepath}")
        
        # Example 3: Play saved audio
        print("Example 3: Playing saved audio...")
        tts.play_audio(filepath)
        
        # Example 4: Change language and speed
        print("Example 4: Testing different language and speed...")
        tts.set_language('es')  # Spanish
        tts.set_speed(slow=True)
        tts.speak("Hola! Esto es una prueba de velocidad lenta.")
        
        # Example 5: Volume control
        print("Example 5: Testing volume control...")
        tts.set_language('en')
        tts.set_speed(slow=False)
        tts.set_volume(0.5)  # 50% volume
        tts.speak("This is playing at 50% volume.")
        
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        # Cleanup
        tts.cleanup()
        print("TTS instance cleaned up.") 