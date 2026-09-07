import time
import numpy as np
import sounddevice as sd
import sounddevice as sd

# Wake up the sound card with 0.1 seconds of silence
#sd.play(np.zeros(4410*10), 44100)
#sd.wait()


def generate_wave(shape, frequency, t):
    """Generates a raw waveform based on the requested shape."""
    shape = shape.lower()
    if shape == "sine":
        return np.sin(2 * np.pi * frequency * t)
    elif shape == "square":
        return np.sign(np.sin(2 * np.pi * frequency * t))
    elif shape in ("saw", "sawtooth"):
        return 2.0 * (frequency * t - np.floor(0.5 + frequency * t))
    else:
        raise ValueError(f"Unsupported shape '{shape}'. Choose 'sine', 'square', or 'saw'.")

def play_beep(
    frequency=440.0,
    duration=0.5,
    volume=0.2,
    shape="sine",
    repeats=1,
    pause=0.0,
    sample_rate=44100
):
    """Plays a sound with specified wave properties."""
    volume = max(0.0, min(1.0, volume))
    t = np.linspace(0, duration, int(sample_rate * duration), False)
    wave = volume * generate_wave(shape, frequency, t)

    for i in range(repeats):
        sd.play(wave, sample_rate)
        sd.wait()
        if i < repeats - 1 and pause > 0:
            time.sleep(pause)

def play_arpegio(
    duration=0.1, 
    volume=0.2, 
    shape="square", 
    gap=0.03, 
    arpeggio = [261.63, 329.63, 392.00]
):
    """Plays a C Major arpeggio."""
    for freq in arpeggio:
        play_beep(
            frequency=freq,
            duration=duration,
            volume=volume,
            shape=shape,
            repeats=1
        )
        if gap > 0:
            time.sleep(gap)

# Test it out directly
if __name__ == "__main__":
    play_beep(volume=0.001, duration=0.5)
    play_arpegio(arpeggio = [261.63, 329.63, 392.00])
    play_arpegio(arpeggio = [392.00, 329.63, 261.63])
    