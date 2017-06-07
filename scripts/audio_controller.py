from audiolazy import *

rate = 44100 # Sampling rate, in samples/second
s, Hz = sHz(rate) # Seconds and hertz
ms = 1e-3 * s
start1 = 110
start2 = 220
multiplier = 2
for i in range(5):
    note1 = karplus_strong(start1*pow(multiplier, i) * Hz) # Pluck "digitar" synth
    note2 = zeros(300 * ms).append(karplus_strong(start2 * pow(multiplier, i) * Hz))
    notes = (note1 + note2) * .5 #.5 is Amplitude
    sound = notes.take(int(1.4 * s)) # 2 seconds of a Karplus-Strong note
    with AudioIO(True) as player: # True means "wait for all sounds to stop"
      player.play(sound, rate=rate)
