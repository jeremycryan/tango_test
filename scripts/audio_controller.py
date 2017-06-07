from audiolazy import *

if __name__ == '__main__':
    main()
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

def freq(name):
    try:
        letter = name[0]
        name = name[1:]
        mod = False
        if name[0] in ("#", "b"):
            letter += name[0]
            name = name[1:]
        octv = int(name)

        freqs = {"C" : 16.35, "C#" : 17.32, "Db" : 17.32, "D" : 18.35,
                "D#" : 19.45, "Eb" : 19.45, "E" : 20.60, "E#" : 21.83, "Fb" : 20.60,
                "F" : 21.83, "F#" : 23.12, "Gb" : 23.12, "G" : 24.50, "G#" : 25.96,
                "Ab" : 25.96, "A" : 27.50, "A#" : 29.14, "Bb" : 29.14, "B" : 30.87,
                "B#" : 32.70, "Cb" : 15.49}

        return freqs[letter] * (2 ** octv)

    except:
        print "That is not a note."
