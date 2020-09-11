import struct 

def getSignalFrequency():
    freq = -1.0
    while(freq < 1.0 or freq > 250.0):
        # input returns a string, convert it to a float
        freq = float(input('enter signal frequency in range [1.0,250.0] '))
    return freq



freq = getSignalFrequency()
print("freq before rounding: ", end='')
print(freq)

# round to 2 decimal places
freq = round(freq,2)
print("freq rounded to 2 decimal places: ", end='')
print(freq)

print("freq data type: ", end='')
print(type(freq))

# > big endian, standard size 4 bytes
freqBytes = bytearray(struct.pack('f',freq))

print("freqBytes data type: ", end='')
print(type(freqBytes))

print("freqBytes: ",end='')
print(freqBytes)

print("length of bytes object: ", end='')
print(len(freqBytes))

print([ "0x%02x" % b for b in freqBytes])