from math import pi

def countsToRad(counts):
    return (counts * 2 * pi / 2048)

def countsToW(count1, count2, time):
    pos1 = countsToRad(count1)
    pos2 = countsToRad(count2)
    theta = pos2 - pos1
    if theta >= 0:
        return theta / time
    else:
        return (theta + 2 * pi) / time