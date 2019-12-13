h = [0,0,0,0]

def testValSB(p):
    global h
    if p < 256:
        h[0] = p
        h[1] = 0
    elif p > 256:
        h[0] = p % 256
        h[1] = p // 256


# main
testValSB(13)
print(h)
testValSB(300)
print(h)
