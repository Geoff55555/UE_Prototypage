array = None

def testFunc():
    global array
    array = [0,1,2,3]
    return True


def main():
    testFunc()
    print(array)


main()
