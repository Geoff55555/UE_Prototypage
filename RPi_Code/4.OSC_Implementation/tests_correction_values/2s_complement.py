#inspired from table in https://en.wikipedia.org/wiki/Two%27s_complement
def two_complement(nbr, bits):
    last_bit = 1 << (bits-1)  # to check the MSBit
    if nbr & last_bit:  # if the MSB is 1 we should :
        # 1.Calculate the 2's complement
        nbr ^= (pow(2, bits)-1)  # inverts all bits
        nbr += 1  # 2C OK
        # and 2. negate the sign
        nbr *= -1
    return nbr

# test : OK
x = 0
y = two_complement(x, 3)
print(y)
x = 1
y = two_complement(x, 3)
print(y)
x = 2
y = two_complement(x, 3)
print(y)
x = 3
y = two_complement(x, 3)
print(y)
x = 4
y = two_complement(x, 3)
print(y)
x = 5
y = two_complement(x, 3)
print(y)
x = 6
y = two_complement(x, 3)
print(y)
x = 7
y = two_complement(x, 3)
print(y)

# test with real rawQuat value
# x = 50050
x = 33949

# Get the 2C
y = two_complement(x, 16)
print(y)

# Now calculate from qpoint to float...
y*=pow(2,-14)
print(y)