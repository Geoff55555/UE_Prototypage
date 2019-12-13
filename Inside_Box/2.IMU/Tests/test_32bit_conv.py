# for setFeatureCommandSpecConf
microsBetweenReports = 500000

w = (microsBetweenReports & 0xFF)                      # Report Interval (LSB from 32-bit) in microseconds. 0x7A120 = 500ms
x = (microsBetweenReports & 0xFF00) // pow(256,1)      # Report Interval
y = (microsBetweenReports & 0xFF0000) // pow(256,2)    # Report Interval
z = (microsBetweenReports & 0xFF000000) // pow(256,3)  # Report Interval (MSB)

print(z, y, x, w)

# for sendSPIPacket
headerBuffer = [0,0]
packetLength = 270
if packetLength < 256:
    headerBuffer[0] = packetLength  # packet length LSB
    headerBuffer[1] = 0             # packet length MSB
elif packetLength >= 256:
    headerBuffer[0] = packetLength % 256  # packet length LSB
    headerBuffer[1] = packetLength // 256  # packet length MSB
    
print(headerBuffer)
