import json

# raw data received
raw_data = '[{"data":{"coordinatesType":1,"coordinates":{"z":1000,"x":6898,"y":17331},"anchorData":[{"rss":-92.15,"tagId":"26920","anchorId":"22367"},{"rss":-87.22,"tagId":"26920","anchorId":"24660"},{"rss":-84.74,"tagId":"26920","anchorId":"17671"},{"rss":-92.52,"tagId":"26920","anchorId":"4226"}],"tagData":{"sensors":[{"value":"","name":"CUSTOM_PAYLOAD"}],"blinkIndex":13296},"metrics":{"latency":18,"rates":{"packetLoss":0,"success":1.05,"update":1.05}},"extras":{"version":"0.1","zones":[]}},"success":true,"timestamp":1569252240.7322404,"tagId":"26920","version":"1.4"}]'

# to perform json.loads, need to get rid of the brackets
string_without_brackets = raw_data[1:-1]  # negative index is a shortcut for telling "len(raw_data)" before the negative value https://www.w3schools.com/python/python_tuples.asp

# json.loads creates a  python dictionary according to json data
py_dict = json.loads(string_without_brackets)
coord = [0,0,0]
coord[0] = py_dict["data"]["coordinates"]["x"]
coord[1] = py_dict["data"]["coordinates"]["y"]
coord[2] = py_dict["data"]["coordinates"]["z"]

# to format as needed, for now just being made as understandable string
data_wanted = 'Tag_ID = ' + str(py_dict["tagId"]) + ' --> Coord = ' + str(coord)

print(data_wanted)