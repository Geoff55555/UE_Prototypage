tags_id = []
tags_coord = []  # link between the 2 arrays is that we'll be using the same index for both to refer to the same tag

id = 11
coord = [22,22,22]
try:
    id_current_tag_received_index = tags_id.index(id)  # check if this id has been already received
except ValueError:
    tags_id.append(id)  # if not, we add the id of the tag in the tags_id list
    tags_coord.append([0, 0, 0])  # immediatly after, we also create a new line in tags_coord array so the indexes
    # match in both arrays refering to the same tag
    id_current_tag_received_index = tags_id.index(id)
tags_coord[id_current_tag_received_index] = coord  # we link this tag id to its own coord currently received

print(tags_id)
print(tags_coord)

id = 11
coord = [3,3,3]
try:
    id_current_tag_received_index = tags_id.index(id)  # check if this id has been already received
except ValueError:
    tags_id.append(id)  # if not, we add the id of the tag in the tags_id list
    tags_coord.append([0, 0, 0])  # immediatly after, we also create a new line in tags_coord array so the indexes
    # match in both arrays refering to the same tag
    id_current_tag_received_index = tags_id.index(id)
tags_coord[id_current_tag_received_index] = coord  # we link this tag id to its own coord currently received

print(tags_id)
print(tags_coord)

id = 1
coord = [3,2,1]
try:
    id_current_tag_received_index = tags_id.index(id)  # check if this id has been already received
except ValueError:
    tags_id.append(id)  # if not, we add the id of the tag in the tags_id list
    tags_coord.append([0, 0, 0])  # immediatly after, we also create a new line in tags_coord array so the indexes
    # match in both arrays refering to the same tag
    id_current_tag_received_index = tags_id.index(id)
tags_coord[id_current_tag_received_index] = coord  # we link this tag id to its own coord currently received

print(tags_id)
print(tags_coord)
