import json

class JParser:
    def __init__(self):
        return
    
    def parse_json_file(self, file_path):
        with open(file_path, 'r') as f:
            data = json.load(f)
        return data
    
class PathParser:
    def __init__(self):
        return
    
    def parse_the_file(data):
        node_list_list = []
        arrive_list_list = []
        start_list_list = []
        for line in data:
            node_list = []
            arrive_list = []
            start_list = []
            line = line[line.find(":")+3:]
            while True:
                c1 = line.find(",")
                if c1 == -1:
                    break
                c2 = line.find(")")
                c3 = line.find(">")
                time_arrive = float(line[0:c1])
                time_start = float(line[c1+1:c2-1])
                node_num = f"{int(line[c2+1:c3-1]):04d}"

                node_list.append(node_num)
                arrive_list.append(time_arrive)
                start_list.append(time_start)
                line = line[c3+2:]
            node_list_list.append(node_list)
            arrive_list_list.append(arrive_list)
            start_list_list.append(start_list)

        return node_list_list, arrive_list_list, start_list_list
