from enum import Enum

class Schedule:
    def __init__(self):
        self.node_list = []
        self.time_arrive_list = []
        self.time_start_list = []

class Status(Enum):
    IDLE = 0
    MOVE = 1
    FINISHIED = 2

def getNodePose(target_name, json_data):
    # global json_data
    for idx, node in enumerate(json_data["nodes"]):
        name = node["name"]
        if name == target_name:
            # print(f"NAME {name}")
            x = json_data["nodes"][idx]["position"]["x"]
            y = json_data["nodes"][idx]["position"]["y"]
            return x, y
    return None, None

class Robot:
    def __init__(self):
        self.id = -1
        self.cur_node = None
        self.next_node = None
        self.time_cur_arrive = 0
        self.time_cur_start = 0
        self.time_next_arrive = 0
        self.node_idx = 0
        self.time_scale = 0.1

        self.json_data = None
        # self.time_next_end = 0

        self.status = Status.IDLE
        self.schedule = Schedule()
        self.x = 0
        self.y = 0

        self.dx = 0
        self.dy = 0
        return
    
    def setNodeData(self):
        self.cur_node = self.schedule.node_list[self.node_idx]
        self.next_node = self.schedule.node_list[self.node_idx+1]
        self.time_cur_arrive = self.schedule.time_arrive_list[self.node_idx]
        self.time_cur_start = self.schedule.time_start_list[self.node_idx]
        self.time_next_arrive = self.schedule.time_arrive_list[self.node_idx+1]

        self.x, self.y = getNodePose(self.cur_node, self.json_data)
        
    def shouldIStart(self, time):
        # print(f"Time {self.time_cur_start} cur {time}")
        if self.time_cur_start <= time:
            return True
        return False
    
    def didIArrive(self):

        goal_x, goal_y = getNodePose(self.next_node, self.json_data)

        dist_x = abs(goal_x - self.x)
        dist_y = abs(goal_y - self.y)
        # print(f"            DIST {dist_x+dist_y}")
        if (dist_x + dist_y) < 0.1:
            self.x = goal_x
            self.y = goal_y
            return True
        return False
    
    def culDxDy(self, time):

        goal_x, goal_y = getNodePose(self.next_node, self.json_data)
        dist_time = self.time_next_arrive - self.time_cur_start

        # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        # print(f"Cur Node {self.cur_node}  Next {self.next_node}")
        # print(f"start Time {self.time_cur_start} Arrive Time {self.time_next_arrive}")
        # print(f"start Pose {self.x} {self.y} Arrive Pose {goal_x} {goal_y}")
        dist_x = (goal_x - self.x) / dist_time
        dist_y = (goal_y - self.y) / dist_time
        self.dx = dist_x*self.time_scale
        self.dy = dist_y*self.time_scale
        # print(f"    DX Pose {self.dx} {self.dy} Arrive Pose {dist_x} {dist_y}")

    
    def move(self):
        self.x += self.dx
        self.y += self.dy
        return
    def updateRobot(self, time):
        # print(f"Im robot {self.id}  Status {self.status}")
        # print(f"    Idx {self.node_idx} coord  {self.x} {self.y}")
        if self.status == Status.IDLE:
            if self.shouldIStart(time):
                # print(" I Will start!!")
                self.status = Status.MOVE
                self.culDxDy(time)
            else:
                return

        elif self.status == Status.MOVE:
            if self.didIArrive():
                self.node_idx +=1
                # print
                if len(self.schedule.node_list) == self.node_idx+1:
                    self.status = Status.FINISHIED
                    return
                self.status = Status.IDLE
                self.setNodeData()
            else:
                self.move()
            return
        
        elif self.status == Status.FINISHIED:
            return
        