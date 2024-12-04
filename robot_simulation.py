import pygame
import math

class Envir:
    def __init__(self,dimention):
        #colors
        self.black=(0,0,0)
        self.white = (255,255,255)
        self.green = (0,255,0)
        self.blue = (0,0,255)
        self.red = (255,0,0)
        self.yel = (255,255,0)
        #map dimention
        self.height = dimention[0]
        self.width = dimention[1]
        #window setting
        pygame.display.set_caption("Differental drive robot")
        self.map = pygame.display.set_mode((self.width,
                                            self.height))        
        #text variables
        self.font = pygame.font.SysFont("Tajawal-Regular.tff",50)
        self.text = self.font.render('default',True,self.black,self.white)
        self.textRect = self.text.get_rect()
        self.textRect.center = (dimention[1]-1100,dimention[0]-50)
        #trail
        self.trail_set = []
        
    def write_info(self,vl,vr,theta):
        dtheta = int(-math.degrees(theta))
        txt = f"Vl = {vl} Vr = {vr} Theta = {dtheta}"
        self.text = self.font.render(txt,True,self.black,self.white)
        self.map.blit(self.text,self.textRect)
    
    def trail(self,pos):
        # queue FIFO
        for i in range(0,len(self.trail_set)-1):
            pygame.draw.line(self.map,self.blue,(self.trail_set[i][0],self.trail_set[i][1]),
                             (self.trail_set[i+1][0],self.trail_set[i+1][1]))
        if self.trail_set.__sizeof__() > 30000:
            self.trail_set.pop(0)
        self.trail_set.append(pos)
    
    def robot_frame(self,pos, rotation):
        #the center point is the robot location
        n = 80
        centerx , centery = pos
        x_axis = (centerx + n*math.cos(-rotation), centery + n*math.sin(-rotation))
        y_axis = (centerx + n*math.cos(-rotation - (math.pi/2)), centery + n*math.sin(-rotation - (math.pi/2)))

        pygame.draw.line(self.map,self.red,(centerx,centery),x_axis,3)
        pygame.draw.line(self.map,self.green,(centerx,centery),y_axis,3)
    
    def draw_rect(self):                      #x  #y  #w #h
        pygame.draw.rect(self.map,self.black,(550,0,100,275))
        pygame.draw.rect(self.map,self.black,(800,425,100,175))
        pygame.draw.rect(self.map,self.black,(850,225,350,100))
        pygame.draw.rect(self.map,self.black,(200,225,150,150))
        pygame.draw.rect(self.map,self.blue,(32,132,4,4))
        pygame.draw.rect(self.map,self.yel,(1200-32,550-32,4,4))
       
class Robot:
    def __init__(self,startpos,robotImg,width,algorithm):
        self.m2p = 3779.52 #meter to pexel
        # robot dims
        self.w = width
        self.x = startpos[0]
        self.y = startpos[1]
        self.theta = 0
        self.ang = 0
        self.vl = 0.01*self.m2p
        self.vr = 0.01*self.m2p
        self.maxspeed = 0.02*self.m2p
        self.minspeed = -0.02*self.m2p
        self.u = 10
        self.a = 20
        self.W = 0
        # graphics
        self.img = pygame.image.load(robotImg)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x,self.y))
       
        self.cord = {
                      'S':(float(startpos[0]),float(startpos[1])),
                      'A':(200.0 - 32, 225.0 - 32),
                      'B':(350.0 + 32, 225.0 - 32),
                      'C':(200.0 - 32, 375.0 + 32),
                      'D':(350.0 + 32, 375.0 + 32),
                      'E':(550.0 - 32, 0.0   + 32),
                      'F':(550.0 - 32, 275.0 + 32),
                      'G':(650.0 + 32, 275.0 + 32),
                      'H':(850.0 - 32, 225.0 - 32),
                      'I':(850.0 - 32, 325.0 + 32),
                      'J':(800.0 - 32, 425.0 - 32),
                      'K':(900.0 + 32, 425.0 - 32),
                      'T':(1200.0 -32, 550 - 32)
                     }
        
        
        if algorithm == 'dijkstra':
            self.graph = {
                'S':[('A',self.distance_b2n(self.cord.get('S'),self.cord.get('A'))),('C',self.distance_b2n(self.cord.get('S'),self.cord.get('C')))],
                'A':[('B',self.distance_b2n(self.cord.get('A'),self.cord.get('B'))),('E',self.distance_b2n(self.cord.get('A'),self.cord.get('E'))),('C',self.distance_b2n(self.cord.get('A'),self.cord.get('C')))],
                'B':[('E',self.distance_b2n(self.cord.get('B'),self.cord.get('E'))),('F',self.distance_b2n(self.cord.get('B'),self.cord.get('F')))] ,
                'C':[('D',self.distance_b2n(self.cord.get('C'),self.cord.get('D'))),('J',self.distance_b2n(self.cord.get('C'),self.cord.get('J')))] ,
                'D':[('F',self.distance_b2n(self.cord.get('D'),self.cord.get('F'))),('G',self.distance_b2n(self.cord.get('D'),self.cord.get('G'))),('H',self.distance_b2n(self.cord.get('D'),self.cord.get('H'))),('I',self.distance_b2n(self.cord.get('D'),self.cord.get('I'))),('J',self.distance_b2n(self.cord.get('D'),self.cord.get('J')))] ,
                'E':[('F',self.distance_b2n(self.cord.get('E'),self.cord.get('F')))] ,
                'F':[('G',self.distance_b2n(self.cord.get('F'),self.cord.get('G'))),('J',self.distance_b2n(self.cord.get('F'),self.cord.get('J'))),('K',self.distance_b2n(self.cord.get('F'),self.cord.get('K')))] ,
                'G':[('H',self.distance_b2n(self.cord.get('G'),self.cord.get('H'))),('I',self.distance_b2n(self.cord.get('G'),self.cord.get('I'))),('J',self.distance_b2n(self.cord.get('G'),self.cord.get('J'))),('K',self.distance_b2n(self.cord.get('G'),self.cord.get('K')))] ,
                'H':[('I',self.distance_b2n(self.cord.get('H'),self.cord.get('I')))] ,
                'I':[('J',self.distance_b2n(self.cord.get('I'),self.cord.get('J'))),('K',self.distance_b2n(self.cord.get('I'),self.cord.get('K'))),('T',self.distance_b2n(self.cord.get('I'),self.cord.get('T')))] ,
                'J':[('K',self.distance_b2n(self.cord.get('J'),self.cord.get('K')))] ,
                'K':[('T',self.distance_b2n(self.cord.get('K'),self.cord.get('T')))] ,
                'T':[]
            }
            solution = self.dijkstra(self.graph,'S','T')
            self.optimal_path = []
            for i in solution:
                self.optimal_path.append(i[0])
            print('Solution is ',self.optimal_path)
            print("Cost of Solution is",self.path_cost(solution)[0])    
        elif algorithm == 'BFS':
            self.graph = {
                            'S':['A','C'],
                            'A':['B','E','C'],
                            'B':['E','F','J','D'] ,
                            'C':['D','J'] ,
                            'D':['F','G','H','I','J'] ,
                            'E':['F'] ,
                            'F':['G','J','K'] ,
                            'G':['H','I','J','K'] ,
                            'H':['I'] ,
                            'I':['J','K','T'] ,
                            'J':['K'] ,
                            'K':['T'] ,
                            'T':[]            
                        }
            solution = self.BFS(self.graph,'S','T')
            self.optimal_path = []
            for i in solution:
                self.optimal_path.append(i[0])
            print('Solution is ',self.optimal_path)
        elif algorithm == 'DFS':
            self.graph = {
                            'S':['A','C'],
                            'A':['B','E','C'],
                            'B':['E','F','J','D'] ,
                            'C':['D','J'] ,
                            'D':['F','G','H','I','J'] ,
                            'E':['F'] ,
                            'F':['G','J','K'] ,
                            'G':['H','I','J','K'] ,
                            'H':['I'] ,
                            'I':['J','K','T'] ,
                            'J':['K'] ,
                            'K':['T'] ,
                            'T':[]
                        }  
            solution = self.DFS(self.graph,'S','T')
            self.optimal_path = []
            for i in solution:
                self.optimal_path.append(i[0])
            print('Solution is ',self.optimal_path)
        elif algorithm == 'A STAR':
            self.graph = {
                'S':[('A',self.distance_b2n(self.cord.get('S'),self.cord.get('A'))),('C',self.distance_b2n(self.cord.get('S'),self.cord.get('C')))],
                'A':[('B',self.distance_b2n(self.cord.get('A'),self.cord.get('B'))),('E',self.distance_b2n(self.cord.get('A'),self.cord.get('E'))),('C',self.distance_b2n(self.cord.get('A'),self.cord.get('C')))],
                'B':[('E',self.distance_b2n(self.cord.get('B'),self.cord.get('E'))),('F',self.distance_b2n(self.cord.get('B'),self.cord.get('F')))] ,
                'C':[('D',self.distance_b2n(self.cord.get('C'),self.cord.get('D'))),('J',self.distance_b2n(self.cord.get('C'),self.cord.get('J')))] ,
                'D':[('F',self.distance_b2n(self.cord.get('D'),self.cord.get('F'))),('G',self.distance_b2n(self.cord.get('D'),self.cord.get('G'))),('H',self.distance_b2n(self.cord.get('D'),self.cord.get('H'))),('I',self.distance_b2n(self.cord.get('D'),self.cord.get('I'))),('J',self.distance_b2n(self.cord.get('D'),self.cord.get('J')))] ,
                'E':[('F',self.distance_b2n(self.cord.get('E'),self.cord.get('F')))] ,
                'F':[('G',self.distance_b2n(self.cord.get('F'),self.cord.get('G'))),('J',self.distance_b2n(self.cord.get('F'),self.cord.get('J'))),('K',self.distance_b2n(self.cord.get('F'),self.cord.get('K')))] ,
                'G':[('H',self.distance_b2n(self.cord.get('G'),self.cord.get('H'))),('I',self.distance_b2n(self.cord.get('G'),self.cord.get('I'))),('J',self.distance_b2n(self.cord.get('G'),self.cord.get('J'))),('K',self.distance_b2n(self.cord.get('G'),self.cord.get('K')))] ,
                'H':[('I',self.distance_b2n(self.cord.get('H'),self.cord.get('I')))] ,
                'I':[('J',self.distance_b2n(self.cord.get('I'),self.cord.get('J'))),('K',self.distance_b2n(self.cord.get('I'),self.cord.get('K'))),('T',self.distance_b2n(self.cord.get('I'),self.cord.get('T')))] ,
                'J':[('K',self.distance_b2n(self.cord.get('J'),self.cord.get('K')))] ,
                'K':[('T',self.distance_b2n(self.cord.get('K'),self.cord.get('T')))] ,
                'T':[]
            }            
            self.H_table = {
                        'S':[(self.distance_b2n(self.cord.get('S'),self.cord.get('T')))],
                        'A':[(self.distance_b2n(self.cord.get('A'),self.cord.get('T')))],
                        'B':[(self.distance_b2n(self.cord.get('B'),self.cord.get('T')))] ,
                        'C':[(self.distance_b2n(self.cord.get('C'),self.cord.get('T')))] ,
                        'D':[(self.distance_b2n(self.cord.get('D'),self.cord.get('T')))] ,
                        'E':[(self.distance_b2n(self.cord.get('E'),self.cord.get('T')))] ,
                        'F':[(self.distance_b2n(self.cord.get('F'),self.cord.get('T')))] ,
                        'G':[(self.distance_b2n(self.cord.get('G'),self.cord.get('T')))] ,
                        'H':[(self.distance_b2n(self.cord.get('H'),self.cord.get('T')))] ,
                        'I':[(self.distance_b2n(self.cord.get('I'),self.cord.get('T')))] ,
                        'J':[(self.distance_b2n(self.cord.get('J'),self.cord.get('T')))] ,
                        'K':[(self.distance_b2n(self.cord.get('K'),self.cord.get('T')))] ,
                        'T':[(0.0)]
                    }
                        
            solution = self.A_STAR(self.graph,'S','T')
            self.optimal_path = []
            for i in solution:
                self.optimal_path.append(i[0])
            print('Solution is ',self.optimal_path)
            print("cost of Solution is :", self.path_f_cost(solution)[0])
        else:
            s = '#'*20
            print(s,"\n","Wrong Algorithm Name\n",s)   
            
            self.optimal_path = [] 
            self.vl = 0
            self.vr = 0
            self.u = 0
            self.a = 0
            self.w = 0
    ##################################################
    #####################dijkstra#####################
    def path_cost(self,path):
        # Calculate the total cost of a path
        total_cost = 0
        for (node, cost) in path:
            total_cost += cost
        return total_cost , path[-1][0]#D,C or..

    def dijkstra(self,graph, start, goal):
        visited = []
        queue = [[(start,0)]]
        while queue:
            queue.sort(key=self.path_cost) #sorting by cost
            path = queue.pop(0) # choosing least cost
            node = path[-1][0]
            if node in visited:
                continue
            visited.append(node)
            if node == goal:
                return path
            else:
                adjacent_nodes = graph.get(node,[])
                #print(adjacent_nodes)
                for (node2, cost) in adjacent_nodes:
                    new_path = path.copy()
                    new_path.append((node2, cost))
                    queue.append(new_path)
    
    
    #####################dijkstra#####################
    ##################################################
        
       
    ##################################################
    ##################    BFS   ###################### 
    def BFS(self,graph,start,goal):
        visited = []
        queue = [[start]] # queue is FIFO (first in first out)
        while queue:
            path = queue.pop(0)
            node = path[-1]
            if node in visited:
                continue
            visited.append(node)
            if node == goal:
                return path
            else:
                adjacent_nodes = graph.get(node,[])
                for node2 in adjacent_nodes:
                    new_path = path.copy()
                    new_path.append(node2)
                    queue.append(new_path)
        
    
    ##################    BFS   ######################
    ##################################################
    
    
    ##################################################
    ##################    DFS   ######################
    def DFS(self,graph,start,goal):
        visited = []
        stack = [[start]]#stack is LIFO (last in first out)
        while stack:
            path = stack.pop()
            node = path[-1]
            if node in visited:
                continue
            visited.append(node)
            if node == goal:
                return path
            else:
                adjacent_nodes = graph.get(node,[])
                for node2 in adjacent_nodes:
                    new_path = path.copy()
                    new_path.append(node2)
                    stack.append(new_path)

    ##################    DFS   ######################
    ##################################################
    
    
    ##################################################
    ##################  A STAR  ######################
    def path_f_cost(self,path):
        g_cost=0
        for node, cost in path:
            #print(node,cost)
            g_cost += cost
        last_node = path[-1][0]
        h_cost = self.H_table[last_node][0]
        f_cost = g_cost + h_cost
        return f_cost , last_node

    def A_STAR(self,graph, start, goal):
        visisted=[]
        queue = [[(start,0)]]
        while queue:
            queue.sort(key=self.path_f_cost)#sorting by f_cost
            path = queue.pop(0)# choosing least f_cost
            node = path[-1][0]
            if node in visisted:
                continue
            visisted.append(node)
            if node == goal:
                return path
            else:
                adjacent_nodes = graph.get(node,[])
                for (node2 , cost) in adjacent_nodes:
                    new_path = path.copy()
                    new_path.append((node2, cost))
                    queue.append(new_path)
    
    ##################  A STAR  ######################
    ##################################################
     
    def distance_b2n(self,n1,n2):
        x = n1[0] - n2[0]
        y = n1[1] - n2[1]
        return (x**2 + y**2)**0.5

    def angle_b2n(self,n1,n2):
        x = n1[0] - n2[0]
        y = n1[1] - n2[1]
        if n2[1] > n1[1]:
            return math.degrees(math.atan(y/x))
        elif n2[1] < n1[1]:
            return -math.degrees(math.atan(y/x))
        else:
            return 0

    def draw(self,map):
        map.blit(self.rotated,self.rect)
        
    def move(self,event=None):        
        self.x += (self.u*math.cos(self.theta) - self.a*math.sin(self.theta)*self.W)*dt
        self.y += (self.u*math.sin(self.theta) + self.a*math.cos(self.theta)*self.W)*dt
        self.theta += self.W*dt
        self.vl = self.u/2
        self.vr = self.u/2
        
        self.rotated = pygame.transform.rotozoom(self.img,math.degrees(-self.theta),1)
        self.rect = self.rotated.get_rect(center = (self.x,self.y))
        
        self.follow_path()
        
    def follow_path(self):
        
        if len(self.optimal_path) > 0:
            target = self.cord.get(self.optimal_path[0])
            
            if self.x + 1 < target[0]:
                delta_x = target[0] - self.x
                delta_y = target[1] - self.y
                self.u = delta_x*math.cos(self.theta) + delta_y*math.sin(self.theta)
                self.W = (-1/self.a)*delta_x*math.sin(self.theta) + (1/self.a)*delta_y*math.cos(self.theta)
            else:
                self.u = 0
                self.W = 0
                self.optimal_path.pop(0)
        
        
            
pygame.init()

# start position
start = (32,132)

#dimention
dims = (600,1200)

#running or not
running = True

# the envir
environment = Envir(dims)   

#the robot
#we can chose the algorithm from here :
# 1- dijkstra
# 2- BFS
# 3- DFS
# 4- A STAR
robot = Robot(start,'./assets/robot.png',0.01*3779.52,'BFS')

dt =0
last_time = pygame.time.get_ticks()



#simulation loop
while running:
    # activate the quite button
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running =False
    
    dt = (pygame.time.get_ticks() - last_time)/1000
    last_time = pygame.time.get_ticks()
    pygame.display.update()
    environment.map.fill(environment.white)
    environment.trail((robot.x,robot.y))
    environment.write_info(int(robot.vl),int(robot.vr),robot.theta)
    robot.draw(environment.map)
    environment.robot_frame((robot.x,robot.y),-robot.theta)
    environment.draw_rect()
    robot.move()
