import random

import math
from queue import PriorityQueue
import time



def dfs(maze, initial, goal, size): #Nodes for initial and goal and all future nodes are stored as 2-tuples with with first value being x value and second value being y value
    fringe=[]   #Initialized the fringe as a list, using append and pop to act as a LIFO stack
    fringe.append(initial)                                  
    prev=[[-9999 for x in range(size)] for y in range(size)]    #Initialized a matrix that will act as pointers to the nodes previous node

    closed=set()    #closed set
    prev[initial[0]][initial[1]] = None #Mark the previous node of the initial node as None

    nodes=0     #counter for Nodes explored
    
    while not not fringe:   #tracks fringe being not empty
        current=fringe.pop()    #gets Last input node

        nodes+=1    #increments total explored Nodes
        
        if current==goal:   #We are at the goal node and have found a path 
            pntr=current    #assigns pntr to current node at goal    
            path=[]
            length=0    #tracks the length of the path
            while pntr is not None:     #Backtracks through prev matrix storing path in path list
                path.append(pntr)
                pntr=prev[pntr[0]][pntr[1]]
                length+=1
            path.reverse()
            return (1, path,length,nodes)   #Returns 1 indictaing success, the path, the length and the total nodes explored


        #next block is used to see if all neighbors of current node can be added to the fringe. Made sure to add bottom and left last as they will ideally be going to the goal
        
        if((current[0]-1 >=0) and not ((current[0]-1,current[1]) in closed)):
            if(maze[current[0]-1][current[1]] == 0):
                prev[current[0]-1][current[1]]=current
                fringe.append((current[0]-1,current[1]))
        if((current[1]-1 >=0) and not ((current[0],current[1]-1) in closed)):
            if(maze[current[0]][current[1]-1] == 0):
                prev[current[0]][current[1]-1]=current
                fringe.append((current[0],current[1]-1))
        if((current[0]+1 <size) and not ((current[0]+1,current[1]) in closed)):
            if(maze[current[0]+1][current[1]] == 0):
                prev[current[0]+1][current[1]]=current
                fringe.append((current[0]+1,current[1]))
        if((current[1]+1 <size) and not ((current[0],current[1]+1) in closed)):
            if(maze[current[0]][current[1]+1] == 0):
                prev[current[0]][current[1]+1]=current
                fringe.append((current[0],current[1]+1))

                
        closed.add(current)     #adds current node to the closed set

    return (0,None, None,nodes) #reached if fringe is empty, returns 0 for fail None for path and length and total nodes explored



def bfs(maze, initial, goal, size):     #very similar to dfs but using queue instead of stack as fringe
    fringe=[]
    fringe.append(initial)  #adding initial node to fringe
    prev=[[-9999 for x in range(size)] for y in range(size)] #making previous matrix

    closed=set()    #initiallizing the closed set
    prev[initial[0]][initial[1]] = None #Making the previous node of the initial node None, indicating it is the first node in the previous matrix

    nodes=0     #counter for number of nodes explored

    while not not fringe:
        current=fringe.pop(0)   #pops the first node adding thus making it a queue

        if((current[0],current[1]) in closed):  #had to be added as otherwise nodes already in the fringe but not in closed set would be added to fringe ballooning the total run time 
            continue

        nodes+=1
        
        if current==goal:   #goal is found
            pntr=current
            path = []
            length=0
            while pntr is not None:     #done to find path and its length
                path.append(pntr)
                pntr=prev[pntr[0]][pntr[1]]
                length+=1
            path.reverse()
            return (1, path,length,nodes)   #returning 1 for success, the path, its length and the total number of nodes

        #looks at all neighbors of current node and sees it they can be added to the fringe
        
        if((current[0]-1 >=0) and not ((current[0]-1,current[1]) in closed)):
            if(maze[current[0]-1][current[1]] == 0):
                prev[current[0]-1][current[1]]=current
                fringe.append((current[0]-1,current[1]))
        if((current[1]-1 >=0) and not ((current[0],current[1]-1) in closed)):
            if(maze[current[0]][current[1]-1] == 0):
                prev[current[0]][current[1]-1]=current
                fringe.append((current[0],current[1]-1))
        if((current[0]+1 <size) and not ((current[0]+1,current[1]) in closed)):
            if(maze[current[0]+1][current[1]] == 0):
                prev[current[0]+1][current[1]]=current
                fringe.append((current[0]+1,current[1]))
        if((current[1]+1 <size) and not ((current[0],current[1]+1) in closed)):
            if(maze[current[0]][current[1]+1] == 0):
                prev[current[0]][current[1]+1]=current
                fringe.append((current[0],current[1]+1))

        closed.add(current)     #add current node to the closed set

    return (0,None, None,nodes) #only reached if fringe is empty, returns 0 for fail, None for path and length and total nodes explored

def aStar(maze, initial, goal, size,heur): #inputs are the maze, initial and goal nodes, size of matrix and "heuristic matrix" if needed. Heur is only used in strategy 3 would be None for normal a* operation
    fringe= PriorityQueue() #fringe is a priority queue

    distDonePrev=[[(math.inf, 0, None) for x in range(size)] for y in range(size)]    #matrix storing values of the current distance of current node, whether that node was prossesed and the previous node. Values are stored inside a tuple and initialized to inf, marked as not processed and no prev.

    x=initial[0]; y=initial[1]
    temp=distDonePrev[x][y] #splotting up tuple just for ease of understanding
    dist=temp[0]
    done=temp[1]
    prev=temp[2]
    dist=0
    if heur is None:    #if heur matrix is not fed in, use euclidian distance as heuristic matrix, else use node in heur matriz cooresponding to current node
        h=math.sqrt((size-1-x)^2+(size-1-y)^2)
    else:
        h=heur[x][y]    
    fringe.put((0+h,h,initial)) #input distance plus heuristic, heuristic and node in to fringe. heuristic is added so you can subtract it to have just the total traveled distance weight
    prev=(x,y)
    distDonePrev[x][y]=(dist, done, prev)   #new values stored in distDonePrev matrix

    nodes=0     #counting total explored nodes
    
    while not fringe.empty():
        
        temp=fringe.get()
        d=temp[0]   #distance
        h=temp[1]   #heuristic

        d=d-h       #find distance without heuristic factored in
        current=temp[2] #current 
        
        x=current[0]; y=current[1]
        temp=distDonePrev[x][y]     #breaking down distDonePrev for ease
        distV=temp[0]
        doneV=temp[1]
        prevV=temp[2]
        
        if(doneV==0):   #current node is not processed

            nodes+=1

            if current==goal:   #goal node is found, finding path and length
                pntr=(size-1,size-1)
                path=[]
                length=1
                while pntr != initial:
                    path.append(pntr)
                    pntr=distDonePrev[pntr[0]][pntr[1]]
                    pntr=pntr[2]
                    length+=1
                path.append(initial)
                path.reverse()
                return (1, path,length,nodes) #returning 1 for success, path, length and nodes

            #checking if neighbors are able to added to the fringe and preparing them for addition
            if(x-1 >=0):
                if (maze[x-1][y] ==0):      
                    temp=distDonePrev[x-1][y]   #breakinf down distDonePrev for child
                    distU=temp[0]
                    doneU=temp[1]
                    prevU=temp[2]
                    if (d+1<distU): #checking to see if distance traveled plus weight of moving to U (1) is less than the distance stored in U
                        distU=d+1
                        if heur is None:    #calculating/getting heuristic
                            h=math.sqrt((size-1-(x-1))^2+(size-1-y)^2)
                        else:
                            h=heur[x-1][y]
                        fringe.put((distU+h,h,(x-1,y))) #inputing child into fringe
                        prevU=(x,y)
                        distDonePrev[x-1][y]=(distU,doneU,prevU) #updating distDonePrev of child

            #all children are processed in same way as above
            
            if(y-1 >=0):
                if (maze[x][y-1] ==0):
                    temp=distDonePrev[x][y-1]
                    distU=temp[0]
                    doneU=temp[1]
                    prevU=temp[2]
                    if (d+1<distU):
                        distU=d+1
                        if heur is None:
                            h=math.sqrt((size-1-(x))^2+(size-1-(y-1)^2))
                        else:
                            h=heur[x][y-1]
                        fringe.put((distU+h,h,(x,y-1)))
                        prevU=(x,y)
                        distDonePrev[x][y-1]=(distU,doneU,prevU)
            
            if(x+1 <size):
                if (maze[x+1][y] ==0):
                    temp=distDonePrev[x+1][y]
                    distU=temp[0]
                    doneU=temp[1]
                    prevU=temp[2]
                    if (d+1<distU):
                        distU=d+1
                        if heur is None:
                            h=math.sqrt((size-(x))^2+(size-1-(y)^2))
                        else:
                            h=heur[x+1][y]
                        fringe.put((distU+h,h,(x+1,y)))
                        prevU=(x,y)
                        distDonePrev[x+1][y]=(distU,doneU,prevU)
            
            if(y+1 <size):
                if (maze[x][y+1] ==0):
                    temp=distDonePrev[x][y+1]
                    distU=temp[0]
                    doneU=temp[1]
                    prevU=temp[2]
                    if (d+1<distU):
                        distU=d+1
                        if heur is None:
                            h=math.sqrt((size-(x))^2+(size-(y)^2))
                        else:
                            h=heur[x][y+1]
                        fringe.put((distU+h,h,(x,y+1)))
                        prevU=(x,y)
                        distDonePrev[x][y+1]=(distU,doneU,prevU)
            doneV=1     #markind current node as processed
            distDonePrev[x][y]=(distV,doneV,prevV)  #updating distDonePrev matrix of current node
    return (0, None, None,nodes) #returning 0 for fail, None for path and length and total nodes explored

def strategy1 (maze, initial, goal,size,fire):  #taking maze, initial and nodes, size of maze and flammability rate as inputs
    path = bfs(maze,initial,goal,size)[1]   #perform bfs and store the path of the result
    if path is None:    #path is None means that bfs failed
        return 0

    pathsize = len(path)
    for i in range(pathsize):   #for each node in the path
        maze = advanceFire(maze, fire)  #advancing the fire step on the maze and storing it in our current maze
        pntr = path[i]  #advancing the pointer to the next node in the path
        if maze[pntr[0]][pntr[1]] == 5: #current position is on fire, agent has died return 0 for fail
            return 0
    return 1    #agent reached the goal, return 1 for success

def strategy2 (maze, initial, goal, size,fire):  #taking maze, initial and nodes, size of maze and flammability rate as inputs
    pntr =initial
    while 1:    #runs unitl agent catches fire or reaches goal
        path = bfs(maze,pntr,goal,size)[1]  #perform bfs on maze from current location to goal and store the path
        if path is None:
            return 0
        pathsize = len(path)
        maze = advanceFire(maze, fire)  #advance the fire one step
        pntr = path[1]  #move pntr to second node in path as that will be the next step taken
        if maze[pntr[0]][pntr[1]] == 5: #agent is on fire, return 0 for fail
            return 0
        if pntr==goal:  #agent reached the goal, return 1 for success
            return 1
    return 0

def strategy3 (maze, initial, goal, size,fire):  #taking maze, initial and nodes, size of maze and flammability rate as inputs

    pntr=initial
    while 1: #run infinite loop until return is hit marking failure or success

        mazeCopy=[[0 for x in range(size)] for y in range(size)]
        for i in range(size):
            mazeCopy[i]=maze[i].copy()

        for i in range(5):
            mazeCopy=advanceFire(mazeCopy,fire)
        
        probMat=[[0 for x in range(size)] for y in range(size)] #generate a matrix that will store probabilites of a node catching fire
        for i in range(size):       #calculating the probability of node catching fire and storing it in probMat
            for j in range(size):
                k=0
                if mazeCopy[i][j]!=5 and mazeCopy[i][j]!=1:
                    if((i-1 >=0) and mazeCopy[i-1][j] == 5):
                        k+=1
                    if((j-1 >=0) and mazeCopy[i][j-1] == 5):
                        k+=1
                    if((i+1 <size) and mazeCopy[i+1][j] == 5):
                        k+=1
                    if((j+1 <size) and mazeCopy[i][j+1] == 5):
                        k+=1
                    probMat[i][j]=1-(math.pow((1-fire),k))
                elif mazeCopy[i][j]==5:
                    probMat[i][j]=1
        
        path = aStar(maze,pntr,goal,size,probMat)[1] #use aStar algo but input probMat has heur input making it so the path shies away nodes with higher probabilites of being flammable
        if path is None:    #no path, return 0
            return 0
        pathsize = len(path)
        maze = advanceFire(maze, fire) #advance the fire
        pntr = path[1]      #move pointer to second node in path as that os the next step
        if maze[pntr[0]][pntr[1]] == 5:     #agent is on fire, return 0 for failure
            return 0
        if pntr==goal:      #agent made it to the goal, return 1 for success
            return 1
    return 0   
            
    

def advanceFire(maze,q):
    mazeCopy=[[0 for x in range(size)] for y in range(size)]    #make a copy of the array as to not factor in newly calculated fires in current calculations.
    for i in range(size):
        mazeCopy[i]=maze[i].copy()
    fire=5      #value we chose to represent nodes on fire

    #goes through each node and counts the neighbors that are on fire
    for i in range(size):
        for j in range(size):
            k=0
            if maze[i][j]!=fire and maze[i][j]!=1:
                if((i-1 >=0) and maze[i-1][j] == fire):
                    k+=1
                if((j-1 >=0) and maze[i][j-1] == fire):
                    k+=1
                if((i+1 <size) and maze[i+1][j] == fire):
                    k+=1
                if((j+1 <size) and maze[i][j+1] == fire):
                    k+=1
                prob=1-(math.pow((1-q),k))      #calculated probability of flammability
                if(random.uniform(0, 1)<=prob):     #running a random number generator between 0 and 1 and if it is less than the calculated probability, the node is now on fire
                    mazeCopy[i][j]=fire
    return mazeCopy     #return the copy of the maze
        

def mazeGen(size, density):
    maze=[[0 for x in range(size)] for y in range(size)]    #make a matrix with the desired size filling it with zeros

    for i in range(size):
        for j in range(size):
            if(i==j and (i==0 or i==size-1)): #for each node that is not the start or end run a rng to see if it below the desired density, if it is, make the node into 1 signifing a roadblock
                continue
            temp= random.uniform(0, 1)
            if(density>=temp):
                maze[i][j]=1
    count=1
    while 1: #used to generate fire
        i=random.randrange(0, size-1)   #generate random x value
        j=random.randrange(0, size-1)   #generate random y value

        if ((i!=j and (i!=0 or i!=size-1)) and maze[i][j]!=1):  #if x and y do not signify start or end node nor signify a road block, signify node as fire and break
            maze[i][j]=5
            break
        if(count==2*size**2):     #if fire generations runs for 2*size^2 times, it is possible that the maze doesnt have a open spot
            break
        count+=1
        
    return maze     #return the generated maze

size= int(input("Size of Maze? "))


#=======================================================================================================================================

#code used to test


##density= float(input("Obstacle Density? "))
print("\n")


#print("size: ",size, " Density: ", density)


#print(maze)


trials=50
fire=0
totalTime=0

for density in range(0, 11,1):

    success=0
    density=density/10
    bfsNodes=0
    aStarNodes=0
##    stratThree=0
    for i in range(trials):
        
        maze=mazeGen(size, density)

##        for i in range(size):
##            print(maze[
##        stratOne+=strategy1(maze, (0,0),(size-1,size-1),size,fire)
##        stratTwo+=strategy2(maze, (0,0),(size-1,size-1),size,fire)
##        start_time = time.time()
##        stratThree+=strategy3(maze, (0,0),(size-1,size-1),size,fire)
##        end_time = time.time()
##        hitime=end_time-start_time
##        #print("Time: "+str(hitime))
##        totalTime+=(end_time-start_time)
####        print("\n")

##        dfsRes=dfs(maze, (0,0),(size-1,size-1),size)
##        print("dfs: "+str(dfsRes[0]) + " length: " +str(dfsRes[2]))

##        start_time = time.time()
        bfsRes=bfs(maze, (0,0),(size-1,size-1),size)
##        end_time = time.time()
        bfsNodes+= bfsRes[3]
##        print(bfsRes[3])
##        print("bfs: "+str(bfsRes[0]) + " length: " +str(bfsRes[2]) + " time: " + str(end_time-start_time))
    
##        start_time = time.time()    
        aStarRes=aStar(maze,(0,0),(size-1,size-1),size,None)
        aStarNodes+= aStarRes[3]
##        print(aStarRes[3])
##        end_time = time.time()
##        print("aStar: "+str(aStarRes[0]) + " length: " +str(aStarRes[2])+ " time: " + str(end_time-start_time))
    
##        print("\n")

##    stratOne=stratOne/trials
##    stratTwo=stratTwo/trials
##    stratThree=stratThree/trials

    aStarNodes=aStarNodes/trials
    bfsNodes=bfsNodes/trials
    
    print("density: "+str(density)+" Difference: "+str(bfsNodes-aStarNodes))
    print("\n")
#print(totalTime)









