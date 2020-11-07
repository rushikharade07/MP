# agent.py

# AUthor :: sdfsdfsdf

# NIkhil edit


import numpy as np
from math import sqrt

class Agent(object):

    def __init__(self, csvParameters, dhor = 8, goalRadiusSq=1):

        self.id = int(csvParameters[0]) # the id of the agent
        self.gid = int(csvParameters[1]) # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])]) # the position of the agent 
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])]) # the goal of the agent
        self.prefspeed = float(csvParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(sqrt(self.gvel.dot(self.gvel )))*self.prefspeed       
        self.maxspeed = float(csvParameters[7]) # the maximum sped of the agent
        self.radius = float(csvParameters[8]) # the radius of the agent
        self.goalRadiusSq =goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.dhor = dhor # the sensing radius
        self.vnew = np.zeros(2) # the new velocity of the agent     
    
    # tried declaring a function, had issues with calling it
    # def ttc(self,nagent):
    #     rad = np.sqrt(np.random.uniform(0, 4, n))
    #     posi = np.pi * np.random.uniform(0, 2, n)
    #     c = w.dot(w) - rad.rad
    #     if c < 0:
    #         return 0
    #     rel_vel = self.vel - nagent.vel
    #     a = relvel.dot(rel_vel)
    #     b = posi.dot(posi)
    #     if b>0:
    #         return float('inf')
    #     discr = b*b -a*c
    #     if discr<=0:
    #         return  float('inf')
    #     tau = c/(-b+sqrt(discr))
    #     if tau<0:
    #         return float('inf')
    #     return tau

    def computeNewVelocity(self, neighbors=[]):

        neighbouring_agents = [] #defining neighbouring agents list
        cost_function = []  # defining cost function list

        # N = 500 # setting the number of sampling velocities

        # Extra Credit -
        N = 500 # set N = 200 to simulate crossing_agents

        #setting sampling parameters
        radius = np.sqrt(np.random.uniform(0, 4, N))
        theta = np.pi * np.random.uniform(0, 2, N)

        print(radius)
        print("---------------------")
        # setting candidate positions
        cd_x = radius * np.cos(theta)
        cd_y = radius * np.sin(theta)

        # defining sampling velocity
        vel_sample = np.array([cd_x, cd_y])
        vel_sample = vel_sample.transpose()
        # print(vel_sample)
        # print("---------------------")

        # finding time to collision
        for cdv in vel_sample:
            ttc = []    # declaring a list of time to collision
            for agent in neighbors:
                if agent != self:
                    distance = sqrt(((self.pos[0] - agent.pos[0]) ** 2) + (self.pos[1] - agent.pos[1]) ** 2)    # Calcualting the distance between two agents
                    if distance <= self.dhor:
                        neighbouring_agents.append(agent)
                        radius = self.radius + agent.radius
                        position = agent.pos - self.pos
                        c = position.dot(position) - radius * radius
                        relative_velocity = agent.vel - cdv
                        a = relative_velocity.dot(relative_velocity)
                        b = position.dot(relative_velocity)
                        discr = b * b - a * c
                        if discr >= 0 and b < 0:
                            tau = c / (-b + sqrt(discr))
                            if tau <= 0:
                                tau = 0
                        else:
                            tau = float('inf')
                    else:
                        tau = float('inf')
                    ttc.append(tau)
            tc = min(ttc) # calculating minimum time to collision of the two

            # Defining parameters for cost function
            alpha = 1
            beta = 1
            gamma = 2
            cost_function_1 = (alpha*(sqrt((cdv- self.gvel).dot((cdv - self.gvel))))) + (beta* (sqrt((cdv - self.vel). dot((cdv - self.vel)))))

            if tc != 0:
                cost_function_2 = gamma / tc
                cost_function.append(cost_function_1 + cost_function_2)
            else:
                cost_function.append(float('inf'))

        # Indexing least of cost function
        index = np.argmin(cost_function)
        min_cost_function = min(cost_function)
        new_velocity = np.array(vel_sample[index])

        if not self.atGoal:
            self.vnew[:] = new_velocity[:]   # Assigning new velocity to vnew


    def update(self, dt):

        if not self.atGoal:
            self.vel[:] = self.vnew[:]
            self.pos += self.vel*dt   #update the position
        
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed  
            
            
