#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import gurobipy as gp
import numpy as np
import sys
import time
from gurobipy import GRB
import matplotlib.pyplot as plt
#from parameter import *

# # Parameters for Setup
Period_duration=600/3600 # Duration of each period in hrs
Working_Period=4# working period lenght in hrs

T = int(Working_Period/Period_duration)  # Number of periods
R = 3   # Number of robots
C = 3   # Number of charging stations
S=2     # Number of sensors installed on robots
W = 5   # Number of non-navigation tasks
W_N=2  # Number of navigation tasks

q=1
Q_Battery_Weight=q*T*W*W_N/R  # Importance of Battery lifetime on cost function

# create sets
Times = range(0, T)     # Set of periods
Robots = range(0, R)    # Set of robots
Charging_stations = range(0, C) # Set of charging stations
Sensors = range(0, S)   # Set of sensors
Non_Navigation_Tasks = range(0, W)  # Set of non-navigation tasks, e.g., face recognition
Navigation_Tasks=range(0,W_N) # Set of navigation paths

# #creating sets for including maintenance
maintenance_robots = {(i):0 for i in Robots} #set of robots that needs maintenance (MR)
# maintenance_robots[1]= 1 # robot 1 is selected for maintenance
no_of_main_robots = 1
main_robot_list = [2]  ##robots that are selected for maintenance
for i in Robots:
    if i in main_robot_list:
        maintenance_robots[i] = 1

main_dur = 4  #length of maintenance period
main_solution_period= range(0, T- main_dur)   
maintenance_period = range (0, main_dur)

# Battery Energy Levels
Ebat = 111.0     # Battery capacity (Wh)
Edod = round(0.3 * Ebat)   # Depth of Discharge
Emax = round(0.8 * Ebat)  # Preferred max charge level
E_end_min=Edod # desired min energy at end of working period.
Charging_time=0.75 # hrs to completely recharge the battery
Charging_rate=Ebat/Charging_time*Period_duration #Wh recharged during an entire period in charging mode

# # Parameters that must be received from function (TO MODIFY)
#E_Balance_Zero={i:np.random.uniform(low=Edod, high=Ebat, size=None) for i in Robots}     # Initial energy balance (Wh)
#E_Balance_Zero=[80, 40,80]
# E_Balance_Zero={0: 72.9214885234359, 1: 55.82778271484023}

E_Balance_Zero = {0: 60, 1: 106, 2: 106}
Charge_State_Zero={i:0 for i in Robots}       # Initial charge state, i.e., charging(1)/not_charging(0). 


# Task-related Parameters
M_Nav={h:1*10**-1 for h in Navigation_Tasks}
    #M_Nav=np.random.uniform(low=0.1*10**-1, high=3*10**-1, size=W_N)
    #M_Nav=[3.499262193048623126e-02,2.233270584866454966e-01]

M_Task={j:3*10**-1 for j in Non_Navigation_Tasks}
    #M_Task=np.random.uniform(low=0.3*10**-1, high=9*10**-1, size=W)
    #M_Task=[5.834735384327416341e-01,6.063307144569393126e-01,7.921409694627232212e-02,3.743802826951356799e-01,9.024889641913197424e-02]



M_Max={i:20*10**-1 for i in Robots}
Priority={j:1 for j in Non_Navigation_Tasks} #(0,1)
#Priority=np.random.randint(1,10,size=W)/10
#Priority=[2.999999999999999889e-01,9.000000000000000222e-01,5.999999999999999778e-01,4.000000000000000222e-01,2.999999999999999889e-01]


# Computing and Sensing Coefficients and Parameters
Locomotion_Power=21 # in W
Sensing_Power={}
Sensing_Power[0]=(1.3+2.2) # camera power in W
Sensing_Power[1]=(1.9+0.9) # Lidar power in W

Alpha_Loc={h:Locomotion_Power*Period_duration for h in Navigation_Tasks} # Locomotion Coefficient
Alpha_Comp={i:2.5/0.279166818*Period_duration for i in Robots} # Computing Coefficient

Alpha_Sensing={}
Alpha_Sensing[0]= Sensing_Power[0]*Period_duration # Camera coefficient (Wh)
Alpha_Sensing[1]= Sensing_Power[1]*Period_duration  # Lidar coefficient (Wh)

Avg_Access_Time={l:0.01 for l in Sensors}  # Access time for Sensors in seconds

Task_Inference_Time={j:M_Task[j]*0.539/(0.279166818) for j in Non_Navigation_Tasks} # Inference Time for non-navigation tasks (sec)
Nav_Inference_Time={h:M_Nav[h]*0.539/(0.279166818) for h in Navigation_Tasks} # Inference Time for non-navigation tasks (sec)

Tasks_Sensing_Rate={(j,l):1/Task_Inference_Time[j] for j in Non_Navigation_Tasks for l in Sensors} # samples/sec
Nav_Sensing_Rate={(h,l):1/Nav_Inference_Time[h] for h in Navigation_Tasks for l in Sensors} # samples/sec

# Parameters for Robot navigation and distance from stations
Robot_Speed=1*3600 # Average robot speed in meter/hrs
Max_distance={h:200 for h in Navigation_Tasks} # max distance between navigation paths and fartest charging station (meters)
E_NavMax= {h:Locomotion_Power*Max_distance[h]/Robot_Speed for h in Navigation_Tasks} # max energy necessary to navigate the robot back to a charging station (Wh)

Dist_change_max=500 # max distance to change navigation task or to go to a charging station
#E_changeMax=Locomotion_Power*Dist_change_max/Robot_Speed # max energy spent due to changing nav task or to go to recharge
E_changeMax = (Locomotion_Power+Sensing_Power[0]+Sensing_Power[1]+2.5+0.8)*Dist_change_max/Robot_Speed 

Y={(k,j):1 for k in Times for j in Non_Navigation_Tasks}
Gamma_Matrix={(h,j):1 for h in Navigation_Tasks for j in Non_Navigation_Tasks}

start_time = time.time()
# Create a new model
m = gp.Model("qp")
u = {}
x = {}
z = {}
e = {}
e_charged={}
e_res_Task={}
e_res_Nav={}
e_other={}
a = {}
b = {}
aux1={}
aux2={}
aux3={}
Robot_Navigation_state={}
y={}

obj1 = 0
obj2 = 0
aux3_sum = 0
W_obj2=0

for k in Times:
    for i in Robots:
        e[k, i] = m.addVar(vtype=GRB.CONTINUOUS, name='e_(%s,%s)' % (k, i)) #Energy balance
        e_res_Nav[k, i] = m.addVar(vtype=GRB.CONTINUOUS, name='e_res_Nav_(%s,%s)' % (k, i)) #
        e_res_Task[k, i] = m.addVar(vtype=GRB.CONTINUOUS, name='e_res_Task(%s,%s)' % (k, i)) #
        e_other[k, i] = m.addVar(vtype=GRB.CONTINUOUS, name='e_other(%s,%s)' % (k, i)) #
        e_charged[k, i] = m.addVar(vtype=GRB.CONTINUOUS, name='e_charged(%s,%s)' % (k, i)) #
for k in Times:
    for i in Robots:
        for j in Non_Navigation_Tasks:
            for h in Navigation_Tasks:
                x[k, i, h,j] = m.addVar(vtype=GRB.BINARY,name='x_(%s,%s,%s,%s)'%(k,i,h,j))

for k in Times:
    for j in Non_Navigation_Tasks:
        for h in Navigation_Tasks:
            obj1 = obj1 + Priority[j]*(Y[(k,j)]- sum(x[k,i,h,j] for i in Robots))


for k in Times:
    for i in Robots:
        for h in Navigation_Tasks:
            Robot_Navigation_state[k,i,h] = m.addVar(vtype=GRB.BINARY,name='Robot_Navigation_state[%s,%s,%s]'%(k,i,h))
            y[k,i,h]= m.addVar(vtype=GRB.BINARY,name='y[%s,%s,%s]'%(k,i,h))
            
            
            
for k in Times:
    for i in Robots:
        for c in Charging_stations:
            z[k, i, c] = m.addVar(vtype=GRB.BINARY,name='z_(%s,%s,%s)'%(k,i,c))
            a[k, i, c] = m.addVar(vtype=GRB.BINARY,name='a_(%s,%s,%s)'%(k,i,c))
            b[k, i, c] = m.addVar(vtype=GRB.BINARY,name='b_(%s,%s,%s)'%(k,i,c))
        aux1[k, i] = m.addVar(vtype=GRB.CONTINUOUS,name='aux1_(%s,%s)'%(k,i))
        aux2[k, i] = m.addVar(vtype=GRB.CONTINUOUS,name='aux2_(%s,%s)'%(k,i))
            
for k in Times:
    for i in Robots:
        #for h in Navigation_Tasks:
        aux3[k,i] = m.addVar(vtype=GRB.BINARY,name='aux3_(%s,%s)'%(k,i))
        
#defining decision variable for maintenance u[m,i]
if main_dur>0:
    for p in main_solution_period:
        for i in Robots:
            u[p, i] = m.addVar(vtype=GRB.BINARY,name='u_(%s,%s)'%(p,i))  #maintenane variable
            
#Define maintenance_solution_matrix:
if main_dur>0:
    
    msol = {(p,k):0 for p in main_solution_period for k in Times}


for p in main_solution_period:
        for temp in maintenance_period:
            msol[p,p+temp] = 1

    
m.update()

for k in Times:
    for i in Robots:
        for c in Charging_stations:
            obj2 = obj2 + a[k, i, c] * aux1[k,i] + b[k, i, c] * aux2[k,i]
                
for k in Times:
    for i in Robots:
        #for h in Navigation_Tasks:
            aux3_sum=aux3_sum + aux3[k,i]        
        
W_obj2= Q_Battery_Weight*obj2

obj = obj1 + W_obj2

m.update()

#Initialization
for i in Robots:
    e[-1,i]=E_Balance_Zero[i]
    for h in Navigation_Tasks:
        Robot_Navigation_state[-1,i,h]=0
        
        
    for c in Charging_stations:
        z[-1, i, c] = Charge_State_Zero[i]


for k in Times:
    for h in Navigation_Tasks:
        for j in Non_Navigation_Tasks:
            m.addConstr(sum(x[k, i, h,j] for i in Robots) <= Y[(k,j)], name="Task_Activitiy_Status_[%s,%s,%s]" % (k,h,j)) #Constraint 4

for k in Times:
    for i in Robots:
        for h in Navigation_Tasks:
            m.addConstr(sum(x[k, i, h,j] for j in Non_Navigation_Tasks) >= Robot_Navigation_state[k,i,h], name="Nav_Activitiy_Status_[%s,%s,%s]" % (k,i,h)) #Constraint 4bis to avoid that a navigation task is allocated without non-navigation tasks.

for k in Times:
    for i in Robots:
        for h in Navigation_Tasks:
            for j in Non_Navigation_Tasks:
                m.addConstr(x[k, i, h,j]<= Gamma_Matrix[(h,j)], name="State_consistency_[%s,%s,%s,%s]" % (k, i,h,j)) #Constraint 5

for k in Times:
    for i in Robots:
        for h in Navigation_Tasks:
            for j in Non_Navigation_Tasks:
                m.addConstr(Robot_Navigation_state[k,i,h]>=x[k,i,h,j], name="MAX_[%s,%s,%s,%s]" % (k, i,h,j)) #Constraint 6 part1 to force the variable Robot_Navigation_state[k,i,h] to be equal to 0 if all the x are zeros.
                

for k in Times:
    for i in Robots:
        m.addConstr(sum(z[k, i, c] for c in Charging_stations)+sum(Robot_Navigation_state[k,i,h] for h in Navigation_Tasks)<=1, name="State_consistency_[%s,%s,%s]" % (k, i,h)) #Constraint 6 part 2


for k in Times:
    m.addConstr(sum(z[k, i, c]  for c in Charging_stations for i in Robots) <= C, name="Limited_stations_[%s]" % (k)) #Constraint 7


for k in Times:
    for i in Robots:
        m.addConstr(sum(Robot_Navigation_state[k, i, h]*M_Nav[h] for h in Navigation_Tasks)+sum(x[k,i,h,j]*M_Task[j] for h in Navigation_Tasks for j in Non_Navigation_Tasks) <= M_Max[i], name="Limited_Resource_capacity_[%s,%s]" % (k, i)) #Constraint 8


for k in Times:
    for i in Robots:
        #m.addConstr(e[k,i]>=sum(Robot_Navigation_state[k,i,h]*E_NavMax[h] for h in Navigation_Tasks), name="Min_Energy_Charge_[%s,%s]" % (k, i))  #Constraint 9
        m.addConstr(e[k,i]>=0, name="Min_Energy_Charge_[%s,%s]" % (k, i))  #Constraint 9
        if k>=T-1:
            m.addConstr(e[k,i]>=E_end_min, name="Min_Energy_End_[%s,%s]" % (k, i))  #Constraint 9b
        
        
for k in Times:
    for i in Robots:
        m.addConstr(e[k,i]==e[k-1,i]+e_charged[k,i]-e_res_Nav[k,i]-e_res_Task[k,i]-e_other[k,i], name="Energy_Balance_[%s,%s]" % (k, i))  #Constraint 10_1
        m.addConstr(e_charged[k,i]==sum(z[k, i, c]*Charging_rate for c in Charging_stations), name="Energy_Charge_[%s,%s]" % (k, i))  #Constraint 10_1
        m.addConstr(e_res_Task[k,i]==Alpha_Comp[i]*sum(x[k,i,h,j]*M_Task[j] for h in Navigation_Tasks for j in Non_Navigation_Tasks)+sum(Alpha_Sensing[l]*x[k,i,h,j]*Tasks_Sensing_Rate[(j,l)]*Avg_Access_Time[l] for l in Sensors for h in Navigation_Tasks for j in Non_Navigation_Tasks), name="Energy_Res_Task_[%s,%s]" % (k, i))  #Constraint 10_1
        m.addConstr(e_res_Nav[k,i]==Alpha_Comp[i]*sum(Robot_Navigation_state[k,i,h]*M_Nav[h] for h in Navigation_Tasks)+sum(Alpha_Sensing[l]*Robot_Navigation_state[k,i,h]*Nav_Sensing_Rate[(h,l)]*Avg_Access_Time[l] for l in Sensors for h in Navigation_Tasks), name="Energy_Res_Nav[%s,%s]" % (k, i))  #Constraint 10_1
        #m.addConstr(e_other[k,i]==sum(Alpha_Loc[h]*Robot_Navigation_state[k,i,h] for h in Navigation_Tasks), name="Energy_Other[%s,%s]" % (k, i))  #Constraint 10_1
        m.addConstr(e_other[k,i]==sum(Alpha_Loc[h]*Robot_Navigation_state[k,i,h]  for h in Navigation_Tasks) + E_changeMax*aux3[k,i], name="Energy_Other[%s,%s]" % (k, i))  #Constraint 10_1
        
       
for k in Times:
    for i in Robots:
        m.addConstr(e[k,i]<=Ebat, name="Max_Energy_Stored_[%s,%s]" % (k, i))  #Constraint 10

for k in Times:
    for i in Robots:
        for c in Charging_stations:
            m.addConstr(a[k, i, c] == (1 - z[k - 1, i, c]) * z[k, i, c], name="Aux_[%s,%s,%s]" % (k, i, c))
            m.addConstr(aux1[k, i] >= ((e[k-1, i] - Edod) / Ebat), name="Aux3_[%s,%s,%s]" % (k, i, c))
            m.addConstr(aux1[k, i] >= -((e[k-1, i] - Edod) / Ebat), name="Aux3_[%s,%s,%s]" % (k, i, c))
            
            m.addConstr(b[k, i, c] == z[k - 1, i, c] * (1 - z[k, i, c]), name="Aux2_[%s,%s,%s]" % (k, i, c))
            m.addConstr(aux2[k, i] >= ((Emax - e[k-1, i]) / Ebat), name="Aux4_[%s,%s,%s]" % (k, i, c))
            m.addConstr(aux2[k, i] >= - ((Emax - e[k-1, i]) / Ebat), name="Aux4_[%s,%s,%s]" % (k, i, c))



for k in Times:
    for i in Robots:
          #m.addConstr(aux3[k,i] == sum((Robot_Navigation_state[k-1,i,h]*(1-Robot_Navigation_state[k,i,h]) + ((1-Robot_Navigation_state[k-1,i,h])*Robot_Navigation_state[k,i,h])) for h in Navigation_Tasks), name="Aux3_[%s,%s]" % (k, i))
          m.addConstr(aux3[k,i] == sum( y[k,i,h] for h in Navigation_Tasks), name="Aux3_[%s,%s]" % (k, i))
         
         
for k in Times:
    for i in Robots:   
        for h in Navigation_Tasks:
            m.addConstr(y[k,i,h]<= Robot_Navigation_state[k-1,i,h] + Robot_Navigation_state[k,i,h] , name="Y_[%s,%s,%s]" % (k, i, h))
            m.addConstr(y[k,i,h]>= Robot_Navigation_state[k-1,i,h] - Robot_Navigation_state[k,i,h] , name="Y_[%s,%s,%s]" % (k, i, h))
            m.addConstr(y[k,i,h]>= Robot_Navigation_state[k,i,h] - Robot_Navigation_state[k-1,i,h] , name="Y_[%s,%s,%s]" % (k, i, h))
            m.addConstr(y[k,i,h]<= 2 - Robot_Navigation_state[k-1,i,h] - Robot_Navigation_state[k,i,h] , name="Y_[%s,%s,%s]" % (k, i, h))
         
m.update()       
 #Defining constraints for maintenance
if main_dur>0:   
    for k in Times:
        for i in Robots:
            for h in Navigation_Tasks:
                for j in Non_Navigation_Tasks:
                    for p in main_solution_period:
                        m.addConstr(x[k, i, h,j] <= 1-u[p,i]*msol[p,k], name="no_task_allocated_while_in_maintenance_[%s,%s,%s,%s,%s]" % (k, i,h,j,p))
    
    for i in Robots:
        m.addConstr(sum(u[p,i] for p in main_solution_period)== maintenance_robots[i], name="only_one_robots_goes_to_maintenance_[%s]" % (i))
    
    for k in Times:
        for i in Robots:
            for c in Charging_stations:
                for p in main_solution_period:
                    m.addConstr(z[k, i, c] <= 1-u[p,i]* msol[p,k], name="no_CS_will_be_allocated_while_in_maintenance_[%s,%s,%s,%s]" % (k, i,c,p))

m.update() 
#*********************************************


m.setObjective(obj)
m.Params.MIPgap = 0.01
m.setParam(GRB.Param.TimeLimit, 100.0)
m.update()
m.write('problem.lp')
m.optimize()
end_time = time.time()
total_time = end_time - start_time
print('Obj: %g' % obj.getValue())

print('Obj1: %g' % obj1.getValue())
print('Obj2: %g' % obj2.getValue())
print('Obj2 weighted: %g' % W_obj2.getValue())
print('Aux3_sum: %g' % aux3_sum.getValue())
print("total time:", total_time)



#*********************************************    

#for v in m.getVars():
#    print('%s %g' % (v.varName, v.x))



# for i in Robots:
#     for k in Times:
#         #print(m.getVarByName('e_(%s,%s)'%(k,i))) 
#         print(m.getVarByName('aux3_(%s,%s)'%(k,i)))
#         for h in Navigation_Tasks:
#             #
#             print(m.getVarByName('Robot_Navigation_state[%s,%s,%s]'%(k,i,h)))
#             #sum((m.getVarByName('Robot_Navigation_state[%s,%s,%s]'%(k-1,i,h)).x*(1-m.getVarByName('Robot_Navigation_state[%s,%s,%s]'%(k,i,h)).x) + ((1-m.getVarByName('Robot_Navigation_state[%s,%s,%s]'%(k-1,i,h)).x)*m.getVarByName('Robot_Navigation_state[%s,%s,%s]'%(k,i,h)).x)) for h in Navigation_Tasks)


# for k in Times:
#     for i in Robots:
#         print(m.getVarByName('e_(%s,%s)' % (k, i)) )       

Task_Downtime=np.zeros((T+1, W_N))
for k in Times:
    for h in Navigation_Tasks:
        for j in Non_Navigation_Tasks:
            Task_Downtime[k+1,h]=Task_Downtime[k+1,h]+(Y[k,j]-sum(m.getVarByName('x_(%s,%s,%s,%s)'%(k,i,h,j)).x for i in Robots))
    
allocated_tasks=np.zeros((T+1, R, W_N))
for i in Robots:
    for h in Navigation_Tasks:
        for k in Times:
            for j in Non_Navigation_Tasks:
                #print(m.getVarByName('x_(%s,%s,%s,%s)'%(k,0,0,j)))
                allocated_tasks[k+1,i,h]= allocated_tasks[k+1,i,h]+m.getVarByName('x_(%s,%s,%s,%s)'%(k,i,h,j)).x

# allocation_x = np.zeros((T, R, W_N, W))
# for i in Robots:
#     for h in Navigation_Tasks:
#         for k in Times:
#             for j in Non_Navigation_Tasks:
#                 # print(m.getVarByName('x_(%s,%s,%s,%s)'%(k,0,0,j)))
#                 allocation_x[k, i, h, j] = m.getVarByName('x_(%s,%s,%s,%s)' % (k, i, h, j)).x

charging_allocation=np.zeros((T+1, R))
for i in Robots:
    charging_allocation[0,i]=Charge_State_Zero[i]
    for k in Times:
        for c in Charging_stations:
            #print(m.getVarByName('x_(%s,%s,%s,%s)'%(k,0,0,j)))
            charging_allocation[k+1,i]=charging_allocation[k+1,i]+m.getVarByName('z_(%s,%s,%s)'%(k,i,c)).x

state_of_charge=np.zeros((T+1, R))
for i in Robots:
    state_of_charge[0,i]=E_Balance_Zero[i]/Ebat*100
    for k in Times:
            #print(m.getVarByName('x_(%s,%s,%s,%s)'%(k,0,0,j)))
           state_of_charge[k+1,i]=m.getVarByName('e_(%s,%s)'%(k,i)).x /Ebat*100

variable_aux3 = np.zeros((T,R))
for k in Times:
    for i in Robots:
        variable_aux3[k,i] = m.getVarByName('aux3_(%s,%s)'%(k,i)).x    

maintenance_opt=np.zeros((T-main_dur, R))
for p in main_solution_period:
        for i in Robots:
            maintenance_opt[p, i] = m.getVarByName('u_(%s,%s)'%(p,i)).x  #maintenane variable
            



for i in Robots:
    state_of_charge[0,i]=E_Balance_Zero[i]/Ebat*100
    for k in Times:
            #print(m.getVarByName('x_(%s,%s,%s,%s)'%(k,0,0,j)))
           state_of_charge[k+1,i]=m.getVarByName('e_(%s,%s)'%(k,i)).x /Ebat*100
    
SoC_violation = np.zeros((T, R))    
for k in Times:
    for i in Robots:
        SoC_violation[k,i] = max((state_of_charge[k,i] - Emax), 0) + max((Edod - state_of_charge[k,i]), 0)

Soc_vio = 0        
for k in Times:
    Soc_vio = Soc_vio + sum((SoC_violation[k,i]) for i in Robots)
final_soc_violation = (100/Ebat) * Soc_vio        

print("soc violation:", final_soc_violation)
    
for h in Navigation_Tasks:
    plt.bar(range(0,T+1),Task_Downtime[:,h])
    plt.ylabel('# of Unallocated Tasks for Navigation %s'%(h),fontweight='bold')
    plt.xlabel('Time Period',fontweight='bold')
    plt.xlim([0,T+1])
    plt.show()
    plt.clf()


for i in Robots:
    for h in Navigation_Tasks:
        plt.bar(range(0,T+1),allocated_tasks[:,i,h],label="Navigation Task %s"%h)
    plt.ylabel('Number of Assigned Tasks to Robot %s '%(i),fontweight='bold')
    plt.xlabel('Time Period',fontweight='bold')
    plt.xlim([0,T+1])
    plt.ylim(0, W+3)
    plt.legend(loc="best")
    plt.show()
    plt.clf()


for i in Robots:
    plt.bar(range(0,T+1),charging_allocation[:,i],label="Robot %s"%i)
    plt.ylabel('Charging State',fontweight='bold')
    plt.xlabel('Time Period',fontweight='bold')
    plt.legend(loc="best")
    plt.xlim([0,T])
    plt.show()
    plt.clf()
    
    

for i in Robots:
    plt.bar(main_solution_period,maintenance_opt[:,i],label="Robot %s"%i)
    plt.ylabel('Maintenance Start Period for Robot %s'%(i),fontweight='bold')
    plt.xlabel('Time Period',fontweight='bold')
    plt.legend(loc="best")
    plt.xlim([0,T])
    plt.show()
    plt.clf()
####SoC plot    
plt.plot(range(0, T + 1), state_of_charge[:, 0], linestyle='-', color='darkorange', label="AMR 0")  
plt.plot(range(0, T + 1), state_of_charge[:, 1], linestyle='dashed', color='blue', label="AMR 1")
plt.plot(range(0, T + 1), state_of_charge[:, 2], linestyle='dashdot', color='green', label="AMR 2")  

plt.axhline(y=Emax / Ebat * 100, color='r', linestyle='-', label="Emax")
plt.axhline(y=Edod / Ebat * 100, color='r', linestyle='-', label="Edod")
# plt.axhline(y=E_changeMax, color='black', linestyle='-', label="lowest_level")
# plt.axhline(y=Ebat, color='black', linestyle='-', label="highest_level")
# plt.axhline(y=0, color='blue', linestyle='--', label="Edod")
# plt.axhline(y=100, color='blue', linestyle='--', label="Edod")
plt.ylabel('State of Charge (%)', fontsize = 14, fontweight='bold')
plt.xlabel('Time Periods', fontsize = 14,fontweight='bold')
plt.xlim([0, T])
# plt.ylim([-20, 110]) 
plt.ylim([0, 119]) 

plt.xticks(range(0,T+1, 2), fontsize = 14)#
plt.yticks(fontsize = 14)
for l in range(0, T, 2):
    plt.axvline(x=[l], color='grey', alpha=0.1)
# plt.legend(bbox_to_anchor=(0., -0.5, 1., -0.11), loc='best',
#             ncol=5, mode="expand", borderaxespad=0.)
plt.legend(ncol =3, fontsize = 12, loc ="upper right", bbox_to_anchor=(1.02, 1.03))
plt.show()
plt.clf() 
# =============================================================================
# for i in Robots:
#     plt.plot(range(0,T+1),state_of_charge[:,i], label="Robot %s"%i)
# plt.axhline(y=Emax/Ebat*100, color='r', linestyle='-', label ="Emax")
# plt.axhline(y=Edod/Ebat*100, color='r', linestyle='--', label ="Edod")
# plt.ylabel('State of Charge (%)',fontweight='bold')
# plt.xlabel('Time Period',fontweight='bold')
# plt.xlim([0,T])
# plt.ylim([0,100])
# plt.legend(loc="best")
# plt.show()
# plt.clf()
# =============================================================================
