
import gurobipy as gp
from gurobipy import GRB
import numpy as np
import sys
import matplotlib.pyplot as plt
import time
import math
from operator import itemgetter
import pandas as pd
import ast
import random 
from munkres import Munkres, print_matrix, DISALLOWED
import matplotlib.pyplot as plt


####file execution##### 
File_name = '1_exp.csv'
Setting_df=pd.read_csv(File_name)


    

def MTC_file_parameters(Exp_no):    # parameters collected from .csv file
    
    Period_duration = Setting_df.loc[Exp_no,'Period_duration']
    Working_Period = Setting_df.loc[Exp_no,'Working_Period']
    #T = itemgetter('T')(Parameters)
    T = math.ceil(Working_Period / Period_duration)  # Number of periods
    R = Setting_df.loc[Exp_no,"No_of_robots"]
    C = Setting_df.loc[Exp_no,"No_of_chargers"]
    S = Setting_df.loc[Exp_no,"No_of_sensors"]
    W = Setting_df.loc[Exp_no,"No_of_non_nav_tasks"]
    W_N = Setting_df.loc[Exp_no,"No_of_nav_task"]
    q = Setting_df.loc[Exp_no,"q"]
    
    Charging_time = Setting_df.loc[Exp_no,"Charging_time"]     

    # Battery Energy Levels
    Ebat = Setting_df.loc[Exp_no,'Ebat']  # Battery capacity (Wh)
    Edod = Setting_df.loc[Exp_no,'Edod']  # 0.2 * Ebat   # Depth of Discharge
    Emax = Setting_df.loc[Exp_no,'Emax'] # 1 * Ebat  # Preferred max charge level

    ###maintenance parameters
    # main_percentage = Setting_df.loc[Exp_no,'main_percentage']  # percentage of robots need to be in maintenance
    no_of_main_robots = Setting_df.loc[Exp_no,'No of robs in main']

    main_robot_list_string = Setting_df.loc[Exp_no,'list of main_robs']
    main_robot_list = ast.literal_eval(main_robot_list_string)
    main_dur = Setting_df.loc[Exp_no,'Duration of Main']
    main_dur = int(main_dur)
    
    # Locomotion_Power = Setting_df.loc[Exp_no,"Locomotion_Power"]

    Robot_Speed = Setting_df.loc[Exp_no,"Robot_Speed"]


    # Max_distance_nav = (Setting_df.loc[Exp_no,'Max_distance']) # max distance between navigation paths and fartest charging station (meters)
    # Max_distance = ast.literal_eval(Max_distance_nav) 
    
    
    Dist_change_max = Setting_df.loc[Exp_no,"Dist_change_max"] 
    
    
    gamma_string = Setting_df.loc[Exp_no,"Gamma_Matrix"]
    Gamma_Matrix = ast.literal_eval(gamma_string) 
    # Gamma_Matrix = {(0, 0): 1, (0, 1): 1, (0, 2): 1, (0, 3): 1, (0, 4): 1, (1, 0): 1, (1, 1): 1, (1, 2): 1, (1, 3): 0, (1, 4): 0}

    
    initial_charge_string = Setting_df.loc[Exp_no,"E_Balance_Zero"]
    E_Balance_Zero = ast.literal_eval(initial_charge_string)    
    
    # E_changeMax = Setting_df.loc[Exp_no,"E_changeMax"] 
    
    Priority_string = Setting_df.loc[Exp_no,"Priority"]
    Priority = ast.literal_eval(Priority_string) 
    
    

    # Locomotion_Power = (Setting_df.loc[Exp_no,'Locomotion_Power'])  # in W
    # sensing_power_string = Setting_df.loc[Exp_no,"Sensing_Power"]
    # Sensing_Power = ast.literal_eval(sensing_power_string)


    MTC_Parameters = { 'Exp_no':Exp_no, 'no_of_main_robots': no_of_main_robots, 'main_robot_list':main_robot_list, 'main_dur': main_dur, 'Period_duration': Period_duration, 
                  'Working_Period': Working_Period, 'T': T, 'R': R, 'C': C, 'S': S, 'W': W, 'W_N': W_N,'Ebat':Ebat, 'Edod':Edod, 'Emax':Emax, 'Charging_time': Charging_time, 'E_Balance_Zero': E_Balance_Zero,
                  'Dist_change_max':Dist_change_max, 'Priority':Priority, 'Gamma_Matrix':Gamma_Matrix,
                  'Robot_Speed':Robot_Speed,'q':q}
    

    
    return MTC_Parameters 



if 'TCM' not in Setting_df:
    Setting_df['TCM'] = 0
exp_start_time = time.time()

for Exp_no in range(0, len(Setting_df) ): # len(Setting_df)
    if Setting_df.loc[Exp_no,"TCM"] != 1 :  
        print(f"-----start of Exp_no: {Exp_no}-----")
        Parameters = MTC_file_parameters(Exp_no)
        # break 
#####################TCM function##########################

        Period_duration = itemgetter('Period_duration')(Parameters)
        Working_period = itemgetter('Working_Period')(Parameters)
        T = itemgetter('T')(Parameters)
        # T = math.ceil(Working_period/Period_duration)
        R = int(itemgetter('R')(Parameters))
        C = int(itemgetter('C')(Parameters))
        S = int(itemgetter('S')(Parameters))
        W = int(itemgetter('W')(Parameters))
        W_N = int(itemgetter('W_N')(Parameters))
        q = float(itemgetter('q')(Parameters))
        Ebat = itemgetter('Ebat')(Parameters)
        Edod = itemgetter('Edod')(Parameters)
        Emax = itemgetter('Emax')(Parameters)
        Charging_time = itemgetter('Charging_time')(Parameters)
        E_Balance_Zero = itemgetter('E_Balance_Zero')(Parameters)
        Priority = itemgetter('Priority')(Parameters)
        Gamma_Matrix = itemgetter('Gamma_Matrix')(Parameters)
        # Locomotion_Power = itemgetter('Locomotion_Power')(Parameters)
        # Sensing_Power = itemgetter('Sensing_Power')(Parameters)
        Robot_Speed = itemgetter('Robot_Speed')(Parameters)
        Dist_change_max = itemgetter('Dist_change_max')(Parameters)
        # main_percentage= float(itemgetter('main_percentage')(Parameters))
        no_of_main_robots= itemgetter('no_of_main_robots')(Parameters)
        main_robot_list= itemgetter('main_robot_list')(Parameters)
        main_dur = itemgetter('main_dur')(Parameters)
        main_dur = int(main_dur)
        E_end_min = Edod

        # create sets
        Times = range(0, T)     # Set of periods
        Ex_Times =  range(-1,T) # Set of extended periods which includes -1
        Robots = range(0, R)    # Set of robots
        Charging_stations = range(0, C)  # Set of charging stations
        Sensors = range(0, S)   # Set of sensors
        Non_Navigation_Tasks = range(0, W)  # Set of non-navigation tasks, e.g., face recognition
        Navigation_Tasks = range(0, W_N)  # Set of navigation paths
        print("times", Times)
        # creating sets for including maintenance
        # main_robot_list = random.sample(range(0, R), no_of_main_robots)##robots that are selected for maintenance
        maintenance_robots = {(i): 0 for i in Robots}
        # main_robot_list = []  ##robots that are selected for maintenance
        for i in Robots:
            if i in main_robot_list:
                maintenance_robots[i] = 1


        main_solution_period = range(0, (T - main_dur))
        maintenance_period = range(0, main_dur)
        print("main solution period", main_solution_period)
        random_maintenance = 1  ###indicates if we want random maintenance or not (o = no, 1= yes)
        single_exp = 1


        print("-------------output------------")



        print(f" priority array : {Priority}")
        # Priority = [0.3, 0.7, 0.9, 0.5, 0.4]
        # Priority=[2.999999999999999889e-01,9.000000000000000222e-01,5.999999999999999778e-01,4.000000000000000222e-01,2.999999999999999889e-01]
        j_p = max(Priority.keys(), key=(lambda x: Priority[x]))
        j_sort = sorted(Priority.keys(), key=lambda x: Priority[x], reverse=True)
        j_sort_ascending = sorted(Priority.keys(), key=lambda x: Priority[x], reverse=False)###keeps the obj task number that has lower to higher priority



        Charging_rate = Ebat/Charging_time*Period_duration  # Wh recharged during an entire period in charging mode
        Charge_State_Zero = {i: 0 for i in Robots} ### Initial charge state, i.e., charging(1)/not_charging(0).
        print(f"initial energy array {E_Balance_Zero}")

        print("###-------########")
        Q_Battery_Weight = T*W*W_N/R  # Importance of Battery lifetime on cost function
        batDegWeight = Q_Battery_Weight * q

        # Task-related Parameters
        M_Nav={h:1*10**-1 for h in Navigation_Tasks}
        #M_Nav=np.random.uniform(low=1.5*10**-1, high=2.5*10**-1, size=W_N)
        # M_Nav=[3.499262193048623126e-02,2.233270584866454966e-01]

        M_Task = {j: 3*10**-1 for j in Non_Navigation_Tasks}
        #M_Task=np.random.uniform(low=6.5*10**-1, high=8*10**-1, size=W)
        # M_Task=[5.834735384327416341e-01,6.063307144569393126e-01,7.921409694627232212e-02,3.743802826951356799e-01,9.024889641913197424e-02]

        M_Max = {i: 20*10**-1 for i in Robots}


        # Computing and Sensing Coefficients and Parameters
        Locomotion_Power = 21  # in W
        Sensing_Power = {}
        Sensing_Power[0]=(1.3+2.2) # camera power in W
        Sensing_Power[1]=(1.9+0.9) # Lidar power in W
        Sensing_Power = {0: 3.5, 1: 2.8}

        # Locomotion Coefficient
        Alpha_Loc = {h: Locomotion_Power*Period_duration for h in Navigation_Tasks}
        # Computing Coefficient
        Alpha_Comp = {i: 2.5/0.279166818*Period_duration for i in Robots}

        Alpha_Sensing = {}
        Alpha_Sensing[0] = Sensing_Power[0]*Period_duration  # Camera coefficient (Wh)
        Alpha_Sensing[1] = Sensing_Power[1]*Period_duration  # Lidar coefficient (Wh)

        # Access time for Sensors in seconds
        Avg_Access_Time = {l: 0.01 for l in Sensors}

        # Inference Time for non-navigation tasks (sec)
        Task_Inference_Time = {j: M_Task[j]*0.539 /(0.279166818) for j in Non_Navigation_Tasks}
        # Inference Time for non-navigation tasks (sec)
        Nav_Inference_Time = {h: M_Nav[h]*0.539 /(0.279166818) for h in Navigation_Tasks}

        Tasks_Sensing_Rate = {(j, l): 1/Task_Inference_Time[j] for j in Non_Navigation_Tasks for l in Sensors}  # samples/sec
        Nav_Sensing_Rate = {(h, l): 1/Nav_Inference_Time[h] for h in Navigation_Tasks for l in Sensors}  # samples/sec


        Max_distance = {h: 200 for h in Navigation_Tasks}
        # max energy necessary to navigate the robot to/from a charging station (Wh)
        E_NavMax = {h: Locomotion_Power * Max_distance[h]/Robot_Speed for h in Navigation_Tasks}

        ##from akshar
        E_changeMax = (Locomotion_Power+Sensing_Power[0]+Sensing_Power[1]+2.5+0.8)*Dist_change_max/Robot_Speed 

        Y = {(k, j): 1 for k in Times for j in Non_Navigation_Tasks}
        ##-------------------------------------------------------------------
        # initializing matrix to hold the task and charge allocation variables 

        allocation_x_rounded = np.zeros((T, R, W_N, W))
        allocation_z_rounded = np.zeros((T, R, C))
        # Robot_Navigation_state_rounded = np.zeros((T, R, W_N))
        Robot_Navigation_state_rounded = {(k, i, h): 0 for k in Ex_Times for i in Robots for h in Navigation_Tasks}
        variable_aux3_rounded = np.zeros((T, R))

        # maintenance_rounded = np.zeros((T-main_dur+1, R))
        main = {(i, k): 0 for i in Robots for k in Ex_Times}  ##to keep track of the maintenance periods
        # main = {(0, -1): 0, (0, 0): 0, (0, 1): 0, (0, 2): 0, (0, 3): 0, (0, 4): 0, (0, 5): 0, (0, 6): 0, (0, 7): 0, (0, 8): 0, (0, 9): 0, (0, 10): 0, (0, 11): 1, (0, 12): 1, (0, 13): 1, (0, 14): 1, (0, 15): 0, (0, 16): 0, (0, 17): 0, (1, -1): 0, (1, 0): 0, (1, 1): 0, (1, 2): 0, (1, 3): 0, (1, 4): 0, (1, 5): 0, (1, 6): 0, (1, 7): 0, (1, 8): 0, (1, 9): 0, (1, 10): 0, (1, 11): 0, (1, 12): 0, (1, 13): 0, (1, 14): 0, (1, 15): 0, (1, 16): 0, (1, 17): 0, (2, -1): 0, (2, 0): 0, (2, 1): 0, (2, 2): 0, (2, 3): 0, (2, 4): 0, (2, 5): 0, (2, 6): 0, (2, 7): 0, (2, 8): 0, (2, 9): 0, (2, 10): 0, (2, 11): 0, (2, 12): 0, (2, 13): 1, (2, 14): 1, (2, 15): 1, (2, 16): 1, (2, 17): 0}

        ####temp variable for draft test###
        temp_x = np.zeros((T, R, W_N, W))
        temp_z = np.zeros((T, R, C))
        temp_RNS = {(k, i, h): 0 for k in Ex_Times for i in Robots for h in Navigation_Tasks}
        temp_variable_aux3_rounded = np.zeros((T, R))


        #############robot lists#########
        available_robots= [i for i in range(0, R)]
        available_nav_tasks = [i for i in range(0, W_N)]
        nav_task_to_robot = [-1 for i in range(0, W_N)]
        remaining_obj_task_matrix = Gamma_Matrix.copy()


        need_charge_robots =[]
        charging_robots = []
        available_CStations = [i for i in range(0, C)]
        available_C = C
        robot_c_map = {}

        #functions*********************************************
        ###temp e function
        def temp_calc_e(k, i, allocation_x_rounded, Robot_Navigation_state_rounded, allocation_z_rounded, aux):
            for c in Charging_stations:
                if (allocation_z_rounded[k, i, c] == 1):
                    e_charged[k, i] = allocation_z_rounded[k, i, c] * Charging_rate
                    break
                else:
                    e_charged[k, i] = 0    
            
            e_res_Nav[k,i] = Alpha_Comp[i] * sum( Robot_Navigation_state_rounded[k, i, h] * M_Nav[h] for h in Navigation_Tasks) + sum( Alpha_Sensing[l] * Robot_Navigation_state_rounded[k, i, h] * Nav_Sensing_Rate[(h, l)] * Avg_Access_Time[l] for l in Sensors for h in Navigation_Tasks)
            e_res_Task[k,i] = Alpha_Comp[i] * sum( allocation_x_rounded[k, i, h, j] * M_Task[j] for h in Navigation_Tasks for j in Non_Navigation_Tasks) + sum( Alpha_Sensing[l] * allocation_x_rounded[k, i, h, j] * Tasks_Sensing_Rate[(j, l)] * Avg_Access_Time[l] for l in Sensors for h in Navigation_Tasks for j in Non_Navigation_Tasks)
            e_other[k,i] = sum(Alpha_Loc[h] * Robot_Navigation_state_rounded[k, i, h] for h in Navigation_Tasks) + E_changeMax * aux

            if k == 0:  # calculate energy for the first time
                temp_e[k, i] = E_Balance_Zero[i] + e_charged[k, i] - e_res_Nav[k, i] - e_res_Task[k, i] - e_other[k, i]
            else:
                temp_e[k, i] = temp_e[k-1, i] + e_charged[k, i] - e_res_Nav[k, i] - e_res_Task[k, i] - e_other[k, i]
            return temp_e[k,i] 



        ###function to calculate energy level at the end of each period
        def calc_e(k, i, allocation_x_rounded, Robot_Navigation_state_rounded, allocation_z_rounded):
            temp_variable_aux3_rounded[k, i]= sum((Robot_Navigation_state_rounded[k - 1, i, h] * (1 - Robot_Navigation_state_rounded[k, i, h]) + ( (1 - Robot_Navigation_state_rounded[k - 1, i, h]) * Robot_Navigation_state_rounded[k, i, h])) for h in Navigation_Tasks)
            # calculate total charge added on this period
            for c in Charging_stations:
                if (allocation_z_rounded[k, i, c] == 1):
                    e_charged[k, i] = allocation_z_rounded[k, i, c] * Charging_rate
                    break
                else:
                    e_charged[k, i] = 0    
            
            e_res_Nav[k,i] = Alpha_Comp[i] * sum( Robot_Navigation_state_rounded[k, i, h] * M_Nav[h] for h in Navigation_Tasks) + sum( Alpha_Sensing[l] * Robot_Navigation_state_rounded[k, i, h] * Nav_Sensing_Rate[(h, l)] * Avg_Access_Time[l] for l in Sensors for h in Navigation_Tasks)
            e_res_Task[k,i] = Alpha_Comp[i] * sum( allocation_x_rounded[k, i, h, j] * M_Task[j] for h in Navigation_Tasks for j in Non_Navigation_Tasks) + sum( Alpha_Sensing[l] * allocation_x_rounded[k, i, h, j] * Tasks_Sensing_Rate[(j, l)] * Avg_Access_Time[l] for l in Sensors for h in Navigation_Tasks for j in Non_Navigation_Tasks)
            e_other[k,i] = sum(Alpha_Loc[h] * Robot_Navigation_state_rounded[k, i, h] for h in Navigation_Tasks) + E_changeMax * temp_variable_aux3_rounded[k, i]

            if k == 0:  # calculate energy for the first time
                temp_e[k, i] = E_Balance_Zero[i] + e_charged[k, i] - e_res_Nav[k, i] - e_res_Task[k, i] - e_other[k, i]
            else:
                temp_e[k, i] = temp_e[k-1, i] + e_charged[k, i] - e_res_Nav[k, i] - e_res_Task[k, i] - e_other[k, i]
            # print("current energy after executing task", e[k,i])  
            return temp_e[k,i] 


        #####function to calc lowest required energy##########
        def calc_lowest_required_energy(h, i, j):
            e_Nav= Alpha_Comp[i] * M_Nav[h]  + sum( Alpha_Sensing[l] * Nav_Sensing_Rate[(h, l)] * Avg_Access_Time[l] for l in Sensors)
            e_Task= Alpha_Comp[i] * ( 1 * M_Task[j]) + sum( Alpha_Sensing[l] * 1 * Tasks_Sensing_Rate[(j, l)] * Avg_Access_Time[l] for l in Sensors)
            e_o= Alpha_Loc[h] + E_changeMax
            required_energy = e_Nav + e_Task + e_o 
            return required_energy


        
        #########function to calculate obj2     
        def calc_obj2(E, charge= True):
            val1 = abs(E - Edod)
            val2 = abs(Emax - E)
            if charge == True:
                obj2 =(val1/Ebat) * batDegWeight
            elif charge == False:
                obj2 =(val2/Ebat) * batDegWeight
            return obj2    

        ###func to calc E_waste####
        def calc_e_waste(k, i):
            if Robot_Navigation_state_rounded[k-1, i, h]==0 and (sum((Robot_Navigation_state_rounded[k - 1, i, h]) for h in Navigation_Tasks)) > 0:
                e_waste = 2 * E_changeMax
            if Robot_Navigation_state_rounded[k-1, i, h]==0 and (sum((Robot_Navigation_state_rounded[k - 1, i, h]) for h in Navigation_Tasks)) == 0: 
                e_waste = E_changeMax 
            if Robot_Navigation_state_rounded[k-1, i, h]==1:
                e_waste = 0
            return e_waste          
            
        #########function to calculate obj2     
        def temp_calc_obj2(E, charge= True):
            val1 = abs(E - Edod)
            val2 = abs(Emax - E)
            if charge == True:
                obj2 =(val1/Ebat) * batDegWeight
            elif charge == False:
                obj2 =(val2/Ebat) * batDegWeight
            return obj2 
                
        #######function to calculate temporary total obj cost
        def obj_with_all_tasks_done(k,i,h,W,remaining_obj_task_matrix):
            temp_x = np.zeros((T, R, W_N, W))
            temp_z = np.zeros((T, R, C))
            temp_RNS = {(k, i, h): 0 for k in Ex_Times for i in Robots for h in Navigation_Tasks}
            
            for j in range(0, W):
                if remaining_obj_task_matrix[h,j] == 1:
                    temp_x[k, i, h, j] = 1
                    temp_RNS[k,i,h] = 1        
                
            
            energy= temp_calc_e(k,i, temp_x, temp_RNS, temp_z, 1)
            if k>0:
                if Robot_Navigation_state_rounded[k-1, i, h]==0 and (sum((Robot_Navigation_state_rounded[k - 1, i, h]) for h in Navigation_Tasks)) > 0:
                    energy= temp_calc_e(k,i, temp_x, temp_RNS, temp_z, 2)
                if Robot_Navigation_state_rounded[k-1, i, h]==1:
                    energy = energy + E_changeMax
                
            # print(f"at k={k}, energy after all tasks assigned for robot {i} is {energy}")
            obj1_rounded=0
            for j in Non_Navigation_Tasks:
                obj1_rounded = obj1_rounded + Priority[j]*(Y[(k, j)]*remaining_obj_task_matrix[(h, j)] - temp_x[k, i, h, j])
            obj2_rounded = calc_obj2(energy, charge= True)
            total_obj = obj1_rounded + obj2_rounded
            # print("total obj with all tasks done", total_obj)
            return total_obj, temp_x, temp_RNS, energy

        def cost_matrix_weight_calc(k,i,h,remaining_obj_task_matrix):
            temp_task_removed=0
            task_removed=0
            obj1_rounded=0
            total_obj_tasks = sum(remaining_obj_task_matrix[h,j] for j in Non_Navigation_Tasks)
            prev_obj, temp_x, temp_RNS, energy= obj_with_all_tasks_done(k,i,h,W, remaining_obj_task_matrix)
            tasks=[] # will hold the index of j for the obj tasks of that navigation
            removed_obj_tasks= [] #will hold the index of j for the obj tasks that are removed of that navigation
            remaining_obj_task= [] #will hold the index of j for the obj tasks that can not be allocated
            for j in Non_Navigation_Tasks:
                if remaining_obj_task_matrix[h,j]==1:
                    tasks.append(j)
            # print(f"tasks : {tasks}")
            tasks_sorted = [item for item in j_sort_ascending if item in tasks] #will hold the index of j for the obj tasks of that navigation in lower priority to higher priority
            ##--------------------new addition to handle min energy threshold--------------#####
            if energy < E_changeMax:
                # print("energy going below  e_changmax")
                prev_obj=10000000
                for j in Non_Navigation_Tasks:
                    if remaining_obj_task_matrix[h,j_sort_ascending[j]] == 1:
                        temp_x[k, i, h, j_sort_ascending[j]] = 0
                        temp_task_removed+=1
                        if temp_task_removed == total_obj_tasks:
                            temp_RNS[k,i,h] = 0
                            
                        tempp_e = temp_calc_e(k,i, temp_x, temp_RNS, temp_z, 1) ###previously 1
                        if k>0:
                            if Robot_Navigation_state_rounded[k-1, i, h]==0 and (sum((Robot_Navigation_state_rounded[k - 1, i, h]) for h in Navigation_Tasks)) > 0:
                                if temp_task_removed == total_obj_tasks:
                                    tempp_e = tempp_e + E_changeMax
                                else:
                                    tempp_e = tempp_e - E_changeMax
                            if Robot_Navigation_state_rounded[k-1, i, h]==1:
                                tempp_e = tempp_e + E_changeMax   
                            if Robot_Navigation_state_rounded[k-1, i, h]==0 and (sum((Robot_Navigation_state_rounded[k - 1, i, h]) for h in Navigation_Tasks)) == 0:
                                if temp_task_removed == total_obj_tasks:
                                    tempp_e = tempp_e + E_changeMax - E_changeMax     
                        # print(f"after {temp_task_removed} task removed energy is {tempp_e}")
                        
                        
                        #########################
                            
                        if tempp_e >= E_changeMax:
                            obj1_rounded = obj1_rounded + Priority[j_sort_ascending[j]]*(Y[(k, j_sort_ascending[j])]*remaining_obj_task_matrix[(h, j_sort_ascending[j])] - temp_x[k, i, h,  j_sort_ascending[j]])
                            obj2_rounded = calc_obj2(tempp_e, charge= True)
                            total_obj = obj1_rounded + obj2_rounded
                            if total_obj > prev_obj:
                                break
                            task_removed = temp_task_removed
                            removed_obj_tasks.append(j_sort_ascending[j])
                            prev_obj = total_obj
                            if task_removed == total_obj_tasks:
                                temp_RNS[k,i,h] = 0
                                prev_obj = -1
                                remaining_obj_task = tasks_sorted 
                                break    
                        elif temp_task_removed == total_obj_tasks:
                            temp_RNS[k,i,h] = 0
                            prev_obj = -1
                            remaining_obj_task = tasks_sorted 
                            break
                        else:
                            continue 
            
            #####----to handle obj changes---------------------###
            else:
                for j in Non_Navigation_Tasks:
                    if remaining_obj_task_matrix[h,j_sort_ascending[j]] == 1:
                        temp_x[k, i, h, j_sort_ascending[j]] = 0
                        # print(f"allocation table after {j} no task removed: {allocation_x_rounded}")
                        temp_task_removed+=1
                        if temp_task_removed == total_obj_tasks:
                            temp_RNS[k,i,h] = 0
                            
                        tempp_e= temp_calc_e(k,i, temp_x, temp_RNS, temp_z, 1)
                        
                            
                        if k>0:
                            if Robot_Navigation_state_rounded[k-1, i, h]==0 and (sum((Robot_Navigation_state_rounded[k - 1, i, h]) for h in Navigation_Tasks)) > 0:
                                if temp_task_removed == total_obj_tasks:
                                    tempp_e = tempp_e +E_changeMax
                                else:
                                    tempp_e = tempp_e - E_changeMax
                                        
                            if Robot_Navigation_state_rounded[k-1, i, h]==1:
                                tempp_e = tempp_e + E_changeMax
                            if Robot_Navigation_state_rounded[k-1, i, h]==0 and (sum((Robot_Navigation_state_rounded[k - 1, i, h]) for h in Navigation_Tasks)) == 0:
                                if temp_task_removed == total_obj_tasks:
                                    tempp_e = tempp_e + E_changeMax - E_changeMax
                                
                                
                        if k==0 and temp_task_removed == total_obj_tasks:#########
                            tempp_e = tempp_e + E_changeMax ##########    
                                
                        obj1_rounded = obj1_rounded + Priority[j_sort_ascending[j]]*(Y[(k, j_sort_ascending[j])]*remaining_obj_task_matrix[(h, j_sort_ascending[j])] - temp_x[k, i, h,  j_sort_ascending[j]])
                        obj2_rounded = calc_obj2(tempp_e, charge= True)
                        total_obj = obj1_rounded + obj2_rounded
                        
                        
                        new_obj=total_obj
                        if new_obj > prev_obj:
                            break
                        energy = tempp_e
                        task_removed = temp_task_removed
                        removed_obj_tasks.append(j_sort_ascending[j])
                        if task_removed == total_obj_tasks:
                            temp_RNS[k,i,h] = 0
                            prev_obj = -1
                            remaining_obj_task = tasks_sorted 
                            break
                        
                        prev_obj = new_obj
                    else:
                        continue    
                        
            if task_removed == 0:
                remaining_obj_task = []
            else:    
                remaining_obj_task = removed_obj_tasks
            allocated_obj_task = [item for item in tasks if item not in remaining_obj_task]
            return prev_obj, allocated_obj_task
                
            
        def munkres_func(matrix):
            m = Munkres()
            indexes = m.compute(matrix)
            indexes_sorted= indexes.sort(key=lambda tup: tup[0])
            selected_nav_task = [i for i, j in indexes]
            selected_robot = [lis[-1] for lis in indexes]
            res = {selected_nav_task[i]: selected_robot[i] for i in range(len(selected_nav_task))}
            return res

        def nav_to_robot_assign(e, k, available_robots, available_nav_tasks, remaining_obj_task_matrix):
            rows, cols = (len(available_nav_tasks), len(available_robots))
            weight_matrix = [[0 for _ in range(cols)] for _ in range(rows)]
            allocated_obj = [[[] for _ in range(cols)] for _ in range(rows)]
            disallowed_robots=[]
            
            for h in range(0, len(available_nav_tasks)):
                for i in range(0, len(available_robots)):
                    # print(f"----------showing for nav task {available_nav_tasks[h]} and robot {available_robots[i]}--------------")
                    weight, allocated_obj_task = cost_matrix_weight_calc(k,available_robots[i], available_nav_tasks[h], remaining_obj_task_matrix)
                    if weight == -1:
                        weight_matrix[h][i] = DISALLOWED
                    else:
                        weight_matrix[h][i] = round(weight, 4)
                    allocated_obj[h][i] = allocated_obj_task    
                    temp_RNS[k,i,h] = 0
                    
            allocated_obj = np.array(allocated_obj, dtype=object) ###this line is to not getting a warning as this array holds list of lists
            ##### now we have to tune the weight matrix . col_index holds the col index of the martix having all the disaalowed###---------------#
            col_index= []
            for i in range(len(weight_matrix[0])):
                flag = True
                for j in range(len(weight_matrix)):
                    if weight_matrix[j][i] != DISALLOWED:
                        flag = False
            
                if flag == True:
                    col_index.append(i)
            
            
            for i in col_index:
                disallowed_robots.append(available_robots[i])
                
            available_robots = [item for item in available_robots if item not in disallowed_robots]
            
            ####this is for deleting the columns with all disallowed entry
            iter = len(col_index)           
            if iter > 0:
                weight_matrix  = np.delete(weight_matrix, col_index, 1).tolist()
                allocated_obj  = np.delete(allocated_obj, col_index, 1).tolist()
                
        ##########--------------------------###########         
            
            result = munkres_func(weight_matrix)
            #####as in the rsult matrix the indexs are not the actual robot and nav task index so we need to map that result with the actual
            ## robot and nav task no as follows til line
            allocated_obj_task = {}  ##it holds the index of the allocated obj tasks per nav to construct the remaining_obj_matrix later
            for h in result:
                allocated_obj_task[h] = allocated_obj[h][result[h]]
                
            new_allocated_obj_task={}
            for h in allocated_obj_task:
                new_allocated_obj_task[available_nav_tasks[h]]= allocated_obj_task[h]    
            
            task_robot_pairs={}
            for h in result:
                task_robot_pairs[available_nav_tasks[h]]= available_robots[result[h]]
            return task_robot_pairs, new_allocated_obj_task, disallowed_robots   
            

        ###functions for allocation and charging###
        def allocation(k, available_robots, available_nav_tasks, need_charge_robots,
                    remaining_obj_task_matrix, e, allocation_x_rounded, Robot_Navigation_state_rounded, allocation_z_rounded):
            v=0
            while len(available_robots) != 0 and len(available_nav_tasks) != 0:
                v= v+1
                task_robot_pairs, new_allocated_obj_task, disallowed = nav_to_robot_assign(e, k, available_robots, available_nav_tasks, remaining_obj_task_matrix)
                ####next 4 lines is to check disallowed robots and put them to need charge list##
                for i in Robots:
                    if i in disallowed:
                        need_charge_robots.append(i)
                available_robots = [item for item in available_robots if item not in need_charge_robots]        
                ####next 4 line is for allocation###
                for h in task_robot_pairs.keys():
                    for j in new_allocated_obj_task[h]:
                        allocation_x_rounded[k,task_robot_pairs[h],h,j] = 1
                    Robot_Navigation_state_rounded[k, task_robot_pairs[h], h]=1
                    available_robots.remove(task_robot_pairs[h])####remove that robot from the avilable list
                ###after allocation we will update the obj task matrix to only hve 1 with those j index that are not allocated yet####
                for row, col in new_allocated_obj_task.items():####this three line is to update the rmaining obj task matrix
                    for c in col:
                        remaining_obj_task_matrix[(row, c)] = 0
                
                for h in task_robot_pairs:
                    s= sum(remaining_obj_task_matrix[h, j] for j in Non_Navigation_Tasks)
                    if s<=0:
                        available_nav_tasks.remove(h)
                        
            for i in Robots:
                e[k,i]= calc_e(k, i, allocation_x_rounded, Robot_Navigation_state_rounded, allocation_z_rounded)
                
            remaining_obj_task_matrix = Gamma_Matrix.copy()  ###this is to set the remaining obj matrix new for next period
            
            return available_robots, available_nav_tasks, e, need_charge_robots,remaining_obj_task_matrix, e, allocation_x_rounded, Robot_Navigation_state_rounded, allocation_z_rounded                

        ####charging decision part###
            
        ####charging decision part###    
        def charging(k,need_charge_robots, available_CStations, charging_robots, e, allocation_x_rounded, Robot_Navigation_state_rounded, allocation_z_rounded, robot_c_map):
            need_charge_robots = [item for item in need_charge_robots if item not in charging_robots] 
            while len(need_charge_robots) != 0 and len(available_CStations) != 0:
                for i in Robots:
                    if i in need_charge_robots:
                        for c in Charging_stations:
                            if c in available_CStations:
                                allocation_z_rounded[k,i,c] = 1
                                charging_robots.append(i)
                                robot_c_map[i] = c
                                available_CStations.remove(c)
                                need_charge_robots.remove(i)
                                break
                        for h in Navigation_Tasks:
                            Robot_Navigation_state_rounded[k,i,h] = 0
            need_charge_robots = [item for item in need_charge_robots if item not in charging_robots]         
            #####now update robot energies###
            for i in Robots:
                e[k,i]= calc_e(k, i, allocation_x_rounded, Robot_Navigation_state_rounded, allocation_z_rounded)               
            return need_charge_robots, available_CStations, charging_robots, e, allocation_x_rounded, Robot_Navigation_state_rounded, allocation_z_rounded, robot_c_map


        #### function to check if we should keep charging the robots###
        def keep_charging(k, i, cur_energy, available_nav_tasks, robot_needed = True):
            if robot_needed == True:
                print(f"robot needed")
                nav_obj = {h: 0 for h in available_nav_tasks}
                for h in available_nav_tasks:
                    for j in Non_Navigation_Tasks:
                        nav_obj[h] = nav_obj[h] + Priority[j]*Gamma_Matrix[(h, j)]
                h_max = max(nav_obj, key=nav_obj.get)
                obj1_without_TA = nav_obj[h_max]
                energy_if_charging = cur_energy + min(Charging_rate, (Ebat - cur_energy))
                obj2_without_TA= calc_obj2(energy_if_charging, charge= False)
                total_obj_without_TA =  obj1_without_TA + obj2_without_TA
                energy_if_not_charging = cur_energy
                obj1_with_TA = 0
                obj2_with_TA= calc_obj2(energy_if_not_charging, charge = False)
                total_obj_with_TA =  obj1_with_TA + obj2_with_TA
                if total_obj_without_TA > total_obj_with_TA:
                    print("stop charging here and make it available")
                    decision= False
                    return decision, h_max
                else:
                    decision = True
                    print("keep charging in this period")
                    return decision, h_max
                
                
                    
            elif robot_needed == False:
                print("robot is not needed just check if any of the robots needs to get out of charging")
                obj1 = 0
                energy_if_charging = cur_energy + min(Charging_rate, (Ebat - cur_energy))
                
                obj2_if_keep_charging= calc_obj2(energy_if_charging, charge= False)
                total_obj_if_keep_charging =  obj1 + obj2_if_keep_charging
                energy_if_not_charging = cur_energy
                obj2_if_not_charging = calc_obj2(energy_if_not_charging, charge= False)
                total_if_not_charging =  obj1 + obj2_if_not_charging
                if total_obj_if_keep_charging > total_if_not_charging:
                    print("stop charging here and make it available")
                    decision= False
                    return decision
                else:
                    print("keep charging in this period")
                    decision = True
                    return decision
                


        #####------------------ main body -----------------------------################
        Start_time_with_LP = time.time()

        #######run the LP#######---------------------
        if random_maintenance == 0:
            m = gp.Model("qp")
                
            x = {}
            z = {}
            u = {}
            ee = {}
            ee_charged = {}
            ee_res_Task = {}
            ee_res_Nav = {}
            ee_other = {}
            a = {}
            b = {}
            aux1 = {}
            aux2 = {}
            aux3 = {}
            Robot_Navigation_state = {}
            y = {}

            obj1 = 0
            obj2 = 0
            aux3_sum = 0
            W_obj2 = 0

            for k in Times:
                for i in Robots:
                    ee[k, i] = m.addVar(vtype=GRB.CONTINUOUS,
                                    name='e_(%s,%s)' % (k, i))  # Energy balance
                    ee_res_Nav[k, i] = m.addVar(
                        vtype=GRB.CONTINUOUS, name='e_res_Nav_(%s,%s)' % (k, i))
                    ee_res_Task[k, i] = m.addVar(
                        vtype=GRB.CONTINUOUS, name='e_res_Task(%s,%s)' % (k, i))
                    ee_other[k, i] = m.addVar(vtype=GRB.CONTINUOUS,
                                            name='e_other(%s,%s)' % (k, i))
                    ee_charged[k, i] = m.addVar(
                        vtype=GRB.CONTINUOUS, name='e_charged(%s,%s)' % (k, i))
            for k in Times:
                for i in Robots:
                    for j in Non_Navigation_Tasks:
                        for h in Navigation_Tasks:
                            x[k, i, h, j] = m.addVar(
                                vtype=GRB.CONTINUOUS, name='x_(%s,%s,%s,%s)' % (k, i, h, j))

            for k in Times:
                for j in Non_Navigation_Tasks:
                    for h in Navigation_Tasks:
                        obj1 = obj1 + \
                            Priority[j]*(Y[(k, j)]*Gamma_Matrix[(h, j)] -
                                        sum(x[k, i, h, j] for i in Robots))


            for k in Times:
                for i in Robots:
                    for h in Navigation_Tasks:
                        Robot_Navigation_state[k, i, h] = m.addVar(
                            vtype=GRB.CONTINUOUS, name='Robot_Navigation_state[%s,%s,%s]' % (k, i, h))
                        y[k, i, h] = m.addVar(vtype=GRB.CONTINUOUS,
                                            name='y[%s,%s,%s]' % (k, i, h))


            for k in Times:
                for i in Robots:
                    for c in Charging_stations:
                        z[k, i, c] = m.addVar(vtype=GRB.CONTINUOUS,
                                            name='z_(%s,%s,%s)' % (k, i, c))
                        a[k, i, c] = m.addVar(vtype=GRB.CONTINUOUS,
                                            name='a_(%s,%s,%s)' % (k, i, c))
                        b[k, i, c] = m.addVar(vtype=GRB.CONTINUOUS,
                                            name='b_(%s,%s,%s)' % (k, i, c))
                    aux1[k, i] = m.addVar(vtype=GRB.CONTINUOUS,
                                        name='aux1_(%s,%s)' % (k, i))
                    aux2[k, i] = m.addVar(vtype=GRB.CONTINUOUS,
                                        name='aux2_(%s,%s)' % (k, i))

            for k in Times:
                for i in Robots:
                    aux3[k, i] = m.addVar(vtype=GRB.CONTINUOUS,
                                        name='aux3_(%s,%s)' % (k, i))

            # defining decision variable for maintenance u[m,i]
            if main_dur > 0:
                for p in main_solution_period:
                    for i in Robots:
                        # maintenane variable
                        u[p, i] = m.addVar(vtype=GRB.CONTINUOUS, name='u_(%s,%s)' % (p, i))

            # Define maintenance_solution_matrix:
            if main_dur > 0:
                msol = {(p, k): 0 for p in main_solution_period for k in Times}


            for p in main_solution_period:
                for temp in maintenance_period:
                    msol[p, p+temp] = 1


            m.update()


            for k in Times:
                for i in Robots:
                    obj2 = obj2 + aux1[k, i] + aux2[k, i]


            for k in Times:
                for i in Robots:
                    aux3_sum = aux3_sum + aux3[k, i]

            W_obj2 = Q_Battery_Weight*obj2

            obj = obj1 + W_obj2

            m.update()

            # Initialization
            for i in Robots:
                ee[-1, i] = E_Balance_Zero[i]
                for h in Navigation_Tasks:
                    Robot_Navigation_state[-1, i, h] = 0
                for c in Charging_stations:
                    z[-1, i, c] = Charge_State_Zero[i]

            for k in Times:
                for i in Robots:
                    for j in Non_Navigation_Tasks:
                        for h in Navigation_Tasks:
                            m.addConstr(x[k, i, h, j] >= 0, name='Constrain_x_(%s,%s,%s,%s)' % (
                                k, i, h, j))  # Constraint x0
                            m.addConstr(x[k, i, h, j] <= 1, name='Constrain_x_(%s,%s,%s,%s)' % (
                                k, i, h, j))  # Constraint x1

            for k in Times:
                for i in Robots:
                    for h in Navigation_Tasks:
                        m.addConstr(Robot_Navigation_state[k, i, h] >= 0, name='Constrain_RNS_(%s,%s,%s)' % (
                            k, i, h))  # Constraint RNS0
                        m.addConstr(Robot_Navigation_state[k, i, h] <= 1, name='Constrain_RNS_(%s,%s,%s)' % (
                            k, i, h))  # Constraint RNS1
                        m.addConstr(y[k, i, h] >= 0, name='y_(%s,%s,%s)' %
                                    (k, i, h))  # Constraint y0
                        m.addConstr(y[k, i, h] <= 1, name='y_(%s,%s,%s)' %
                                    (k, i, h))  # Constraint y1

            for k in Times:
                for i in Robots:
                    for c in Charging_stations:
                        m.addConstr(z[k, i, c] >= 0, name='Constrain_z_(%s,%s,%s)' %
                                    (k, i, c))  # Constraint z0
                        m.addConstr(z[k, i, c] <= 1, name='Constrain_z_(%s,%s,%s)' %
                                    (k, i, c))  # Constraint z1

                        m.addConstr(a[k, i, c] >= 0, name='Constrain_a_(%s,%s,%s)' %
                                    (k, i, c))  # Constraint a0
                        m.addConstr(a[k, i, c] <= 1, name='Constrain_a_(%s,%s,%s)' %
                                    (k, i, c))  # Constraint a1

                        m.addConstr(b[k, i, c] >= 0, name='Constrain_b_(%s,%s,%s)' %
                                    (k, i, c))  # Constraint b0
                        m.addConstr(b[k, i, c] <= 1, name='Constrain_b_(%s,%s,%s)' %
                                    (k, i, c))  # Constraint b1

            for k in Times:
                for i in Robots:
                    m.addConstr(aux3[k, i] >= 0, name='Constrain_aux3_(%s,%s)' %
                                (k, i))  # Constraint aux3_0
                    m.addConstr(aux3[k, i] <= 1, name='Constrain_aux3_(%s,%s)' %
                                (k, i))  # Constraint aux3_1


            for k in Times:
                for h in Navigation_Tasks:
                    for j in Non_Navigation_Tasks:
                        m.addConstr(sum(x[k, i, h, j] for i in Robots) <= Y[(
                            k, j)], name="Task_Activitiy_Status_[%s,%s,%s]" % (k, h, j))  # Constraint 4

            for k in Times:
                for i in Robots:
                    for h in Navigation_Tasks:
                        m.addConstr(sum(x[k, i, h, j] for j in Non_Navigation_Tasks) >= Robot_Navigation_state[k,
                                                                                                            i, h], name="Nav_Activitiy_Status_[%s,%s,%s]" % (k, i, h))  # Constraint 4

            for k in Times:
                for i in Robots:
                    for h in Navigation_Tasks:
                        for j in Non_Navigation_Tasks:
                            m.addConstr(x[k, i, h, j] <= Gamma_Matrix[(
                                h, j)], name="State_consistency_[%s,%s,%s,%s]" % (k, i, h, j))  # Constraint 5

            for k in Times:
                for i in Robots:
                    for h in Navigation_Tasks:
                        for j in Non_Navigation_Tasks:
                            m.addConstr(Robot_Navigation_state[k, i, h] >= x[k, i, h, j], name="MAX_[%s,%s,%s,%s]" % (
                                k, i, h, j))  # Constraint 6 part1


            for k in Times:
                for i in Robots:
                    m.addConstr(sum(z[k, i, c] for c in Charging_stations)+sum(Robot_Navigation_state[k, i, h]
                                                                            for h in Navigation_Tasks) <= 1, name="State_consistency_[%s,%s,%s]" % (k, i, h))  # Constraint 6 part 2


            for k in Times:
                m.addConstr(sum(z[k, i, c] for c in Charging_stations for i in Robots)
                            <= C, name="Limited_stations_[%s]" % (k))  # Constraint 7


            for k in Times:
                for i in Robots:
                    m.addConstr(sum(Robot_Navigation_state[k, i, h]*M_Nav[h] for h in Navigation_Tasks)+sum(x[k, i, h, j]*M_Task[j]
                                                                                                            for h in Navigation_Tasks for j in Non_Navigation_Tasks) <= M_Max[i], name="Limited_Resource_capacity_[%s,%s]" % (k, i))  # Constraint 8


            for k in Times:
                for i in Robots:
                    # m.addConstr(e[k,i]>=sum(Robot_Navigation_state[k,i,h]*E_NavMax[h] for h in Navigation_Tasks), name="Min_Energy_Charge_[%s,%s]" % (k, i))  #Constraint 9
                    m.addConstr(ee[k, i] >= 0, name="Min_Energy_Charge_[%s,%s]" %
                                (k, i))  # Constraint 9
                    if k >= T-1:
                        # Constraint 9b
                        m.addConstr(ee[k, i] >= E_end_min,
                                    name="Min_Energy_End_[%s,%s]" % (k, i))


            for k in Times:
                for i in Robots:
                    m.addConstr(ee[k, i] == ee[k-1, i]+ee_charged[k, i]-ee_res_Nav[k, i]-ee_res_Task[k, i] -
                                ee_other[k, i], name="Energy_Balance_[%s,%s]" % (k, i))  # Constraint 10_1
                    m.addConstr(ee_charged[k, i] == sum(z[k, i, c]*Charging_rate for c in Charging_stations),
                                name="Energy_Charge_[%s,%s]" % (k, i))  # Constraint 10_1
                    m.addConstr(ee_res_Task[k, i] == Alpha_Comp[i]*sum(x[k, i, h, j]*M_Task[j] for h in Navigation_Tasks for j in Non_Navigation_Tasks)+sum(Alpha_Sensing[l]*x[k, i, h, j] *
                                                                                                                                                        Tasks_Sensing_Rate[(j, l)]*Avg_Access_Time[l] for l in Sensors for h in Navigation_Tasks for j in Non_Navigation_Tasks), name="Energy_Res_Task_[%s,%s]" % (k, i))  # Constraint 10_1
                    m.addConstr(ee_res_Nav[k, i] == Alpha_Comp[i]*sum(Robot_Navigation_state[k, i, h]*M_Nav[h] for h in Navigation_Tasks)+sum(Alpha_Sensing[l]*Robot_Navigation_state[k,
                                                                                                                                                                                    i, h]*Nav_Sensing_Rate[(h, l)]*Avg_Access_Time[l] for l in Sensors for h in Navigation_Tasks), name="Energy_Res_Nav[%s,%s]" % (k, i))  # Constraint 10_1
                    # m.addConstr(e_other[k,i]==sum(Alpha_Loc[h]*Robot_Navigation_state[k,i,h] for h in Navigation_Tasks), name="Energy_Other[%s,%s]" % (k, i))  #Constraint 10_1
                    m.addConstr(ee_other[k, i] == sum(Alpha_Loc[h]*Robot_Navigation_state[k, i, h]
                                                    for h in Navigation_Tasks) + E_changeMax*aux3[k, i], name="Energy_Other[%s,%s]" % (k, i))  # Constraint 10_1


            for k in Times:
                for i in Robots:
                    m.addConstr(ee[k, i] <= Ebat, name="Max_Energy_Stored_[%s,%s]" %
                                (k, i))  # Constraint 10

            for k in Times:
                for i in Robots:
                    for c in Charging_stations:
                        m.addConstr(a[k, i, c] >= ((0 - z[k - 1, i, c]) +
                                                z[k, i, c]), name="Aux_[%s,%s,%s]" % (k, i, c))
                        m.addConstr(aux1[k, i] >= ((ee[k-1, i] - Edod) / Ebat) -
                                    (1-a[k, i, c]), name="Aux3_[%s,%s,%s]" % (k, i, c))
                        m.addConstr(aux1[k, i] >= -((ee[k-1, i] - Edod) / Ebat) -
                                    (1-a[k, i, c]), name="Aux3_[%s,%s,%s]" % (k, i, c))

                        m.addConstr(b[k, i, c] >= (z[k - 1, i, c] +
                                                (0 - z[k, i, c])), name="Aux2_[%s,%s,%s]" % (k, i, c))
                        m.addConstr(aux2[k, i] >= ((Emax - ee[k-1, i]) / Ebat) -
                                    (1-b[k, i, c]), name="Aux4_[%s,%s,%s]" % (k, i, c))
                        m.addConstr(aux2[k, i] >= -((Emax - ee[k-1, i]) / Ebat) -
                                    (1-b[k, i, c]), name="Aux4_[%s,%s,%s]" % (k, i, c))


            for k in Times:
                for i in Robots:
                    #m.addConstr(aux3[k,i] == sum((Robot_Navigation_state[k-1,i,h]*(1-Robot_Navigation_state[k,i,h]) + ((1-Robot_Navigation_state[k-1,i,h])*Robot_Navigation_state[k,i,h])) for h in Navigation_Tasks), name="Aux3_[%s,%s]" % (k, i))
                    m.addConstr(aux3[k, i] == sum(y[k, i, h]
                                                for h in Navigation_Tasks), name="Aux3_[%s,%s]" % (k, i))


            for k in Times:
                for i in Robots:
                    for h in Navigation_Tasks:
                        m.addConstr(y[k, i, h] <= Robot_Navigation_state[k-1, i, h] +
                                    Robot_Navigation_state[k, i, h], name="Y_[%s,%s,%s]" % (k, i, h))
                        m.addConstr(y[k, i, h] >= Robot_Navigation_state[k-1, i, h] -
                                    Robot_Navigation_state[k, i, h], name="Y_[%s,%s,%s]" % (k, i, h))
                        m.addConstr(y[k, i, h] >= Robot_Navigation_state[k, i, h] -
                                    Robot_Navigation_state[k-1, i, h], name="Y_[%s,%s,%s]" % (k, i, h))
                        m.addConstr(y[k, i, h] <= 2 - Robot_Navigation_state[k-1, i, h] -
                                    Robot_Navigation_state[k, i, h], name="Y_[%s,%s,%s]" % (k, i, h))

            m.update()
            # Defining constraints for maintenance
            if main_dur > 0:
                for k in Times:
                    for i in Robots:
                        for h in Navigation_Tasks:
                            for j in Non_Navigation_Tasks:
                                for p in main_solution_period:
                                    m.addConstr(x[k, i, h, j] <= 1-u[p, i]*msol[p, k],
                                                name="no_task_allocated_while_in_maintenance_[%s,%s,%s,%s,%s]" % (k, i, h, j, p))

                for i in Robots:
                    m.addConstr(sum(u[p, i] for p in main_solution_period) == maintenance_robots[i],
                                name="only_one_robots_goes_to_maintenance_[%s]" % (i))

                for k in Times:
                    for i in Robots:
                        for c in Charging_stations:
                            for p in main_solution_period:
                                m.addConstr(z[k, i, c] <= 1-u[p, i] * msol[p, k],
                                            name="no_CS_will_be_allocated_while_in_maintenance_[%s,%s,%s,%s]" % (k, i, c, p))

            m.update()
            # # *********************************************

            m.setObjective(obj)
            m.Params.MIPgap = 0.01
            m.update()
            m.write('problem.lp')
            m.optimize()
            total_obj_LP =obj.getValue()
            print("total obj from LP:", total_obj_LP)

            # end_time_with_LP = time.time()
            # total_time_LP = end_time_with_LP - Start_time_with_LP

            # *********************************************
            
            maintenance_opt = np.zeros((T-main_dur+1, R))
            for p in main_solution_period:
                for i in Robots:
                    maintenance_opt[p, i] = m.getVarByName('u_(%s,%s)' % (p, i)).x  # maintenane variable
            maint_periods= {}
            #print("times:", Times)
            def calc_main_period(i):
                main_max = 0
                main_period_best = 0
                for k in Times:
                    #If it never was above 0 and we reach the last opportunity to do all the maintenance
                    #Do the maintenance at the first period we wanted, which is period 0.
                    if k > len(Times) - main_dur - 1 and main_max == 0:
                        main_period_best = 0 #This code is repeated on 992 for security it'd be done.
                        print("no value more than 0 found so maintenance period is:", main_period_best)
                        break
                    if k > len(Times) - main_dur - 1:
                        #Last period, stop checking
                        break
                    else:
                    #Simple recursion to find best period and replace lower values with better period values.
                        if maintenance_opt[k, i] > main_max:
                            main_max = maintenance_opt[k, i]
                            main_period_best = k
                return main_period_best
                    
            if no_of_main_robots > 0:
                for i in main_robot_list:
                    maint_periods[i]= calc_main_period(i)
                    
                print("maintenance periods from LP:", maint_periods)
            ## this 4 lines are for setting the main table to 1 from the maintenance period got from LP
            if no_of_main_robots > 0:
                for i in main_robot_list:
                    for k in range(maint_periods[i],maint_periods[i] + main_dur):
                        main[i,k] = 1
            print("**********end of LP---------------")
        ########***************end of LP*****************************###################
        end_time_with_LP = time.time()
        total_time_LP = end_time_with_LP - Start_time_with_LP

        ############for randomized maintenance start selection
        if random_maintenance == 1:
            ("----in random maintenance----")
            maint_periods= {}
            for i in main_robot_list:
                maint_periods[i] = random.randint(0, (T- main_dur)+1)

            if no_of_main_robots > 0:
                for i in main_robot_list:
                    for k in range(maint_periods[i],maint_periods[i] + main_dur):
                        main[i,k] = 1    


        ########______________________main RMA_______________############
        e = np.zeros((T, R))
        temp_e = np.zeros((T, R))
        state_of_charge = np.zeros((T, R))
        e_charged = np.zeros((T, R))
        e_res_Task = np.zeros((T, R))
        e_res_Nav = np.zeros((T, R))
        e_other = np.zeros((T, R))

        lowest_energy = calc_lowest_required_energy(0,0,0)            
                
        ##########start of RMA algorithm ######        
        for k in Times:
            if k==0:
                available_robots= [i for i in range(0, R)]
                available_nav_tasks = [i for i in range(0, W_N)]
                for i in Robots:
                    e[k,i] = calc_e(k, i, allocation_x_rounded, Robot_Navigation_state_rounded, allocation_z_rounded)
                ####next 9 lines are checked for maintenance periods#####
                for i in Robots:
                    if maintenance_robots[i] == 1:
                        print("main robot", i)
                        if main[i,k-1] == 0 and main[i,k] == 1:
                            ("------start of maintenance-----")
                            available_robots.remove(i) if i in available_robots else available_robots 
                            # need_charge_robots.remove(i) if i in need_charge_robots else need_charge_robots
                            # charging_robots.remove(i) if i in charging_robots else charging_robots         
                for i in Robots:
                    if i in available_robots and e[k,i] <= E_changeMax:
                        need_charge_robots.append(i)
                        available_robots.remove(i)
                            
                available_robots, available_nav_tasks, e, need_charge_robots,remaining_obj_task_matrix, e, allocation_x_rounded, Robot_Navigation_state_rounded, allocation_z_rounded= allocation(k, available_robots, available_nav_tasks, need_charge_robots,
                    remaining_obj_task_matrix, e, allocation_x_rounded, Robot_Navigation_state_rounded, allocation_z_rounded)
                
                ####charging decision part###
                need_charge_robots, available_CStations, charging_robots, e, allocation_x_rounded, Robot_Navigation_state_rounded, allocation_z_rounded, robot_c_map = charging(k, need_charge_robots, available_CStations, charging_robots, e, allocation_x_rounded, Robot_Navigation_state_rounded, allocation_z_rounded, robot_c_map)
                print(f"energy after k = {k} is {e[k,:]}")
                print("########-----------------#######")
                
            ####now start of the next period
            elif k > 0:
                print(f"*********-----------start of k = {k}-------------*************")
                ###because at the start of every period the tasks are reset to original
                available_robots= [i for i in range(0, R)]
                available_nav_tasks = [i for i in range(0, W_N)]
                ####next 9 lines are checked for maintenance periods#####
                for i in Robots:
                    if maintenance_robots[i] == 1:
                        if main[i,k-1] == 0 and main[i,k] == 1:
                            print("----start of maintenance-----")
                            available_robots.remove(i) if i in available_robots else available_robots 
                            need_charge_robots.remove(i) if i in need_charge_robots else need_charge_robots
                            charging_robots.remove(i) if i in charging_robots else charging_robots
                            
                        if main[i,k-1] == 1 and main[i,k] == 0:
                            print("-----end of maintenance-----")
                            if e[k-1,i] <= E_changeMax:
                                need_charge_robots.append(i)
                                available_robots.remove(i) if i in available_robots else available_robots 
                            else:
                                if i not in available_robots:
                                    available_robots.append(i)
                        if main[i,k-1] == 1 and main[i,k] == 1:###this is for if the robot is in the mid of the maintenance
                            print("-----------robot is in the maintenance----")
                            available_robots.remove(i) if i in available_robots else available_robots 
                            need_charge_robots.remove(i) if i in need_charge_robots else need_charge_robots
                            charging_robots.remove(i) if i in charging_robots else charging_robots
                            
                available_robots = [item for item in available_robots if item not in need_charge_robots and item not in charging_robots]
                ####to check if any robots reach max charge level
                for i in Robots:
                    if i in charging_robots and e[k-1,i]>= Ebat:
                        charging_robots.remove(i)
                        available_robots.append(i)
                print("charging robots", charging_robots)
                ###the next 4 lines are to sort the charging robot list where the robot having highest energy stays front in the list
                charging_rob_dict = {}
                for i in charging_robots:
                    charging_rob_dict[i] = e[k-1,i]
                # print("charging rob dict:", charging_rob_dict)
                charging_robots_sorted = sorted(charging_rob_dict, key=charging_rob_dict.get, reverse=True)    
                # charging_robots_sorted= get_robots_sorted_by_energy(k,charging_robots, e)
                print("charging robot sorted:", charging_robots_sorted)
                add_robots_needed = (len(available_nav_tasks) - len(available_robots))
                
                ##------count of the robots that has less than energy which is required to do atleast 1 obj task
                robs_less_lowest_energy = 0   
                for i in available_robots:
                    if e[k-1,i] < lowest_energy:
                        robs_less_lowest_energy +=1
                add_robots_needed = add_robots_needed + robs_less_lowest_energy
                ####---------------------
                
                charging_robot_checked= 0
                while add_robots_needed > 0 and charging_robot_checked < len(charging_robots_sorted):
                    # print(f"additional robot needed {add_robots_needed}")
                    for i in charging_robots_sorted:
                        decision1, h_max = keep_charging(k,i, e[k-1,i], available_nav_tasks, robot_needed = True)
                        charging_robot_checked += 1
                        if decision1 == False:
                            available_robots.append(i)
                            charging_robots.remove(i)
                            allocation_z_rounded[k,i, robot_c_map[i]] = 0
                            available_CStations.append(robot_c_map[i])
                            robot_c_map.pop(i)
                            available_nav_tasks.remove(h_max)
                            add_robots_needed = (len(available_nav_tasks) - len(available_robots))
                            
                        else:
                            allocation_z_rounded[k,i, robot_c_map[i]] = 1
                ###as charging robot list changed again make charging robot sorted list
                charging_rob_dict = {}
                for i in charging_robots:
                    charging_rob_dict[i] = e[k-1,i]
                # print("charging rob dict:", charging_rob_dict)
                charging_robots_sorted = sorted(charging_rob_dict, key=charging_rob_dict.get, reverse=True) 
                        
                for i in charging_robots_sorted:
                    decision2 =  keep_charging(k,i, e[k-1,i], available_nav_tasks, robot_needed = False)   
                    if decision2 == False:
                        available_robots.append(i)
                        charging_robots.remove(i)
                        allocation_z_rounded[k,i, robot_c_map[i]] = 0
                        available_CStations.append(robot_c_map[i])
                        robot_c_map.pop(i)
                        
                    else:
                        allocation_z_rounded[k,i, robot_c_map[i]] = 1
                                
                available_nav_tasks = [i for i in range(0, W_N)]
                
                available_robots, available_nav_tasks, e, need_charge_robots,remaining_obj_task_matrix, e, allocation_x_rounded, Robot_Navigation_state_rounded, allocation_z_rounded= allocation(k, available_robots, available_nav_tasks, need_charge_robots,
                    remaining_obj_task_matrix, e, allocation_x_rounded, Robot_Navigation_state_rounded, allocation_z_rounded)
                
                ####charging decision part###
                need_charge_robots, available_CStations, charging_robots, e, allocation_x_rounded, Robot_Navigation_state_rounded, allocation_z_rounded, robot_c_map = charging(k, need_charge_robots, available_CStations, charging_robots, e, allocation_x_rounded, Robot_Navigation_state_rounded, allocation_z_rounded, robot_c_map)        
                print(f"energy after k = {k} is {e[k,:]}")
                

                
                
        End_time = time.time()
        if no_of_main_robots == 0:
            run_time_TCM = End_time - end_time_with_LP
        else:
            run_time_TCM = End_time - Start_time_with_LP
        
        run_time_only_TCM =  End_time - end_time_with_LP      
        
        ###------------obj calculations--------
        obj1_rounded = 0
        obj2_rounded = 0

        for k in Times:
            if k != 0:
                for i in Robots:
                    val1 = abs(e[k - 1, i] - Edod)
                    val2 = abs(Emax - e[k - 1, i])

                    for c in Charging_stations:
                        obj2_rounded = obj2_rounded + ((1-allocation_z_rounded[k-1, i, c]) * allocation_z_rounded[k, i, c] * (
                            val1/Ebat)) + ((1 - allocation_z_rounded[k, i, c]) * allocation_z_rounded[k-1, i, c] * (val2/Ebat))

        W_obj2_rounded = batDegWeight * obj2_rounded


        for k in Times:
            for j in Non_Navigation_Tasks:
                for h in Navigation_Tasks:
                    allox_sum = 0
                    for i in Robots:
                        allox_sum = allox_sum + allocation_x_rounded[k, i, h, j]
                    
                    obj1_rounded = obj1_rounded + Priority[j]*(Y[(k, j)]*Gamma_Matrix[(h, j)] - allox_sum)

        obj_rounded = obj1_rounded + W_obj2_rounded
        total_tasks_to_perform = 0
        for k in Times:
            total_tasks_to_perform = total_tasks_to_perform +  sum( (Gamma_Matrix[h, j]) for h in Navigation_Tasks for j in Non_Navigation_Tasks)

        tasks_done = total_tasks_to_perform - obj1_rounded

        total_wasted = (sum(temp_variable_aux3_rounded[k,i] for i in Robots for k in Times)) * E_changeMax
        e_consumed = np.zeros((T, R))

        for k in Times:
            for i in Robots:
                if k==0:
                    if sum(Robot_Navigation_state_rounded[k,i,h] for h in Navigation_Tasks) > 0:
                        e_consumed[k,i] = E_Balance_Zero[i] - e[k,i]        
                else:
                    if sum(Robot_Navigation_state_rounded[k,i,h] for h in Navigation_Tasks) > 0:
                        e_consumed[k,i] = e[k-1,i] - e[k,i]   
                        
        total_consumed = sum(e_consumed[k,i] for i in Robots for k in Times)
                    


        EEF= total_consumed/(total_consumed - total_wasted)
            
        SoC_violation = np.zeros((T, R))    
        for k in Times:
            for i in Robots:
                SoC_violation[k,i] = max((e[k,i] - Emax), 0) + max((Edod - e[k,i]), 0)

        Soc_vio = 0        
        for k in Times:
            Soc_vio = Soc_vio + sum((SoC_violation[k,i]) for i in Robots)
        final_soc_violation = (100/Ebat) * Soc_vio        
        print("---------------")

        total_allocated_tasks= sum( allocation_x_rounded[k, i, h, j] for k in Times for i in Robots  for h in Navigation_Tasks for j in Non_Navigation_Tasks)
        TCM_task_downtime = total_tasks_to_perform - total_allocated_tasks

        print('Obj_rounded: {}'.format(obj_rounded))
        print('total_tasks_to_perform: {}'.format(total_tasks_to_perform))
        print('Obj1_rounded: {}'.format(obj1_rounded))
        print('Obj2_rounded: {}'.format(obj2_rounded))
        # print("weight given to obj2:", q*Q_Battery_Weight)
        print("weight given to obj2:", batDegWeight)
        print('Obj2_rounded weighted: {}'.format(W_obj2_rounded))
        print('Full Time: {}'.format(End_time - Start_time_with_LP))
        print("task downtime:", TCM_task_downtime)
        print("tasks done:", total_allocated_tasks)
        print("soc violation:", final_soc_violation) 
        print("EEF:", EEF) 
        print(f"------end of Exp_no: {Exp_no}---------")
        

        if random_maintenance == 0:
            Setting_df.loc[Exp_no,"Rand_Alloc_LP_obj1"] = obj1_rounded
            Setting_df.loc[Exp_no,"Rand_Alloc_LP_obj2"] = obj2_rounded
            Setting_df.loc[Exp_no,"Rand_Alloc_LP_obj2_weighted"] = W_obj2_rounded
            Setting_df.loc[Exp_no,"Rand_Alloc_LP_total_cost"] = obj_rounded
            Setting_df.loc[Exp_no,"total_cost_LP"] = total_obj_LP
            Setting_df.loc[Exp_no,"Rand_Alloc_LP_Total_tasks"] = total_tasks_to_perform
            Setting_df.loc[Exp_no,"Rand_Alloc_LP_task_downtime"] = TCM_task_downtime
            Setting_df.loc[Exp_no,"Rand_Alloc_LP_TA%"] = (total_allocated_tasks/total_tasks_to_perform)
            Setting_df.loc[Exp_no,"TCM_run_time_with_LP"] = run_time_TCM
            Setting_df.loc[Exp_no,"TCM_run_time_without_LP"] = run_time_only_TCM

            Setting_df.loc[Exp_no,"Rand_Alloc_LP_EEF"] = EEF
            Setting_df.loc[Exp_no,"Rand_Alloc_LP_SoC_v"] = final_soc_violation
            Setting_df.loc[Exp_no,"run_time_LP"] = total_time_LP
            Setting_df.loc[Exp_no,"TCM"] = 0
        
        ###for storing the other version TCM+randomized maintenance
        elif random_maintenance == 1:
            Setting_df.loc[Exp_no,"Rand_Alloc_rand_main_obj1"] = obj1_rounded
            Setting_df.loc[Exp_no,"Rand_Alloc_rand_main_obj2"] = obj2_rounded
            Setting_df.loc[Exp_no,"Rand_Alloc_rand_main_obj2_weighted"] = W_obj2_rounded
            Setting_df.loc[Exp_no,"Rand_Alloc_rand_main_total_cost"] = obj_rounded
            # Setting_df.loc[Exp_no,"total_cost_LP"] = total_obj_LP
            Setting_df.loc[Exp_no,"Rand_Alloc_rand_main_Total_tasks"] = total_tasks_to_perform
            Setting_df.loc[Exp_no,"Rand_Alloc_rand_main_task_downtime"] = TCM_task_downtime
            Setting_df.loc[Exp_no,"Rand_Alloc_rand_main_TA%"] = (total_allocated_tasks/total_tasks_to_perform)

            Setting_df.loc[Exp_no,"Rand_Alloc_rand_main_run_time"] = run_time_only_TCM
            Setting_df.loc[Exp_no,"Rand_Alloc_rand_main_EEF"] = EEF
            Setting_df.loc[Exp_no,"Rand_Alloc_rand_main_SoC_v"] = final_soc_violation
            Setting_df.loc[Exp_no,"run_time_LP"] = total_time_LP
            Setting_df.loc[Exp_no,"TCM"] = 0


        Setting_df.to_csv(File_name, index = False)
        
exp_end_time = time.time()
exp_total_time = exp_end_time - exp_start_time
print("----***--experiment total time:---***", exp_total_time)

        ##_______________________________Plots

Task_Downtime = np.zeros((T + 1, W_N))
for k in Times:
    for h in Navigation_Tasks:
        for j in Non_Navigation_Tasks:
            Task_Downtime[k + 1, h] = Task_Downtime[k + 1, h] + ((Gamma_Matrix[h, j]) - sum(allocation_x_rounded[k, i, h, j] for i in Robots))        

state_of_charge_a = np.zeros((T + 1, R))
for i in Robots:
    state_of_charge_a[0, i] = E_Balance_Zero[i] / Ebat * 100
    for k in Times:
        state_of_charge_a[k + 1, i] = e[k, i] / Ebat * 100          

allocated_tasks = np.zeros((T + 1, R, W_N))
for i in Robots:
    for h in Navigation_Tasks:
        for k in Times:
            for j in Non_Navigation_Tasks:
                # print(m.getVarByName('x_(%s,%s,%s,%s)'%(k,0,0,j)))
                allocated_tasks[k + 1, i, h] = allocated_tasks[k + 1, i, h] + allocation_x_rounded[k, i, h, j] 


#print("allocated tasks", allocated_tasks)

# for i in Robots:
#     plt.plot(range(0, T + 1), state_of_charge_a[:, i], label="Robot %s" % i)
if single_exp == 1:
    plt.plot(range(0, T + 1), state_of_charge_a[:, 0], linestyle='-', color='darkorange', label="AMR 0")  
    plt.plot(range(0, T + 1), state_of_charge_a[:, 1], linestyle='dashed', color='blue', label="AMR 1")
    plt.plot(range(0, T + 1), state_of_charge_a[:, 2], linestyle='dashdot', color='green', label="AMR 2")  

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

    # sys.exit()

                    
    for i in Robots:
        for h in Navigation_Tasks:
            plt.bar(range(0, T + 1), allocated_tasks[:, i, h], label="Navigation Task %s" % h)
        plt.ylabel('Number of Assigned Tasks to Robot %s ' % (i), fontweight='bold')
        plt.xlabel('Time Period', fontweight='bold')
        plt.xlim([-1, T + 1])
        plt.ylim(0, W + 3)
        plt.legend(loc="best")
        plt.show()
        plt.clf()
        
        
    for h in Navigation_Tasks:
        plt.bar(range(0,T+1),Task_Downtime[:,h])
        plt.ylabel('# of Unallocated Tasks for Navigation %s'%(h),fontweight='bold')
        plt.xlabel('Time Period',fontweight='bold')
        plt.xlim([0,T+1])
        plt.show()
        plt.clf()


        