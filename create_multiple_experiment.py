import numpy as np
import pandas as pd
import random 
File_name = 'test_file'+'.csv'

def create_setups(Exp_no, i ):
    random.seed(a=None, version=2)
    Period_duration = 600/3600 # Duration of each period in hrs
  
    Working_Period = 10
    T = int(Working_Period/Period_duration)  # Number of periods
    # R_to_C_ratio = round(random.uniform(1.5, 2.01), 2)
    # R_to_C = [3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, 8, 8.5, 9, 9.5, 10]
    # R_to_C = [11, 12, 13, 14, 15]
    # R_to_C_ratio = R_to_C[i]
    R_to_C_ratio = 2
    # R_to_Task_ratio = 1
    # r_to_t = [0.8, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5]
    # R_to_Task_ratio= r_to_t[i]
    R_to_Task_ratio = 0.8
    main_percentage = 0.8
    
    # R_to_Task_ratio = 0.8
    # C = random.randint(1, 10)
    # R = random.randint(0,21)
    # C = R
    # C=random.randint(1, 5)
    C= 8
    R = round(R_to_C_ratio * C)
    # R = 8
    W_N = round(R * R_to_Task_ratio)
    # W_N = 6
    W = 5
    S = 2
    
    q = 1
    Ebat = 111.0
    Edod = round(0.3 * Ebat)
    Emax = round(0.8 * Ebat)
    Charging_time=0.75
    
    
    no_of_main_robots= round(R * main_percentage)
    no_of_main_robots = 6
    main_robot_list = random.sample(range(0, R), no_of_main_robots)
    # main_robot_list = [3, 2, 0, 9, 7]
    main_dur = round(T/3)
    main_dur = 6
    main_dur = int(main_dur)
    
    
    Robots = range(0, R)
    Non_Navigation_Tasks = range(0, W)  # Set of non-navigation tasks, e.g., face recognition
    Navigation_Tasks=range(0,W_N) # Set of navigation paths
     
    
    # Gamma_Matrix={(h,j):np.random.randint(0,2) for h in Navigation_Tasks for j in Non_Navigation_Tasks}
    Gamma_Matrix = {(h, j): 1 for h in Navigation_Tasks for j in Non_Navigation_Tasks}

    # Assigning at least one Navigation task to a non navigation task
    # for j in Non_Navigation_Tasks:  
    #     if sum(Gamma_Matrix[h, j] <= 0 for h in Navigation_Tasks):
    #         n = np.random.randint(0, W_N)
    #         Gamma_Matrix[n, j] = 1  
            
    Priority = {j: 1 for j in Non_Navigation_Tasks}
    # Priority = {j: np.random.randint(1,11)/10 for j in Non_Navigation_Tasks}
    E_Balance_Zero = {i:np.random.randint(low=(Edod + 5), high=Ebat, size=None) for i in Robots}     # Initial energy balance (Wh)
    # E_Balance_Zero = {i:np.random.randint(low=80, high=Emax, size=None) for i in Robots}     # Initial energy balance (Wh)
    # E_Balance_Zero= {0: 80, 1: 105, 2: 72, 3: 89, 4: 91, 5: 59, 6: 100, 7: 53, 8: 41, 9: 82, 10: 45, 11: 99, 12: 109, 13: 96, 14: 81, 15: 77, 16: 100, 17: 105, 18: 70, 19: 53, 20: 76, 21: 60, 22: 42, 23: 47, 24: 93}
    # E_Balance_Zero = {0: 98, 1: 105, 2: 105, 3: 71, 4: 75, 5: 77, 6: 56, 7: 47, 8: 71, 9: 85, 10: 88, 11: 71, 12: 59, 13: 100, 14: 108, 15: 64, 16: 91, 17: 69}
    
    Robot_Speed = 1*3600 # Average robot speed in meter/hrs
    Dist_change_max = 500 # max distance to change navigation task or to go to a charging station
    
    
    
    
    
    Setting_df= {'Experiment_no':Exp_no, 'Maintenance (%)' : main_percentage, 'No of robs in main': no_of_main_robots, 'list of main_robs' : main_robot_list, 'Duration of Main' : main_dur, 'q' : q, 'Period_duration': Period_duration, 'Working_Period': Working_Period,'Total_periods': T, 'No_of_robots':R, 'No_of_chargers':C, 'No_of_sensors':S, 'No_of_non_nav_tasks':W, 
                                        'No_of_nav_task':W_N, 'Charging_time':Charging_time, 'Ebat':Ebat, 'Edod':Edod, 'Emax':Emax, 'Robot_Speed':Robot_Speed, 'Dist_change_max':Dist_change_max, 
                                        'Gamma_Matrix':Gamma_Matrix, 'E_Balance_Zero':E_Balance_Zero, 'Priority':Priority}
    return Setting_df



no_of_run = 20
df_final= pd.DataFrame()
i = 0

for Exp_no in range(no_of_run):
    df1 = create_setups(Exp_no, i)
    df_final =df_final.append(df1, ignore_index=True)
    i = i+1
   

print(df_final) 
df_final.to_csv(File_name, index = False)   