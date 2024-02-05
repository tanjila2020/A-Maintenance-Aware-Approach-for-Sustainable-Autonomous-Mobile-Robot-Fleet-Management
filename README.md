
**Abstract**
Autonomous mobile robots (AMRs) are capable of carrying out operations continuously for 24/7, which enables them to optimize tasks, increase throughput, and meet demanding operational requirements. To ensure seamless and uninterrupted operations, an effective coordination of task allocation and charging schedules is crucial while considering the preservation of battery sustainability. Moreover, regular preventive maintenance plays an important role in enhancing the robustness of AMRs against hardware failures and abnormalities during task execution. 
However, existing works do not consider the influence of properly scheduling AMR maintenance on both task downtime and battery lifespan. In this paper, we propose MTC, a maintenance-aware task and charging scheduler designed for fleets of AMR operating continuously in highly automated environments. MTC leverages Linear Programming (LP) to first help decide the best time to schedule maintenance for a given set of AMRs. Subsequently, the Kuhn-Munkres algorithm, a variant of the Hungarian algorithm, is used to finalize task assignments and carry out the
charge scheduling to minimize the combined cost of task downtime and battery degradation. Experimental results demonstrate the effectiveness of MTC, reducing the combined total cost up to 3.45 times and providing up to 68% improvement in battery capacity degradation compared to the baselines.
Following are the important packages that are required to execute the MTC.py and MINLP.py files:
|package | pip installation command | 
| ------------- | ------------- | 
| [gurobipy](https://www.gurobi.com/documentation/9.5/quickstart_linux/cs_using_pip_to_install_gr.html)| `python -m pip install gurobipy`   | 
| [numpy](https://numpy.org/install/)  | `pip install numpy`  |   
| [matplotlib](https://matplotlib.org/stable/users/installing/index.html) | `python -m pip install -U matplotlib`|  
| [time](https://pypi.org/project/times/) | `pip install times`|  
| [pandas](https://pandas.pydata.org/docs/getting_started/install.html) | `pip install pandas`|  
| [math](https://pypi.org/project/python-math/) | `pip install python-math` |


**Note:** to use gurobypy, a Gurobi license is needed. This license is free for academic use and can be obtained from this [website](https://www.gurobi.com/academia/academic-program-and-licenses/). Gurobi is required for solving the optimization problems in MINLP, MTC.


This repository contains the proposed MTC algorithm and other baselines MINLP, RMA and RM explained in the paper. It also contains a `create_multiple_experiment.py` file which creates initial parameters in random order for different experiment setups and can create multiple experiments to run. 
### Baselines:
**RMA** is a simple baseline which selects both the maintenance period and the task allocation randomly but uses the same charging policy. 

**RM** is a simple baseline which randomly selects the maintenance period but uses the same task allocation and charging policy of the MTC algorithm.

**MINLP** is a simple MINLP model for the proposed approach, which schedules maintenance periods, task allocation, and charging times considering total cost formulated as defined by Equation (1) in the paper.

**MTC** is the proposed algorithm. 


### Creating multiple test scenarios:
To create different scenarios, change the range of variables used in `create_multiple_experiment.py`, This will create a CSV file with different experimental setups. Each row in the CSV(e.g., test.csv) is a different experimental setup. Use this file as input by changing the `File_name` variable in MTC and other baselines to get the solution for each scenario. MTC and other baselines will write their respective results in the same file. ( **Note that the CSV file must be in the same folder along with other scripts** )
For defining the number of experiments you want to create go to the line no. 87 of `create_multiple_experiment.py` and set the value of the parameter "no_of_run" as required, for example: 1 for single experiment and so on.


### Executing scripts:
1. Update the `File_name` variable with the input CSV file name (for example, test.csv) in line no 18 in MTC.py. This will make the script get the initial parameters from the CSV file and write the results to the CSV file. 
2. The script will automatically update the experiment results in the designated CSV file upon completion.
3. For conducting a single experiment, set the "single_exp" parameter to 1 on line 153 of MTC.py. For multiple experiments, adjust this value to 0.
4. Repeat the steps outlined in instructions 1-3 for executing baseline scripts RMA.py, RM.py, and MINLP.py.
5. To visualize the State of Charge and other metrics graphs for an individual experiment, ensure the input CSV contains only one data row and that the "single_exp" parameter is set to 1.
6. Within the repository, two sample input CSV files are provided, named "1_exp.csv" and "multiple_exp.csv", to facilitate the setup of initial experiments for singular and multiple runs, respectively. Note that all the variable named with "TCM" in the code refer to the MTC in the paper.

**Citation**

@ARTICLE{10323097,
  author={Atik, Syeda Tanjila and Chavan, Akshar Shravan and Grosu, Daniel and Brocanelli, Marco},
  journal={IEEE Transactions on Mobile Computing}, 
  title={A Maintenance-Aware Approach for Sustainable Autonomous Mobile Robot Fleet Management}, 
  year={2023},
  volume={},
  number={},
  pages={1-14},
  keywords={Task analysis;Batteries;Degradation;Schedules;Navigation;Job shop scheduling;Resource management;Autonomous mobile robots;battery degradation;preventive maintenance;task and charge scheduling},
  doi={10.1109/TMC.2023.3334589}}
