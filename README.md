# PATCHER ver 0.1
Synthesizing scheduler patch for repairing control safety violations

This is part of Repeatability Evaluation Package for ICCPS-2025, for the paper titled "Repairing Control Safety Violations via Scheduler Patch Synthesis". 

# LICENSE
MIT License

# DESCRIPTION
PATCHER ("patcher-n.py") is a Python3 script that analyzes a control specification, task specification along with safety property. PATCHER checks if the given control systems meet safety specification, when implemented as a set of tasks that are scheduled on a unicore processor under NP-EDF scheduling policy. On discovering a safety violation, PATCHER tries to synthesize a drop set i.e. jobs that are *not* to be scheduled at runtime, such that control safety is achieved. Sample case studies are provided in the scripts "patcher-n.py". The task, control and safety specs are provided in the function systemspec(). See the comments in the script and edit the lines as required. 

# CONTROL SYSTEM SPECIFICATION
PATCHER requires augmented, discretized control system specs as input. Example *continuous time* models are located in the file "models.py". The script "example.py" augments the system with a one-period control delay and designs a discrete-time controller for the chosen discretization step. The discretized system with the controller forms the control spec to be given as input to PATCHER. Additionally, "example.py" can output a simulation run from a given starting state, which forms the ideal trajectory (i.e., without any missing control updates) to be given as (optional) input to PATCHER. 
The scripts "models.py", "example.py" and "controllers.py" are from https://github.com/Ratfink/ControlTimingSafety.jl , courtesy Shengjie Xu, Clara Hobbs and team.   

# NOTES
1) PATCHER outputs detailed messages at runtime. To suppress messages, comment out appropriate logging.info() calls in the code. 
2) NP-RM scheduling policy and "hold-and-kill" missing control update policy is not yet tested.
3) Analysis time is sensitive to number of jobs spawned ("horizon" value), number of tasks including  sporadic ones, number and size of control systems.

# INSTALLATION & DEPENDENCIES
PATCHER requires Python3, and package z3 (to install z3, run *pip3 install z3-solver*). Tested on Ubuntu 22.04. 
Control spec generation (optional step) requires packages numpy, control.

# HOW TO RUN
0) If required, discretize your (continuous) control system, with the "example.py" or "example-modified.py" script. The discretization timestep chosen forms the period for the corresponding task. To discretize, choose a model from "models.py", or enter new A,B,C,D matrices, and run *python3 example.py* after selecting the right model.

1) Execute the PATCHER script with an optional timeout and time command, since the main loop in the script is designed to explore the full state space of patches, and might take long to terminate:    *timeout <15m> time python3 patcher-n.py*

# LIMITATIONS
1) PATCHER currently does not automatically increment the analysis horizon (e.g., by adding the LCM of periods to current horizon), it has to be done manually when PATCHER fails to synthesize a patch for the given horizon.
2) PATCHER currently does not automatically adjust (increase) the initial state box bounds on discovering new control states during simulation. The init bounds have to be manually adjusted (increased), after examining PATCHER logs for simulations values. Refer to the paper on increasing init bounds. 
