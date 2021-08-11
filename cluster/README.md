# Running Cases on PACE cluster
## Why use the cluster?
The advantage of the cluster is the ability to run many cases in parallel. Individual runs do not 
leverage parallelism to speed computations (each agent plans sequentially). The cluster is useful 
for large parametric studies.

## How to run a simulation on the cluster?
Running a single simulation on the Georgia Tech Pace cluster is very easy. The only thing you need 
to do is setup the conda environment, clone the repo and modify the pbs file as required (account name, 
file containing the case inputs, case number).

## How to run multiple cases on the cluster?
It depends on whether you are using the embers or inferno queue. Embers is free but comes with some 
constraints (max runtime, max number of cases that can run at a given time, cases can get preempted). 

If you are using inferno you can simply use a job array as illustrated in the job_array.pbs file. 
You must first create a json file containing the inputs of all the simulations you will be running 
(generate_input_file.py).

I am not including the method I used to run my cases on embers. It relied on a small library developed 
by Raphael Gautier that is not publicly available. It used GNU-Parallel to spawn workers that would 
request a case number from a distant server (running in a free AWS machine). It was a bit clunky. 
There is probably a better way to do it using pyLauncher.