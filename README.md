# README #


This repo contains the codes for paper [Integrated Task Assignment and Path Planning for Capacitated Multi-Agent Pickup and Delivery](https://arxiv.org/abs/2110.14891).


## Compiling


Clone this repo.

```
$ mkdir build
$ cd build
$ cmake ../
$ make
```

### Dependencies


**Boost**, **Google-sparsehash** are required for compiling.

If using ubuntu, you can install them by:
```
$ sudo apt-get update
$ sudo apt-get install sparsehash
$ sudo apt-get install libboost-all-dev
```

They are also available on Homebrew, if using mac os.


## Usage Examples and Arguments


Kiva Instances are provided in **examples** folder. 

**kiva-agent-maps**: Including a kiva map with different amount of agents on the map. In each map, "@" indicate obstcale, "." indicate an open space, "e" indicate an endpoint, and "r" indicate an agent initial location.
For example, kiva-10-500-5 indicate a map with 10 agents on the map. All maps have same layout, 
just agents are different.

**kiva-tasks**: Including 25 kiva task instance with different release frequency. The number in first column is task release timestep, second column is the task starting endpoint ID, the third column is the task goal endpoint ID.
For example, the folder with name 0.2-500 indicate each instance have 500 tasks with release frequency of 0.2. 
For folder without frequency, all tasks are released at timestep 0.

```
$ ./mapd -m path/to/kiva-agent-maps/kiva-10-500-5.map 
    -a path/to/kiva-agent-maps/kiva-10-500-5.map
    -t path/to/kiva-tasks/1-500/0.task
    -c 60
    -s PP
    --capacity 1
    --objective total-travel-delay
    --only-update-top
    --kiva
    -o path/to/output/file
```

**-m [path]** indicate a map to load

**-a [path]** indicate the agents to load

**-t [path]** indicate the task to load

**-s pp** must have this to run coupled task and path planning

**--capacity** the maximun capacity of agents

**--kiva** must have this argument to load kiva instances in examples.

**--objective [total-travel-delay]** The optimize objective, can be total-travel-delay or makespan

**--online** run example in lifelong mode. Without this argument, codes run in offline mode.

**--only-update-top** must have this to only update the top elements in the heaps.

**--regret** run in RMCA. Without this argument codes run as MCA

**--anytime** enable anytime improvement after assignment.

**--group-size [8]** group size for anytime improvement.

**--destory-method [random]** can be "destory-max","multi-max","random"

**-c [60]** only limit each anytime optimization time. For offline mode, you can set -c as long as possible. For online mode, we only give several seconds for optimze as optimize repeat many times by the assignment process.


## Example arguments to recreate algorithms in the paper


### MCA

Offline:
```
--only-update-top --objective total-travel-delay
```

offline with any time improvement(change numbers and options as your requirements):
```
--only-update-top --objective total-travel-delay --anytime -c 60 --group-size 5 --destory-method random
```

Online:
```
--only-update-top --objective total-travel-delay --online
```

online with any time improvement(change numbers and options as your requirements):
```
--only-update-top --objective total-travel-delay --online --anytime -c 1 --group-size 5 --destory-method random
```


### RMCA

Offline:
```
--regret --only-update-top --objective total-travel-delay
```

offline with any time improvement(change numbers and options as your requirements):
```
--regret --only-update-top --objective total-travel-delay --anytime -c 60 --group-size 5 --destory-method random
```

Online:
```
--regret --only-update-top --objective total-travel-delay --online
```

online with any time improvement(change numbers and options as your requirements):
```
--regret --only-update-top --objective total-travel-delay --online --anytime -c 1 --group-size 5 --destory-method random
```

**Hints**

Offline examples does not works well with release frequency smaller than 1, as the low level search(path planning) will be extreme slow if a late released task is assigned frist to pick up.











