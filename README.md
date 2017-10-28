# Pendulum Simulation
A simple simulation of a multi-link pendulum (an assignment for Animations CPSC 526 - UBC).
The starting code was written by [Michiel van de Panne](https://www.cs.ubc.ca/~van/).

## How to use

### Installation
```pip install -r requirements.txt```

### How to run the code

```
git clone https://github.com/farzadab/pendulum_simulation/
cd pendulum_simulation

# for a single-link pendulum run:
python3 sim.py --no-gplane

# for a multi-link pendulum run:
python3 sim.py --nlinks 4 --no-gplane

# for a multi-link pendulum with a ground plane:
python3 sim.py --nlinks 4 --gplane

# to plot the K/E/T energies at the end of the simulation:
python3 sim.py --nlinks 4 --no-gplane --plot
```

### Parameters
| Parameter | Description | Default value |
| ------------- |:-------------|:-----:|
|nlinks | number of links | 1 |
|timestep | timestep in seconds | 0.01 |
|scv | constant value for velocity stabilization | 2 |
|scp | constant value for position stabilization | 0.2 |
|gsv | ground stiffness factor for velocity | 100 |
|gsp | ground stiffness factor for position | 100 |
|gplane | add a ground plane (can be disabled by `no-gplane`) | True |
|plot | plot energies at the end (uses a browser) | False|

### Keyboard commands
| Key | Effect |
| ------------- |:-------------|
|q or ESC | will quit simulation|
|space | pauses the sim|
|r | resets the simulation|


## Images
![4 link demo](https://github.com/farzadab/pendulum_simulation/raw/master/demo/4links.png)
