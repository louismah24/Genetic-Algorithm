# Genetic-Algorithm
This project builds upon the existing Genetic Algorithm/Creatures case study, which originally explored the evolution of simulated modular creatures within a virtual environment. The
foundational system employed genetic encoding to procedurally generate articulated bodies
using a simplified genome, allowing evolutionary processes such as crossover, mutation, and
selection to optimize physical forms for movement-based tasks.

# Mountain Climbing Genetic Algorithm

A Genetic Algorithm (GA) simulation that evolves modular creatures capable of climbing a procedurally generated mountain using the PyBullet physics engine.

This project extends the original "Creatures" evolutionary algorithm by replacing the flat terrain with a challenging mountain environment, encouraging creatures to evolve more effective climbing behaviours through natural selection.

## Features

- Procedurally generated 3D mountain terrain
- Physics simulation using PyBullet
- Evolution of articulated creatures from genetic genomes
- Multiple mutation operators:
  - Point mutation
  - Grow mutation
  - Shrink mutation
- Crossover and roulette-wheel parent selection
- Elitism to preserve the best-performing individual
- Fitness evaluation based on:
  - Maximum climbing height
  - Distance to the mountain peak

## Technologies

- Python
- PyBullet
- NumPy
- URDF
- Genetic Algorithms
- Evolutionary Computation

## Project Structure

```
mov-envt.py      # Main evolutionary simulation
creature.py      # Creature genome and body generation
motor.py         # Motor controllers
simulation.py    # Physics simulation
```

## Fitness Function

The fitness score combines two objectives:

- Reach a higher elevation on the mountain
- Move closer to the mountain peak

```
Fitness =
Maximum Height +
0.5 × (15 - Distance to Centre)
```

This encourages creatures to both climb and navigate towards the summit.

## Experiments

The project investigates the effect of several evolutionary parameters:

- Population size
- Genome size
- Mutation rate
- Number of generations

An additional experiment introduces new motor waveforms:

- Pulse
- Sine
- Triangle
- Random

to study how different control strategies affect climbing performance.

## Results

Key findings include:

- Increasing generations significantly improved fitness.
- A higher mutation rate (0.8) produced the best climbing performance.
- A population size of 30 achieved better results than 20 or 40.
- Smaller genomes evolved more efficiently.
- Allowing evolution to choose among multiple motor waveforms outperformed using a single motor type.


## Basic Experiments results
<img width="790" height="399" alt="image" src="https://github.com/user-attachments/assets/7f634e45-e99d-4fcb-a2f0-00751a572499" />


## Advanced Experiments results
<img width="795" height="89" alt="image" src="https://github.com/user-attachments/assets/796e8ce6-e70b-42c5-b94e-0e42f3f51263" />

## Future Improvements

- Parallel fitness evaluation
- More complex terrains
- Adaptive mutation rates
- Multi-objective optimisation
- Neural-network-based controllers

## Video Demonstration
https://github.com/user-attachments/assets/383bc6df-d7df-4e90-9376-18491d996919



