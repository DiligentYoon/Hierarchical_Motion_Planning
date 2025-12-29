# Simulink-based Highway Lane Change Planner

This project implements a highway lane change planner using Simulink. It is designed to simulate and test the decision-making and trajectory generation logic for an autonomous vehicle performing lane changes in a multi-lane highway environment.

## Project Structure

The repository is organized as follows:

- **`Ours/`**: Contains the core Simulink models and MATLAB helper functions for the lane change planner.
    - `HighwayLaneChangePlanner.slx`: The main planner model.
    - `HighwayLaneChangePlannerTestBench.slx`: The test bench for running and verifying the planner with different scenarios.
    - `helper*.m`: Various MATLAB scripts for setup, bus creation, and plotting.
    - `Results/`: Contains simulation results, including GIFs.
- **`Baselines/`**: Contains baseline models for comparison.
- **`Scenarios/`**: Contains MATLAB scripts that define various driving scenarios. Each script sets up the road, surrounding vehicles, and the ego vehicle's initial state.
- **`resources/`**: Project-related resources and configurations.

## How to Run a Simulation

1.  **Open MATLAB.**
2.  **Set the MATLAB path:** Add the project folder and its subfolders to the MATLAB path.
3.  **Load a Scenario:** Run one of the scenario scripts from the `Scenarios/` directory (e.g., `run('Scenarios/scenario_01_DecisionTrigger.m')`). This will load the `scenario` and `egoVehicle` objects into the MATLAB workspace.
4.  **Open the Test Bench:** Open the `Ours/HighwayLaneChangePlannerTestBench.slx` file in Simulink.
5.  **Run the Simulation:** Click the "Run" button in the Simulink model to start the simulation. The model will use the variables loaded from the scenario script.

## Simulation Result Examples

The following GIFs show the results of running different scenarios. The ego vehicle is colored blue.

### Scenario 1: Merging Scenario
<p align="center">
<img src="Ours/Results/scenario_1_result.gif" width="480"/>
</p>  

### Scenario 2: Cluttered Scenario
<p align="center">
<img src="Ours/Results/scenario_3_result.gif" width="480"/>
</p>  