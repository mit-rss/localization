| Deliverable | Due Date              |
|---------------|----------------------------------------------------------------------------|
| Part A individual submission (gradescope) | Friday, March 22nd at 11:59PM EST |
| Part B team submission (gradescope) | Monday, April 8th at 11:59PM EST |
| Pushed code | Monday, April 8th at 11:59PM EST |
| OPTIONAL: Parts D and E | Monday, April 8th at 11:59PM EST |
| **Briefing slides (slides on github pages)**  | Monday, April 8th at 1:00PM EST |
| Briefing (8 min presentation + 3 min Q&A) | Wednesday, April 10th at 1:00PM EST |
| Report (on team github pages website) | Wednesday, April 10th at 11:59PM EST |
| [Team Member Assessment](https://forms.gle/Rcg1j11pwGrZT2up7) | Wednesday, April 10th at 11:59PM EST |


**notebook with detailed instructions for each module: [README.ipynb](README.ipynb)**

# Lab 5: Monte Carlo Localization (MCL)

## Introduction

Determining a robot’s orientation and position in a known environment, also known as localization, is a critical problem in the field of robotics. As is common in robotics, this seemingly simple problem is surprisingly difficult, and remains an active research area. In this lab, you will solve robotic localization by implementing Monte Carlo Localization (aka MCL or particle filter). This is a challenging lab and we'd recommend starting early and moving fast.

This lab consists of a number of modules, some required and some extra credit, some individual and some with your team - for details on submission, grading, and deliverables, see the next section.

<img src="figures/pf.png" width="600">

## Submission and Grading

This section describes the grading scheme for Lab 5, while the instructions to get started on the lab at available in the [instructions notebook](README.ipynb).
The deliverables for this lab consist of five parts (two of which are optional) for a total of 10 points with 2 possible extra credit points. Parts A and E must be submitted individually, while the rest are teamwork. There will also be a report and briefing for this lab, and each teammate must submit the [team member assessment form](https://forms.gle/Rcg1j11pwGrZT2up7). *Note that part A is due sooner than the others, to encourage getting an early start on it - see the deliverables table at the top of this handout for all deadlines.*

Parts A, B, and C are required and will be graded out of 10 points. Parts D and E are optional and 2 extra credit points are possible for a maximum grade of 12/10 points with extra credit. This grade out of 10 points is then combined with the report and briefing grades (each also out of 10 points - same rubrics used as in the previous labs for [reports](https://docs.google.com/document/d/1B6l7vKJFN3CPPcMn8cKKArHUU_Bq_YUZ5KxKoP6qMk0/edit?usp=sharing) and [briefings](https://docs.google.com/document/d/1NmqQP7n1omI9bIshF1Y-MP70gfDkgEeoMjpWv8hjfsY/edit?usp=sharing)). The grades will be weighted according to the table below for an overall lab grade out of 10 points. *Note that due to the individual nature of parts A and E, overall lab grades may differ across teammates.*

| Deliverable Grade | Weighting              |
|---------------|----------------------------------------------------------------------------|
| briefing grade (out of 10)  | 20% |
| report grade (out of 10) | 40% |
| grade for parts A-E (out of 10, up to 12/10 with extra credit) | 40% |

-   **Part A - (Writing Assignment)** Understand the motion and sensor model.
-   **Part B - (Programming Assignment)** Develop and test the particle filter algorithm in 2D racecar simulation environment, upload solution to gradescope for autograder evaluation.
-   **Part C - (Localization)** Adapt your solution from part B to work in your car and conduct experimental analysis for your report and briefing.
-   *Part D - (OPTIONAL: Extra Credit) From localization to SLAM: Coming Soon!*
-   *Part E - (OPTIONAL: Extra Credit) Derive the Bayes' Filter presented in Lecture 10.*

### Before You Begin: Initial Setup

In order to build this package, we need to include a few dependencies that are not already included on the car. Note that this does **not** affect your work in the simulator.

Please come find an instructor to do this for you, as this step requires an internet connection.


============================= **Note for TAs** ============================= 

Pull the updated docker image (`sebagarc/hardwareros2-production`), which contains:

- the re-routed `racecar_simulator`
- `SIM_WS` environment variable
- two additional apt packages.

Verify that the localization package can be built.

=====================================================================


### Part A: Grading for writing assignment (3 points) - **INDIVIDUAL EFFORT**, *REQUIRED*

Submit your answers **individually** to the writing assignment on gradescope, preferably LaTeXed. You must show work (derivations, formulas used, etc.) to receive full credit. You may collaborate on problems but you cannot simply share answers. You must write up your solutions independently. The 3 points of part A will be assigned based on the submitted written exercises:
- 1 point for part (i) in question 1
- 1 point for part (ii) in question 1
- 1 point for question 2

Submit your numeric answers along with your justifications to the gradescope assignment **Lab 5 Part A: Individual Submission**. You may check your numeric results by putting your answers in `/autograder/solutions_go_here.py` and uploading to the gradescope autograder **Lab 5 Part A: OPTIONAL**, but your grade will be based only on your explanations and answers submitted for the written portion. Just writing the final answer will not give total credit, even if correct. These questions will help you understand the algorithm before diving into coding.

### Part B: Grading for simulation implementation (4 points) - **TEAMWORK**, *REQUIRED*

Implement MCL in the simulator. Augment the simulated odometry data with various types of noise and compare how your solution does compared to the ground truth odometry. Ask a TA for a checkoff. Points will be assigned based on completion and the convergement of your solution in the 2D racecar simulation environment.

Before your briefing, you should seek out a TA to check off your solution during lab hours *before the day of briefings*. In the case that you do not finish this part during lab hours, please contact your point TA about scheduling another time for checkoff *with timely notice* or see if any TAs are free during offices hours. Still, the priority during office hours is to help students with their questions not do lab check offs. At least one member of your team needs to be present for the checkoff and be ready to answer questions about your implementation.

You should submit your implementation to the **Lab 5 Part B: Localization in Simulation** assignment on gradescope as a zip of your localization package.

**Running Unit Tests (updated 1 April 2024)**

We have provided a few unit tests for you to test your sensor model and motion model. To run these tests, do:

```bash
# ====== motion model ======
ros2 launch localization motion_model_test.launch.py
# ==========================
# ====== sensor model ======
ros2 launch localization sensor_model_test.launch.py
# this will wait for you to run test_map.launch.xml in another terminal
ros2 launch localization test_map.launch.xml
# ==========================
```

If your code errors out or fails, the console will indicate that. Otherwise, you should see a few messages indicating that the tests have passed, and the script will exit. If you are curious about what might have gone wrong, please inspect the `launch/unit_tests/*.py` launch files. 

### Part C: Grading for localization in ROBOT (3 points) - **TEAMWORK**, *REQUIRED*

For this part you will need to adapt your MCL implementation from part B to work in your car, and conduct experimental analysis of your algorithm's performance for your report and briefing. See part C of the [instructions notebook](README.ipynb) for more details on how to adapt your code to run in your car.

In your report and briefing, make sure to provide:
- Numerical evidence that your algorithm is working in the form of charts / data
    - Convergence rates, cross track error, etc
    - Show how the robust the simulator is in the presence of **noisy odometry**, using **ground truth odometry** for comparison.
- An [illustrative video](https://www.youtube.com/watch?v=-c_0hSjgLYw&t=6s) of your particle filter working, overlaying
    - Visualization of inferred position
    - Visualization of the particle distribution
    - The known map
    - Laser scan data in the coordinate frame of your inferred position (it should align fairly well with the walls in the known map)
    - Any other byproducts of your algorithm which you find worthy of visualization

### Part D: Grading for SLAM (1 bonus point) - **TEAMWORK**, *OPTIONAL EXTRA-CREDIT*

Experiment with SLAM 

### Part E: Grading for the Bayes' filter derivation (1 bonus point) - **INDIVIDUAL EFFORT**, *OPTIONAL EXTRA-CREDIT*

Derive the form of the Bayes' Filter presented in Lecture 10. Submit as a typed PDF uploaded to the **Lab 5 Part E: OPTIONAL** gradescope assignment.

## Lab Modules

The instructions to get started with Lab 5 are available in the [instructions notebook](README.ipynb).
