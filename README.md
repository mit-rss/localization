| Deliverable | Due Date              |
|---------------|----------------------------------------------------------------------------|
| Part A individual submission (gradescope) | Wednesday, March 16th at 1:00PM EST |
| Briefing (8 min presentation + 3 min Q&A) (slides on github pages)  | Wednesday, March 30th at 1:00PM EST |
| Report (on team github pages website) | Friday, April 1st at 11:59PM EST |
| Part B team submission (gradescope) | Friday, April 1st at 11:59PM EST |
| Pushed code | Friday, April 1st at 11:59PM EST |
| [Team Member Assessment](https://forms.gle/qwQtuj4cMP6Qiy518) | Friday, April 1st at 11:59PM EST |
| OPTIONAL: Parts D and E | Friday, April 1st at 11:59PM EST |

**notebook with detailed instructions for each module: [README.ipynb](README.ipynb)**

# Lab 5: Monte Carlo Localization (MCL)

## Introduction

Determining a robot’s orientation and position in a known environment, also known as localization, is a critical problem in the field of robotics. As is common in robotics, this seemingly simple problem is surprisingly difficult, and remains an active research area. In this lab, you will solve robotic localization by implementing Monte Carlo Localization (aka MCL or particle filter). This is a challenging lab and we'd recommend starting early and moving fast.

This lab consists of a number of modules, some required and some extra credit, some individual and some with your team - for details on submission, grading, and deliverables, see the next section.

<img src="figures/pf.png" width="600">

## Submission and Grading

This section describes the grading scheme for Lab 5, while the instructions to get started on the lab at available in the [instructions notebook](README.ipynb).
The deliverables for this lab consist of five parts (two of which are optional) for a total of 10 points with 2 possible extra credit points. Parts A and E must be submitted individually, while the rest are teamwork. There will also be a report and briefing for this lab, and each teammate must submit the [team member assessment form](https://docs.google.com/forms/d/e/1FAIpQLSeH7moDd1OhA5nKpBgstc7plhaBMFm3L1H99joylZSgegmQYw/viewform?usp=sf_link). *Note that part A is due sooner than the others, to encourage getting an early start - see the deliverables table at the top of this handout for all deadlines.*

Parts A, B, and C are required and will be graded out of 10 points. Parts D and E are optional and 2 extra credit points are possible for a maximum grade of 12/10 points with extra credit. This grade out of 10 points is then combined with the report and briefing grades (each also out of 10 points - same rubrics used as in the previous labs for [reports](https://docs.google.com/document/d/1B6l7vKJFN3CPPcMn8cKKArHUU_Bq_YUZ5KxKoP6qMk0/edit?usp=sharing) and [briefings](https://docs.google.com/document/d/1NmqQP7n1omI9bIshF1Y-MP70gfDkgEeoMjpWv8hjfsY/edit?usp=sharing)). The grades will be weighted according to the table below for an overall lab grade out of 10 points. *Note that due to the individual nature of parts A and E, overall lab grades may differ across teammates.*

| Deliverable Grade | Weighting              |
|---------------|----------------------------------------------------------------------------|
| briefing grade (out of 10)  | 20% |
| report grade (out of 10) | 40% |
| grade for parts A-E (out of 10, up to 12/10 with extra credit) | 40% |

-   **Part A - (Writing Assignment)** Understand the motion and sensor model.
-   **Part B - (Programming Assignment)** Develop and test the particle filter algorithm in 2D racecar simulation environment, upload solution to gradescope for autograder evaluation.
-   **Part C - (Localization)** Adapt your solution from part B to work in your car and conduct experimental analysis for your report and briefing.
-   *Part D - (OPTIONAL: Extra Credit) From localization to SLAM: configure and run Google Cartographer in the simulator.*
-   *Part E - (OPTIONAL: Extra Credit) Derive the Bayes' Filter presented in Lecture 10.*

### Part A: Grading for writing assignment (3 points) - **INDIVIDUAL EFFORT**, *REQUIRED*

Submit your answers **individually** to the writing assignment on gradescope, preferably LaTeXed. You must show work (derivations, formulas used, etc.) to receive full credit. You may collaborate on problems but you cannot simply share answers. You must write up your solutions independently. The 3 points of part A will be assigned based on the submitted written exercises:
- 1 point for part (i) in question 1
- 1 point for part (ii) in question 1
- 1 point for question 2

Submit your numeric answers along with your justifications to the gradescope assignment **Lab 5 Part A: Individual Submission**. You may check your numeric results by putting your answers in `/autograder/solutions_go_here.py` and uploading to the gradescope autograder **Lab 5 Part A: OPTIONAL**, but your grade will be based only on your explanations and answers submitted for the written portion. Just writing the final answer will not give total credit, even if correct. These questions will help you understand the algorithm before diving into coding.

### Part B: Grading for implementation and autograder (4 points) - **TEAMWORK**, *REQUIRED*

Implement MCL in the simulator. Augment the simulated odometry data with various types of noise and compare how your solution does compared to the ground truth odometry. Run your implementation of MCL on the gradescope autograder. Points will be assigned based on performance as compared to the TA solution in the 2D racecar simulation environment.

You should submit your implementation to the **Lab 5 Part B: Localization in Simulation** assignment on gradescope as a zip of your localization package. See section 5 of part B in the [instructions notebook](README.ipynb) for more details on submission format and how exactly the autograder will evaluate your implementation. *Only one member of your team needs to submit the code to the autograder.*

**Note that while the autograder will add noise to the odometry when evaluating your solution, you must augment the odometry with your own noise when using the 2D simulation environment, both in order to increase your confidence in your solution and to evaluate your implementation in a realistic environment when providing analysis in your report and briefing.**

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

### Part D: Grading for SLAM with Google Cartographer (1 bonus point) - **TEAMWORK**, *OPTIONAL EXTRA-CREDIT*

Experiment with SLAM by configuring and running Google Cartographer in the simulation environment. Include your results in your lab report and presentation.

- Build a map of the Bldg 31 OR STATA basement using Cartographer (these are the usual provided maps)
- Localize using Cartographer
- Show visualization evidence that Cartographer is working in the simulator

### Part E: Grading for the Bayes' filter derivation (1 bonus point) - **INDIVIDUAL EFFORT**, *OPTIONAL EXTRA-CREDIT*

Derive the form of the Bayes' Filter presented in Lecture 10. Submit as a typed PDF uploaded to the **Lab 5 Part E: OPTIONAL** gradescope assignment.

## Lab Modules

The instructions to get started with Lab 5 are available in the [instructions notebook](README.ipynb).
