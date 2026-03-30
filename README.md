| Deliverable                                                                                                    | Due Date                            |
|----------------------------------------------------------------------------------------------------------------|-------------------------------------|
| Part A individual submission (Gradescope)                                                                      | Friday, March 20st at 11:59PM EST   |
| Part B team submission (Gradescope)                                                                            | Monday, April 6th at 11:59PM EST    |
| OPTIONAL: Parts D and E                                                                                        | Monday, April 6th at 11:59PM EST    |
| Pushed code to Git                                                                                             | Monday, April 6th at 11:59PM EST    |
| Briefing (8 min presentation + 3 min Q&A)                                                                      | Monday, April 6th at 1:00PM EST     |
| Report (uploaded to your team's website and [Canvas](https://canvas.mit.edu/courses/36874/assignments/449550)) | Friday, April 10th at 11:59PM EST   |
| TA Checkoff                                                                                                    | Wednesday, April 8th at 7:00PM EST  |
| [Team Member Assessment](https://forms.gle/BLvxNQFBBh2gAFzz5)                                                  | Wednesday, April 8th at 11:59PM EST |

*Note: We will assume the lowest score if the team member assessment form is not submitted on time*

*Note: Late checkoffs will incur a percentage penalty to your participation grade. Missed checkoffs will result in -1% to your overall grade*

**A lot of this lab's instructions are in this notebook with detailed instructions for each module: [README.ipynb](README.ipynb) We include them in a python notebook to take advantage of mathematical formatting that markdown doesn't support.** 

# Lab 5: Monte Carlo Localization (MCL)

## Introduction

Determining a robot's orientation and position in a known environment, also known as localization, is a critical problem in the field of robotics. As is common in robotics, this seemingly simple problem is surprisingly difficult, and remains an active research area. In this lab, you will solve robotic localization by implementing Monte Carlo Localization (aka MCL or particle filter). 

Localization is one major aspect of navigation; the next lab will serve as an extension of this one and cover the other major aspect, path planning. This is a challenging lab and we'd recommend starting early and moving fast, as your code/analysis for this lab will lay the groundwork for the next one.

This lab consists of a number of modules, some required and some extra credit, some individual and some with your team - for details on submission, grading, and deliverables, see the next section.

<img src="figures/pf.png" width="600">

## Submission and Grading

This section describes the grading scheme for Lab 5, while the instructions to get started on the lab at available in the [instructions notebook](README.ipynb).
The deliverables for this lab consist of five parts (two of which are optional) for a total of 10 points with 2 possible extra credit points. Parts A and D must be submitted individually, while the rest are teamwork. Parts A, B, and C are required and will be graded out of 10 points. Parts D and E are optional and 2 extra credit points are possible for a maximum grade of 12/10 points with extra credit. *Note that part A is due sooner than the others, to encourage getting an early start on it - see the deliverables table at the top of this handout for all deadlines. In addition, due to the individual nature of parts A and D, overall lab grades may differ across teammates.*

There will also be a report and briefing for this lab, and each teammate must submit the [team member assessment form](https://forms.gle/BLvxNQFBBh2gAFzz5). The lab 5 report will serve as a basis for your next lab, and you will be expected to revise and update sections in the future when writing the lab 6 report. 

This grade out of 10 points is then combined with the report and briefing grades (each also out of 10 points - same rubrics used as in the previous labs for [reports](https://canvas.mit.edu/courses/31106/assignments/393140) and [briefings](https://canvas.mit.edu/courses/31106/assignments/385210)). 
The grades will be weighted according to the table below for an overall lab grade out of 10 points.  

| Deliverable Grade           | Weighting |
|-----------------------------|-----------|
| Briefing grade (out of 10)  | 20%       |
| Report grade (out of 10)    | 40%       |
| Technical completion of lab | 40%       |

- **Part A - (Writing Assignment, 3pts)** Understand the motion and sensor model.
- **Part B - (Programming Assignment, 4pts)** Develop and test the particle filter algorithm in 2D racecar simulation environment.
- **Part C - (Localization, 3pts)** Adapt your solution from part B to work in your car and conduct experimental analysis for your report and briefing.
- *Part D - (OPTIONAL: Extra Credit, 1pts) Derive the Bayes' Filter presented in Lecture 10.*
- *Part E - (OPTIONAL: Extra Credit, 1pts) From localization to SLAM: Exploring SLAM and Visualizing with Foxglove!*

### Initial Setup

In order to build this package, we need to include a few dependencies that are not already included on the car. Note that this does **not** affect your work in the simulator.

Please pull the new docker image using the command `sudo docker pull staffmitrss/racecar2026`.

### Part A: Grading for writing assignment (3 points) - **INDIVIDUAL EFFORT**, *REQUIRED*

Part A requires you to answer several questions with written answers. The questions are listed in the [instructions notebook](README.ipynb). Submit your answers **individually** to the writing assignment on gradescope, preferably LaTeXed. You must show work (derivations, formulas used, etc.) to receive full credit. You may collaborate on problems but you cannot simply share answers - please note collaborators in your submission. You must write up your solutions independently. The 3 points of part A will be assigned based on the submitted written exercises:
- 1 point for part (i) in question 1
- 1 point for part (ii) in question 1
- 1 point for question 2

Submit your numeric answers along with your justifications to the gradescope assignment **Lab 5 Part A: Individual Submission**. You may check your numeric results by putting your answers in `/autograder/solutions_go_here.py` and uploading to the gradescope autograder **Lab 5 Part A: OPTIONAL**, but your grade will be based only on your explanations and answers submitted for the written portion. Just writing the final answer will not give total credit, even if correct. These questions will help you understand the algorithm before diving into coding. 

### Part B: Grading for simulation implementation (4 points) - **TEAMWORK**, *REQUIRED*

Implement MCL in the simulator. Remember to read the detailed description in [instructions notebook](README.ipynb). We want you to augment the simulated odometry data with various types of noise and compare how your solution does compared to the ground truth odometry. Points will be assigned based on completion and the convergement of your solution in the 2D racecar simulation environment.

You should submit your implementation to the **Lab 5 Part B: Localization in Simulation** assignment on gradescope as a zip of your localization package. We will expect to see your implementation in simulation during checkoffs. 

### [Running Unit Tests](#running-unit-tests)

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

If your code errors out or fails, the console will indicate that. Otherwise, you should see a few messages indicating that the tests have passed, and the script will exit. If you are curious about what might have gone wrong, please inspect the `localization/test/*.py` files. 

A couple notes about the tests, should you wish to use them:

- In the motion model test, we assume a deterministic motion model to keep things simple. For this reason, please have your motion model behavior be controlled by a `self.deterministic` field. If `self.deterministic = True`, the motion model `evaluate` should not add noise to the odometry. If `self.deterministic = False`, the motion model should add noise to the odometry (needed for localization).

- You may notice that the sensor_model precompute test is difficult to debug. To make this process easier, we have included a file `assets/debug_precomputed_table.pkl` containing precomputed tables you should be getting if `alpha = 1` for each of `alpha_hit`, `alpha_rand`, `alpha_max`, `alpha_short`. For example, if you load the dict into the variable `results_each`, `results_each['hit']` gives the table for `alpha_hit = 1` and `alpha_rand = alpha_short = alpha_max = 0`. 

- Please note that for short, the first column may be `nan` if you are trying to normalize the columns. As `np.nan != np.nan`, we followed the convention of `0/0 = 0`. This will not be an issue in practice, since you won't be having `alpha_short = 1`.

### Part C: Grading for localization in real world (3 points) - **TEAMWORK**, *REQUIRED*

For this part you will need to adapt your MCL implementation from part B to work in your car, and conduct experimental analysis of your algorithm's performance for your report and briefing. See part C of the [instructions notebook](README.ipynb) for more details on how to adapt your code to run in your car.

You are almost certainly going to need different parameters for your particle filter for running on the real robot in the real world, such as the number of samples and the noise coefficients on your motion and sensor models. To tune these parameters, you should have a small number of bags in different conditions, and you should be able to replay the bags with MCL running with different parameters, and measure the performance in terms of metrics that you choose. You should also carefully measure the time it takes for each update of your MCL implementation. If each update seems to be taking a very long time, you probably need to look carefully at your implementation. Also be sure to read the "Tips and tricks" section below. If your code uses several `for` loops, you will want to change that. 

In your report and briefing, make sure to provide:
- Numerical evidence that your algorithm is working in the form of charts / data
    - Specific metrics such as convergence rates, cross track error, etc, especially as a function of different parameter choices
    - In your lab report, please report on the runtime of your filter updates, especially as you change the number of samples and number of lidar rays you process 
    - Show how the robust the simulator is in the presence of **noisy odometry**, using **ground truth odometry** for comparison.
- An [illustrative video](https://www.youtube.com/watch?v=-c_0hSjgLYw&t=6s) of your particle filter working, overlaying
    - Visualization of inferred position
    - Visualization of the particle distribution
    - The known map
    - Laser scan data in the coordinate frame of your inferred position (it should align fairly well with the walls in the known map)
    - Any other byproducts of your algorithm which you find worthy of visualization
 
### Part B & C: Tips and Tricks

As the algorithm must run in realtime with a large number of particles, **an efficient implementation is a requirement for success**. There are a few tricks you can use, primarily:
- **Use numpy arrays for absolutely everything**
    - Use numpy functions on numpy arrays to do any computations.
    - Avoid Python for loops like the plague.
    - [Slice indexing is your (best) friend.](https://numpy.org/doc/1.20/reference/arrays.indexing.html)
    - Cache and reuse important numpy arrays by setting them to the right size during initialization of your particle filter as `self` variables.
- **Downsample your laser scan**: your lidar has > 1000 beams but many of them are redundant. Downsample to ~100 for good performance (you can try lower as well). This will make the probability distribution over your state space less "peaked" and increase the number of particles you can maintain in real time.
- **Start with ~200 particles**; don't go crazy with particles. You can probably get your code running with thousands of particles but it will take some well crafted code to run in real time.
- You may find it helpful to "squash" your sensor model output probability by raising it to a power of less than one (1/3 for example) to make your distribution even less peaked. For more details, look at [4,5].
- Your sensor model and motion model don't need to run at the same rate! The motion model is probably much faster and over short periods of time it will accurately track the motion of the car. The sensor model can correct the drift of the motion model at a slower rate if necessary.
- Use `ros2 topic hz` to check the rate at which you are publishing the expected transformation from the map to the car's position. It should be greater than 20hz for realtime performance.
- Use the smallest number of operations required to perform your arithmetic, avoid unnecessary memory allocations, and avoid excessive function calls.
- Identify your critical code paths, and keep them clean. Conversely, don't worry too much about code that is called infrequently.
- Publishing visualization messages may cause your system to be slower. It is recommended to set a ROS parameter to enable/disable visualizations.
- If you want an even faster (albeit more complicated to interface with) ray tracer check out [range_libc](https://github.com/kctess5/range_libc). This was written by RSS TA Corey Walsh and it is heavily optimized.

### Part D: Grading for the Bayes' filter derivation (1 bonus point) - **INDIVIDUAL EFFORT**, *OPTIONAL EXTRA-CREDIT*

Derive the form of the Bayes' Filter presented in Lecture 10. Submit as a typed PDF uploaded to the **Lab 5 Part D: OPTIONAL** gradescope assignment.

### Part E: Grading for SLAM (1 bonus point) - **TEAMWORK**, *OPTIONAL EXTRA-CREDIT* 

Use the provided SLAM package to create your own map of an area of your choosing, and then show your localization from Part C running in this area, for at least 10 seconds, driving at least 10 euclidean meters (loops encouraged). Specifically, we would like to see two videos side by side of the browser visualization and real-life car moving through the area you chose. Please choose a large enough room or area that is driveable, has plenty of obstacles/features/landmarks, and does not change too often. Foot traffic or other non-static objects may cause unwanted artifacts in your map. Additionally, we would like to hear a brief explanation during your checkoff on the differences between MCL (Mone Carlo Localization), and graph-based localization that is used in visual slam algorithms. 

## Lab Modules

The instructions to get started with Lab 5 are available in the [instructions notebook](README.ipynb).
