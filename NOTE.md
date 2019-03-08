### About using local modified Pkg:   
  - Please look into _interface.jl_ to see how to include local packages. Although it's not a perfect solution.  

### About generating local Pkg:  
  - For better encapsulation of codes and parallel computing, it's encouraged to move complete functional block into package. For instance, here I moved the Geometry block into Pkg "Geo". Just be careful to expose functions.

### Program Rules
  - Do not use global variables even in test files. Instead, use them in function block like main(), which will really speed up the program.
  - Break the first rule only when you want to test generated policy or other results in REPL.

### Ideas  
  1. By checking the process and intermediate output of QMDP solver, I found that the solver didn't even call observation() when generating a policy. This function is only called by Simulator and only when I explicitly introduce o as a parameter. So the idea is that we keep our observation model untouched and do not generate the wanted observation in our observation model. Instead we write a custom simulator to control every step of the simulation. Every step when next state is sampled, we generate a observation according to this state and the initial route to update the belief. In this way we can have a control of the direction of state/belief-update.  

### Performance  
  1. Because the default POMDPs.update() function, which implements a Bayesian Filter, has nested for loops, in which observation and transition function are called, which also have for loops in themselves. These leads to a very poor performance, thus a parallelization is necessary.  
  2. In Bayesian Filter, the states with a small probability can be ignored, which can significantly speed up the iterations (even parallelization is no more needed).
  3. Within Bayesian Filter, transition() is called for [length(state_space)]^2 times (currently 33075^2=1.1 billion), while observation() is called for length(state_space) times. So optimizing transition() and observation() is meaningful.
  4. In order to further refine the steps of s, v and a, which will cause space explosion, a parallelization of the QMDP-Solver is necessary. Please look into the source code of this solver and understand the algorithm.

### Problems
  1. The ego car behaves itself very extreme:  
     - The first step of the ego car is braking with a = -1. That should be right. Just imagine what happens if both cars keeps running without braking.
     - The result of the second step is a = 0.0, which is very abnormal. The probability of collision should still be high in this step.
     - In the third step, ego car chose to accelerate itself with a = 1.0. Dose it think that the other car brakes and it can go just as there is no other cars?
     - The fourth step is normal.
  2. If set acc_k = 0.0, the behavior of ego car has not changed. That's really weird.
  3. If the step of s is too rough, the R_crash function could not evaluate the state/action/behavior well, because the position of a state my be far away from junction point. So there will be no collision according to every state pair. Solution could be set one state pair according to the junction point, or refine s.
  4. What if collision were still not avoidable even if ego car chooses to brake itself with min(a)? The Ego Car would rather not brake in order not to be further punished in R_v() and R_acc() functions. So maybe a further reward rule should be added in R_crash function to encourage that if a collision isn't avoidable, the v and a should still be minimized(if V > 0 => a = -a_max; if V = 0 => a = 0). (done)
  5. Do we need to calculate the reward based on belief in our simulator? Currently based only on sampled state.
  6. Use random method to sample state/observation or just choose state/observation with highest probability?
