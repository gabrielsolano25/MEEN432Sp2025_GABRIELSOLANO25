
Overview  
--------
This Simulink model simulates an electric vehicle (EV) driving around an oval track. It includes lateral and longitudinal dynamics, 
a motor and battery system, regenerative and friction braking, and a simple driver controller. 
The model is designed to evaluate performance under rules like staying on track and maintaining battery SOC. I have also implemented the non-horizontal track specific to this individual submission. The hill gradient is now factored in and the car achieves a maximum distance in 1 hour without leaving the track as specified. 

How to Use It  
---------
1. Open the Simulink model file: `Project4_MODEL_FINAL.slx` and `p4_init.slx`.

2. Run the script: `Project4_FINAL_Script.m`and `p4_init.slx`. This sets all parameters and starts the simulation.

3. After running the script, view the results:
   - Lap Time and Lap Completion Results
   - Battery SOC
   - Braking forces (friction and regen)

4. The simulation includes a condition to stop applying torque when SOC drops below 10%.

How It Works 
---------
- The driver uses a “lookahead” algorithm to steer toward a point ahead on the track.
- Lateral dynamics compute forces based on tire slip and yaw rate.
- Longitudinal dynamics balance motor force, drag, and braking.
- Regenerative braking feeds power back to the battery when decelerating.
- Friction braking is enforced when speed is low or SOC is too high.
- Implementation of Sloped force now factored into acceleration equation 

---------
