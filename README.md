# CarND-Controls-PID
A PID controller is used to direct the vehicle around the course of the udacity simulator. The components of the controller have the following function:
- P: directly acts on the crosstrack error (cte) and produces a steering angle command proportional to the cte.
- I: produces a steering angle command that is proportional to the integral of the cte and thus can reduce errors that would persist over time if only P control were to be used. This is due to the fact that by integrating the cte the compensation signal increases over time.
- D: increases controller dynamics by producing a compenstation signal proportional to the derivative of the cte.

The controller parameters are tuned with the twiddle algorithm from the udacity tutorials that was ported to c++ for this purpose.