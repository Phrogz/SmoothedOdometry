# Smoothed Odometry

This is an experimental simulation used to quickly test and iterate on techniques
for fusing together velocity-based odometry with one or more AprilTag detections.

## How Do I Use It?

Copy the files into a directory and open the HTML file with a web browser. If
all goes well you should see something similar to this video:
https://youtu.be/wmUtdmmf6cU

Use WASD or the arrow keys to move the robot. Use Q/E to rotate the bot.


### What Am I Looking At?

The orange box is the robot, whose camera is pointed in the direction of the arrow.

When the robot can detect an AprilTag, the (noisy) pose it produces it shown
as a red dot on the screen (which fade to pink over time). 

An orange trace shows where the center of the bot truly is over time.
Occasionally the bot gets hit by another robot (that we can't see) that bumps
its real location suddenly.

The blue and green dots (which are often so similar as to be on top of each other)
show the predicted and estimated locations of the bot over time. (Blue prediction
is velocity-based odometry only; green estimate is the fusion of predictions
and AprilTags.)


### How Does It Work?

The short answer is:

1. Initialize the prediction and estimate with a best guess of
   where the robot starts.
2. Each update, recalculate the prediction (e.g. based on wheel rotations).
3. Then, update the estimate by moving it towards the prediction by some percent;
   the higher the value, the more it means you trust your odometry.
   * _This simulation currently uses 70% for this value._
4. Then, if any AprilTags are detected and gave you a pose, move the estimate
   towards each pose by a some percent. The noisier the AprilTag data, the
   lower your percent should be.
   * _Low percents smooth out heavy noise over time, but also take time to
     converge. When the robot is ~stopped, this simulation uses 10%._
   * _Higher percents reduce the amount of time lag needed to converge.
     When the robot is moving, this simulation uses 50%._

See the few lines in the `estimatePose()` function for implementation.
Read on below for additional background.


## Discussion

### Background
This simulation represents a robot in the [FRC 2024 Crescendo][1] competition.
We get reasonably good odometry data from the robot as it drives around the field
based solely on information from the robot's wheels. However, this data can drift
out of synchronization over time to the degree that the measurements are imperfect,
or when external forces move our bot suddenly (e.g. other bots bumping into us).

As a result, our bot also includes a vision system that recognizes the
[AprilTags][2] placed around the field. These are used to "localize" the robot.
In an ideal world, just seeing an AprilTag allows the robot to know precisely
where it is located and how it is oriented. In practice, the poses reported
from the vision system can have significant amounts of noise in the data. The
amount of noise increases based on distance from the AprilTags (as the image
seen by the camera is smaller and increasingly less precise) and also the more
the AprilTag is off-axis from the camera. Further, AprilTags are not always
visible from everywhere on the field.

So, we want to "blend" together our internal odometry information--which is
usually right, until it isn't--with noisy AprilTag detections in a way that
produces a result that is better than either on its own.


### Kalman Filters?

Conventional wisdom says that the "right" way to perform this sort of prediction
is to use a [Kalman Filter][3]. This technique has several benefits to
recommend it, and is even [included in WPILib][4]. The following, however,
are contraindications to using it:

* Using a Kalman filter requires substantial investment in understanding the
  math and concepts. The WPILib documentation suggests a minimum of 40 pages
  of a 500 page textbook to understand the basics, though most of the textbook
  is applicable.

* My (limited) investigation implies that many of the provably-correct benefits
  of Kalman filters only apply during linear scenarios. Once non-linearities
  (like the robot accelerating) are introduced, and once multiple data sources
  need to be fused together, we enter the domain of varations on the Kalman
  filter. While "Nonlinear", "Unscented", "Extended", and "Ensemble" Kalman
  Filters share the name and concepts of the original, the no longer produce
  the same correctness as the original.

This simulation is an attempt to see if a far-simpler solution
-- both in terms of the math used and the concepts needed --
can suffice to produce good results.


### IIR Filters

An "infinite impulse response" (IIR) filter is a very simple system that 
produces a smoothed value, with very low computation and memory requirements.
In pseudo-code:

```python
trustAmount = 0.7   # Move 70% towards a new value each update
smoothedValue = someInitialGuess()
for each update:
    newValue = nextValue()
    smoothedValue += (newValue - smoothedValue) * trustAmount
```

Unlike a moving average, this only requires a single variable to keep track of
the history, and by adjusting `trustAmount` between `0.0` and `1.0` you can
fully tune the amount of smoothing, from flat-line ignoring of new values
to completely trusting the new value, and everything in between.

As in a Kalman filter, the more we trust a data source, the more it should
contribute towards our next result, the higher our `trustAmount` should be.

In action, our `estimate` is the smoothed location of the robot. Every frame
we move the old value towards our `prediction` of where the robot is, heavily
weighting this (because we trust our prediction). Then when we get AprilTag
poses, we gently nudge the estimate towards the pose we get. Because we don't
trust the noisiness of this data, we only move it a little. Over many frames,
this pulls the estimate to the ~average of all the noisy samples.


### Conclusion

It's not perfect. Where a true Kalman filter would (I believe) move a static
bot's `estimate` towards the true average of the noisy values over time, an
IIR filter continues to be influenced by the noise. However, from a visual
inspection of this simulation, it appears that even with TERRIBLY noise
AprilTag poses and odometry which is far poorer than our actual data, the
paired IIR filters used in this approach appear to produce results that
converge to "good enough" values quickly enough.


## Credits

Simulation originally written by Gavin Kistner for Team 2972 (Gears & Buccaneers)

This file uses the [victor.js library][7] for simple
manipulation of 2D vectors. My thanks to the authors for sharing their work.

[1]: https://www.firstinspires.org/robotics/frc/game-and-season
[2]: https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html
[3]: https://en.wikipedia.org/wiki/Kalman_filter
[4]: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html
[7]: http://victorjs.org