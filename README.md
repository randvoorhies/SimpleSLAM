The following is a simple reference implementation of Simultaneous Localization
And Mapping written in MATLAB. The code simulates a robot cruising around in
a 2D world which contains 5 uniquely identifiable landmarks at unknown
locations. The robot is equipped with a sensor that can measure the range and
angle to these landmarks, but each measurement is corrupted by noise.

This implementation is based on Thrun et. al's FastSLAM 1.0 algorithm, though
all optimizations have been removed for clarity. The algorithm works by
representing the position of the robot via a Particle Filter. Each particle
contains 5 completely independent Extended Kalman Filters, each of which is
responsible for tracking a single landmark. After each measurement of
a landmark, each particle is given a score based on how closely the latest
landmark measurement matched its expectation. The list of particles is then
resampled such that particles with high scores are likely to be replicated, and
those with low scores are likely to be deleted.

To run this demo, just checkout this project, and run slam.m 

![](https://github.com/randvoorhies/SimpleSLAM/raw/master/screenshot.png)