# Kidnapped Vehicle Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
Self-Driving Car Engineer Nanodegree Program

[screenshot1]: ./screenshot.png

## Requirements

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* Udacity Simulator [Download link](https://github.com/udacity/self-driving-car-sim/releases)


Tips for setting up the environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d).

## Compilation and Execution

1. Clone this repo
2. cd to repo root dir.
3. `mkdir build`
4. `cd build`
5. `cmake ..`
6. `make`
7. `./particle_filter`
8. Run simulator

## Rubric

![screenshot1][screenshot1]

### Accuracy

#### Does your particle filter localize the vehicle to within the desired accuracy?

Yes, by running the our particle filter with the simulator the output is "Success! Your particle filter passed!". The accuracy is within the limits: [x: 0.114, y:0.108, yaw: 0.004].

### Performance

#### Does your particle run within the specified time of 100 seconds?

Yes, this criteria is automatically checked when running the simulator with our particle filter implementation. The output message says "Success! Your particle filter passed!" with a system time of 48.88 seconds.

### General

#### Does your code use a particle filter to localize the robot?

Yes, our particle filter implements the required initialization (`ParticleFilter::init`), prediction (`ParticleFilter::prediction` with additional Gaussian noise), association (`ParticleFilter::dataAssociation` using a nearest neighbor approach), weight updating (`ParticleFilter::updateWeights` taking into account sensor range, transformation to map coordinates, landmark association, and multivariate weighting), and resampling (`ParticleFilter::resample` with a resampling wheel depending on particle weight).
