/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  //

  if (is_initialized)
    return;

  std::cout << "Initializing...\n";

  // Initialize the number of particles.
  num_particles = 100;

  // Fetch standard deviations for x, y, and theta.
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  // Create normal (Gaussian) distributions for x, y, and theta around the estimates.
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);

  // Sample particles.
  for (unsigned int i = 0; i < num_particles; ++i) {

    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;

    particles.push_back(p);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  // Fetch standard deviations.
  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];

  // Create normal (Gaussian) distributions for noise addition.
  normal_distribution<double> dist_x(0, std_x);
  normal_distribution<double> dist_y(0, std_y);
  normal_distribution<double> dist_theta(0, std_theta);

  // Calculate new state for each particle.
  for (unsigned int i = 0; i < num_particles; ++i) {

    double theta = particles[i].theta;

    if (fabs(yaw_rate) < 0.00001) {

      particles[i].x += velocity * delta_t * cos(theta);
      particles[i].y += velocity * delta_t * sin(theta);

    }
    else {
      
      particles[i].x += velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
      particles[i].y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }

    // Add noise.
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

  unsigned int num_predictions = predicted.size();
  unsigned int num_observations = observations.size();

  for (unsigned int i = 0; i < num_observations; ++i) {

    double min_dist = std::numeric_limits<double>::max();
    int landmark_id = -1;

    for (unsigned int j = 0; j < num_predictions; ++j) {

      double current_dist = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

      if (current_dist < min_dist) {

        min_dist = current_dist;
        landmark_id = predicted[j].id;
      }
    }

    observations[i].id = landmark_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
  //
  
  unsigned int num_landmarks = map_landmarks.landmark_list.size();
  double sensor_range_2 = sensor_range * sensor_range;

  // Fetch standard deviations.
  double std_range = std_landmark[0];
  double std_bearing = std_landmark[1];

  for (unsigned int i = 0; i < num_particles; ++i) {

    // Fetch current particle data.
    double particle_x = particles[i].x;
    double particle_y = particles[i].y;
    double particle_theta = particles[i].theta;

    // Get map landmark locations within sensor range.
    std::vector<LandmarkObs> landmarks_in_range;

    for (unsigned int j = 0; j < num_landmarks; ++j) {

      // Fetch current landmark data.
      float landmark_x = map_landmarks.landmark_list[j].x_f;
      float landmark_y = map_landmarks.landmark_list[j].y_f;
      int landmark_id = map_landmarks.landmark_list[j].id_i;

      // Store landmark if in range.
      if (dist(particle_x, particle_y, landmark_x, landmark_y) <= sensor_range_2)
        landmarks_in_range.push_back(LandmarkObs{landmark_id, landmark_x, landmark_y});
    }

    // Transform observation coordinates to map coordinates.
    std::vector<LandmarkObs> transformed_observations;

    for (unsigned int j = 0; j < observations.size(); ++j) {

      double x_t = cos(particle_theta) * observations[j].x - sin(particle_theta) * observations[j].y + particle_x;
      double y_t = sin(particle_theta) * observations[j].x + cos(particle_theta) * observations[j].y + particle_y;
      transformed_observations.push_back(LandmarkObs{observations[j].id, x_t, y_t});
    }

    // Associate transformed observations to landmarks.
    dataAssociation(landmarks_in_range, transformed_observations);

    // Calculate particle weight.
    particles[i].weight = 1.0;

    for (unsigned int j = 0; j < transformed_observations.size(); ++j) {

      // Fetch current observation data.
      double observation_x = transformed_observations[j].x;
      double observation_y = transformed_observations[j].y;
      int landmark_id = transformed_observations[j].id;

      // Find associated landmark and fetch its data.
      double landmark_in_range_x = 0.0;
      double landmark_in_range_y = 0.0;
      unsigned int k = 0;
      unsigned int num_landmarks_in_range = landmarks_in_range.size();
      bool found_landmark_in_range = false;

      while (!found_landmark_in_range && k < num_landmarks_in_range) {

        if (landmarks_in_range[k].id == landmark_id) {
          
          found_landmark_in_range = true;
          landmark_in_range_x = landmarks_in_range[k].x;
          landmark_in_range_y = landmarks_in_range[k].y;
        }

        ++k;
      }

      // Compute weight for the current observation.
      double dist_x = observation_x - landmark_in_range_x;
      double dist_y = observation_y - landmark_in_range_y;

      double observation_weight = (1 / (2 * M_PI * std_range * std_bearing)) *
                                  exp(
                                      -(dist_x * dist_x / (2 * std_range * std_bearing) 
                                      +(dist_y * dist_y / (2 * std_range * std_bearing))));

      if (observation_weight == 0.0)
        observation_weight = 0.00001;

      particles[i].weight *= observation_weight;
    }
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  // Fetch weights and calculate maximum.
  std::vector<double> weights;
  double max_weight = std::numeric_limits<double>::min();

  for (unsigned int i = 0; i < num_particles; ++i) {

    weights.push_back(particles[i].weight);

    if (particles[i].weight > max_weight)
      max_weight = particles[i].weight;
  }

  // Create distributions for sampling.
  uniform_int_distribution<int> dist_id(0, num_particles-1);
  uniform_real_distribution<double> dist_weight(0.0, max_weight);

  // Resampling wheel.
  int idx = dist_id(gen);
  double beta = 0.0;

  std::vector<Particle> resampled_particles;

  for (unsigned int i = 0; i < num_particles; ++i) {

    beta += dist_weight(gen) * 2.0;

    while (beta > weights[idx]) {
      
      beta -= weights[idx];
      idx = (idx + 1) % num_particles;
    }

    resampled_particles.push_back(particles[idx]);
  }

  // Replace old particles with resampled ones.
  particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
