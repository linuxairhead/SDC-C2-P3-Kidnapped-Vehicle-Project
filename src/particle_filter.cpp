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

   KVP_DEBUG("init", "start");
   // Set the number of particles. Initialize all particles to first position (based on estimates of
   //   x, y, theta and their uncertainties from GPS) and all weights to 1. 
   // Add random Gaussian noise to each particle.
   // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

   // resize the particle vector and weights vector
   num_particles = 20;
   default_random_engine gen;

   KVP_DEBUG("init", "resize the particle vector and weights vector");

   // a normal (Gaussian) distribution for x, y, and theta with std (standard deviations)
   std::normal_distribution<double> dist_x(x, std[0]);
   std::normal_distribution<double> dist_y(y, std[1]);
   std::normal_distribution<double> dist_theta(theta, std[2]);
   KVP_DEBUG("init", "normal distribution for x, y, and theta");

   // initialized all the particle with random generate value

   for (int i = 0; i < num_particles; ++i) {
      Particle particle;
      particle.id = i;
      particle.x = dist_x(gen);
      particle.y = dist_y(gen);
      particle.theta = dist_theta(gen);
      particle.weight = 1.0;

      particles.push_back(particle);
      weights.push_back(particle.weight);
   }

   KVP_DEBUG("init", "initialized all the particle with random gen");
   is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
   KVP_DEBUG("prediction", "start");
   // TODO: Add measurements to each particle and add random Gaussian noise.
   // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
   //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   //  http://www.cplusplus.com/reference/random/default_random_engine/
   default_random_engine gen;

   double pred_x, pred_y, pred_theta, tempTheta;

   // for each particle, predict new particle location according to the vehicle velocity and yaw_rate
   for(int i = 0; i < num_particles; ++i) {
      // if yaw_rate is closer to zero, the car is going closer to straight line
      if( fabs(yaw_rate) < 0.0001 ) {
          pred_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
          pred_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
          pred_theta = particles[i].theta;
      } else {
          tempTheta = particles[i].theta + yaw_rate * delta_t;
          pred_x = particles[i].x + velocity / yaw_rate * (sin(tempTheta) - sin(particles[i].theta));
          pred_y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(tempTheta));
          pred_theta = particles[i].theta + yaw_rate * delta_t;
      }

      normal_distribution<double> x_noise(pred_x, std_pos[0]);
      normal_distribution<double> y_noise(pred_y, std_pos[1]);
      normal_distribution<double> theta_noise(pred_theta, std_pos[2]);
      KVP_DEBUG("prediction", "defined gaussian noise distribution");

      // for each particle added ramdomly pick noise from gaussian distribution.
      particles[i].x = x_noise(gen);
      particles[i].y = y_noise(gen);
      particles[i].theta = theta_noise(gen);
   }

   KVP_DEBUG("prediction", "calculated new particle position");
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations,
                                     double sensor_range ) {
   KVP_DEBUG("dataAssociation", "start");
   // TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
   //   observed measurement to this particular landmark.
   // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
   //   implement this method and use it as a helper during the updateWeights phase.
   KVP_DEBUG("dataAssociation", "init, set max sensor range");

   for(int i = 0; i < observations.size(); i++) {
      double closest_obj = sensor_range * sqrt(2);
      int closest_id = -1;

      for(int j = 0; j < predicted.size(); j++) {
         double obj_dist = dist(observations[i].x, observations[i].y,
                           predicted[j].x, predicted[j].y);
         if(closest_obj > obj_dist) {
            closest_obj = obj_dist;
            closest_id = predicted[j].id;
            KVP_DEBUG("dataAssociation", "closest " + observations[i].id );
         }
       }
       observations[i].id = closest_id;
       KVP_DEBUG("dataAssociation", " " + to_string(closest_id));
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
   KVP_DEBUG("updateWeights", "start");
   // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
   //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
   //   according to the MAP'S coordinate system. You will need to transform between the two systems.
   //   Keep in mind that this itransformation requires both rotation AND translation (but no scaling).
   //   The following is a good resource for the theory:
   //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   //   and the following is a good resource for the actual equation to implement (look at equation 
   //   3.33
   //   http://planning.cs.uiuc.edu/node99.html

  /*This variable is used for normalizing weights of all particles and bring them in the range
    of [0, 1]*/
  double weight_normalizer = 0.0;

  for (int i = 0; i < num_particles; i++) {
    double particle_x = particles[i].x;
    double particle_y = particles[i].y;
    double particle_theta = particles[i].theta;

    KVP_DEBUG("updateWeights", "step1 transform from vehicle coordinate to map coordinate");
    vector<LandmarkObs> mapCoordinate;
    //Transform observations from vehicle's co-ordinates to map co-ordinates.
    for (int j = 0; j < observations.size(); j++) {
      LandmarkObs obj;
      obj.id = j;
      obj.x = particle_x + (cos(particle_theta) * observations[j].x) - (sin(particle_theta) * observations[j].y);
      obj.y = particle_y + (sin(particle_theta) * observations[j].x) + (cos(particle_theta) * observations[j].y);
      mapCoordinate.push_back(obj);
    }

    /*Step 2: Filter map landmarks to keep only those which are in the sensor_range of current
    particle. Push them to predictions vector.*/
    KVP_DEBUG("updateWeights", "create near landmarks list within sensor range by map coordinate");
    vector<LandmarkObs> predLandmarks;
    for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      Map::single_landmark_s landmark = map_landmarks.landmark_list[j];
      if ((fabs((particle_x - landmark.x_f)) <= sensor_range) && (fabs((particle_y - landmark.y_f)) <= sensor_range)) {
        predLandmarks.push_back(LandmarkObs {landmark.id_i, landmark.x_f, landmark.y_f});
      }
    }

    /*Step 3: Associate observations to lpredicted andmarks using nearest neighbor algorithm.*/
    KVP_DEBUG("updateWeights", "identify landmark id");
    //Associate observations with predicted landmarks
    dataAssociation(predLandmarks, mapCoordinate, sensor_range);

    /*Step 4: Calculate the weight of each particle using Multivariate Gaussian distribution.*/
    KVP_DEBUG("updateWeights", "Calculate the weight of each particle using Multivariate Gaussian distribution");
    //Reset the weight of particle to 1.0
    particles[i].weight = 1.0;
    double normalizer = (1.0/(2.0 * M_PI * std_landmark[0] * std_landmark[1]));

    /*Calculate the weight of particle based on the multivariate Gaussian probability function*/
    for (int j = 0; j < mapCoordinate.size(); j++) {
      for (int k = 0; k < predLandmarks.size(); k++) {
        if (mapCoordinate[j].id == predLandmarks[k].id) {
          double xt = pow((mapCoordinate[j].x-predLandmarks[k].x),2) / (2.0 * pow(std_landmark[0],2));
          double yt = pow((mapCoordinate[j].y-predLandmarks[k].y),2) / (2.0 * pow(std_landmark[1],2));

          particles[i].weight *= normalizer * exp(-1.0 * (xt + yt));
        }
      }
    }
    weight_normalizer += particles[i].weight;
  }

  /*Step 5: Normalize the weights of all particles since resmapling using probabilistic approach.*/
  KVP_DEBUG("updateWeights", "Normalize the weights of all particles since resampling using probabilistic approach");
  for (int i = 0; i < particles.size(); i++) {
    particles[i].weight /= weight_normalizer;
    weights[i] = particles[i].weight;
  }
}

void ParticleFilter::resample() {
   KVP_DEBUG("resample", "start");
   // TODO: Resample particles with replacement with probability proportional to their weight. 
   // NOTE: You may find std::discrete_distribution helpful here.
   //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   vector<Particle> resampleP;

   default_random_engine gen;
   uniform_int_distribution<int> random_p(0, num_particles - 1);
   int index = random_p(gen);

   // get the largest weight
   double maxW = 2.0 * *max_element(weights.begin(), weights.end());
   double sumW = 0.0;

   KVP_DEBUG("resample", "maxW selected ");
   for(int i=0; i<particles.size(); i++) {
      uniform_real_distribution<double> random_w(0.0, maxW);
      sumW += random_w(gen);

      while( sumW > weights[index] ) {
         sumW -= weights[index];
         index = (index + 1) % num_particles;
      }
      resampleP.push_back(particles[index]);
   }
   particles = resampleP;
   KVP_DEBUG("resample", "resampled");
}


Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    KVP_DEBUG("SetAssociations", "start");
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
    KVP_DEBUG("getAssociation", "start");
    vector<int> v = best.associations;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
    KVP_DEBUG("getSenseX", "start");
    vector<double> v = best.sense_x;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
    KVP_DEBUG("getSenseY", "start");
    vector<double> v = best.sense_y;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
