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

   if(is_initialized)
     return;

   // resize the particle vector and weights vector
   num_particles = 100;
   particles.resize(num_particles);
   weights.resize(num_particles);
   KVP_DEBUG("init", "resize the particle vector and weights vector");

   // a normal (Gaussian) distribution for x, y, and theta with std (standard deviations)
   std::normal_distribution<double> dist_x(x, std[0]);
   std::normal_distribution<double> dist_y(y, std[1]);
   std::normal_distribution<double> dist_theta(theta, std[2]);
   KVP_DEBUG("init", "normal distribution for x, y, and theta");

   // initialized all the particle with random generate value
   default_random_engine gen;
   for (int i = 0; i < num_particles; ++i) {
      particles[i].id = i;
      particles[i].x = dist_x(gen);
      particles[i].y = dist_y(gen);
      particles[i].theta = dist_theta(gen);
      particles[i].weight = 1.0;
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

   std::normal_distribution<double> x_noise(0, std_pos[0]);
   std::normal_distribution<double> y_noise(0, std_pos[1]);
   std::normal_distribution<double> theta_noise(0, std_pos[2]);
   KVP_DEBUG("prediction", "defined gaussian noise distribution");

   default_random_engine gen;
   gen.seed(321);

   // for each particle, predict new particle location according to the vehicle velocity and yaw_rate
   for(int i = 0; i < num_particles; ++i) {
      // if yaw_rate is closer to zero, the car is going closer to straight line
      if( fabs(yaw_rate) < 0.0001 ) {
          particles[i].x += velocity * delta_t * cos(particles[i].theta);
          particles[i].y += velocity * delta_t * sin(particles[i].theta);
      } else {
          double newTheta = particles[i].theta + yaw_rate * delta_t;
          particles[i].x += velocity / yaw_rate * (sin(newTheta) - sin(particles[i].theta));
          particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(newTheta));
          particles[i].theta += yaw_rate * delta_t;
      }

      // for each particle added ramdomly pick noise from gaussian distribution.
      particles[i].x += x_noise(gen);
      particles[i].y += y_noise(gen);
      particles[i].theta += theta_noise(gen);
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
   double closest = sensor_range * 100;
   int closest_id = 0;

   for(int i = 0; i < observations.size(); i++) {
      for(int j = 0; j < predicted.size(); j++) {
         double obj_dist = dist(observations[i].x, observations[i].y,
                           predicted[j].x, predicted[j].y);
         if(closest > obj_dist) {
            closest = obj_dist;
            closest_id = predicted[j].id;
            KVP_DEBUG("dataAssociation", "closest " + observations[i].id );
         }
       }
       observations[i].id = closest_id;
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
   double weight_normalizer = 0.0;

   for(int i = 0; i < num_particles; i++) {

      KVP_DEBUG("updateWeights", "transform from vehicle coordinate to map coordinate");
      std::vector<LandmarkObs> mapCoordObj;

      for(int j = 0; j < observations.size(); j++) {
         LandmarkObs obj;
         obj.x = particles[i].x + observations[j].x*cos(particles[i].theta) - observations[j].y*sin(particles[i].theta);
         obj.y = particles[i].y + observations[j].x*sin(particles[i].theta) + observations[j].y*cos(particles[i].theta);
         obj.id = observations[j].id;
         mapCoordObj.push_back(obj);
      }

      KVP_DEBUG("updateWeights", "create near landmarks list within sensor range by map coordinate");
      std::vector<LandmarkObs> nearLandmarks;

      for(int j = 0 ; j < map_landmarks.landmark_list.size(); j++) {
        Map::single_landmark_s landmark = map_landmarks.landmark_list[i];
        double nearDist = dist(particles[i].x, particles[i].y, landmark.x_f, landmark.y_f);
        if( nearDist < sensor_range) {
           nearLandmarks.push_back(LandmarkObs {landmark.id_i, landmark.x_f, landmark.x_f});
        }
      }

      KVP_DEBUG("updateWeights", "identify landmark id");
      dataAssociation(nearLandmarks, mapCoordObj, sensor_range);

      KVP_DEBUG("updateWeights", "Calculate the weight of each particle using Multivariate Gaussian distribution");
      for(int j = 0; j < mapCoordObj.size() ; j++) {
         for(int k = 0; k < nearLandmarks.size() ; k++) {
            if(mapCoordObj[j].id == nearLandmarks[k].id) {
               double xt = pow((mapCoordObj[j].x - nearLandmarks[k].x),2) / (2 * pow(std_landmark[0],2));
               double yt = pow((mapCoordObj[j].y - nearLandmarks[k].y),2) / (2 * pow(std_landmark[1],2));
               double w = exp(-1 * (xt + yt))/(2*M_PI*std_landmark[0]*std_landmark[1]);
               particles[i].weight *= w;
            }
         }
      }
      weight_normalizer += particles[i].weight;
   }
   KVP_DEBUG("updateWeights", "Normalize the weights of all particles since resampling using probabilistic approach");
   for(int i = 0; i < particles.size(); i++ ) {
      particles[i].weight /= weight_normalizer;
      weights[i] = particles[i].weight;
   }
}

void ParticleFilter::resample() {
   KVP_DEBUG("resample", "start");
   // TODO: Resample particles with replacement with probability proportional to their weight. 
   // NOTE: You may find std::discrete_distribution helpful here.
   //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   default_random_engine gen;
   discrete_distribution<> dist_particles(weights.begin(), weights.end());

   vector<Particle> resampleP;
   resampleP.resize(num_particles);

   for(int i=0; i<num_particles; i++) {
      int p_id = dist_particles(gen);
      resampleP[i] = particles[p_id];
   }

   particles = resampleP;

   weights.clear();
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
