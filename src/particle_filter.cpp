/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::discrete_distribution;
using std::uniform_real_distribution;
using std::uniform_int_distribution;

static std::default_random_engine gen;
  
void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles  
  
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  for(int i =0U; i<num_particles; i++) {
  	Particle p;
    p.x = dist_x(gen);
    p.y= dist_y(gen);
    p.theta = dist_theta(gen);    
    p.weight=1U;
    particles.push_back(p);
    weights.push_back(p.weight);
  }
  
  
  is_initialized=true;
  
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);
  
  for(int i=0U; i < num_particles; i++) {
    
    if (fabs(yaw_rate) < 0.0000001) {  
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    }else {
      particles[i].x += (velocity/yaw_rate)*(sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
      particles[i].y += (velocity/yaw_rate)*(cos(particles[i].theta) - (cos(particles[i].theta + yaw_rate*delta_t)));
      particles[i].theta += (yaw_rate*delta_t);   
    }    
  
  	particles[i].x += dist_x(gen);
  	particles[i].y += dist_y(gen);
  	particles[i].theta += dist_theta(gen);
    
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  double minimum_dist;
  double distance;
  
  for(unsigned int i=0 ; i< observations.size(); i++) {
  	minimum_dist=std::numeric_limits<double>::max();
    
    for(unsigned int j=0; j< predicted.size(); j++) {
      
      distance = dist(observations[i].x,observations[i].y,predicted[j].x,predicted[j].y);
      
      if(distance < minimum_dist) {
      	minimum_dist = distance;
        observations[i].id = predicted[j].id;
      }
    }
    
  }
  
  
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  double weights_sum=0;
  LandmarkObs transObs;
  double o_x, o_y, r_x, r_y,std_x,std_y,obs_w;
      
  for(int i=0U; i < num_particles; i++) {
    vector<LandmarkObs> withinRange;
  
    for(unsigned int j=0; j < map_landmarks.landmark_list.size() ; j++) {
     
    	double x_delta = map_landmarks.landmark_list[j].x_f - particles[i].x;
    	double y_delta = map_landmarks.landmark_list[j].y_f - particles[i].y;
    	double landmark_dist_square = x_delta * x_delta + y_delta * y_delta;
    	double sensor_range_square = sensor_range * sensor_range;
    	/* Only consider landmarks which are within the sensor range */
    	if (landmark_dist_square <= sensor_range_square) {
          withinRange.push_back(LandmarkObs{ map_landmarks.landmark_list[j].id_i, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f });
    	}
    }
  	
    
    
    vector<LandmarkObs> trans_o;
    
    for (unsigned int j = 0; j < observations.size(); j++) {
      transObs.id = observations[j].id;
      transObs.x = cos(particles[i].theta)*observations[j].x - sin(particles[i].theta)*observations[j].y + particles[i].x;
      transObs.y = sin(particles[i].theta)*observations[j].x + cos(particles[i].theta)*observations[j].y + particles[i].y;
      trans_o.push_back(transObs);
    }
    
    dataAssociation(withinRange, trans_o);

	particles[i].weight = 1U;
    weights[i] = 1U;

    for (unsigned int j = 0; j < trans_o.size(); j++) {
      
      // placeholders for observation and associated prediction coordinates
      o_x = trans_o[j].x;
      o_y = trans_o[j].y;

      int ass_id = trans_o[j].id;

      // get the x,y coordinates of the prediction associated with the current observation
      for (unsigned int k = 0; k < withinRange.size(); k++) {
        if (withinRange[k].id == ass_id) {
          r_x = withinRange[k].x;
          r_y = withinRange[k].y;
          break;
        }
      }

      // calculate weight for this observation with multivariate Gaussian
      std_x = std_landmark[0];
      std_y = std_landmark[1];
      obs_w = ( 1/(2*M_PI*std_x*std_y)) * exp( -( pow(o_x-r_x,2)/(2*pow(std_x, 2)) + (pow(o_y-r_y,2)/(2*pow(std_y, 2))) ) );

      // product of this observation weight with total observations weight
      particles[i].weight *= obs_w;    
    }
    
    weights[i] = particles[i].weight;
    weights_sum += weights[i];
  }

  if (fabs(weights_sum) > 0.0)
   {
       for (unsigned k = 0; k < weights.size(); k++)
       {
           weights[k] = weights[k] / weights_sum;
       }
   }
  
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  
  vector<Particle> resample_particles;

  // get all of the current weights
  vector<double> weights;
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
  }

  uniform_int_distribution<int> uniintdist(0, num_particles-1);
  auto index = uniintdist(gen);

  double max_weight = *max_element(weights.begin(), weights.end());

  uniform_real_distribution<double> unirealdist(0.0, max_weight);

  double beta = 0.0;

  // spin the resample wheel!
  for (int i = 0; i < num_particles; i++) {
    beta += unirealdist(gen) * 2.0;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    resample_particles.push_back(particles[index]);
  }

  particles = resample_particles;  
  
  
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}