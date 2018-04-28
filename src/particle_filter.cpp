/*
 * particle_filter.cpp
 *
 *  Created on: Apr 28, 2018
 *      Author: Mohamed ZIEDAN
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

#define PARTICLES_NUMBER 12 // Can be decreased (even 12 particles can pass the test)
#define EPS 0.001

using namespace std;

static default_random_engine _generator;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.

	num_particles = PARTICLES_NUMBER;

	// Create normal distributions for x, y and theta.
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	particles.resize(num_particles); // Resize the `particles` vector to fit desired number of particles
	weights.resize(num_particles);	 // Resize the `weights` vector to fit desired number of particles

	double init_weight = 1.0 / num_particles;

	for (int x = 0; x < num_particles; x++)
	{
		particles[x].id = x;
		particles[x].x = dist_x(_generator);
		particles[x].y = dist_y(_generator);
		particles[x].theta = dist_theta(_generator);
		particles[x].weight = init_weight;
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
	// TODO: Add measurements to each particle and add random Gaussian noise.

	normal_distribution<double> dist_x(0.0, std_pos[0]);
	normal_distribution<double> dist_y(0.0, std_pos[1]);
	normal_distribution<double> dist_theta(0.0, std_pos[2]);

	const double velocity_dt = velocity * delta_t;
	const double yaw_dt = yaw_rate * delta_t;
	const double vel_yaw = velocity / yaw_rate;

	for (int x = 0; x < num_particles; x++)
	{
		if (fabs(yaw_rate) < EPS)
		{
			particles[x].x += velocity_dt * cos(particles[x].theta);
			particles[x].y += velocity_dt * sin(particles[x].theta);
		}
		else
		{
			const double theta_new = particles[x].theta + yaw_dt;
			particles[x].x += vel_yaw * (sin(theta_new) - sin(particles[x].theta));
			particles[x].y += vel_yaw * (cos(particles[x].theta) - cos(theta_new));
			particles[x].theta = theta_new;
		}
		// Adding random Gaussian noise
		particles[x].x += dist_x(_generator);
		particles[x].y += dist_y(_generator);
		particles[x].theta += dist_theta(_generator);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs> &observations)
{
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.

	for (unsigned int x = 0; x < observations.size(); x++)
	{
		// Grab current observation
		LandmarkObs current_opservation = observations[x];

		// Initialize the  minimum distance to maximum possiblity
		double min_dist = numeric_limits<double>::max();

		int map_id = -1;

		for (unsigned int j = 0; j < predicted.size(); j++)
		{
			LandmarkObs current_prediction = predicted[j];
			double current_dist = dist(current_opservation.x, current_opservation.y, current_prediction.x, current_prediction.y);

			// Find the predicted landmark nearest the current observed landmark
			if (current_dist < min_dist)
			{
				min_dist = current_dist;
				map_id = current_prediction.id;
			}
		}
		// Set the observation's ID to the nearest predicted landmark's ID
		observations[x].id = map_id;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
																	 const std::vector<LandmarkObs> &observations, const Map &map_landmarks)
{
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution.
	for (int x = 0; x < num_particles; x++)
	{
		double p_x = particles[x].x;
		double p_y = particles[x].y;
		double p_theta = particles[x].theta;

		vector<LandmarkObs> predictions;

		for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++)
		{
			// Get ID and x,y coordinates
			float landMark_x = map_landmarks.landmark_list[j].x_f;
			float landMark_y = map_landmarks.landmark_list[j].y_f;
			int landMark_id = map_landmarks.landmark_list[j].id_i;

			if (fabs(landMark_x - p_x) <= sensor_range && fabs(landMark_y - p_y) <= sensor_range)
			{
				predictions.push_back(LandmarkObs{landMark_id, landMark_x, landMark_y});
			}
		}

		vector<LandmarkObs> transformed_os;
		for (unsigned int j = 0; j < observations.size(); j++)
		{
			double t_x = cos(p_theta) * observations[j].x - sin(p_theta) * observations[j].y + p_x;
			double t_y = sin(p_theta) * observations[j].x + cos(p_theta) * observations[j].y + p_y;
			transformed_os.push_back(LandmarkObs{observations[j].id, t_x, t_y});
		}

		dataAssociation(predictions, transformed_os);

		particles[x].weight = 1.0;
		for (unsigned int j = 0; j < transformed_os.size(); j++)
		{
			double o_x, o_y, pr_x, pr_y;
			o_x = transformed_os[j].x;
			o_y = transformed_os[j].y;
			int associated_prediction = transformed_os[j].id;

			for (unsigned int k = 0; k < predictions.size(); k++)
			{
				if (predictions[k].id == associated_prediction)
				{
					pr_x = predictions[k].x;
					pr_y = predictions[k].y;
				}
			}

			double std_x = std_landmark[0];
			double std_y = std_landmark[1];
			double observationWeight = (1 / (2 * M_PI * std_x * std_y)) * exp(-(pow(pr_x - o_x, 2) / (2 * pow(std_x, 2)) + (pow(pr_y - o_y, 2) / (2 * pow(std_y, 2)))));

			particles[x].weight *= observationWeight;
		}
	}
}

void ParticleFilter::resample()
{
	// TODO: Resample particles with replacement with probability proportional to their weight.
	vector<Particle> new_particles;

	vector<double> weights;
	for (int x = 0; x < num_particles; x++)
	{
		weights.push_back(particles[x].weight);
	}

	uniform_int_distribution<int> uniintdist(0, num_particles - 1);
	auto index = uniintdist(_generator);

	// Get max weight
	double max_weight = *max_element(weights.begin(), weights.end());

	// Uniform random distribution [0.0, max_weight)
	uniform_real_distribution<double> unirealdist(0.0, max_weight);

	double beta = 0.0;

	// Spin the resample wheel
	for (int x = 0; x < num_particles; x++)
	{
		beta += unirealdist(_generator) * 2.0;
		while (beta > weights[index])
		{
			beta -= weights[index];
			index = (index + 1) % num_particles;
		}
		new_particles.push_back(particles[index]);
	}

	particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle &particle, const std::vector<int> &associations,
																				 const std::vector<double> &sense_x, const std::vector<double> &sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	particle.associations = associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y;
	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1); // get rid of the trailing space
	return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1); // get rid of the trailing space
	return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1); // get rid of the trailing space
	return s;
}
