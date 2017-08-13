//
// Created by nick on 17-8-11.
//

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double *std) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/
    default_random_engine gen;
    normal_distribution<double> N_x_init(0,std[0]);
    normal_distribution<double> N_y_init(0,std[1]);
    normal_distribution<double> N_theta_init(0,std[2]);

    num_particles = 200;

    is_initialized = true;

    weights = vector<double>(num_particles);
    particles = vector<Particle>(num_particles);

    for (int i = 0; i < num_particles; ++i) {
        weights.at(i) = 1;
        particles.at(i).weight = 1;
        particles.at(i).theta = theta + N_theta_init(gen);
        particles.at(i).x = x + N_x_init(gen);
        particles.at(i).y = y + N_y_init(gen);
        particles.at(i).id = i;
    }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/
    default_random_engine gen;
    normal_distribution<double> N_x_init(0,std_pos[0]);
    normal_distribution<double> N_y_init(0,std_pos[1]);
    normal_distribution<double> N_theta_init(0,std_pos[2]);

    double x0 = 0;
    double y0 = 0;
    double theta0 = 0;

    double x_new = 0;
    double y_new = 0;
    double theta_new = 0;

    for (int i = 0; i < num_particles; ++i) {
        x0 = particles.at(i).x;
        y0 = particles.at(i).y;
        theta0 = particles.at(i).theta;

        if (fabs(yaw_rate) < 1e-5){
            x_new = x0 + velocity * delta_t * cos(theta0);
            y_new = y0 + velocity * delta_t * sin(theta0);
            theta_new = theta0;
        } else {
            x_new = x0 + (velocity/yaw_rate) * (sin(theta0 + yaw_rate * delta_t) - sin(theta0));
            y_new = y0 + (velocity/yaw_rate) * (cos(theta0) - cos(theta0 + yaw_rate * delta_t));
            theta_new = theta0 + yaw_rate * delta_t;
        }

        particles.at(i).x = x_new + N_x_init(gen);
        particles.at(i).y = y_new + N_y_init(gen);
        particles.at(i).theta = theta_new + N_theta_init(gen);
    }

}


Particle mapObservationToMapCoordinates(LandmarkObs observation, Particle particle) {
    double x = observation.x;
    double y = observation.y;

    double xt = particle.x;
    double yt = particle.y;
    double theta = particle.theta;

    Particle mapCoordinates;

    mapCoordinates.x = x * cos(theta) - y * sin(theta) + xt;
    mapCoordinates.y = x * sin(theta) + y * cos(theta) + yt;

    return mapCoordinates;

}

/**
 * Find the distance between a map landmark and a transformed particel
 * @param land_mark A map landmark
 * @param map_coordinates Particle coordinates in globel coordinate system
 * @return Distance between landmark and particle
 */
double calculateDistance(Map::single_landmark_s land_mark, Particle map_coordinates) {
    double x1 = land_mark.x_f;
    double y1 = land_mark.y_f;

    double x2 = map_coordinates.x;
    double y2 = map_coordinates.y;

    return sqrt(pow((x1 - x2),2) + pow((y1 - y2),2));
}

/**
 * Find the closest map landmark to a given transformd particle observation
 * @param map_coordinates Particle coordinates in global coordinate system
 * @param map_landmarks Map landmarks
 * @return The closet landmark to the particle
 */
Map::single_landmark_s findClosestLandmark(Particle map_coordinates, Map map_landmarks) {
    Map::single_landmark_s closest_landmark = map_landmarks.landmark_list.at(0);
    double distance = calculateDistance(map_landmarks.landmark_list.at(0),map_coordinates);

    for (int i = 1; i <map_landmarks.landmark_list.size() ; ++i) {
        Map::single_landmark_s current_landmark = map_landmarks.landmark_list.at(i);
        double current_distance = calculateDistance(current_landmark,map_coordinates);

        if (current_distance < distance) {
            distance = current_distance;
            closest_landmark = current_landmark;
        }
    }

    return closest_landmark;
}


double findObservationProbability(Particle map_coordinates, Map::single_landmark_s closest_landmark, double std_landmark[]) {
    double mew_x = closest_landmark.x_f;
    double mew_y = closest_landmark.y_f;

    double x = map_coordinates.x;
    double y = map_coordinates.y;

    double sigma_x = std_landmark[0];
    double sigma_y = std_landmark[1];

    return (1/ (2 * M_PI * sigma_x * sigma_y))
        * pow(M_E, -(pow(x - mew_x,2)/(2 * pow(sigma_x, 2)) + pow(y - mew_y,2)/(2 * pow(sigma_y,2))));
}


void ParticleFilter::updateWeights(double sensor_range, double *std_landmark, std::vector<LandmarkObs> observations,
                                   Map map_landmarks) {
    // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation
    //   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account
    //   for the fact that the map's y-axis actually points downwards.)
    //   http://planning.cs.uiuc.edu/node99.html

    for (int i = 0; i < particles.size() ; ++i) {
        Particle particle = particles.at(i);
        double new_weight = 1;
        for (LandmarkObs observation:observations) {
            // Convert the observations as seen by the particle into map coordinates
            Particle map_coordinates = mapObservationToMapCoordinates(observation, particle);
            Map::single_landmark_s closest_landmark = findClosestLandmark(map_coordinates,map_landmarks);
            double observation_probability = findObservationProbability(map_coordinates,closest_landmark,std_landmark);
            new_weight *= observation_probability;
        }
        particle.weight = new_weight;
        weights.at(i) = new_weight;
    }

}


void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    random_device rd;
    mt19937 gen(rd());
    discrete_distribution<> distribution(weights.begin(), weights.end());

    vector<Particle> new_particles = vector<Particle>(num_particles);
    for (int i = 0; i < particles.size(); ++i) {
        new_particles.at(i) = particles.at(distribution(gen));
        weights.at(i) = particles.at(distribution(gen)).weight;
    }
    particles = new_particles;

}


void ParticleFilter::write(std::string filename) {
    ofstream datafile;
    datafile.open(filename,ios::app);
    for (int i = 0; i < num_particles ; ++i) {
        datafile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
    }
    datafile.close();
}
























