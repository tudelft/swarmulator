#include "randomgenerator.h"
#include <iostream>
#include "stdint.h"
#include "auxiliary.h"

random_generator::random_generator()
{
  random_device rd;
  generator.seed(rd());
};

float random_generator::uniform_float(const float &min, const float &max)
{
  uniform_real_distribution<> dist(min, max);
  return dist(generator);
};

int random_generator::uniform_int(const int &min, const int &max)
{
  uniform_int_distribution<> dist(min, max);
  return dist(generator);
};

float random_generator::gaussian_float(const float &mean, const float &stddev)
{
  normal_distribution<> dist(mean, stddev);
  return dist(generator);
};

bool random_generator::bernoulli(const float &p)
{
  bernoulli_distribution dist(p);
  return dist(generator);
}

vector<float> random_generator::gaussian_float_vector(const int &length, const float &mean, const float &std)
{
  // Generate the random vector
  vector<float> v(length, 0);
  for (uint8_t i = 0; i < length; i++) {
    v[i] = gaussian_float(mean, std);
  }

  return v;
}

vector<float> random_generator::uniform_float_vector(const int &length, const float &min, const float &max)
{
  // Generate the random vector
  vector<float> v(length, 0);
  for (uint8_t i = 0; i < length; i++) {
    v[i] = uniform_float(min, max);
  }

  return v;
}
