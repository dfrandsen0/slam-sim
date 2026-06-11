#include <random>

#include "utilities.h"
#include "..\config.h"

namespace {
     std::mt19937& GetGen() {
        static thread_local std::random_device rd;
        static thread_local std::mt19937 gen(rd());
        return gen;
    }
}

int Utilities::GetRandomInt(int min, int max) {
    static thread_local std::uniform_int_distribution<int> distr(min, max);
    return distr(GetGen());
}

double Utilities::GetRandomNoise(double mean, double sigma) {
    static thread_local std::normal_distribution<double> randomAccuracy(0.0, 1.0);

    return (randomAccuracy(GetGen()) * mean * sigma);
}

double Utilities::GetFixedNoise(double sigma) {
    static thread_local std::normal_distribution<double> accuracy(0.0, 1.0);
    
    return accuracy(GetGen()) * sigma;
}

// Excludes max
double Utilities::GetUniformEpsilon(double max) {
    static thread_local std::uniform_real_distribution<double> epsilonDistro(0.0, max);

    return epsilonDistro(GetGen());
}