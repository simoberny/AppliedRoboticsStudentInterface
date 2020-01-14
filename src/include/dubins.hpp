#pragma once

#include "utils.hpp"

#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>

using namespace std;

struct arc {
    double x0;
    double y0;
    double th0;
    double xf;
    double yf;
    double thf;
    double k;
    double L;
};

struct curve {
  arc a1;
  arc a2;
  arc a3;
  double L;
};

class Dubins{
    private: 
        double x0, y0, th0;
        double xf, yf, thf;
        double kmax;
    public: 
        Dubins();
        void setParams(double x0, double y0, double th0, double xf, double yf, double thf, double kmax);
        pair<int, curve> shortest_path();
        Path getPath(curve c);
};



