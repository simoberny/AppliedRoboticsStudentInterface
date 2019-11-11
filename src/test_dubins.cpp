#include "include/dubins.hpp"
#include <math.h>
#include <iostream>
#include <tuple>
#include <vector>

using namespace std;

int main(){
    double x0 = 0, y0 = 0, th0 = 0;
    double xf = 1.28, yf = 1.03, thf = M_PI/2;
    double Kmax = 1.0;

    Dubins dub;
    dub.setParams(x0, y0, th0, xf, yf, thf, Kmax);
    pair<int, curve> ret = dub.shortest_path();

    curve cur = ret.second;

    if(ret.first >= 0){
        cout << "Lunghezza totale: " << cur.L << endl;
    }else{
        cout << "Failed" << endl;
    }

    return 0;
}