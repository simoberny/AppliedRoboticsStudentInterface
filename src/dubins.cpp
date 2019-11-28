#include "include/dubins.hpp"

Dubins::Dubins() {}

void Dubins::setParams (double x0, double y0, double th0, double xf, double yf, double thf, double kmax) { 
    this->x0 = x0;
    this->y0 = y0;
    this->th0 = th0;
    this->xf = xf; 
    this->yf = yf;
    this->thf = thf;
    this->kmax = kmax;
}

double sinc(double t){
    double s;

    if(abs(t) < 0.002){
        s = 1 - pow(t, 2) * 1/6 * (1 - pow(t, 2)/120);
    }else{
        s = sin(t)/t;
    }

    return s;
}

double mod2pi(double ang){
    double out = ang;

    while(out < 0){
        out = out + 2 * M_PI;
    }

    while(out >= 2 * M_PI){
        out = out - 2 * M_PI;
    }

    return out;
}

tuple<double, double, double> circline(double s, double x0, double y0, double th0, double k){
    double x = x0 + s * sinc(k * s / 2.0) * cos(th0 + k * s /2);
    double y = y0 + s * sinc(k * s / 2.0) * sin(th0 + k * s /2);
    double th = mod2pi(th0 + k * s);

    return make_tuple(x, y, th);
}

arc dubinsarc(double x0, double y0, double th0, double k, double L){
    arc a;
    a.x0 = x0;
    a.y0 = y0;
    a.th0 = th0;
    a.k = k;
    a.L = L;
    
    tie(a.xf, a.yf, a.thf) = circline(L, x0, y0, th0, k);

    return a;
}

curve dubinscurve (double x0, double y0, double th0, 
                    double s1, double s2, double s3, double k0, double k1, double k2){
    curve d;
    d.a1 = dubinsarc(x0, y0, th0, k0, s1);
    d.a2 = dubinsarc(d.a1.xf, d.a1.yf, d.a1.thf, k1, s2);
    d.a3 = dubinsarc(d.a2.xf, d.a2.yf, d.a2.thf, k2, s3);
    d.L = d.a1.L + d.a2.L + d.a3.L;

    return d;
}

// LSL
tuple<bool, double, double, double> LSL(double sc_th0, double sc_thf, double sc_Kmax){
    double invK = 1 / sc_Kmax;

    // differenza coseni
    double C = cos(sc_thf) - cos(sc_th0);

    // differenza seni
    double S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);

    // metà dell'angolo che serve per portare da iniziale a finale
    double temp1 = atan2(C, S);
    // percorro la prima parte di ancgolo necessario
    double sc_s1 = invK * mod2pi(temp1 - sc_th0);

    double temp2 = 2 + 4 * pow(sc_Kmax, 2) - 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
    if(temp2 < 0){
        return make_tuple(false, 0, 0, 0);
    }

    double sc_s2 = invK * sqrt(temp2);
    double sc_s3 = invK * mod2pi(sc_thf - temp1);

    return make_tuple(true, sc_s1, sc_s2, sc_s3);
}

// RSR
tuple<bool, double, double, double> RSR(double sc_th0, double sc_thf, double sc_Kmax){
    double invK = 1 / sc_Kmax;

    // differenza coseni
    double C = cos(sc_th0) - cos(sc_thf);

    // differenza seni
    double S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);

    // angolo finale completo
    double temp1 = atan2(C, S);
    double sc_s1 = invK * mod2pi(sc_th0 - temp1);

    double temp2 = 2 + 4 * pow(sc_Kmax, 2) - 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
    if(temp2 < 0){
        return make_tuple(false, 0, 0, 0);
    }

    double sc_s2 = invK * sqrt(temp2);
    double sc_s3 = invK * mod2pi(temp1 - sc_thf);

    return make_tuple(true, sc_s1, sc_s2, sc_s3);
}

// LSR
tuple<bool, double, double, double> LSR(double sc_th0, double sc_thf, double sc_Kmax){
    double invK = 1 / sc_Kmax;

    // differenza coseni
    double C = cos(sc_th0) + cos(sc_thf);

    // differenza seni
    double S = 2 * sc_Kmax + sin(sc_th0) + sin(sc_thf);

    // metà dell'angolo che serve per portare da iniziale a finale
    double temp1 = atan2(-C, S);
    double temp3 = 4 * pow(sc_Kmax, 2) - 2 + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
    
    if(temp3 < 0){
        return make_tuple(false, 0, 0, 0);
    }

    double sc_s2 = invK * sqrt(temp3);
    double temp2 = -atan2(-2, sc_s2 * sc_Kmax);
    double sc_s1 = invK * mod2pi(temp1 + temp2 - sc_th0);
    double sc_s3 = invK * mod2pi(temp1 + temp2 - sc_thf);

    return make_tuple(true, sc_s1, sc_s2, sc_s3);
}

// RSL
tuple<bool, double, double, double> RSL(double sc_th0, double sc_thf, double sc_Kmax){
    double invK = 1 / sc_Kmax;

    // differenza coseni
    double C = cos(sc_th0) + cos(sc_thf);

    // differenza seni
    double S = 2 * sc_Kmax - sin(sc_th0) - sin(sc_thf);

    // metà dell'angolo che serve per portare da iniziale a finale
    double temp1 = atan2(C, S);
    double temp3 = 4 * pow(sc_Kmax, 2) - 2 + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
    
    if(temp3 < 0){
        return make_tuple(false, 0, 0, 0);
    }

    double sc_s2 = invK * sqrt(temp3);
    double temp2 = atan2(2, sc_s2 * sc_Kmax);
    double sc_s1 = invK * mod2pi(sc_th0 - temp1 + temp2);
    double sc_s3 = invK * mod2pi(sc_thf - temp1 + temp2);

    return make_tuple(true, sc_s1, sc_s2, sc_s3);
}

// RLR
tuple<bool, double, double, double> RLR(double sc_th0, double sc_thf, double sc_Kmax){
    double invK = 1 / sc_Kmax;

    // differenza coseni
    double C = cos(sc_th0) - cos(sc_thf);

    // differenza seni
    double S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);

    // angolo finale completo
    double temp1 = atan2(C, S);
    double temp2 = 0.125 * (6 - 4 * pow(sc_Kmax, 2) + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));

    if(abs(temp2) > 1){
        return make_tuple(false, 0, 0, 0);
    }

    double sc_s2 = invK * mod2pi(2 * M_PI - acos(temp2));
    double sc_s1 = invK * mod2pi(sc_th0 - temp1 + 0.5 * sc_s2 * sc_Kmax);
    double sc_s3 = invK * mod2pi(sc_th0 - sc_thf + sc_Kmax * (sc_s2 - sc_s1));

    return make_tuple(true, sc_s1, sc_s2, sc_s3);
}

//LRL
tuple<bool, double, double, double> LRL(double sc_th0, double sc_thf, double sc_Kmax){
    double invK = 1 / sc_Kmax;

    // differenza coseni
    double C = cos(sc_thf) - cos(sc_th0);

    // differenza seni
    double S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);

    // angolo finale completo
    double temp1 = atan2(C, S);
    double temp2 = 0.125 * (6 - 4 * pow(sc_Kmax, 2) + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));

    if(abs(temp2) > 1){
        return make_tuple(false, 0, 0, 0);
    }

    double sc_s2 = invK * mod2pi(2 * M_PI - acos(temp2));
    double sc_s1 = invK * mod2pi(temp1 - sc_th0 + 0.5 * sc_s2 * sc_Kmax);
    double sc_s3 = invK * mod2pi(sc_thf - sc_th0 + sc_Kmax * (sc_s2 - sc_s1));

    return make_tuple(true, sc_s1, sc_s2, sc_s3);
}

tuple<double, double, double> scaleFromStandard(double lambda, double sc_s1, double sc_s2, double sc_s3){
    double s1 = sc_s1 * lambda;
    double s2 = sc_s2 * lambda;
    double s3 = sc_s3 * lambda;

    return make_tuple(s1,s2,s3);
}

// Trasforma il problema da 7  a 3 parametri
tuple<double, double, double, double> scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax){
    double dx = xf - x0;
    double dy = yf - y0;
    double phi = atan2(dy, dx);
    double lambda = hypot(dx, dy)/2;

    double sc_th0 = mod2pi(th0 - phi); 
    double sc_thf = mod2pi(thf - phi); 
    double sc_Kmax = Kmax * lambda;

    return make_tuple(sc_th0, sc_thf, sc_Kmax, lambda);
}

pair<int, curve> Dubins::shortest_path () {
    double sc_s1, sc_s2, sc_s3;
    
    // Calcolo valori scalati
    double sc_th0, sc_thf, sc_Kmax, lambda;
    tie(sc_th0, sc_thf, sc_Kmax, lambda) = scaleToStandard(this->x0, this->y0, this->th0, this->xf, this->yf, this->thf, this->kmax);

    vector<tuple<bool, double, double, double> (*)(double, double, double)> primitives = 
        {&LSL, &RSR, &LSR, &RSL, &RLR, &LRL};
        
    vector<string> f_name = {"LSL", "RSR", "LSR", "RSL", "RLR", "LRL"};

    double ksigns[6][6] = {
        { 1, 0, 1 }, //LSL
        {-1, 0,-1 }, //RSR
        { 1, 0,-1 }, //LSR
        {-1, 0, 1 }, //RSL
        {-1, 1,-1 }, //RLR
        { 1,-1, 1 } //LRL
    };

    double L = INFINITY;
    int pidx = -1;
    for(int i = 0; i < primitives.size(); i++){
        bool ok;
        double sc_s1_c, sc_s2_c, sc_s3_c;

        tie(ok, sc_s1_c, sc_s2_c, sc_s3_c) = primitives[i](sc_th0, sc_thf, sc_Kmax);
        double Lcur = sc_s1_c + sc_s2_c + sc_s3_c;

        //cout << endl << f_name[i] << endl;

        double s1, s2, s3;
        tie(s1, s2, s3) = scaleFromStandard(lambda, sc_s1_c, sc_s2_c, sc_s3_c);

        curve tempcur = dubinscurve(x0, y0, th0, s1, s2, s3, ksigns[i][0]*kmax, ksigns[i][1]*kmax, ksigns[i][2]*kmax);       
        //cout << "L: " << tempcur.L << " \n s1: " << s1 << " \n s2: " << s2 << " \n s3: " << s3 << endl;


        if(ok && Lcur < L){
            L = Lcur;
            sc_s1 = sc_s1_c; 
            sc_s2 = sc_s2_c;
            sc_s3 = sc_s3_c;

            pidx = i;
        }
    }

    curve cur; 

    if(pidx >= 0){
        double s1, s2, s3;
        tie(s1, s2, s3) = scaleFromStandard(lambda, sc_s1, sc_s2, sc_s3);

        cur = dubinscurve(x0, y0, th0, s1, s2, s3, ksigns[pidx][0]*kmax, ksigns[pidx][1]*kmax, ksigns[pidx][2]*kmax);
        
        // Assertion  
    }else{
        cout << "PIDX < 0" << endl;
    }

    return make_pair(pidx, cur);
}

Path Dubins::getPath(curve c){
    Path p;
    int npts = 100;
    double x, y, th;

    cout << "Lunghezze singole: " << c.a1.L << " - " << c.a2.L << " - " << c.a3.L << endl;

    double s_g = c.L/npts;

    for(int i = 0; i < floor(c.a1.L/s_g); i++){
        double s = s_g * i;

        tie(x, y, th) = circline(s, c.a1.x0, c.a1.y0, c.a1.th0, c.a1.k);
        p.points.emplace_back(s,x,y,th,c.a1.k);
    }

    for(int i = 0; i < floor(c.a2.L/s_g); i++){
        double s =  s_g * i;

        tie(x, y, th) = circline(s, c.a2.x0, c.a2.y0, c.a2.th0, c.a2.k);
        p.points.emplace_back(s,x,y,th,c.a2.k);
    }

    for(int i = 0; i < floor(c.a3.L/s_g); i++){
        double s =  s_g * i;

        tie(x, y, th) = circline(s, c.a3.x0, c.a3.y0, c.a3.th0, c.a3.k);
        p.points.emplace_back(s,x,y,th,c.a3.k);
    }

    return p;
}

void print_arc(arc a, string nome){
    cout << "--- " << nome << " ---" << endl;
    cout << "(x0: " << a.x0 << ", y0: " << a.y0 << ", th0: "<< a.th0 << ")" << endl;
    cout << "(xf: " << a.xf << ", yf: " << a.yf << ", thf: "<< a.thf << ")" << endl;
    cout << "K: " << a.k << "- L: " << a.L << ")" << endl;
}