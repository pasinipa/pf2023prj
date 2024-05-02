#pragma once

struct SimPars {
    double ds{ 1 };
    double d{ 20 };
    double s{ 5 };
    double a{ 0.7 };
    double c{ 0.3 };

    SimPars (double p1, double p2, double p3, double p4, double p5)
    : ds{ p1 }, d{ p2 }, s{ p3 }, a{ p4 }, c{ p5 } {}

    SimPars () = default;
};
