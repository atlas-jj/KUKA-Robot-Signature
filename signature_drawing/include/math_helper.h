/*************************************************************************
	> File Name: triangular_dist.h
	> Author:
	> Mail:
	> Created Time: Mon 20 Feb 2017 09:23:05 PM MST
 ************************************************************************/

#ifndef _TRIANGULAR_DIST_H
#define _TRIANGULAR_DIST_H
#endif
#include <iostream>
#include<math.h>
#include<stdlib.h>
using namespace std;

class math_helper
{
    public:
       math_helper(double b);
       ~math_helper();
       double getPx(double x);
       double getRandX();
       double getRandY();
    private:
       double st_d;
};
