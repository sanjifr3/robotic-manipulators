#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

int main()
{
	//Parameters of Pyramid
	//Limit: 238 < sqrt(x^2 + y^2) < 559
	double a = 40;//mm - side lengths of cube
	double s = 25;//mm - spacing of cusbes
    double alphai = 90;//deg - Pick Up Angle ((+) = CW from (+X) axis)
    double alphaf = 0;//deg - Drop off Angle ((+) = CW from (+X) axis)
    double XfSt = 250;//mm - X coordinate of left middle bottom block
    double YfSt = 0;//mm - Y coordinate of left middle bottom block
    double YiSt = -150;//mm - Y coordinate of first initial block
    double XiSt = 200;//mm - X coordinate of first initial block
    double Zzero = 205;//mm - Minimum Z coordinate
    double Zoff = 20;//mm - Z offset
   
    MatrixXd XYZi(14,3);
    MatrixXd XYZf(14,3);

    // Initial X Coordinates
    XYZi(0,0) = XYZi(5,0) = XYZi(10,0) = XiSt;
    XYZi(1,0) = XYZi(6,0) = XYZi(11,0) = XiSt + 1.5*a;
    XYZi(2,0) = XYZi(7,0) = XYZi(12,0) = XiSt + 3*a;
    XYZi(3,0) = XYZi(8,0) = XYZi(13,0) = XiSt + 4.5*a;
    XYZi(4,0) = XYZi(9,0) = XiSt + 6*a;

    // Initial Y Coordinates
    XYZi(0,1) = XYZi(1,1) = XYZi(2,1) = XYZi(3,1) = XYZi(4,1) = YiSt;
    XYZi(5,1) = XYZi(6,1) = XYZi(7,1) = XYZi(8,1) = XYZi(9,1) = YiSt - 2*a;
    XYZi(10,1) = XYZi(11,1) = XYZi(12,1) = XYZi(13,1) = YiSt -4*a;

    // Initial Z Coordinates
    for (int i = 0; i < 14; i++)
    {
    	XYZi(i,2) = Zzero + Zoff + 0.5*a;
    }

	// Final X Coordinates
    XYZf(2,0) = XYZf(5,0) = XYZf(8,0) = XfSt + 0.5*a;
    XYZf(1,0) = XYZf(4,0) = XYZf(7,0) = XYZf(13,0) = XfSt + 1.5*a + s;
    XYZf(0,0) = XYZf(3,0) = XYZf(6,0) = XfSt + 2.5*a + 2*s;
    XYZf(10,0) = XYZf(12,0) = XfSt + a + 0.5*s;
    XYZf(9,0) = XYZf(11,0) = XfSt + 2*a + 1.5*s;

    // Final Y Coordinates
    XYZf(0,1) = XYZf(1,1) = XYZf(2,1) = YfSt + a + s;
    XYZf(3,1) = XYZf(4,1) = XYZf(5,1) = XYZf(13,1) = YfSt;
    XYZf(6,1) = XYZf(7,1) = XYZf(8,1) = YfSt - (a + s);
    XYZf(9,1) = XYZf(10,1) = YfSt + 0.5*(a + s);
    XYZf(11,1) = XYZf(12,1) = YfSt - 0.5*(a + s);

    // Final Z Coordinates
    for(int i=0; i<9; i++)
    {
         XYZf(i,2) = Zzero + Zoff + 0.5*a;	
    }
 
    for(int i=9; i<13; i++)
    {
         XYZf(i,2) = Zzero + Zoff + 1.5*a;
    }
    
    XYZf(13,2) = Zzero + 2.5*a + Zoff;

    cout << "\nInitial Coordinates:\n   x   y    z\n" << XYZi << endl;
    cout << "\nFinal Coordinates:\n   x      y    z\n" << XYZf << endl;
}
