#include <cmath>
#include <iostream>

// rLam is the reference longitude (in degrees)
// rPhi is the reference latitude (in degrees)
// The reference angles define where the 0,0,0 point is in the local NED coordinates
static double rPhi = 0;
static double rLam = 0;

void printUsage() {
    std::cout << "Usage: ./a.out latitude longitude altitude" << std::endl;
}

void printNED(NED_s ned) {

}

NED_s GPS2NED(double phi, double lambda, double h);

int main(int argc, char* argv[]) {
    if(argc < 4) {
        printUsage();
    }

    double lati = atof(argv[1]);
    double longi = atof(argv[2]);
    double alti = atof(argv[3]);

    printNED(GPS2NED(lati, longi, alti));
}

NED_s GPS2NED(double phi, double lambda, double h)
{
	// send in phi (latitude) as an angle in degrees ex.38.14626
	// send in lambda (longitude) as an angle in degrees ex. 76.42816
	// both of those are converted into radians
	// send in h as a altitude (mean sea level) provo = about 1500, maryland = 6.7056

	// CONSTANTS
	double piD180 = 3.1415926535897932 / 180.0;
	double a = 6378137.0;						// length of Earth’s semi-major axis in meters
	double b = 6356752.3142;					// length of Earth’s semi-minor axis in meters
    double e2 = 1. - pow((b / a), 2);			// first numerical eccentricity
    
    double rPhiRad = rPhi*piD180;
    double rLamRad = rLam*piD180;
    
	double chi = sqrt(1 - e2*sin(rPhiRad)*sin(rPhiRad));

	// Convert the incoming angles to radians
	phi = phi*piD180;
    lambda = lambda*piD180;

	// Convert the angles into Earth Centered Earth Fixed Reference Frame
	double x = (a / chi + h)* cos(phi)*cos(lambda);
	double y = (a / chi + h)* cos(phi)*sin(lambda);
	double z = (a*(1 - e2) / chi + h)*sin(phi);

	// Find the difference between the point x, y, z to the reference point in ECEF 
	double dx = x - xr;
	double dy = y - yr;
	double dz = z - zr;

	// Rotate the point in ECEF to the Local NED
	NED_s ned;
	ned.N = (-sin(rPhiRad)*cos(rLamRad)*dx) + (-sin(rPhiRad)*sin(rLamRad)*dy) + cos(rPhiRad)*dz;
	ned.E = (sin(rLamRad)*dx) - (cos(rLamRad)*dy);
	ned.D = (-cos(rPhiRad)*cos(rLamRad)*dx) + (-cos(rPhiRad)*sin(rLamRad)*dy) + (-sin(rPhiRad)*dz);
	return ned;
}