#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <fstream>

#define DEBUG // Run tests

/*
    Converts latitude and longitude to NED coordinates. We can 
    use this to generate NED coordinates for testing given GPS locations.
*/

// rLam is the reference longitude (in degrees)
// rPhi is the reference latitude (in degrees)
// The reference angles define where the 0,0,0 point is in the local NED coordinates
static double rPhi = 0;
static double rLam = 0;
static double rH = 0; // MSL, positive, in meters

struct NED {
    double N; // North
    double E; // East
    double D; // Down. Down is negative. 10 feet above ground = -10.
};

struct LatLon {
    double LAT;
    double LON;
};

// Prototypes
void printUsage();
void printNED(NED ned);
LatLon str2LatLon(std::string lat, std::string lon);
NED GPS2NED(double phi, double lambda, double h);
void runTests();

int main(int argc, char* argv[]) {
#ifdef DEBUG
    runTests();
#endif
#ifndef DEBUG
    if(argc < 4) {
        printUsage();
    }

    double lati = atof(argv[1]);
    double longi = atof(argv[2]);
    double alti = atof(argv[3]);

    printNED(GPS2NED(lati, longi, alti));
#endif
}

void printUsage() {
    std::cout << "Usage: ./a.out latitude longitude altitude" << std::endl;
}

void printNED(NED ned) {
    std::cout << "N: " << ned.N << "\nE: " << ned.E << "\nD: " << ned.D << std::endl;
}

LatLon str2LatLon(std::string lat, std::string lon) {
    LatLon result;
    result.LAT = std::stod(lat.substr(1, 2)) + std::stod(lat.substr(4, 2)) / 60.0 + stod(lat.substr(7, 5)) / 3600.0;
    result.LON = std::stod(lon.substr(2, 3)) + std::stod(lon.substr(5, 2)) / 60.0 + stod(lon.substr(8, 5)) / 3600.0;
    return result;
}

NED GPS2NED(double phi, double lambda, double h)
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
    
    // Convert reference points into radians.
    double rPhiRad = rPhi*piD180;
    double rLamRad = rLam*piD180;
    double rHRad = rH*piD180;
    
    // Convert the angles into Earth Centered Earth Fixed Reference Frame
    double chi = sqrt(1 - e2*sin(rPhiRad)*sin(rPhiRad));
	double xr = (a / chi + rHRad)* cos(rPhi)*cos(rLam);
	double yr = (a / chi + rHRad)* cos(rPhi)*sin(rLam);
	double zr = (a*(1 - e2) / chi + rHRad)*sin(rPhi);

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
	NED ned;
	ned.N = (-sin(rPhiRad)*cos(rLamRad)*dx) + (-sin(rPhiRad)*sin(rLamRad)*dy) + cos(rPhiRad)*dz;
	ned.E = (sin(rLamRad)*dx) - (cos(rLamRad)*dy);
	ned.D = (-cos(rPhiRad)*cos(rLamRad)*dx) + (-cos(rPhiRad)*sin(rLamRad)*dy) + (-sin(rPhiRad)*dz);
	return ned;
}

void runTests() {
    // Set the reference point
    LatLon ref = str2LatLon("N38-09-01.50", "W076-25-29.70");
    rPhi = ref.LAT;
    rLam = ref.LON;
    rH = 6.7056;

    // Read the test_input file and compare it to test_expected_output
    std::ifstream input, expected_output;
    input.open("test_input.txt");
    expected_output.open("test_expected_output.txt");

    std::string lat, lon;
    LatLon latlon;
    NED actual;
    int testNum = 0;
    while (!input.eof()) {
        testNum ++;
        input >> lat >> lon;
        latlon = str2LatLon(lat, lon);
        NED actualNed = GPS2NED(latlon.LAT, latlon.LON, rH);

        // Compare to expected
        double n, e;
        expected_output >> n >> e;
        if(n != actualNed.N || e != actualNed.E) {
            std::cout << "Test " << testNum << " failed" << std::endl;
            exit(1);
        }
    }
    
    std::cout << "All tests passed" << std::endl;
}