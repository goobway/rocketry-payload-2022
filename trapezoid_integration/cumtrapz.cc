#include <iostream>
#include <math.h>
#include <fstream>
#include <string.h>
#include <vector>
#include <sstream>

using namespace std;

vector<float> cum_trapz(vector<float> time, vector<float> data, int offset, int max, bool first)
{
    float lowerBound, upperBound, trapezoid, h;
    vector<float> integration;

    if (first == true)
    {
        for (int i = offset; i < max; i++)
        {
            lowerBound = time[i - 1];
            upperBound = time[i];
            h = upperBound - lowerBound;
            trapezoid = h * ((data[i - 1] + data[i]) / 2);
            //cout << i << ": " << trapezoid << endl;
            integration.push_back(trapezoid);
        }
    }
    else
    {
        for (int i = 0; i < max; i++)
        {
            lowerBound = time[(i + offset) - 1];
            upperBound = time[i + offset];
            h = upperBound - lowerBound;
            trapezoid = h * ((data[i - 1] + data[i]) / 2);
            //cout << i << ": " << trapezoid << endl;
            integration.push_back(trapezoid);
        }
    }

    return integration;
}

int main()
{
    /* INPUT: READ FROM CSV FILE */
    vector<float> time, accX, accY, accZ;

    // Temp variables to store elements in string stream
    std::string tempTime, tempAccX, tempAccY, tempAccZ, line;

    std::ifstream file("acc-data-20ft");
    bool firstLine = true;

    while (std::getline(file, line))
    {
        istringstream iss(line); // Construct string stream from line

        // Read line until comma saving value to temporary strings
        getline(iss, tempTime, ',');
        getline(iss, tempAccX, ',');
        getline(iss, tempAccY, ',');
        getline(iss, tempAccZ, ',');
        if (firstLine == false)
        { // skip first line
            time.push_back(std::stof(tempTime) / 1000.0); // convert to seconds
            accX.push_back(std::stof(tempAccX));
            accY.push_back(std::stof(tempAccY));
            accZ.push_back(std::stof(tempAccZ));
        }
        firstLine = false;
    }

    /* CALCULATIONS */
    vector<float> velX = cum_trapz(time, accX, 829, 1001, true);
    vector<float> velY = cum_trapz(time, accY, 829, 1001, true);
    vector<float> velZ = cum_trapz(time, accZ, 829, 1001, true);

    vector<float> dispX = cum_trapz(time, velX, 829, velX.size() + 1, false);
    vector<float> dispY = cum_trapz(time, velX, 829, velY.size() + 1, false);
    vector<float> dispZ = cum_trapz(time, velX, 829, velZ.size() + 1, false);

    return 0;
}