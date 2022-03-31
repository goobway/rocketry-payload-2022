#include <iostream>
#include <math.h>
#include <fstream>
#include <string.h>
#include <vector>
#include <sstream>

using namespace std;

vector<float> cum_trapz(vector<float> time, vector<float> data, int offset, int max, bool velocity)
{
    float initial, final, trapezoid, dt, sum = 0;
    vector<float> integration;
    vector<float> velCur;
    vector<float> disTot;

    if (velocity)
    {
        for (int i = offset; i < max; i++)
        {
            initial = time[i];
            final = time[i + 1];
            dt = final - initial; // time step
            trapezoid = dt * ((data[i] + data[i + 1]) / 2);
            integration.push_back(trapezoid);
        }
        for (int j = 0; j < max; j++)
        {
            velCur.push_back(velCur[j] + integration[j + 1]);
        }
        return velCur;
    }
    else
    {
        for (int i = 0; i < max; i++)
        {
            initial = time[i + offset];
            final = time[(i + offset) + 1];
            dt = final - initial;
            trapezoid = dt * ((data[i] + data[i + 1]) / 2);
            integration.push_back(trapezoid);
        }
        for (int j = 0; j < max; j++)
        {
            disTot.push_back(disTot[j] + integration[j + 1]);
        }
        return disTot;
    }

    return integration;
}

int main()
{
    /* INPUT: READ FROM CSV FILE */
    vector<float> time, accX, accY, accZ;

    // Temp variables to store elements in string stream
    std::string tempTime, tempAccX, tempAccY, tempAccZ, line;

    std::ifstream file("acc-data-20ft-cal2");
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
        {                                                 // skip first line
            time.push_back(std::stof(tempTime) / 1000.0); // convert to seconds
            accX.push_back(std::stof(tempAccX));
            accY.push_back(std::stof(tempAccY));
            accZ.push_back(std::stof(tempAccZ));
        }
        firstLine = false;
    }

    /* CALCULATIONS */
    vector<float> velX = cum_trapz(time, accX, 863, 1000, true);
    vector<float> velY = cum_trapz(time, accY, 863, 1000, true);
    vector<float> velZ = cum_trapz(time, accZ, 863, 1000, true);

    vector<float> dispX = cum_trapz(time, velX, 863, velX.size(), false);
    vector<float> dispY = cum_trapz(time, velY, 863, velY.size(), false);
    vector<float> dispZ = cum_trapz(time, velZ, 863, velZ.size(), false);

    cout << velX.size() << endl;
    cout << dispX.size() << endl;

    /* SAVE DATA TO NEW FILES */

    int offset = 863;

    /* Velocity */
    ofstream velFile("velocity.csv");
    for (int n = 0; n < velX.size(); n++)
    {
        velFile << time[n + offset] << "," << velX[n] << "," << velY[n] << "," << velZ[n] << endl;
    }

    /* Displacement */
    ofstream dispFile("displacement.csv");
    for (int n = 0; n < dispX.size(); n++)
    {
        dispFile << time[n + offset] << "," << dispX[n] << "," << dispY[n] << "," << dispZ[n] << endl;
    }

    return 0;
}