#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/

void computeBasicMultipliers(int _nextCheckpointAngle, int _nextCheckpointDist, float _currentSpeed, float &_angleMultiplier, float &_distMultiplier)
{
    const float MIN_DISTANCE_SLOWDOWN = 3000.0f;
    const float MAX_ANGLE_ENGINE = 120.0f;
    const float MIN_ANGLE_ENGINE = 45.0f;

    if(abs(_nextCheckpointAngle) > MAX_ANGLE_ENGINE)
    {
        _angleMultiplier = 0.0f;
    }

    else if(abs(_nextCheckpointAngle) > MIN_ANGLE_ENGINE)
    {
        //Angle Multiplier slows down the larger the angle to next waypoint is
        float tempAngleMultiplier = (45.0f / abs(_nextCheckpointAngle) * (45.0f / abs(_nextCheckpointAngle)));

        //When dealing with large distances, slowing down costs more time than doing a large loop with keeping max speed
        float largeDistanceMultiplier = (_nextCheckpointDist / MIN_DISTANCE_SLOWDOWN) * (_nextCheckpointDist / MIN_DISTANCE_SLOWDOWN);

        _angleMultiplier = clamp(tempAngleMultiplier * largeDistanceMultiplier, 0.0f, 1.0f);

        //Ignore speed change based solely on distance when having a large angle distance is present
        _distMultiplier = 1.0f;
    }

    else
    {
        if(_nextCheckpointDist * (100 / _currentSpeed) < MIN_DISTANCE_SLOWDOWN)
        {
            _distMultiplier = 0.2f + clamp((_nextCheckpointDist / MIN_DISTANCE_SLOWDOWN), 0.0f, 1.0f) * 0.8f;
        }
    }
}

int main()
{
    static bool usedBoost = false;

    int prevX;
    int prevY;

    // game loop
    while (1) {
        int x;
        int y;
        int nextCheckpointX; // x position of the next check point
        int nextCheckpointY; // y position of the next check point
        int nextCheckpointDist; // distance to the next checkpoint
        int nextCheckpointAngle; // angle between your pod orientation and the direction of the next checkpoint
        cin >> x >> y >> nextCheckpointX >> nextCheckpointY >> nextCheckpointDist >> nextCheckpointAngle; cin.ignore();
        int opponentX;
        int opponentY;
        cin >> opponentX >> opponentY; cin.ignore();

        // Write an action using cout. DON'T FORGET THE "<< endl"
        // To debug: cerr << "Debug messages..." << endl;


        // You have to output the target position
        // followed by the power (0 <= thrust <= 100)
        // i.e.: "x y thrust"

        float currentSpeed = sqrt((x - prevX) * (x - prevX) +  (y - prevY) * (y - prevY));

        float finalMultiplier = 1.0f;

        float distMultiplier = 1.0f;
        float angleMultiplier = 1.0f;

        computeBasicMultipliers(nextCheckpointAngle, nextCheckpointDist, currentSpeed, angleMultiplier, distMultiplier);
        
        if(!usedBoost && nextCheckpointDist > 8000 && abs(nextCheckpointAngle) < 5)
        {
            usedBoost = true;

            cout << nextCheckpointX << " " << nextCheckpointY << " BOOST" << endl;

            prevX = x;
            prevY = y;

            continue;
        }
        finalMultiplier = distMultiplier * angleMultiplier;
        int finalSpeed = round(clamp(100 * finalMultiplier, 0.0f, 100.0f));

        prevX = x;
        prevY = y;

        cout << nextCheckpointX << " " << nextCheckpointY << " " << finalSpeed << endl;
    }
}
