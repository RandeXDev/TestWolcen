#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;

struct Vector2
{
public:
float x;
float y;

//CONSTRUCTORS
Vector2(void)
{
    x = 0.0f;
    y = 0.0f;
}
Vector2(float xValue, float yValue)
{
    x = xValue;
    y = yValue;
}
Vector2(const Vector2 & v)
{
    x = v.x;
    y = v.y;
}
    
//ASSINGMENT AND EQUALITY OPERATIONS
inline Vector2& operator = (const Vector2 & v) { x = v.x; y = v.y; return *this; }
inline Vector2& operator = (const float & f) { x = f; y = f; return *this; }
inline Vector2& operator - (void) { x = -x; y = -y; return *this; }
inline bool operator == (const Vector2 & v) const { return (x == v.x) && (y == v.y); }
inline bool operator != (const Vector2 & v) const { return (x != v.x) || (y != v.y); }

//VECTOR2 TO VECTOR2 OPERATIONS
inline const Vector2 operator + (const Vector2 & v) const { return Vector2(x + v.x, y + v.y); }
inline const Vector2 operator - (const Vector2 & v) const { return Vector2(x - v.x, y - v.y); }
inline const Vector2 operator * (const Vector2 & v) const { return Vector2(x * v.x, y * v.y); }
inline const Vector2 operator / (const Vector2 & v) const { return Vector2(x / v.x, y / v.y); }

//VECTOR2 TO THIS OPERATIONS
inline Vector2& operator += (const Vector2 & v) { x += v.x; y += v.y; return *this; }
inline Vector2& operator -= (const Vector2 & v) { x -= v.x; y -= v.y; return *this; }
inline Vector2& operator *= (const Vector2 & v) { x *= v.x; y *= v.y; return *this; }
inline Vector2& operator /= (const Vector2 & v) { x /= v.x; y /= v.y; return *this; }

//SCALAR TO VECTOR2 OPERATIONS
inline const Vector2 operator + (float v) const { return Vector2(x + v, y + v); }
inline const Vector2 operator - (float v) const { return Vector2(x - v, y - v); }
inline const Vector2 operator * (float v) const { return Vector2(x * v, y * v); }
inline const Vector2 operator / (float v) const { return Vector2(x / v, y / v); }

//SCALER TO THIS OPERATIONS
inline Vector2& operator += (float v) { x += v; y += v; return *this; }
inline Vector2& operator -= (float v) { x -= v; y -= v; return *this; }
inline Vector2& operator *= (float v) { x *= v; y *= v; return *this; }
inline Vector2& operator /= (float v) { x /= v; y /= v; return *this; }


float Length() const { return sqrt(x * x + y * y); }
float LengthSquared() const { return x * x + y * y; }
float Distance(const Vector2 & v) const { return sqrt(((x - v.x) * (x -     v.x)) + ((y - v.y) * (y - v.y))); }
float DistanceSquared(const Vector2 & v) const { return ((x - v.x) * (x -     v.x)) + ((y - v.y) * (y - v.y)); }
float Dot(const Vector2 & v) const { return x * v.x + y * v.y; }
float Cross(const Vector2 & v) const { return x * v.y + y * v.x; }
float Angle(const Vector2 & v) const { return atan2(x * v.y - y * v.x , x * v.x + y * v.y) * 180.0f / M_PI ;}
Vector2 Normalize()  const { return *this / sqrt(x * x + y * y); }
void Normalize() {*this /= sqrt(x * x + y * y);}
void Invert() { int temp = x; x = y; y = temp;}

Vector2 ProjectOnLine(const Vector2 & point, const Vector2 & direction)  const
{
    Vector2 linePointToPoint = *this - point;

    float t = linePointToPoint.Dot(direction);

    return point + direction * t;
}

};

class Waypoint
{
    protected:

    float MIN_DISTANCE = 3000.0f;
    float MAX_SIDE_DEVIATION_PERCENT = 0.2f;
    float MAX_SIDE_DEVIATION = 750.0f;

    int MAX_DEPTH = 2;

    float touchRadius = 600.0f;

    public:

    bool isMajorWp = false;
    Vector2 position;
    
    Waypoint *nextWp = nullptr;
    Waypoint *prevWp = nullptr;

    int subWaypointDepth = 0;
    float distanceToNextWp = 0.0f;

    Waypoint()
    {
        nextWp = nullptr;
        prevWp = nullptr;
    }

    Waypoint(Waypoint &_prev, Waypoint &_next)
    {
        nextWp = &_prev;
        prevWp = &_next;
    }

    bool TryGenerate(Waypoint &_prev, Waypoint &_next, int _depth)
    {
        subWaypointDepth = _depth;

        prevWp = &_prev;
        nextWp = &_next;

        //cerr << _prev.position.x << " " << _prev.position.y << " / ";
        //cerr << nextWp->position.x << " " << nextWp->position.y << endl;

        Vector2 currentDirection = (nextWp->position - prevWp->position).Normalize();
        Vector2 exitDirection = (nextWp->nextWp->position - nextWp->position).Normalize();
        float exitAngleDiff = currentDirection.Angle(exitDirection);

        float previousWpDist = (nextWp->position - prevWp->position).Length();
        Vector2 deviationDirection;

        if(exitAngleDiff > 20.0f)
        {
            //Rotate 90 clockwise
            deviationDirection = Vector2(currentDirection.y, -currentDirection.x);
        }

        else if(exitAngleDiff < -20.0f)
        {
            //Rotate 90 counter-clockwise
            deviationDirection = Vector2(-currentDirection.y, currentDirection.x);
        }

        else
        {
            return false;
        }

        Vector2 onTrajectoryPosition = prevWp->position + currentDirection * (previousWpDist / 2 + (previousWpDist / 6) * abs(exitAngleDiff / 180.0f));
        Vector2 finalTryPosition = onTrajectoryPosition + deviationDirection * MAX_SIDE_DEVIATION * abs(exitAngleDiff / 180.0f);

        if((finalTryPosition - nextWp->position).Length() < MIN_DISTANCE || (finalTryPosition - prevWp->position).Length() < MIN_DISTANCE )
        {
            return false;
        }

        cerr << "New Wp Generated at: " << finalTryPosition.x << " " << finalTryPosition.y << endl;

        position = finalTryPosition;

        prevWp->nextWp = this;
        nextWp->prevWp = this;

        return true;
    }

    void SplitWithNextWp(vector<Waypoint> &_otherWaypoints)
    {
        //cerr << "Try Splitting :" << subWaypointDepth << endl;
        if(subWaypointDepth <= MAX_DEPTH)
        {
            _otherWaypoints.push_back(Waypoint());
            Waypoint *newWaypoint = &_otherWaypoints.back();

            bool success = newWaypoint->TryGenerate(*this, *nextWp, subWaypointDepth + 1);

            //cerr << "Max depth not reached: " << subWaypointDepth << " / Success: " << success << endl;

            if(success)
            {
                //cerr << "Splitting :" << subWaypointDepth << endl;

                newWaypoint->SplitWithNextWp(_otherWaypoints);
                SplitWithNextWp(_otherWaypoints);
            }
        }
    }
};

class MajorWaypoint : public Waypoint
{
    public:

    MajorWaypoint() : Waypoint()
    {
        isMajorWp = true;
    }

    MajorWaypoint(Waypoint &_prev, Waypoint &_next) : Waypoint(_prev, _next)
    {
        isMajorWp = true;
    }

    int checkPointId;

    MajorWaypoint *nextMajorWp = nullptr;
    MajorWaypoint *prevMajorWp  = nullptr;
};

class BasePod
{
    public:

    enum Faction
    {
        Friendly,
        Enemy
    };

    Faction podFaction;

    int nextCheckpointID;

    Vector2 position;
    Vector2 inertiaDirection;
    float speedValue;

    float myRotationAngle;
    Vector2 rotationDirection;

    string name;
    vector<int> checkPointCoords_X;
    vector<int> checkPointCoords_Y;

    vector<MajorWaypoint> *majorWaypoints;
    vector<Waypoint> *allWaypoints;

    MajorWaypoint *prevMajorWp = nullptr;
    MajorWaypoint *nextMajorWp = nullptr;

    Waypoint *nextWaypoint = nullptr;
    Vector2 nextCheckPointPos;

    float nextCheckpointAngle;
    float nextCheckpointDist;
    float nextMajorCheckPointDist;
    Vector2 nextCheckpointDirection;

    virtual void InitPod(vector<int> &_coordsX, vector<int> &_coordsY, vector<MajorWaypoint> &_majorWaypoints, vector<Waypoint> &_allWaypoints, string _name)
    {
        name = _name;

        checkPointCoords_X = _coordsX;
        checkPointCoords_Y = _coordsY;

        allWaypoints = &_allWaypoints;
        majorWaypoints = &_majorWaypoints;

        prevMajorWp = &majorWaypoints->front();
        nextMajorWp = prevMajorWp->nextMajorWp;
    }
};

class EnemyPod : public BasePod
{
    public:

    EnemyPod()
    {
        podFaction = Faction::Enemy;
    }

    Vector2 GetPosition()
    {
        return position;
    }

    Vector2 GetInertialDirection()
    {
        return inertiaDirection;
    }

    void UpdateEnemy(Vector2 _newPos, Vector2 speed, float _rotationAngle, int _nextCheckpointID)
    {
        //Calculations

        nextCheckpointID = _nextCheckpointID;

        nextCheckPointPos = Vector2(checkPointCoords_X.at(nextCheckpointID), checkPointCoords_Y.at(nextCheckpointID));
        myRotationAngle = _rotationAngle;
        rotationDirection = Vector2(cos(myRotationAngle * M_PI / 180.0f), sin(myRotationAngle * M_PI / 180.0f));

        //cerr << "Rotation angle: " << myRotationAngle << endl;
        //cerr << "Rotation Direction: " << rotationDirection.x << " " << rotationDirection.y << endl;

        position = _newPos;
        inertiaDirection = speed;
        speedValue = speed.Length();

        //cerr << "Next CP Pos: " << nextCheckPointPos.x << " " << nextCheckPointPos.y  << " | My Pos: " << position.x << " " << position.y << endl;

        nextCheckpointDirection = (nextCheckPointPos - position);
        nextCheckpointDirection.Normalize();

        //cerr << "Next CP Dir: " << nextCheckpointDirection.x << " " << nextCheckpointDirection.y  << endl;

        nextCheckpointAngle = nextCheckpointDirection.Angle(rotationDirection);

        //cerr << "Next CP Angle: " << nextCheckpointAngle << endl << endl;

        nextCheckpointDist = (nextCheckPointPos - position).Length();
        nextMajorCheckPointDist = nextCheckpointDist;

        position = _newPos;
    }
};

class PodRacer : public BasePod
{
    private:

    //Movement multipliers
    //--------------------------------
    float angleMultiplier;
    float distanceMultiplier;
    bool usedBoost = false;
    bool boostAvailable = false;
    bool driftPossible = false;

    float avoidanceMultiplier;
    float finalMultiplier;
    Vector2 finalDirection;
    float driftMultiplier;

    bool collisionImminent;

    public: 

    Vector2 calculatedTargetPosition;

    PodRacer()
    {
        podFaction = Faction::Friendly;
    }

    Vector2 GetPosition()
    {
        return position;
    }

    Vector2 GetInertialDirection()
    {
        return inertiaDirection;
    }

    void InitPod(vector<int> &_coordsX, vector<int> &_coordsY, vector<MajorWaypoint> &_majorWaypoints, vector<Waypoint> &_allWaypoints, string _name)
    {
        BasePod::InitPod(_coordsX, _coordsY, _majorWaypoints, _allWaypoints, _name);

        /*
        Waypoint *initialWaypoint = &majorWaypoints->front();
        Waypoint *currentWaypoint = &majorWaypoints->front();

        cerr << endl << endl;

        cerr << "<<" << initialWaypoint << "," << currentWaypoint << ">>" << endl;
        
        cerr << endl << endl;

        while(true)
        {
            cerr << "(" <<currentWaypoint->position.x << "," <<currentWaypoint->position.y << ")" << " / Depth: " << currentWaypoint->subWaypointDepth << endl;
            //cerr << "(" <<currentWaypoint->nextWp->position.x << "," <<currentWaypoint->nextWp->position.y << ")" << " / Depth: " << currentWaypoint->subWaypointDepth << endl;
            currentWaypoint = currentWaypoint->nextWp;

            cerr << "<<" << initialWaypoint << "," << currentWaypoint << ">>" << endl;

            if(initialWaypoint == currentWaypoint)
            {
                break;
            }
        }
        */
    }

    void BeginRun(Waypoint &_firstWaypoint)
    {
        nextWaypoint = &_firstWaypoint;
    }

    void UpdateRacer_gold(Vector2 _newPos, Vector2 speed, float _rotationAngle, int _nextCheckpointID)
    {
        // Re-init variables
        angleMultiplier = 1.0f;
        distanceMultiplier = 1.0f;
        boostAvailable = false;
        avoidanceMultiplier = 1.0f;
        driftMultiplier = 1.0f;
        finalMultiplier = 1.0f;
        Vector2 finalDirection = Vector2(1.0f,1.0f);
        collisionImminent = false;

        //Calculations

        nextCheckpointID = _nextCheckpointID;

        nextCheckPointPos = Vector2(checkPointCoords_X.at(nextCheckpointID), checkPointCoords_Y.at(nextCheckpointID));
        myRotationAngle = _rotationAngle;
        rotationDirection = Vector2(cos(myRotationAngle * M_PI / 180.0f), sin(myRotationAngle * M_PI / 180.0f));

        //cerr << "Rotation angle: " << myRotationAngle << endl;
        //cerr << "Rotation Direction: " << rotationDirection.x << " " << rotationDirection.y << endl;

        position = _newPos;
        inertiaDirection = speed;
        speedValue = speed.Length();

        //cerr << "Next CP Pos: " << nextCheckPointPos.x << " " << nextCheckPointPos.y  << " | My Pos: " << position.x << " " << position.y << endl;

        nextCheckpointDirection = (nextCheckPointPos - position);
        nextCheckpointDirection.Normalize();

        //cerr << "Next CP Dir: " << nextCheckpointDirection.x << " " << nextCheckpointDirection.y  << endl;

        nextCheckpointAngle = nextCheckpointDirection.Angle(rotationDirection);

        //cerr << "Next CP Angle: " << nextCheckpointAngle << endl << endl;

        nextCheckpointDist = (nextCheckPointPos - position).Length();
        nextMajorCheckPointDist = nextCheckpointDist;

        position = _newPos;
    }

    void UpdateRacer_waypoints(Vector2 _newPos, Vector2 speed, float _rotationAngle, int _nextCheckpointID)
    {
        // Re-init variables
        
        angleMultiplier = 1.0f;
        distanceMultiplier = 1.0f;
        boostAvailable = false;
        avoidanceMultiplier = 1.0f;
        driftMultiplier = 1.0f;
        finalMultiplier = 1.0f;
        Vector2 finalDirection = Vector2(1.0f,1.0f);
        collisionImminent = false;
        

        //Calculations

        nextCheckpointID = _nextCheckpointID;
        position = _newPos;
        inertiaDirection = speed;
        speedValue = speed.Length();

        if(nextCheckpointID != nextMajorWp->checkPointId)
        {
            prevMajorWp = nextMajorWp;
            nextMajorWp = nextMajorWp->nextMajorWp;
            cerr << name << " Checkpoint change: -> next wp: " << nextMajorWp->checkPointId << endl;
        }

        //Find Current Intermediary Checkpoint

        Vector2 majorWpLine = (nextMajorWp->position - position).Normalize();
        Vector2 minorWpLine = (prevMajorWp->nextWp->position - prevMajorWp->position).Normalize();

        Waypoint *currentTestedWaypoint = prevMajorWp->nextWp;
        float toNextWpDist = (currentTestedWaypoint->position - position).Length();
        Vector2 toNextWpLine = (currentTestedWaypoint->position - position).Normalize();

        bool usingMinorWp = true;

        while(true)
        {
            if(currentTestedWaypoint == nextMajorWp)
            {
                usingMinorWp = false;
                break;
            }
            else
            {
                if(abs(toNextWpLine.Angle(minorWpLine)) > 45 || toNextWpDist < 600.0f)
                {
                    currentTestedWaypoint = currentTestedWaypoint->nextWp;
                    toNextWpLine = (currentTestedWaypoint->position - position).Normalize();
                    toNextWpDist = (currentTestedWaypoint->position - position).Length();
                }
                else
                {
                    break;
                }
            }
        }

        nextWaypoint = currentTestedWaypoint;

        nextCheckPointPos = nextWaypoint->position;
        myRotationAngle = _rotationAngle;
        rotationDirection = Vector2(cos(myRotationAngle * M_PI / 180.0f), sin(myRotationAngle * M_PI / 180.0f));

        //cerr << "Next CP Pos: " << nextCheckPointPos.x << " " << nextCheckPointPos.y  << " | My Pos: " << position.x << " " << position.y << endl;

        nextCheckpointDirection = (nextCheckPointPos - position);
        nextCheckpointDirection.Normalize();

        //cerr << "Next CP Dir: " << nextCheckpointDirection.x << " " << nextCheckpointDirection.y  << endl;

        nextCheckpointAngle = nextCheckpointDirection.Angle(rotationDirection);

        //cerr << "Next CP Angle: " << nextCheckpointAngle << endl << endl;

        nextCheckpointDist = (nextCheckPointPos - position).Length();
        nextMajorCheckPointDist = (nextMajorWp->position - position).Length();
    }

    void DetectDriftPossible()
    {
        float DISTANCE_TO_SPEED_RATIO = 9.0f;
        float MIN_SPEED_FOR_DRIFT_START = 300.0f;
        float DIST_TO_SPEED_RATIO_FOR_CANCEL = 5.5f;
        float MAX_ANGLE_MULTIPLIER_FOR_DRIFT_BURN = 0.6f;
        float MAX_ANGLE_MULTIPLIER_FOR_DRIFT_END = 0.9f;

         Vector2 toNextNextWpDirection = (nextWaypoint->nextWp->position - nextWaypoint->position).Normalize();

        float angleToTestAgainst_EndDrift = atan2(600.0f * MAX_ANGLE_MULTIPLIER_FOR_DRIFT_END, nextCheckpointDist) * 180.0f / M_PI;
        float angleToTestAgainstAbs_EndDrift = abs(angleToTestAgainst_EndDrift);

        float angleToTestAgainst_Burn = atan2(600.0f * MAX_ANGLE_MULTIPLIER_FOR_DRIFT_BURN, nextCheckpointDist) * 180.0f / M_PI;
        float angleToTestAgainstAbs_Burn = abs(angleToTestAgainst_Burn);

        float angleToNexNextWp = toNextNextWpDirection.Angle(nextCheckpointDirection);
        float inertialAngle = inertiaDirection.Angle(nextCheckpointDirection);
        float inertialAngleAbs = abs(inertialAngle);

        cerr << name << " Next Cp dist: " << nextCheckpointDist << " / Angle: " << inertialAngleAbs << " / Angle to test: " << angleToTestAgainstAbs_EndDrift <<endl;

        if(inertialAngleAbs < angleToTestAgainstAbs_EndDrift)
        {
            bool sameSign = (angleToNexNextWp > 0 && inertialAngle > 0) || (angleToNexNextWp < 0 && inertialAngle < 0);

            if(driftPossible == false)
            {
                if(speedValue > MIN_SPEED_FOR_DRIFT_START && nextCheckpointDist < speedValue * DISTANCE_TO_SPEED_RATIO)
                {
                    //start drift
                    if((inertialAngleAbs < angleToTestAgainstAbs_Burn && sameSign == true) ||
                    (inertialAngleAbs < angleToTestAgainstAbs_EndDrift && sameSign == false))
                    {
                        driftPossible = true;
                    }
                }
            }
            
            if(driftPossible == true)
            {

                if(sameSign == false || nextWaypoint->isMajorWp == false)
                {
                    driftMultiplier = 1.0f;
                }
                else if(inertialAngleAbs < angleToTestAgainstAbs_Burn)
                {
                    driftMultiplier = 1.0f - abs(inertialAngleAbs / angleToTestAgainstAbs_EndDrift);
                }

                else
                {
                    driftMultiplier = 0.0f;
                }

                if(speedValue * DIST_TO_SPEED_RATIO_FOR_CANCEL < nextCheckpointDist)
                {
                    driftPossible = false;
                    driftMultiplier = 1.0f;
                }
            }
        }
        else
        {
            driftPossible = false;
            driftMultiplier = 1.0f;
        }

        cerr << name << "Dift Possible: " << driftPossible << endl;
    }

    bool DetectCollisionImminent(PodRacer _otherRacer)
    {
        float distanceToOtherTarget = (_otherRacer.GetPosition() - position).Length();
        //cerr << "Dist to other: " << distanceToOtherTarget << endl;
        //cerr << "Player speed: " << speedValue << endl;
        //cerr << "Enemy speed: " << _otherRacer.speedValue << endl;

        Vector2 twoTurnMyPosition = position + inertiaDirection;
        Vector2 twoTurnOtherPosition = _otherRacer.GetPosition() + _otherRacer.GetInertialDirection();
        //cerr << "Two Turn Pos: " << twoTurnMyPosition.x << " " << twoTurnMyPosition.y << endl;
        //cerr << "Two Turn Other Pos: " << twoTurnOtherPosition.x << " " << twoTurnOtherPosition.y << endl;

        float twoTurnDistance = (twoTurnMyPosition - twoTurnOtherPosition).Length();
        //cerr << "Two Turn Distance: " << twoTurnDistance << endl;

        if(twoTurnDistance < 800.0f)
        {
            collisionImminent = true;
            return true;
        }

        return false;
    }

    Vector2 ComputeIdealDirection(Vector2 _targetPosition, float _distanceToWp, float _angleToWp)
    {
        float LARGE_WP_OFFSET = 100.0f;
        float COMPENSATION_MULTIPLIER = 0.7f;

        Vector2 toNextNextWpDirection = (nextWaypoint->nextWp->position - nextWaypoint->position).Normalize();
        cerr << name << " NextWpDirection " << toNextNextWpDirection.x << " " << toNextNextWpDirection.y << endl;

        Vector2 toTargetDirection = (_targetPosition - position).Normalize();

        if(driftPossible)
        {
            cerr << name << " ToTarget Dir: " << toTargetDirection.x << " " << toTargetDirection.y << endl;
            cerr << name << " Current Speed: " << speedValue << endl;

            return nextWaypoint->nextWp->position;
        }

        else if(speedValue > 50.0f)
        {
            cerr << name << " ToTarget Dir: " << toTargetDirection.x << " " << toTargetDirection.y << endl;
            cerr << name << " Current Speed: " << speedValue << endl;

            Vector2 projectedPoint = (position + inertiaDirection).ProjectOnLine(position, toTargetDirection);
            Vector2 compensationVector = (projectedPoint - (position + inertiaDirection));

            cerr << name << "Compensation vector lentgh: " << compensationVector.Length() << endl;

            Vector2 newDirection = position + toNextNextWpDirection * LARGE_WP_OFFSET + (toTargetDirection * speedValue + compensationVector * COMPENSATION_MULTIPLIER) * 5;
            return newDirection;
        }
        else
        {
            return _targetPosition;
        }        
    }

    void ComputeBasicMultipliers()
    {
        const float MIN_DISTANCE_SLOWDOWN = 1200.0f;
        const float MAX_ANGLE_ENGINE = 145.0f;
        const float MIN_ANGLE_ENGINE = 75.0f;

        if(abs(nextCheckpointAngle) > MAX_ANGLE_ENGINE)
        {
            angleMultiplier = 0.0f;
        }

        else if(abs(nextCheckpointAngle) > MIN_ANGLE_ENGINE)
        {
            //Angle Multiplier slows down the larger the angle to next waypoint is
            float tempAngleMultiplier = (MIN_ANGLE_ENGINE / abs(nextCheckpointAngle) * (MIN_ANGLE_ENGINE / abs(nextCheckpointAngle)));

            //When dealing with large distances, slowing down costs more time than doing a large loop with keeping max speed
            float largeDistanceMultiplier = (nextCheckpointDist / MIN_DISTANCE_SLOWDOWN) * (nextCheckpointDist / MIN_DISTANCE_SLOWDOWN);

            angleMultiplier = clamp(tempAngleMultiplier * largeDistanceMultiplier, 0.0f, 1.0f);

            //Ignore speed change based solely on distance when having a large angle distance is present
            distanceMultiplier = 1.0f;
        }

        //Slowing down based on distance irrelevant with drift
        /*
        else
        {
            if(nextCheckpointDist * (100 / speedValue) < MIN_DISTANCE_SLOWDOWN)
            {
                distanceMultiplier = 0.2f + clamp((nextCheckpointDist / MIN_DISTANCE_SLOWDOWN), 0.0f, 1.0f) * 0.8f;
            }
        }
        */

        if(!usedBoost && nextCheckpointDist > 4000 && abs(nextCheckpointAngle) < 5)
        {
            boostAvailable = true;

            cerr << "****************"<< name << " - Boost available" << endl;

            //cout << checkPointCoords_X.at(player1.nextCheckpointID) << " " << checkPointCoords_Y.at(player1.nextCheckpointID) << " BOOST" << " Anakin" << endl;
        }
    }

    bool ComputeAvoidanceMultiplier(PodRacer _enemy)
    {
        const float ENEMY_DISTANCE_THREAT = 3000.0f;

        float enemyDistanceToPlayer = position.Distance(_enemy.GetPosition());
        Vector2 enemyDirectAttackVector = (position - _enemy.GetPosition()).Normalize();
        float enemyDirectAttackAngle = enemyDirectAttackVector.Angle(_enemy.GetInertialDirection());

        if(abs(enemyDirectAttackAngle) < 15.0f && enemyDistanceToPlayer < ENEMY_DISTANCE_THREAT)
        {
            avoidanceMultiplier = ENEMY_DISTANCE_THREAT / enemyDistanceToPlayer;
            return true;
        }

        avoidanceMultiplier = 1.0f;
        return false;
    }

    void ApplyMovement()
    {
        if(boostAvailable == true && usedBoost == false)
        {
            usedBoost = true;

            cerr << "****************"<< name << " - Tried to use boost" << endl;

            cout << checkPointCoords_X.at(nextCheckpointID) << " " << checkPointCoords_Y.at(nextCheckpointID) << " BOOST " << name << endl;
        }

        else
        {
            finalDirection = ComputeIdealDirection(nextCheckPointPos, nextCheckpointDist, nextCheckpointAngle);

            finalMultiplier = distanceMultiplier * angleMultiplier * avoidanceMultiplier * driftMultiplier;

            //cerr << "Dist Multiplier: " << distMultiplier << endl;
            //cerr << "Angle Multiplier: " << angleMultiplier << endl;
            //cerr << "Avoidance Multiplier: " << avoidanceMultiplier << endl;

            float finalSpeed = round(clamp(100 * finalMultiplier, 0.0f, 100.0f));

            if(collisionImminent)
            {
                cout << round(finalDirection.x) << " " << round(finalDirection.y) << " SHIELD" << endl;
            }
            else
            {
                cout << round(finalDirection.x) << " " << round(finalDirection.y) << " " << finalSpeed << " " << name << endl;
            }
        }
    }
};

class PodRanking
{
    public:

    BasePod* pod;
    int nextWpId = 1;
    int ranking = 1;
    int wpPassed = 0;
    float distanceToNextWp = 0;
};

class RankingManager
{
    private:

    static RankingManager *instance;

    public:

    vector<PodRanking> allPods;

    static RankingManager *getInstance() 
    {
        if (!instance)
        {
            instance = new RankingManager;
        }
            
        return instance;
    }

    public:

    RankingManager()
    {
        allPods.reserve(4);
    }

    void Init()
    {
        
    }

    void AddToPods(BasePod &_newPod)
    {
        allPods.push_back(PodRanking());
        allPods.back().pod = &_newPod;

        cerr << "Added pod " << _newPod.name << " / total count: " << allPods.size() << endl;
    }

    void UpdatePod(BasePod &_newPod)
    {
        for (std::vector<PodRanking>::iterator it = allPods.begin() ; it != allPods.end(); ++it)
        {
            if(it->pod == &_newPod)
            {
                if(_newPod.nextCheckpointID != it->nextWpId)
                {
                    it->nextWpId = _newPod.nextCheckpointID;
                    it->wpPassed++;
                }
                it->distanceToNextWp = _newPod.nextMajorCheckPointDist;
            }
        }
    }

    void SortRankings()
    {
        /*
        cerr << "Rankings: (count:" << allPods.size() << ") :";

        for (std::vector<PodRanking>::iterator it = allPods.begin() ; it != allPods.end(); ++it)
        {
            cerr << it->pod->name << "(" << it->wpPassed << ") (" <<it->distanceToNextWp << ") ";
        }

        cerr <<endl;
        */
        
        sort(allPods.begin(), allPods.end(), PodRankingSorter);

        cerr << "Rankings Sorted: ";

        for (std::vector<PodRanking>::iterator it = allPods.begin() ; it != allPods.end(); ++it)
        {
            cerr << it->pod->name << " ";
        }

        cerr <<endl;
        
    }

    int GetRanking(BasePod _pod)
    {
        for (std::vector<PodRanking>::iterator it = allPods.begin() ; it != allPods.end(); ++it)
        {
            
        }

        return 0;
    }

    BasePod* GetTopRankingEnemyPod()
    {
        for (std::vector<PodRanking>::iterator it = allPods.begin() ; it != allPods.end(); ++it)
        {
            if(it->pod->podFaction == BasePod::Faction::Enemy)
            {
                return it->pod;
            }
        }

        return allPods.at(0).pod;
    }

    static bool PodRankingSorter(PodRanking _pod1, PodRanking _pod2)
    {
        if(_pod1.wpPassed > _pod2.wpPassed)
        {
            return true;
        }
        else if(_pod1.wpPassed == _pod2.wpPassed)
        {
            if(_pod1.distanceToNextWp < _pod2.distanceToNextWp)
            {
                return true;
            }
        }
        
        return false;
    }
};

class PodFighter : public BasePod
{
    public:

    bool firstTurn = true;

    Vector2 targetInterceptPosition;
    float toTargetAngle;
    float angleMultiplier;
    float distMultiplier = 1.0f;

    PodFighter()
    {
        podFaction = Faction::Friendly;
    }

    void UpdateFighter(RankingManager &rankManager , Vector2 _newPos, Vector2 speed, float _rotationAngle, int _nextCheckpointID)
    {
        float DISTANCE_DIFF_FOR_SKIP_ATTACK = 1500.0f;
        float DISTANCE_SLOWDOWN = 2500.0f;

        //Calculations

        nextCheckpointID = _nextCheckpointID;

        nextCheckPointPos = Vector2(checkPointCoords_X.at(nextCheckpointID), checkPointCoords_Y.at(nextCheckpointID));
        myRotationAngle = _rotationAngle;
        rotationDirection = Vector2(cos(myRotationAngle * M_PI / 180.0f), sin(myRotationAngle * M_PI / 180.0f));

        //cerr << "Rotation angle: " << myRotationAngle << endl;
        //cerr << "Rotation Direction: " << rotationDirection.x << " " << rotationDirection.y << endl;

        position = _newPos;
        inertiaDirection = speed;
        speedValue = speed.Length();

        //cerr << "Next CP Pos: " << nextCheckPointPos.x << " " << nextCheckPointPos.y  << " | My Pos: " << position.x << " " << position.y << endl;

        nextCheckpointDirection = (nextCheckPointPos - position);
        nextCheckpointDirection.Normalize();

        //cerr << "Next CP Dir: " << nextCheckpointDirection.x << " " << nextCheckpointDirection.y  << endl;

        nextCheckpointAngle = nextCheckpointDirection.Angle(rotationDirection);

        //cerr << "Next CP Angle: " << nextCheckpointAngle << endl << endl;

        nextCheckpointDist = (nextCheckPointPos - position).Length();
        nextMajorCheckPointDist = nextCheckpointDist;

        position = _newPos;

        BasePod* targetEnemy = rankManager.GetTopRankingEnemyPod();

        cerr << name << " targetting " << targetEnemy->name << endl;

        if(firstTurn == false)
        {
            float distanceToEnemy = (position - targetEnemy->position).Length();

            float enemyDistanceToNextWp = targetEnemy->nextCheckpointDist;
            float myDistanceToEnemyNextWp = (targetEnemy->nextCheckPointPos - position).Length();
            float distanceDiff = myDistanceToEnemyNextWp - enemyDistanceToNextWp;

            if(distanceDiff > DISTANCE_DIFF_FOR_SKIP_ATTACK)
            {
                Vector2 nextNextWpDir = (targetEnemy->nextMajorWp->nextMajorWp->position - targetEnemy->nextMajorWp->position).Normalize();
                targetInterceptPosition = targetEnemy->nextMajorWp->position + nextNextWpDir * distanceDiff;

                float targetInterceptDist = (targetInterceptPosition - position).Length();

                if(targetInterceptDist < DISTANCE_SLOWDOWN / 2)
                {
                    distMultiplier = 0.0f;
                    targetInterceptPosition = targetEnemy->position + targetEnemy->inertiaDirection;
                }

                else
                {
                    distMultiplier = clamp(targetInterceptDist / DISTANCE_SLOWDOWN, 0.0f, 1.0f);
                }

                Vector2 toTargetDir = (targetInterceptPosition - position).Normalize();
                toTargetAngle = toTargetDir.Angle(rotationDirection);
                
            }

            else
            {
                Vector2 toTargetDir = (targetInterceptPosition - position).Normalize();
                toTargetAngle = toTargetDir.Angle(rotationDirection);

                cerr << "Target Enemy pos: " << targetEnemy->position.x << " " << targetEnemy->position.y;
                cerr << " Dir: " << targetEnemy->inertiaDirection.x << " " << targetEnemy->inertiaDirection.y;
                cerr << " Dist " << distanceToEnemy << " Speed " << speedValue << endl << endl;

                if(speedValue > 10.0f)
                {
                    targetInterceptPosition = targetEnemy->position + (targetEnemy->inertiaDirection / 4) * (distanceToEnemy / speedValue) + targetEnemy->nextCheckpointDirection * targetEnemy->nextCheckpointDist / 4;
                }
                else
                {
                    targetInterceptPosition = targetEnemy->position;
                }

                distMultiplier = 1.0f;
            }
        }

        else
        {
            firstTurn = false;
            targetInterceptPosition = position + rotationDirection * 300.0f;
            toTargetAngle = 0.0f;
        } 
    }

    void ComputeBasicMultipliers()
    {
        const float MIN_DISTANCE_SLOWDOWN = 1200.0f;
        const float MAX_ANGLE_ENGINE = 145.0f;
        const float MIN_ANGLE_ENGINE = 75.0f;

        if(abs(toTargetAngle) > MAX_ANGLE_ENGINE)
        {
            angleMultiplier = 0.0f;
        }

        else if(abs(toTargetAngle) > MIN_ANGLE_ENGINE)
        {
            //Angle Multiplier slows down the larger the angle to next waypoint is
            float tempAngleMultiplier = (MIN_ANGLE_ENGINE / abs(toTargetAngle) * (MIN_ANGLE_ENGINE / abs(toTargetAngle)));

            //When dealing with large distances, slowing down costs more time than doing a large loop with keeping max speed
            float largeDistanceMultiplier = (nextCheckpointDist / MIN_DISTANCE_SLOWDOWN) * (nextCheckpointDist / MIN_DISTANCE_SLOWDOWN);

            angleMultiplier = clamp(tempAngleMultiplier * largeDistanceMultiplier, 0.0f, 1.0f);
        }

        else
        {
            angleMultiplier = 1.0f;
        }
    }

    Vector2 ComputeIdealDirection(Vector2 _targetPosition)
    {
        float COMPENSATION_MULTIPLIER = 0.7f;

        Vector2 toTargetDirection = (_targetPosition - position).Normalize();

        if(speedValue > 50.0f)
        {
            cerr << name << " ToTarget Dir: " << toTargetDirection.x << " " << toTargetDirection.y << endl;
            cerr << name << " Current Speed: " << speedValue << endl;

            Vector2 projectedPoint = (position + inertiaDirection).ProjectOnLine(position, toTargetDirection);
            Vector2 compensationVector = (projectedPoint - (position + inertiaDirection));

            cerr << name << "Compensation vector lentgh: " << compensationVector.Length() << endl;

            Vector2 newDirection = position + (toTargetDirection * speedValue + compensationVector * COMPENSATION_MULTIPLIER) * 5;
            return newDirection;
        }
        else
        {
            return _targetPosition;
        }        
    }

    void ApplyMovement()
    {
        const float MAX_SPEED = 400.0f;

        float finalMultiplier = angleMultiplier * distMultiplier;
        float finalSpeed = round(clamp(100 * finalMultiplier, 0.0f, 100.0f));

        Vector2 finalDirection = ComputeIdealDirection(targetInterceptPosition);

        if(speedValue < MAX_SPEED)
        {
            cout << round(finalDirection.x) << " " << round(finalDirection.y) << " " << finalSpeed << " " << name << endl;
        }
        else
        {
            cout << round(finalDirection.x) << " " << round(finalDirection.y) << " " << 0.0f << " " << name << endl;
        }
    }
};

RankingManager *RankingManager::instance = 0;

int main()
{
    bool isInit = false;

    PodRacer player1;
    PodFighter player2;
    EnemyPod enemy1;
    EnemyPod enemy2;

    int laps;
    int checkpointCount;
    vector<int> checkPointCoords_X;
    vector<int> checkPointCoords_Y;

    vector<MajorWaypoint> majorWaypoints;
    vector<Waypoint> otherWaypoints;

    RankingManager *rankManager = RankingManager::getInstance();

    // game loop
    while (1) {

        int checkpointX;
        int checkpointY;

        int x;
        int y;
        int vx;
        int vy;
        int angle;
        int nextCheckPointId;

        if(isInit == false)
        {
            rankManager->Init();

            majorWaypoints.reserve(8);
            otherWaypoints.reserve(32);
            checkPointCoords_X.reserve(8);
            checkPointCoords_Y.reserve(8);

            cin >> laps; cin.ignore();
            cin >> checkpointCount; cin.ignore();

            //cerr << "Check Count: " << checkpointCount << endl;

            for(int i = 0; i < checkpointCount; i++)
            {
                cin >> checkpointX >> checkpointY; cin.ignore();

                checkPointCoords_X.push_back(checkpointX);
                checkPointCoords_Y.push_back(checkpointY);

                majorWaypoints.push_back(MajorWaypoint());

                majorWaypoints.at(i).checkPointId = i;
                majorWaypoints.at(i).position = Vector2(checkpointX, checkpointY);
            }

            cerr << "Major Wp count: " << majorWaypoints.size() << endl;

            for(int i = 0; i < majorWaypoints.size(); i++)
            {
                if(i == 0)
                {
                    majorWaypoints.at(0).prevMajorWp = &majorWaypoints.at(majorWaypoints.size() - 1);
                    majorWaypoints.at(0).prevWp = &majorWaypoints.at(majorWaypoints.size() - 1);

                    majorWaypoints.at(majorWaypoints.size() - 1).nextMajorWp = &majorWaypoints.at(i);
                    majorWaypoints.at(majorWaypoints.size() - 1).nextWp = &majorWaypoints.at(i);
                }

                else
                {
                    majorWaypoints.at(i).prevMajorWp = &majorWaypoints.at(i-1);
                    majorWaypoints.at(i).prevWp = &majorWaypoints.at(i-1);

                    majorWaypoints.at(i-1).nextMajorWp = &majorWaypoints.at(i);
                    majorWaypoints.at(i-1).nextWp = &majorWaypoints.at(i);
                }
            }
            cerr << endl;

            for(int i = 0; i < majorWaypoints.size(); i++)
            {
                cerr << "Major Waypoint at: " << majorWaypoints.at(i).position.x << " " <<majorWaypoints.at(i).position.y << endl;

                majorWaypoints.at(i).SplitWithNextWp(otherWaypoints);
            }

            /*
            
            Waypoint *initialWaypoint = &majorWaypoints.at(0);
            Waypoint *currentWaypoint = &majorWaypoints.at(0);

            cerr << "<<" << initialWaypoint << "," << currentWaypoint << ">>" << endl;
            
            while(true)
            {
                cerr << "(" <<currentWaypoint->position.x << "," <<currentWaypoint->position.y << ")" << " / Depth: " << currentWaypoint->subWaypointDepth << endl;
                //cerr << "(" <<currentWaypoint->nextWp->position.x << "," <<currentWaypoint->nextWp->position.y << ")" << " / Depth: " << currentWaypoint->subWaypointDepth << endl;
                
                currentWaypoint = currentWaypoint->nextWp;

                cerr << "<<" << initialWaypoint << "," << currentWaypoint << ">>" << endl;

                if(initialWaypoint == currentWaypoint)
                {
                    break;
                }
            }

            */
            

        }

        //Update racers / frame

        cin >> x >> y >> vx >> vy >> angle >> nextCheckPointId; cin.ignore();
        if(isInit == false)
        {
            player1.InitPod(checkPointCoords_X, checkPointCoords_Y, majorWaypoints, otherWaypoints, "Anakin");
            rankManager->AddToPods(player1);
        }
        player1.UpdateRacer_waypoints(Vector2(x, y), Vector2(vx, vy), angle, nextCheckPointId);
        rankManager->UpdatePod(player1);

        cin >> x >> y >> vx >> vy >> angle >> nextCheckPointId; cin.ignore();
        if(isInit == false)
        {
            player2.InitPod(checkPointCoords_X, checkPointCoords_Y, majorWaypoints, otherWaypoints, "Obi-Wan");
            rankManager->AddToPods(player2);
        }
        player2.UpdateFighter(*rankManager, Vector2(x, y), Vector2(vx, vy), angle, nextCheckPointId);

        cin >> x >> y >> vx >> vy >> angle >> nextCheckPointId; cin.ignore();
        if(isInit == false)
        {
            enemy1.InitPod(checkPointCoords_X, checkPointCoords_Y, majorWaypoints, otherWaypoints, "Enemy 1");
            rankManager->AddToPods(enemy1);
        }
        enemy1.UpdateEnemy(Vector2(x, y), Vector2(vx, vy), angle, nextCheckPointId);
        rankManager->UpdatePod(enemy1);

        cin >> x >> y >> vx >> vy >> angle >> nextCheckPointId; cin.ignore();
        if(isInit == false)
        {
            enemy2.InitPod(checkPointCoords_X, checkPointCoords_Y, majorWaypoints, otherWaypoints, "Enemy 2");
            rankManager->AddToPods(enemy2);
        }
        enemy2.UpdateEnemy(Vector2(x, y), Vector2(vx, vy), angle, nextCheckPointId);
        rankManager->UpdatePod(enemy2);

        rankManager->UpdatePod(player2);


        rankManager->SortRankings();

        if(isInit == false)
        {
            isInit = true;
        }

        //Player 1

        //player1.ComputeAvoidanceMultiplier(enemy1);
        //player1.ComputeAvoidanceMultiplier(enemy2);

        player1.ComputeBasicMultipliers();

        //player1.DetectCollisionImminent(enemy1);
        //player1.DetectCollisionImminent(enemy2);

        player1.DetectDriftPossible();

        player1.ApplyMovement();

        //Player 2

        //player2.ComputeAvoidanceMultiplier(enemy1);
        //player2.ComputeAvoidanceMultiplier(enemy2);

        player2.ComputeBasicMultipliers();

        //player2.DetectCollisionImminent(enemy1);
        //player2.DetectCollisionImminent(enemy2);

        //player2.DetectDriftPossible();

        player2.ApplyMovement();
    }
}
