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
    bool isTempWp = false;
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

    bool collisionImminent;

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

        if(_nextCheckpointID != nextMajorWp->checkPointId)
        {
            prevMajorWp = nextMajorWp;
            nextMajorWp = nextMajorWp->nextMajorWp;
            cerr << name << " Checkpoint change: -> next wp: " << nextMajorWp->checkPointId << endl;
        }

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

    bool firstFrame = true;

    public: 

    vector<Vector2> positionsToAvoid;
    Vector2 calculatedTargetPosition;

    Waypoint* tempRacerWaypoint;

    bool needsAvoidance;

    PodRacer()
    {
        podFaction = Faction::Friendly;
        positionsToAvoid.reserve(3);
    }

    Vector2 GetPosition()
    {
        return position;
    }

    Vector2 GetInertialDirection()
    {
        return inertiaDirection;
    }

    void InitPod(vector<int> &_coordsX, vector<int> &_coordsY, vector<MajorWaypoint> &_majorWaypoints, vector<Waypoint> &_allWaypoints, Waypoint &_tempRacerWp, string _name)
    {
        BasePod::InitPod(_coordsX, _coordsY, _majorWaypoints, _allWaypoints, _name);

        tempRacerWaypoint = &_tempRacerWp;

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

    void UpdateRacer_waypoints(Vector2 _newPos, Vector2 speed, float _rotationAngle, int _nextCheckpointID, vector<BasePod*> &_otherRacers)
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
        positionsToAvoid.clear();

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

        Vector2 currentAvoidancePos;
        float currentOffsetLength = 0.0f;
        needsAvoidance = false;

        if(firstFrame == false)
        {
            for(int i = 0; i < _otherRacers.size(); i++)
            {
                cerr << "Other Racer: " << _otherRacers.at(i)->name << " / ";
            }
            cerr << endl;

            //bool canAvoid = CheckMovementAvoidance(currentAvoidancePos, _otherRacers, currentOffsetLength, needsAvoidance);
            bool canAvoid = false;

            cerr << name << " Needs avoid: " << needsAvoidance << " Can avoid: " << canAvoid << " (d:" << nextCheckpointDist <<")" << endl;

            if(needsAvoidance && canAvoid && nextWaypoint->isTempWp == false)
            {
                tempRacerWaypoint->isTempWp = true;
                tempRacerWaypoint->isMajorWp = false;
                tempRacerWaypoint->position = currentAvoidancePos;
                tempRacerWaypoint->prevWp = nextWaypoint->prevWp;

                if(nextWaypoint->isMajorWp)
                {
                    tempRacerWaypoint->nextWp = nextWaypoint;
                }
                else
                { 
                    tempRacerWaypoint->nextWp = nextWaypoint->nextWp;
                }

                tempRacerWaypoint->distanceToNextWp = (tempRacerWaypoint->nextWp->position - tempRacerWaypoint->position).Length();

                cerr << endl << "---- Temp Wp - Pos: (" << tempRacerWaypoint->position.x << "," << tempRacerWaypoint->position.y << ") / dist: " << tempRacerWaypoint->distanceToNextWp << endl << endl;

                nextWaypoint = tempRacerWaypoint;
            }

            else
            {
                bool isPathToMainClearNow = false;

                if(nextWaypoint->isTempWp == true)
                {
                    canAvoid = CheckMovementAvoidance(currentAvoidancePos, _otherRacers, currentOffsetLength, isPathToMainClearNow);

                    if(isPathToMainClearNow)
                    {
                        nextWaypoint = nextWaypoint->nextWp;
                    }
                    
                }
            }
        }

        nextCheckPointPos = nextWaypoint->position;
        myRotationAngle = _rotationAngle;
        rotationDirection = Vector2(cos(myRotationAngle * M_PI / 180.0f), sin(myRotationAngle * M_PI / 180.0f));

        //cerr << "Next CP Pos: " << nextCheckPointPos.x << " " << nextCheckPointPos.y  << " | My Pos: " << position.x << " " << position.y << endl;

        //Avoidance Stuff



        nextCheckpointDirection = (nextCheckPointPos - position);
        nextCheckpointDirection.Normalize();

        //cerr << "Next CP Dir: " << nextCheckpointDirection.x << " " << nextCheckpointDirection.y  << endl;

        nextCheckpointAngle = nextCheckpointDirection.Angle(rotationDirection);

        //cerr << "Next CP Angle: " << nextCheckpointAngle << endl << endl;

        nextCheckpointDist = (nextCheckPointPos - position).Length();
        nextMajorCheckPointDist = (nextMajorWp->position - position).Length();

        firstFrame = false;
    }

    void DetectDriftPossible()
    {
        float DISTANCE_TO_SPEED_RATIO = 10.0f;
        float MIN_SPEED_FOR_DRIFT_START = 300.0f;
        float DIST_TO_SPEED_RATIO_FOR_CANCEL = 5.5f;
        float MAX_ANGLE_MULTIPLIER_FOR_DRIFT_BURN = 0.7f;
        float MAX_ANGLE_MULTIPLIER_FOR_DRIFT_END = 0.9f;

        Vector2 toNextNextWpDirection = (nextWaypoint->nextWp->position - nextWaypoint->position).Normalize();

        float angleToTestAgainst_EndDrift = atan2(600.0f * MAX_ANGLE_MULTIPLIER_FOR_DRIFT_END, nextCheckpointDist) * 180.0f / M_PI;
        float angleToTestAgainstAbs_EndDrift = abs(angleToTestAgainst_EndDrift);

        float angleToTestAgainst_Burn = atan2(600.0f * MAX_ANGLE_MULTIPLIER_FOR_DRIFT_BURN, nextCheckpointDist) * 180.0f / M_PI;
        float angleToTestAgainstAbs_Burn = abs(angleToTestAgainst_Burn);

        float angleToNexNextWp = toNextNextWpDirection.Angle(nextCheckpointDirection);
        float inertialAngle = inertiaDirection.Angle(nextCheckpointDirection);
        float inertialAngleAbs = abs(inertialAngle);

        //cerr << name << " Next Cp dist: " << nextCheckpointDist << " / Angle: " << inertialAngleAbs << " / Angle to test: " << angleToTestAgainstAbs_EndDrift <<endl;

        if(inertialAngleAbs < angleToTestAgainstAbs_EndDrift)
        {
            bool sameSign = (angleToNexNextWp > 0 && inertialAngle > 0) || (angleToNexNextWp < 0 && inertialAngle < 0);

            if(driftPossible == false)
            {
                if(speedValue > MIN_SPEED_FOR_DRIFT_START && nextCheckpointDist < speedValue * DISTANCE_TO_SPEED_RATIO * clamp(speedValue / 400.0f, 1.0f, 1.5f))
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

        //cerr << name << "Dift Possible: " << driftPossible << endl;
    }

    void DetectPushThrough(BasePod &_targetPod)
    {
        float DISTANCE_TO_SPEED_RATIO = 5.0f;

        if(nextWaypoint->isMajorWp == true && nextCheckpointDist < speedValue * DISTANCE_TO_SPEED_RATIO * clamp(speedValue / 400.0f, 1.0f, 1.5f))
        {
            
            Vector2 toTargetPodDirection = (_targetPod.position - position).Normalize();
            Vector2 projectedPosition = _targetPod.position.ProjectOnLine(position, nextCheckpointDirection);
            Vector2 toProjectionDirection = (projectedPosition - _targetPod.position).Normalize();
            float projectedDistance = (projectedPosition - _targetPod.position).Length();

            float angleToTargetPod = abs(nextCheckpointDirection.Angle(toTargetPodDirection));
            float angleToProjection = abs(toProjectionDirection.Angle(_targetPod.inertiaDirection));

            //Already in way
            if(angleToTargetPod < 25 && projectedDistance < 600.0f)
            {
                //Check if going to be out of way
                float timeToCollision = projectedDistance / speedValue;

                Vector2 enemyInertiaNorm = _targetPod.inertiaDirection;
                enemyInertiaNorm.Normalize();

                Vector2 newEnemyPosition = _targetPod.position +  ((enemyInertiaNorm +_targetPod.rotationDirection) / 2) * timeToCollision * _targetPod.speedValue;

                Vector2 projectedNew = newEnemyPosition.ProjectOnLine(position, nextCheckpointDirection);
                float distToNewProjected = (projectedNew - position).Length();
                if((projectedNew - newEnemyPosition).Length() < 600.0f && nextCheckpointDist > distToNewProjected - 400.0f)
                {
                    driftPossible = false;
                    driftMultiplier = 1.0f;

                    cerr << "//////////// " << name << " - PushThrough!";
                }
            }
        }
    }

    bool CheckMovementAvoidance(Vector2 &currentAvoidancePos, vector<BasePod*> &_otherRacers, float &currentOffsetLengh, bool &_needsAvoidance)
    {
        if(currentOffsetLengh > nextCheckpointDist)
        {
            currentAvoidancePos = nextCheckPointPos;
            //cerr << endl << " +++++++ Too large Offset " << endl;

            return false;
        }

        if(nextWaypoint->isMajorWp && nextCheckpointDist < 2000.0f)
        {
            currentAvoidancePos = nextCheckPointPos;
            //cerr << endl << " +++++++ Too close to wp " << endl;

            return false;
        }

        Vector2 projectedNextNext = nextWaypoint->nextWp->position.ProjectOnLine(position, (nextWaypoint->position - position).Normalize());
        Vector2 offsetTryDirection = (projectedNextNext - nextWaypoint->nextWp->position).Normalize();

        currentAvoidancePos = nextCheckPointPos + offsetTryDirection * currentOffsetLengh;

        //cerr << "Proj init: " << projectedNextNext.x << "," << projectedNextNext.y << " / Current avoid pos: " << currentAvoidancePos.x << "," << currentAvoidancePos.y << endl;

        Vector2 triedMoveDirection = (currentAvoidancePos - position).Normalize();

        for (std::vector<BasePod*>::iterator it = _otherRacers.begin() ; it != _otherRacers.end(); ++it)
        {
            float distanceToOtherTarget = ((*it)->position - position).Length();

            float angleToOther = triedMoveDirection.Angle(((*it)->position - position).Normalize());

            Vector2 otherProjectedPos = (*it)->position.ProjectOnLine(position, triedMoveDirection);

            if(distanceToOtherTarget < nextCheckpointDist && (otherProjectedPos - (*it)->position).Length() < 800.0f)
            {
                //positionsToAvoid.push_back(it->position);
                
                currentOffsetLengh += nextCheckpointDist / 10.0f;
                _needsAvoidance = true;

                return CheckMovementAvoidance(currentAvoidancePos, _otherRacers, currentOffsetLengh, _needsAvoidance);
            }
        }

        return true;
    }

    bool DetectCollisionImminent(BasePod _otherRacer)
    {
        float distanceToOtherTarget = (_otherRacer.position - position).Length();

        float angleToOther = inertiaDirection.Angle((_otherRacer.position - position).Normalize());
        float speedDiff = inertiaDirection.Length() - _otherRacer.inertiaDirection.Length();
        float directionAngleDiff = inertiaDirection.Angle(_otherRacer.inertiaDirection);

        //cerr << "Dist to other: " << distanceToOtherTarget << endl;
        //cerr << "Player speed: " << speedValue << endl;
        //cerr << "Enemy speed: " << _otherRacer.speedValue << endl;

        Vector2 twoTurnMyPosition = position + inertiaDirection * 1 + rotationDirection * 100.0f;
        Vector2 twoTurnOtherPosition = _otherRacer.position + _otherRacer.inertiaDirection * 1.5f + _otherRacer.rotationDirection * 100.0f;
        //cerr << "Two Turn Pos: " << twoTurnMyPosition.x << " " << twoTurnMyPosition.y << endl;
        //cerr << "Two Turn Other Pos: " << twoTurnOtherPosition.x << " " << twoTurnOtherPosition.y << endl;

        float twoTurnDistance = (twoTurnMyPosition - twoTurnOtherPosition).Length();
        //cerr << "Two Turn Distance: " << twoTurnDistance << endl;

        if(twoTurnDistance < 800.0f && ((angleToOther < 60.0f && speedDiff > 150.0f) || directionAngleDiff > 90.0f))
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
        //cerr << name << " NextWpDirection " << toNextNextWpDirection.x << " " << toNextNextWpDirection.y << endl;

        Vector2 toTargetDirection = (_targetPosition - position).Normalize();

        if(driftPossible)
        {
            //cerr << name << " ToTarget Dir: " << toTargetDirection.x << " " << toTargetDirection.y << endl;
            //cerr << name << " Current Speed: " << speedValue << endl;

            return nextWaypoint->nextWp->position;
        }

        else if(speedValue > 50.0f)
        {
            //cerr << name << " ToTarget Dir: " << toTargetDirection.x << " " << toTargetDirection.y << endl;
            //cerr << name << " Current Speed: " << speedValue << endl;

            Vector2 projectedPoint = (position + inertiaDirection).ProjectOnLine(position, toTargetDirection);
            Vector2 compensationVector = (projectedPoint - (position + inertiaDirection));

            //cerr << name << "Compensation vector lentgh: " << compensationVector.Length() << endl;

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
        const float MAX_ANGLE_ENGINE = 125.0f;
        const float MIN_ANGLE_ENGINE = 65.0f;

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

            //cerr << "****************"<< name << " - Tried to use boost" << endl;

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

    enum RaceStatus
    {
        BEHIND,
        CLOSE_BEHIND,
        CLOSE_AHEAD,
        AHEAD_BY_A_LOT
    };

    RaceStatus status;

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
        float DISTANCE_LARGE_LEAD = 7000.0f;

        /*
        cerr << "Rankings: (count:" << allPods.size() << ") :";

        for (std::vector<PodRanking>::iterator it = allPods.begin() ; it != allPods.end(); ++it)
        {
            cerr << it->pod->name << "(" << it->wpPassed << ") (" <<it->distanceToNextWp << ") ";
        }

        cerr <<endl;
        */
        
        sort(allPods.begin(), allPods.end(), PodRankingSorter);

        /*
        cerr << "Rankings Sorted: ";
        for (std::vector<PodRanking>::iterator it = allPods.begin() ; it != allPods.end(); ++it)
        {
            cerr << it->pod->name << " ";
        }

        cerr <<endl;
        */

        float distanceBehind = 0.0f;
        
        //Update Race Status

        BasePod* firstPlace = allPods.begin()->pod;
        BasePod* firstChaser;

        //We're behind
        if(allPods.begin()->pod->podFaction == PodRacer::Faction::Enemy)
        {
            for (std::vector<PodRanking>::iterator it = allPods.begin() ; it != allPods.end(); ++it)
            {
                if(it->pod->podFaction == PodRacer::Faction::Friendly)
                {
                    firstChaser = it->pod;
                    break;
                }
            }
        }
        else
        {
            for (std::vector<PodRanking>::iterator it = allPods.begin() ; it != allPods.end(); ++it)
            {
                if(it->pod->podFaction == PodRacer::Faction::Enemy)
                {
                    firstChaser = it->pod;
                    break;
                }
            }
        }

        MajorWaypoint* currentWaypoint = firstChaser->nextMajorWp;

        int maxWpCompleted = GetWpCompeted(*firstPlace);
        int currentWpCompleted = GetWpCompeted(*firstChaser);

        if(currentWpCompleted < maxWpCompleted)
        {
            distanceBehind += firstChaser->nextCheckpointDist;

            currentWpCompleted++;
            currentWaypoint = currentWaypoint->nextMajorWp;

            while(currentWpCompleted < maxWpCompleted)
            {
                distanceBehind += (currentWaypoint->position - currentWaypoint->prevMajorWp->position).Length();

                currentWpCompleted++;
                currentWaypoint = currentWaypoint->nextMajorWp;
            }

            distanceBehind += (currentWaypoint->position - currentWaypoint->prevMajorWp->position).Length() - firstPlace->nextCheckpointDist;
        }

        else
        {
            distanceBehind = firstChaser->nextMajorCheckPointDist - firstPlace->nextMajorCheckPointDist;
        }

        if(distanceBehind < DISTANCE_LARGE_LEAD)
        {
            if(firstPlace->podFaction == PodRacer::Faction::Friendly)
            {
                status = RaceStatus::CLOSE_AHEAD;
            }
            else
            {
                status = RaceStatus::CLOSE_BEHIND;
            }
        }
        else
        {
            if(firstPlace->podFaction == PodRacer::Faction::Friendly)
            {
                status = RaceStatus::AHEAD_BY_A_LOT;
            }
            else
            {
                status = RaceStatus::BEHIND;
            }
        }
        
        cerr << "*************" << distanceBehind << " / " << status << "*************"  <<endl;

    }

    int GetRanking(BasePod _pod)
    {
        for (std::vector<PodRanking>::iterator it = allPods.begin() ; it != allPods.end(); ++it)
        {
            
        }

        return 0;
    }

    int GetWpCompeted(BasePod &_pod)
    {
        for (std::vector<PodRanking>::iterator it = allPods.begin() ; it != allPods.end(); ++it)
        {
            if(it->pod == &_pod)
            {
                return it->wpPassed;
            }
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

    bool friendlyCollisionAvoidanceActive = false;
    Vector2 friendlyAvoidanceTempPosition;

    PodFighter()
    {
        podFaction = Faction::Friendly;
    }

    void UpdateFighter(RankingManager &rankManager , Vector2 _newPos, Vector2 speed, float _rotationAngle, int _nextCheckpointID)
    {
        float DISTANCE_DIFF_FOR_SKIP_ATTACK = 700.0f;
        float DISTANCE_SLOWDOWN = 4000.0f;
        float ANGLE_FOR_SKIP_ATTACK_BEHIND = 45.0f;
        float ENEMY_TIME_TO_WP_FOR_STOP = 4.0f;

        collisionImminent = false;
        friendlyCollisionAvoidanceActive = false;

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
            Vector2 directionToEnemyNorm = (targetEnemy->position -position).Normalize();
            Vector2 directiontoEnemyWpNorm = (targetEnemy->nextCheckPointPos - position).Normalize();
            float distanceToEnemy = (position - targetEnemy->position).Length();

            float enemyDistanceToNextWp = targetEnemy->nextMajorCheckPointDist;
            float myDistanceToEnemyNextWp = (targetEnemy->nextCheckPointPos - position).Length();
            float distanceDiff = myDistanceToEnemyNextWp - enemyDistanceToNextWp;

            cerr << "--- MD: " << myDistanceToEnemyNextWp << "--- ED: "<< enemyDistanceToNextWp << " --- Distance diff: " << distanceDiff << endl;

            Vector2 averageEnemyEstimatedMovement = targetEnemy->rotationDirection;

            float inertiaMultiplier = distanceToEnemy / 2000.0f;
            float distanceToWpMultiplier = enemyDistanceToNextWp / 3000.0f;

            Vector2 enemyInertialNormalized = targetEnemy->inertiaDirection;

            if(enemyInertialNormalized.x > 10.0f || enemyInertialNormalized.y > 10.0f )
            {
                enemyInertialNormalized.Normalize();
                averageEnemyEstimatedMovement = (enemyInertialNormalized * inertiaMultiplier + targetEnemy->rotationDirection + targetEnemy->nextCheckpointDirection * distanceToWpMultiplier).Normalize();
            }

            if(distanceDiff > DISTANCE_DIFF_FOR_SKIP_ATTACK)
            {
                Vector2 nextNextWpDir = (targetEnemy->nextMajorWp->nextMajorWp->position - targetEnemy->nextMajorWp->position).Normalize();
                float nextNextWpDist = (targetEnemy->nextMajorWp->nextMajorWp->position - targetEnemy->nextMajorWp->position).Length();
                targetInterceptPosition = targetEnemy->nextMajorWp->position + nextNextWpDir * nextNextWpDist / 1.5f;

                float targetInterceptDist = (targetInterceptPosition - position).Length();

                if(targetInterceptDist < DISTANCE_SLOWDOWN / 3 || enemyDistanceToNextWp < ENEMY_TIME_TO_WP_FOR_STOP * targetEnemy->speedValue)
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
                distMultiplier = 1.0f;

                Vector2 projectedPos = position.ProjectOnLine(targetEnemy->position, targetEnemy->nextCheckpointDirection);
                float toProjectionDistance = (projectedPos - position).Length();

                /*
                if(targetEnemy->nextCheckpointDirection.Angle(rotationDirection) > 90.0f && toProjectionDistance > 2000.0f)
                {
                    distMultiplier = 0.4f;
                }
                */

                toTargetAngle = directionToEnemyNorm.Angle(rotationDirection);

                //cerr << "Target Enemy pos: " << targetEnemy->position.x << " " << targetEnemy->position.y;
                //cerr << " Dir: " << targetEnemy->inertiaDirection.x << " " << targetEnemy->inertiaDirection.y;
                //cerr << " Dist " << distanceToEnemy << " Speed " << targetEnemy->speedValue << endl << endl;
                //cerr << "To Target Angle: " << toTargetAngle << endl;

                Vector2 extraPositionOffset = Vector2 (0.0f, 0.0f);

                if(toTargetAngle < 15.0f)
                {
                    //extraPositionOffset = targetEnemy->rotationDirection * 200.0f;
                }

                else if(toTargetAngle < 35.0f)
                {
                    //extraPositionOffset = targetEnemy->rotationDirection * targetEnemy->speedValue * toTargetAngle / 35.0f;
                }

                if(targetEnemy->speedValue > 10.0f)
                {
                    bool enemyStraightForWp = abs(averageEnemyEstimatedMovement.Angle(targetEnemy->nextCheckpointDirection)) > 170.0f;

                    if(distanceToEnemy > 3000.0f && toProjectionDistance < 1000.0f && enemyStraightForWp)
                    {
                        targetInterceptPosition = targetEnemy->position + (averageEnemyEstimatedMovement) * clamp(targetEnemy->speedValue * 2.5f, 0.0f, distanceToEnemy); 
                    }

                    else if(distanceToEnemy > 4500.0f && toProjectionDistance < 2000.0f)
                    {
                        float timeToEnemy = 0.0f;
                        if(speedValue > 10)
                        {
                            timeToEnemy = distanceToEnemy / speedValue;
                        } 
                        targetInterceptPosition = targetEnemy->position + (averageEnemyEstimatedMovement) * sqrt(distanceToEnemy * distanceToEnemy + (timeToEnemy * targetEnemy->speedValue) * (timeToEnemy * targetEnemy->speedValue)); 

                        distMultiplier = clamp(toProjectionDistance / 2000.0f, 0.0f, 1.0f);
                    }
                    else
                    {
                        //targetInterceptPosition = targetEnemy->position + (targetEnemy->inertiaDirection + targetEnemy->rotationDirection * 200.0f / 1.5f) /* * (distanceToEnemy / targetEnemy->speedValue) */ + targetEnemy->nextCheckpointDirection * clamp(targetEnemy->nextCheckpointDist / 4, 0.0f, 400.0f);
                        targetInterceptPosition = targetEnemy->position + (averageEnemyEstimatedMovement) * clamp(targetEnemy->speedValue * 9.5f, 0.0f, distanceToEnemy); 
                        targetInterceptPosition = targetInterceptPosition + extraPositionOffset;
                    }
                }
                else
                {
                    targetInterceptPosition = targetEnemy->position;
                    targetInterceptPosition = targetInterceptPosition + extraPositionOffset;
                }

                
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
        const float MIN_DISTANCE_SLOWDOWN = 600.0f;
        const float MAX_ANGLE_ENGINE = 75.0f;
        const float MIN_ANGLE_ENGINE = 30.0f;

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
            //cerr << name << " ToTarget Dir: " << toTargetDirection.x << " " << toTargetDirection.y << endl;
            //cerr << name << " Current Speed: " << speedValue << endl;

            Vector2 projectedPoint = (position + inertiaDirection).ProjectOnLine(position, toTargetDirection);
            Vector2 compensationVector = (projectedPoint - (position + inertiaDirection));

            //cerr << name << "Compensation vector lentgh: " << compensationVector.Length() << endl;

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

        if(collisionImminent)
        {
            cerr << endl << "Try activate shield" << endl <<endl;
            cout << round(finalDirection.x) << " " << round(finalDirection.y) << " " << "SHIELD" << " " << name << endl;
        }

        else if(friendlyCollisionAvoidanceActive)
        {
            cerr << endl << "Try avoid friendly" << endl <<endl;
            cout << round(friendlyAvoidanceTempPosition.x) << " " << round(friendlyAvoidanceTempPosition.y) << " " << 40.0f << " " << name << endl;
        }

        else if(speedValue < MAX_SPEED)
        {
            cout << round(finalDirection.x) << " " << round(finalDirection.y) << " " << finalSpeed << " " << name << endl;
        }

        else
        {
            cout << round(finalDirection.x) << " " << round(finalDirection.y) << " " << 0.0f << " " << name << endl;
        }
    }

    bool DetectCollisionImminent(BasePod _otherRacer)
    {
        float distanceToOtherTarget = (_otherRacer.position - position).Length();
        //cerr << "Dist to other: " << distanceToOtherTarget << endl;
        //cerr << "Player speed: " << speedValue << endl;
        //cerr << "Enemy speed: " << _otherRacer.speedValue << endl;

        Vector2 twoTurnMyPosition = position + inertiaDirection * 1.0f + rotationDirection * 100.0f;
        Vector2 twoTurnOtherPosition = _otherRacer.position + _otherRacer.inertiaDirection * 1.0f + _otherRacer.rotationDirection * 100.0f;
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

    void AvoidFriendlyCollision(BasePod _otherRacer)
    {
        int FRAME_COUNT_TO_CHECK = 8;

        for(int i = 0; i < FRAME_COUNT_TO_CHECK; i++)
        {
            Vector2 myTestedPosition = position + inertiaDirection * i;
            Vector2 testedPosition = _otherRacer.position + _otherRacer.inertiaDirection * i;

            if((myTestedPosition - testedPosition).Length() < 1200.0f)
            {
                friendlyCollisionAvoidanceActive = true;

                Vector2 avoidanceVector = (testedPosition - myTestedPosition).Normalize();

                friendlyAvoidanceTempPosition = myTestedPosition - avoidanceVector * 1200.0f;
            }
        }
    }
};

class PodFighter2 : public BasePod
{
    public:

    bool firstTurn = true;

    Vector2 targetInterceptPosition;
    float finalSpeed;

    float angleMultiplier = 1.0f;
    float distanceMultiplier = 1.0f;

    PodFighter2()
    {
        podFaction = Faction::Friendly;
    }

    void UpdateFighter(RankingManager &rankManager , Vector2 _newPos, Vector2 speed, float _rotationAngle, int _nextCheckpointID)
    {
        float DISTANCE_DIFF_FOR_SKIP_ATTACK = 700.0f;
        float DISTANCE_SLOWDOWN = 4000.0f;
        float ANGLE_FOR_SKIP_ATTACK_BEHIND = 45.0f;
        float ENEMY_TIME_TO_WP_FOR_STOP = 4.0f;

        collisionImminent = false;

        //Calculations

        nextCheckpointID = _nextCheckpointID;

        nextCheckPointPos = Vector2(checkPointCoords_X.at(nextCheckpointID), checkPointCoords_Y.at(nextCheckpointID));
        myRotationAngle = _rotationAngle;
        rotationDirection = Vector2(cos(myRotationAngle * M_PI / 180.0f), sin(myRotationAngle * M_PI / 180.0f));

        //cerr << "Rotation angle: " << myRotationAngle << endl;
        cerr << "Rotation Direction: " << rotationDirection.x << " " << rotationDirection.y << endl;

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
            Vector2 directionToEnemyNorm = (targetEnemy->position -position).Normalize();
            Vector2 directiontoEnemyWpNorm = (targetEnemy->nextCheckPointPos - position).Normalize();
            float distanceToEnemy = (position - targetEnemy->position).Length();

            float enemyDistanceToNextWp = targetEnemy->nextMajorCheckPointDist;
            float myDistanceToEnemyNextWp = (targetEnemy->nextCheckPointPos - position).Length();
            float distanceDiff = myDistanceToEnemyNextWp - enemyDistanceToNextWp;


            if(distanceDiff > DISTANCE_DIFF_FOR_SKIP_ATTACK)
            {
                cerr << name << "Waiting for next waypoint to attack " << endl;

                Vector2 nextNextWpDir = (targetEnemy->nextMajorWp->nextMajorWp->position - targetEnemy->nextMajorWp->position).Normalize();
                float nextNextWpDist = (targetEnemy->nextMajorWp->nextMajorWp->position - targetEnemy->nextMajorWp->position).Length();
                targetInterceptPosition = targetEnemy->nextMajorWp->position + nextNextWpDir * nextNextWpDist / 1.5f;

                float targetInterceptDist = (targetInterceptPosition - position).Length();

                Vector2 toTargetDir = (targetInterceptPosition - position).Normalize();
                
                finalSpeed = 50.0f;
            }
            else
            {
                cerr << name << "Attacking " << endl;

                Vector2 toEnemyDirection = (targetEnemy->position - position).Normalize();  
                Vector2 enemyInertialDirectionNorm = targetEnemy->inertiaDirection;
                enemyInertialDirectionNorm.Normalize();

                Vector2 totalApproximateEnemyDirection = (enemyInertialDirectionNorm + targetEnemy->rotationDirection).Normalize();

                float angleToEnemy = toEnemyDirection.Angle(rotationDirection);
                float targetDist = (targetEnemy->position - position).Length();

                float timeToRotate = abs(angleToEnemy / 18.0f);

                Vector2 projectedPosition = position.ProjectOnLine(targetEnemy->position, targetEnemy->rotationDirection);
                Vector2 projectionDirection = (projectedPosition - position).Normalize();

                float currentSpeedProjected = ((position + inertiaDirection).ProjectOnLine(position, projectionDirection) - position).Length();

                float timeToIntercept = (currentSpeedProjected + sqrt(currentSpeedProjected * currentSpeedProjected + (4 * 100.0f / 2) * targetDist)) / 200.0f;

                cerr << "Time to intercept: "<< timeToIntercept << endl;

                if(targetEnemy->speedValue > 10.0f)
                {
                    targetInterceptPosition = targetEnemy->position + totalApproximateEnemyDirection * targetEnemy->speedValue * timeToIntercept;
                }
                else
                {
                    targetInterceptPosition = targetEnemy->position;
                }

                finalSpeed = 100.0f;
            }

            
        }

        else
        {
            firstTurn = false;
            targetInterceptPosition = position + rotationDirection * 300.0f;
            finalSpeed = 100.0f;
        } 
    }

    Vector2 ComputeIdealDirection(Vector2 _targetPosition)
    {
        float COMPENSATION_MULTIPLIER = 0.7f;

        Vector2 toTargetDirection = (_targetPosition - position).Normalize();

        if(speedValue > 50.0f)
        {
            //cerr << name << " ToTarget Dir: " << toTargetDirection.x << " " << toTargetDirection.y << endl;
            //cerr << name << " Current Speed: " << speedValue << endl;

            Vector2 projectedPoint = (position + inertiaDirection).ProjectOnLine(position, toTargetDirection);
            Vector2 compensationVector = (projectedPoint - (position + inertiaDirection));

            //cerr << name << "Compensation vector lentgh: " << compensationVector.Length() << endl;

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
        //const float MAX_SPEED = 400.0f;

        Vector2 finalDirection = ComputeIdealDirection(targetInterceptPosition);

        finalSpeed = round(finalSpeed * angleMultiplier * distanceMultiplier);

        if(collisionImminent)
        {
            cerr << endl << "Try activate shield" << endl <<endl;
            cout << round(finalDirection.x) << " " << round(finalDirection.y) << " " << "SHIELD" << " " << name << endl;
            return;
        }

        cout << round(finalDirection.x) << " " << round(finalDirection.y) << " " << finalSpeed << " " << name << endl;
    }

    bool DetectCollisionImminent(BasePod _otherRacer)
    {
        float distanceToOtherTarget = (_otherRacer.position - position).Length();

        Vector2 twoTurnMyPosition = position + inertiaDirection * 1.0f + rotationDirection * 100.0f;
        Vector2 twoTurnOtherPosition = _otherRacer.position + _otherRacer.inertiaDirection * 1.0f + _otherRacer.rotationDirection * 100.0f;
        
        float twoTurnDistance = (twoTurnMyPosition - twoTurnOtherPosition).Length();

        if(twoTurnDistance < 800.0f)
        {
            collisionImminent = true;
            return true;
        }

        return false;
    }

    void ComputeBasicMultipliers()
    {
        const float MIN_DISTANCE_SLOWDOWN = 2500.0f;
        const float MAX_ANGLE_ENGINE = 75.0f;
        const float MIN_ANGLE_ENGINE = 30.0f;

        Vector2 targetInterceptDirection = (targetInterceptPosition - position).Normalize();
        float targetInterceptAngle = abs(targetInterceptDirection.Angle(rotationDirection));

        cerr << name << " - Target intercept angle " << targetInterceptAngle <<endl;

        if(targetInterceptAngle > MAX_ANGLE_ENGINE)
        {
            angleMultiplier = 0.0f;
        }

        else if(targetInterceptAngle > MIN_ANGLE_ENGINE)
        {
            angleMultiplier = 1.0f - (targetInterceptAngle - MIN_ANGLE_ENGINE) / (MAX_ANGLE_ENGINE - MIN_ANGLE_ENGINE);
        }
        else
        {
            angleMultiplier = 1.0f;
        }
    }
};

RankingManager *RankingManager::instance = 0;

int main()
{
    bool isInit = false;

    PodRacer player1;
    PodFighter2 player2;
    EnemyPod enemy1;
    EnemyPod enemy2;

    int laps;
    int checkpointCount;
    vector<int> checkPointCoords_X;
    vector<int> checkPointCoords_Y;

    vector<MajorWaypoint> majorWaypoints;
    vector<Waypoint> otherWaypoints;

    Waypoint tempRacerWayPoint;

    vector<BasePod*> otherRacers;

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
            //otherRacers.reserve(3);

            otherRacers.push_back(&player2);
            otherRacers.push_back(&enemy1);
            otherRacers.push_back(&enemy2);

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
            player1.InitPod(checkPointCoords_X, checkPointCoords_Y, majorWaypoints, otherWaypoints, tempRacerWayPoint, "Anakin");
            rankManager->AddToPods(player1);
        }

        player1.UpdateRacer_waypoints(Vector2(x, y), Vector2(vx, vy), angle, nextCheckPointId, otherRacers);

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

        rankManager->UpdatePod(player1);
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

        player1.DetectCollisionImminent(enemy1);
        player1.DetectCollisionImminent(enemy2);
        player1.DetectCollisionImminent(player2);

        player1.DetectDriftPossible();

        player1.DetectPushThrough(enemy1);
        player1.DetectPushThrough(enemy2);

        player1.ApplyMovement();

        //Player 2

        //player2.ComputeAvoidanceMultiplier(enemy1);
        //player2.ComputeAvoidanceMultiplier(enemy2);

        player2.ComputeBasicMultipliers();

        player2.DetectCollisionImminent(enemy1);
        player2.DetectCollisionImminent(enemy2);

        //player2.AvoidFriendlyCollision(player1);

        //player2.DetectDriftPossible();

        player2.ApplyMovement();
    }
}
