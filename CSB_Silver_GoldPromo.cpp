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
float Angle(const Vector2 & v) const { return atan2((y - v.y), (x - v.x)) * 180.0f / M_PI ;}
Vector2 Normalize()  const { return *this / sqrt(x * x + y * y); }
void Normalize() {*this /= sqrt(x * x + y * y);}

Vector2 ProjectOnLine(const Vector2 & point, const Vector2 & direction)  const
{
    Vector2 linePointToPoint = *this - point;

    float t = linePointToPoint.Dot(direction);

    return point + direction * t;
}

};


class PodRacer
{
    private:

    bool isInit = false;
    bool secondFrameDone = false;
    Vector2 previousPos;

    Vector2 position;
    Vector2 inertiaDirection;

    public: 

    float currentSpeed;

    Vector2 GetPosition()
    {
        return position;
    }

    Vector2 GetDirection()
    {
        return inertiaDirection;
    }

    void UpdateRacer(Vector2 _newPos)
    {
        if(isInit == true)
        {
            previousPos = position;
            position = _newPos;
            inertiaDirection = (position - previousPos);
            //cerr << "Computed Dir Before: " << direction.x << direction.y << endl;

            currentSpeed = (position - previousPos).Length();

            secondFrameDone = true;
        }
        else
        {
            position = _newPos;
            inertiaDirection = Vector2(1.0f, 1.0f);
            currentSpeed = 0.0f;
            isInit = true;
        }
        
    }

    Vector2 computeIdealDirection(Vector2 targetPosition, float distanceToWp, float angleToWp)
    {
        if(secondFrameDone == true)
        {
            Vector2 toTargetDirection = (targetPosition - position).Normalize();
            cerr << "ToTarget Dir: " << toTargetDirection.x << " " << toTargetDirection.y << endl;
            cerr << "Current Speed: " << currentSpeed << endl;

            Vector2 projectedPoint = (position + inertiaDirection).ProjectOnLine(position, toTargetDirection);
            Vector2 compensationVector = (projectedPoint - (position + inertiaDirection));

            Vector2 newDirection = position + toTargetDirection * currentSpeed + compensationVector / 4;
            return newDirection;
        }
        else
        {
            return targetPosition;
        }        
    }
};

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

void updateRacers(PodRacer &_player, PodRacer &_enemy, Vector2 _playerPos, Vector2 _enemyPos)
{
    _player.UpdateRacer(_playerPos);
    _enemy.UpdateRacer(_enemyPos);
}

bool computeAvoidanceMultiplier(int _nextCheckpointAngle, int _nextCheckpointDist, PodRacer _player, PodRacer _enemy, float &_avoidanceMultiplier)
{
    const float ENEMY_DISTANCE_THREAT = 3000.0f;

    float enemyDistanceToPlayer = _player.GetPosition().Distance(_enemy.GetPosition());
    Vector2 enemyDirectAttackVector = (_player.GetPosition() - _enemy.GetPosition()).Normalize();
    float enemyDirectAttackAngle = enemyDirectAttackVector.Angle(_enemy.GetDirection());

    if(abs(enemyDirectAttackAngle) < 15.0f && enemyDistanceToPlayer < ENEMY_DISTANCE_THREAT)
    {
        _avoidanceMultiplier = ENEMY_DISTANCE_THREAT / enemyDistanceToPlayer;
        return true;
    }

    _avoidanceMultiplier = 1.0f;
    return false;
}

int main()
{
    static bool usedBoost = false;

    int prevX;
    int prevY;

    PodRacer player;
    PodRacer enemy;

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
        float avoidanceMultiplier = 1.0f;

        updateRacers(player, enemy, Vector2(x,y), Vector2(opponentX, opponentY));
        
        bool needsAvoidance = computeAvoidanceMultiplier(nextCheckpointAngle, nextCheckpointDist, player, enemy, avoidanceMultiplier);

        if(needsAvoidance)
        {
            cerr << "Needs Avoidance" << endl;
        }

        computeBasicMultipliers(nextCheckpointAngle, nextCheckpointDist, currentSpeed, angleMultiplier, distMultiplier);
        
        if(!usedBoost && nextCheckpointDist > 8000 && abs(nextCheckpointAngle) < 5)
        {
            usedBoost = true;

            cout << nextCheckpointX << " " << nextCheckpointY << " BOOST" << endl;

            prevX = x;
            prevY = y;

            continue;
        }

        Vector2 adaptedDirection = player.computeIdealDirection(Vector2(nextCheckpointX, nextCheckpointY), nextCheckpointDist, nextCheckpointAngle);

        finalMultiplier = distMultiplier * angleMultiplier * avoidanceMultiplier;
        int finalSpeed = round(clamp(100 * finalMultiplier, 0.0f, 100.0f));

        prevX = x;
        prevY = y;

        cout << round(adaptedDirection.x) << " " << round(adaptedDirection.y) << " " << finalSpeed << endl;
    }
}
