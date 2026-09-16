#include "raylib.h"

#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <string>

// ============================================================
// PHYSICALLY BASED BALL SIMULATION
// ============================================================
//
// Foundations of Physically Based Modeling and Animation
// Donald House & John C. Keyser
//
// Assignment 1
//
// REQUIRED PHYSICS:
//   - Gravity
//   - Air resistance
//   - Wind force
//   - Explicit Euler integration
//   - Variable physics timestep
//   - Fractional timestep collision detection
//   - Collision response
//   - Restitution
//   - Friction
//   - Six-sided box
//
// EXTENSION:
//   - Spatially and temporally varying wind
//   - Multiple predefined scenarios
//   - Interactive initial conditions
//   - Adjustable timestep
//   - Velocity visualization
//
// ============================================================


// ============================================================
// VECTOR MATH
// ============================================================

static Vector3 VecAdd(Vector3 a, Vector3 b)
{
    return {
        a.x + b.x,
        a.y + b.y,
        a.z + b.z
    };
}


static Vector3 VecSub(Vector3 a, Vector3 b)
{
    return {
        a.x - b.x,
        a.y - b.y,
        a.z - b.z
    };
}


static Vector3 VecScale(Vector3 a, float s)
{
    return {
        a.x * s,
        a.y * s,
        a.z * s
    };
}


static float VecDot(Vector3 a, Vector3 b)
{
    return a.x * b.x +
           a.y * b.y +
           a.z * b.z;
}


static float VecLength(Vector3 a)
{
    return std::sqrt(
        VecDot(a, a)
    );
}


static Vector3 VecNormalize(Vector3 a)
{
    float length = VecLength(a);

    if (length < 0.000001f)
        return {0.0f, 0.0f, 0.0f};

    return VecScale(
        a,
        1.0f / length
    );
}


static Vector3 VecProject(
    Vector3 v,
    Vector3 n)
{
    return VecScale(
        n,
        VecDot(v, n)
    );
}


static Vector3 VecReject(
    Vector3 v,
    Vector3 n)
{
    return VecSub(
        v,
        VecProject(v, n)
    );
}


// ============================================================
// BALL
// ============================================================

struct Ball
{
    Vector3 position;
    Vector3 velocity;

    float radius;
    float mass;
};


// ============================================================
// BOX
// ============================================================

struct Box
{
    Vector3 center;

    float width;
    float height;
    float depth;
};


// ============================================================
// PHYSICS PARAMETERS
// ============================================================

struct PhysicsParameters
{
    // Gravity acceleration
    Vector3 gravity;

    // Linear drag coefficient
    //
    // F_drag = -k v
    //
    float dragCoefficient;

    // Constant wind force
    Vector3 windForce;

    // Coefficient of restitution
    float restitution;

    // Coefficient of friction
    float frictionCoefficient;

    // Speed below which a contact can be considered resting
    float restingSpeed;
};


// ============================================================
// COLLISION
// ============================================================

struct Collision
{
    bool hit;

    // Time from current state to collision
    float time;

    // Collision normal pointing INTO the box
    Vector3 normal;
};


// ============================================================
// SIMULATION
// ============================================================

struct Simulation
{
    Ball ball;

    double simulationTime;

    unsigned long long collisionCount;

    int scenario;

    bool paused;

    // Physics timestep
    float physicsDt;
};


// ============================================================
// CUSTOM INITIAL CONDITIONS
// ============================================================

Vector3 customInitialPosition = {
    0.0f,
    1.0f,
    0.0f
};


Vector3 customInitialVelocity = {
    8.0f,
    8.0f,
    7.0f
};


// ============================================================
// COLLISION BOUNDARIES
// ============================================================

static float MinX(
    const Box& box,
    const Ball& ball)
{
    return box.center.x -
           box.width * 0.5f +
           ball.radius;
}


static float MaxX(
    const Box& box,
    const Ball& ball)
{
    return box.center.x +
           box.width * 0.5f -
           ball.radius;
}


static float MinY(
    const Box& box,
    const Ball& ball)
{
    return box.center.y -
           box.height * 0.5f +
           ball.radius;
}


static float MaxY(
    const Box& box,
    const Ball& ball)
{
    return box.center.y +
           box.height * 0.5f -
           ball.radius;
}


static float MinZ(
    const Box& box,
    const Ball& ball)
{
    return box.center.z -
           box.depth * 0.5f +
           ball.radius;
}


static float MaxZ(
    const Box& box,
    const Ball& ball)
{
    return box.center.z +
           box.depth * 0.5f -
           ball.radius;
}


// ============================================================
// WIND
// ============================================================
//
// Scenario 1-4:
// Constant wind.
//
// Scenario 5:
// Spatially and temporally varying wind.
//
// ============================================================

static Vector3 CalculateWindForce(
    const Ball& ball,
    const PhysicsParameters& physics,
    double time,
    bool turbulentWind)
{
    if (!turbulentWind)
    {
        return physics.windForce;
    }


    float x = ball.position.x;
    float y = ball.position.y;
    float z = ball.position.z;

    float t =
        static_cast<float>(time);


    // Spatial + temporal wind field.
    //
    // The wind varies according to both position and time.
    //
    Vector3 turbulent = {

        2.0f *
        std::sin(
            0.65f * x +
            1.10f * t
        ) *
        std::cos(
            0.40f * z
        ),

        1.5f *
        std::sin(
            0.55f * y +
            0.90f * t
        ),

        2.0f *
        std::cos(
            0.60f * z +
            1.20f * t
        ) *
        std::sin(
            0.35f * x
        )
    };


    return VecAdd(
        physics.windForce,
        turbulent
    );
}


// ============================================================
// FORCE CALCULATION
// ============================================================
//
// Gravity:
//
//     Fg = m g
//
// Air resistance:
//
//     Fd = -k v
//
// Wind:
//
//     Fw
//
// Total:
//
//     F = Fg + Fd + Fw
//
// Acceleration:
//
//     a = F / m
//
// ============================================================

static Vector3 CalculateAcceleration(
    const Ball& ball,
    const PhysicsParameters& physics,
    double time,
    bool turbulentWind)
{
    Vector3 gravityForce =
        VecScale(
            physics.gravity,
            ball.mass
        );


    Vector3 dragForce =
        VecScale(
            ball.velocity,
            -physics.dragCoefficient
        );


    Vector3 windForce =
        CalculateWindForce(
            ball,
            physics,
            time,
            turbulentWind
        );


    Vector3 totalForce =
        VecAdd(
            gravityForce,
            VecAdd(
                dragForce,
                windForce
            )
        );


    return VecScale(
        totalForce,
        1.0f / ball.mass
    );
}


// ============================================================
// RESTING CONTACT
// ============================================================
//
// This prevents numerical jitter once the ball has genuinely
// lost enough energy to remain against a surface.
//
// Importantly, this does NOT replace collision detection.
//
// Collision detection and fractional timestep handling happen
// separately.
//
// ============================================================

static void ApplyRestingContact(
    Ball& ball,
    const Box& box,
    const PhysicsParameters& physics,
    Vector3 acceleration)
{
    const float epsilon = 0.0005f;


    float minX = MinX(box, ball);
    float maxX = MaxX(box, ball);

    float minY = MinY(box, ball);
    float maxY = MaxY(box, ball);

    float minZ = MinZ(box, ball);
    float maxZ = MaxZ(box, ball);


    // --------------------------------------------------------
    // FLOOR
    // --------------------------------------------------------

    if (ball.position.y <= minY + epsilon)
    {
        ball.position.y = minY;

        // If gravity / forces push into the floor and the ball
        // is moving slowly, maintain contact.
        if (ball.velocity.y <= physics.restingSpeed &&
            acceleration.y < 0.0f)
        {
            ball.velocity.y = 0.0f;
        }
    }


    // --------------------------------------------------------
    // CEILING
    // --------------------------------------------------------

    if (ball.position.y >= maxY - epsilon)
    {
        ball.position.y = maxY;

        if (ball.velocity.y >= -physics.restingSpeed &&
            acceleration.y > 0.0f)
        {
            ball.velocity.y = 0.0f;
        }
    }


    // --------------------------------------------------------
    // X MIN
    // --------------------------------------------------------

    if (ball.position.x <= minX + epsilon)
    {
        ball.position.x = minX;

        if (ball.velocity.x <= physics.restingSpeed &&
            acceleration.x < 0.0f)
        {
            ball.velocity.x = 0.0f;
        }
    }


    // --------------------------------------------------------
    // X MAX
    // --------------------------------------------------------

    if (ball.position.x >= maxX - epsilon)
    {
        ball.position.x = maxX;

        if (ball.velocity.x >= -physics.restingSpeed &&
            acceleration.x > 0.0f)
        {
            ball.velocity.x = 0.0f;
        }
    }


    // --------------------------------------------------------
    // Z MIN
    // --------------------------------------------------------

    if (ball.position.z <= minZ + epsilon)
    {
        ball.position.z = minZ;

        if (ball.velocity.z <= physics.restingSpeed &&
            acceleration.z < 0.0f)
        {
            ball.velocity.z = 0.0f;
        }
    }


    // --------------------------------------------------------
    // Z MAX
    // --------------------------------------------------------

    if (ball.position.z >= maxZ - epsilon)
    {
        ball.position.z = maxZ;

        if (ball.velocity.z >= -physics.restingSpeed &&
            acceleration.z > 0.0f)
        {
            ball.velocity.z = 0.0f;
        }
    }
}


// ============================================================
// FRACTIONAL TIMESTEP COLLISION DETECTION
// ============================================================
//
// Explicit Euler predicts:
//
//     x(t + dt) = x(t) + v(t) dt
//
// If that prediction crosses a boundary, we solve:
//
//     wall = x + v t_c
//
// Therefore:
//
//     t_c = (wall - x) / v
//
// The earliest valid collision is selected.
//
// ============================================================

static Collision FindEarliestCollision(
    const Ball& ball,
    const Box& box,
    float dt)
{
    Collision result;

    result.hit = false;

    result.time = dt;

    result.normal = {
        0.0f,
        0.0f,
        0.0f
    };


    float minX = MinX(box, ball);
    float maxX = MaxX(box, ball);

    float minY = MinY(box, ball);
    float maxY = MaxY(box, ball);

    float minZ = MinZ(box, ball);
    float maxZ = MaxZ(box, ball);


    Vector3 predicted = {

        ball.position.x +
        ball.velocity.x * dt,

        ball.position.y +
        ball.velocity.y * dt,

        ball.position.z +
        ball.velocity.z * dt
    };


    const float velocityEpsilon =
        0.000001f;


    // ========================================================
    // X MIN
    // ========================================================

    if (ball.velocity.x < -velocityEpsilon &&
        predicted.x < minX)
    {
        float collisionTime =
            (minX - ball.position.x) /
            ball.velocity.x;


        if (collisionTime >= 0.0f &&
            collisionTime <= result.time)
        {
            result.hit = true;

            result.time =
                collisionTime;

            result.normal = {
                1.0f,
                0.0f,
                0.0f
            };
        }
    }


    // ========================================================
    // X MAX
    // ========================================================

    if (ball.velocity.x > velocityEpsilon &&
        predicted.x > maxX)
    {
        float collisionTime =
            (maxX - ball.position.x) /
            ball.velocity.x;


        if (collisionTime >= 0.0f &&
            collisionTime <= result.time)
        {
            result.hit = true;

            result.time =
                collisionTime;

            result.normal = {
                -1.0f,
                0.0f,
                0.0f
            };
        }
    }


    // ========================================================
    // Y MIN / FLOOR
    // ========================================================

    if (ball.velocity.y < -velocityEpsilon &&
        predicted.y < minY)
    {
        float collisionTime =
            (minY - ball.position.y) /
            ball.velocity.y;


        if (collisionTime >= 0.0f &&
            collisionTime <= result.time)
        {
            result.hit = true;

            result.time =
                collisionTime;

            result.normal = {
                0.0f,
                1.0f,
                0.0f
            };
        }
    }


    // ========================================================
    // Y MAX / CEILING
    // ========================================================

    if (ball.velocity.y > velocityEpsilon &&
        predicted.y > maxY)
    {
        float collisionTime =
            (maxY - ball.position.y) /
            ball.velocity.y;


        if (collisionTime >= 0.0f &&
            collisionTime <= result.time)
        {
            result.hit = true;

            result.time =
                collisionTime;

            result.normal = {
                0.0f,
                -1.0f,
                0.0f
            };
        }
    }


    // ========================================================
    // Z MIN
    // ========================================================

    if (ball.velocity.z < -velocityEpsilon &&
        predicted.z < minZ)
    {
        float collisionTime =
            (minZ - ball.position.z) /
            ball.velocity.z;


        if (collisionTime >= 0.0f &&
            collisionTime <= result.time)
        {
            result.hit = true;

            result.time =
                collisionTime;

            result.normal = {
                0.0f,
                0.0f,
                1.0f
            };
        }
    }


    // ========================================================
    // Z MAX
    // ========================================================

    if (ball.velocity.z > velocityEpsilon &&
        predicted.z > maxZ)
    {
        float collisionTime =
            (maxZ - ball.position.z) /
            ball.velocity.z;


        if (collisionTime >= 0.0f &&
            collisionTime <= result.time)
        {
            result.hit = true;

            result.time =
                collisionTime;

            result.normal = {
                0.0f,
                0.0f,
                -1.0f
            };
        }
    }


    return result;
}


// ============================================================
// COLLISION RESPONSE
// ============================================================
//
// Let:
//
//     vn = v . n
//
// Normal collision:
//
//     Jn = -(1 + e) m vn
//
// Friction:
//
//     Jt = -min(m|vt|, mu|Jn|) t
//
// Then:
//
//     v' = v + J/m
//
// ============================================================

static bool ResolveCollision(
    Ball& ball,
    Vector3 normal,
    const PhysicsParameters& physics,
    unsigned long long& collisionCount)
{
    float normalVelocity =
        VecDot(
            ball.velocity,
            normal
        );


    // If the ball is moving away from the wall, there is
    // nothing to resolve.
    if (normalVelocity >= 0.0f)
    {
        return false;
    }


    // ========================================================
    // NORMAL IMPULSE
    // ========================================================

    float normalImpulseMagnitude =
        -(1.0f + physics.restitution) *
        normalVelocity *
        ball.mass;


    Vector3 normalImpulse =
        VecScale(
            normal,
            normalImpulseMagnitude
        );


    // ========================================================
    // TANGENTIAL VELOCITY
    // ========================================================

    Vector3 tangentVelocity =
        VecReject(
            ball.velocity,
            normal
        );


    float tangentSpeed =
        VecLength(
            tangentVelocity
        );


    Vector3 frictionImpulse = {
        0.0f,
        0.0f,
        0.0f
    };


    // ========================================================
    // COULOMB FRICTION
    // ========================================================

    if (tangentSpeed > 0.000001f)
    {
        Vector3 tangentDirection =
            VecScale(
                tangentVelocity,
                1.0f / tangentSpeed
            );


        float requiredFrictionImpulse =
            ball.mass *
            tangentSpeed;


        float maximumFrictionImpulse =
            physics.frictionCoefficient *
            normalImpulseMagnitude;


        float frictionMagnitude =
            std::min(
                requiredFrictionImpulse,
                maximumFrictionImpulse
            );


        frictionImpulse =
            VecScale(
                tangentDirection,
                -frictionMagnitude
            );
    }


    // ========================================================
    // TOTAL IMPULSE
    // ========================================================

    Vector3 totalImpulse =
        VecAdd(
            normalImpulse,
            frictionImpulse
        );


    ball.velocity =
        VecAdd(
            ball.velocity,
            VecScale(
                totalImpulse,
                1.0f / ball.mass
            )
        );


    collisionCount++;


    return true;
}


// ============================================================
// EXPLICIT EULER INTEGRATION
// ============================================================
//
// Position:
//
//     x_(n+1) = x_n + v_n dt
//
// Velocity:
//
//     v_(n+1) = v_n + a_n dt
//
// ============================================================

static void IntegrateEuler(
    Ball& ball,
    const PhysicsParameters& physics,
    double time,
    float dt,
    bool turbulentWind)
{
    if (dt <= 0.0f)
        return;


    Vector3 acceleration =
        CalculateAcceleration(
            ball,
            physics,
            time,
            turbulentWind
        );


    // --------------------------------------------------------
    // Euler position
    // --------------------------------------------------------

    ball.position =
        VecAdd(
            ball.position,
            VecScale(
                ball.velocity,
                dt
            )
        );


    // --------------------------------------------------------
    // Euler velocity
    // --------------------------------------------------------

    ball.velocity =
        VecAdd(
            ball.velocity,
            VecScale(
                acceleration,
                dt
            )
        );
}


// ============================================================
// PHYSICS STEP
// ============================================================
//
// The full timestep is subdivided whenever a collision occurs.
//
// Example:
//
//     dt = 0.008333 s
//
//     collision occurs after
//
//     tc = 0.0037 s
//
// Therefore:
//
//     integrate 0.0037 s
//     resolve collision
//     integrate remaining 0.004633 s
//
// This is the required fractional timestep behavior.
//
// ============================================================

static void SimulatePhysics(
    Simulation& simulation,
    const Box& box,
    const PhysicsParameters& physics,
    bool turbulentWind)
{
    float remainingTime =
        simulation.physicsDt;


    const int MAX_COLLISIONS_PER_STEP = 8;

    int collisionIterations = 0;


    while (remainingTime > 0.000001f &&
           collisionIterations <
               MAX_COLLISIONS_PER_STEP)
    {
        // ====================================================
        // ACCELERATION
        // ====================================================

        Vector3 acceleration =
            CalculateAcceleration(
                simulation.ball,
                physics,
                simulation.simulationTime,
                turbulentWind
            );


        // ====================================================
        // RESTING CONTACT
        // ====================================================

        ApplyRestingContact(
            simulation.ball,
            box,
            physics,
            acceleration
        );


        // ====================================================
        // FIND COLLISION
        // ====================================================

        Collision collision =
            FindEarliestCollision(
                simulation.ball,
                box,
                remainingTime
            );


        // ====================================================
        // NO COLLISION
        // ====================================================

        if (!collision.hit)
        {
            IntegrateEuler(
                simulation.ball,
                physics,
                simulation.simulationTime,
                remainingTime,
                turbulentWind
            );


            simulation.simulationTime +=
                remainingTime;


            remainingTime = 0.0f;

            break;
        }


        // ====================================================
        // FRACTIONAL COLLISION TIME
        // ====================================================

        float collisionTime =
            std::max(
                0.0f,
                std::min(
                    collision.time,
                    remainingTime
                )
            );


        // ====================================================
        // INTEGRATE TO COLLISION
        // ====================================================

        if (collisionTime > 0.000001f)
        {
            IntegrateEuler(
                simulation.ball,
                physics,
                simulation.simulationTime,
                collisionTime,
                turbulentWind
            );


            simulation.simulationTime +=
                collisionTime;
        }


        // ====================================================
        // PLACE BALL EXACTLY ON SURFACE
        // ====================================================

        if (collision.normal.x > 0.5f)
        {
            simulation.ball.position.x =
                MinX(
                    box,
                    simulation.ball
                );
        }
        else if (collision.normal.x < -0.5f)
        {
            simulation.ball.position.x =
                MaxX(
                    box,
                    simulation.ball
                );
        }


        if (collision.normal.y > 0.5f)
        {
            simulation.ball.position.y =
                MinY(
                    box,
                    simulation.ball
                );
        }
        else if (collision.normal.y < -0.5f)
        {
            simulation.ball.position.y =
                MaxY(
                    box,
                    simulation.ball
                );
        }


        if (collision.normal.z > 0.5f)
        {
            simulation.ball.position.z =
                MinZ(
                    box,
                    simulation.ball
                );
        }
        else if (collision.normal.z < -0.5f)
        {
            simulation.ball.position.z =
                MaxZ(
                    box,
                    simulation.ball
                );
        }


        // ====================================================
        // COLLISION RESPONSE
        // ====================================================

        ResolveCollision(
            simulation.ball,
            collision.normal,
            physics,
            simulation.collisionCount
        );


        // ====================================================
        // SMALL NUMERICAL SEPARATION
        // ====================================================
        //
        // This is only a floating-point safeguard.
        //
        // It is NOT used to prevent penetration by replacing
        // collision detection.
        //
        // ====================================================

        const float epsilon =
            0.00001f;


        simulation.ball.position =
            VecAdd(
                simulation.ball.position,
                VecScale(
                    collision.normal,
                    epsilon
                )
            );


        // ====================================================
        // REMAINING TIME
        // ====================================================

        remainingTime -=
            collisionTime;


        // Prevent a zero-time collision loop.
        if (collisionTime < 0.000001f)
        {
            remainingTime -=
                0.000001f;
        }


        collisionIterations++;
    }


    // ========================================================
    // FINAL NUMERICAL SAFETY
    // ========================================================

    simulation.ball.position.x =
        std::max(
            MinX(box, simulation.ball),
            std::min(
                MaxX(box, simulation.ball),
                simulation.ball.position.x
            )
        );


    simulation.ball.position.y =
        std::max(
            MinY(box, simulation.ball),
            std::min(
                MaxY(box, simulation.ball),
                simulation.ball.position.y
            )
        );


    simulation.ball.position.z =
        std::max(
            MinZ(box, simulation.ball),
            std::min(
                MaxZ(box, simulation.ball),
                simulation.ball.position.z
            )
        );
}


// ============================================================
// SCENARIOS
// ============================================================
//
// Scenario 1:
//
// Main demonstration.
// Designed specifically to hit:
//   - Floor
//   - Ceiling
//   - +X
//   - -X
//   - +Z
//   - -Z
//
// The ball is NOT being forced toward walls.
// The initial velocity simply gives it enough energy to
// naturally travel through the box.
//
// ============================================================

static void LoadScenario(
    Simulation& simulation,
    int scenario)
{
    simulation.scenario =
        scenario;


    simulation.simulationTime =
        0.0;


    simulation.collisionCount =
        0;


    simulation.paused =
        false;


    switch (scenario)
    {
        // ====================================================
        // SCENARIO 1
        // ALL-FACES DEMONSTRATION
        // ====================================================

        case 1:
        {
            simulation.ball.position = {
                0.0f,
                1.0f,
                0.0f
            };


            simulation.ball.velocity = {
                8.0f,
                9.0f,
                7.5f
            };


            break;
        }


        // ====================================================
        // SCENARIO 2
        // FLOOR + SIDE WALLS
        // ====================================================

        case 2:
        {
            simulation.ball.position = {
                -3.0f,
                3.0f,
                -2.5f
            };


            simulation.ball.velocity = {
                7.0f,
                2.0f,
                8.0f
            };


            break;
        }


        // ====================================================
        // SCENARIO 3
        // CEILING COLLISION
        // ====================================================

        case 3:
        {
            simulation.ball.position = {
                0.0f,
                1.0f,
                0.0f
            };


            simulation.ball.velocity = {
                4.0f,
                11.0f,
                3.0f
            };


            break;
        }


        // ====================================================
        // SCENARIO 4
        // CORNER / 3D COLLISIONS
        // ====================================================

        case 4:
        {
            simulation.ball.position = {
                -3.0f,
                2.0f,
                -3.0f
            };


            simulation.ball.velocity = {
                9.0f,
                8.0f,
                9.0f
            };


            break;
        }


        // ====================================================
        // SCENARIO 5
        // CUSTOM + TURBULENT WIND
        // ====================================================

        case 5:
        {
            simulation.ball.position =
                customInitialPosition;


            simulation.ball.velocity =
                customInitialVelocity;


            break;
        }


        default:
        {
            LoadScenario(
                simulation,
                1
            );

            break;
        }
    }
}


// ============================================================
// DRAW BOX
// ============================================================

static void DrawSimulationBox(
    const Box& box)
{
    // Wireframe only.
    //
    // This ensures that the ball remains visible from all
    // camera angles.

    DrawCubeWires(
        box.center,
        box.width,
        box.height,
        box.depth,
        BLUE
    );
}


// ============================================================
// DRAW COORDINATE AXES
// ============================================================

static void DrawAxes()
{
    float length = 7.0f;


    // X
    DrawLine3D(
        {-length, 0.0f, 0.0f},
        { length, 0.0f, 0.0f},
        RED
    );


    // Y
    DrawLine3D(
        {0.0f, -length, 0.0f},
        {0.0f,  length, 0.0f},
        GREEN
    );


    // Z
    DrawLine3D(
        {0.0f, 0.0f, -length},
        {0.0f, 0.0f,  length},
        BLUE
    );
}


// ============================================================
// DRAW VELOCITY VECTOR
// ============================================================

static void DrawVelocityVector(
    const Ball& ball)
{
    float speed =
        VecLength(
            ball.velocity
        );


    if (speed < 0.01f)
        return;


    const float scale =
        0.30f;


    Vector3 endpoint = {

        ball.position.x +
        ball.velocity.x * scale,

        ball.position.y +
        ball.velocity.y * scale,

        ball.position.z +
        ball.velocity.z * scale
    };


    DrawLine3D(
        ball.position,
        endpoint,
        MAROON
    );


    DrawSphere(
        endpoint,
        0.08f,
        MAROON
    );
}


// ============================================================
// FORMAT FLOAT
// ============================================================

static std::string FormatFloat(
    float value,
    int precision = 3)
{
    std::ostringstream stream;


    stream
        << std::fixed
        << std::setprecision(precision)
        << value;


    return stream.str();
}


// ============================================================
// CUSTOM INITIAL CONDITION CONTROLS
// ============================================================

static void UpdateCustomControls(
    const Box& box,
    const Ball& ball)
{
    const float positionStep =
        0.10f;


    const float velocityStep =
        0.25f;


    // --------------------------------------------------------
    // POSITION
    // --------------------------------------------------------

    if (IsKeyDown(KEY_A))
        customInitialPosition.x -=
            positionStep;


    if (IsKeyDown(KEY_D))
        customInitialPosition.x +=
            positionStep;


    if (IsKeyDown(KEY_W))
        customInitialPosition.y +=
            positionStep;


    if (IsKeyDown(KEY_S))
        customInitialPosition.y -=
            positionStep;


    if (IsKeyDown(KEY_Q))
        customInitialPosition.z -=
            positionStep;


    if (IsKeyDown(KEY_E))
        customInitialPosition.z +=
            positionStep;


    // --------------------------------------------------------
    // VELOCITY
    // --------------------------------------------------------

    if (IsKeyDown(KEY_J))
        customInitialVelocity.x -=
            velocityStep;


    if (IsKeyDown(KEY_L))
        customInitialVelocity.x +=
            velocityStep;


    if (IsKeyDown(KEY_I))
        customInitialVelocity.y +=
            velocityStep;


    if (IsKeyDown(KEY_K))
        customInitialVelocity.y -=
            velocityStep;


    if (IsKeyDown(KEY_U))
        customInitialVelocity.z -=
            velocityStep;


    if (IsKeyDown(KEY_O))
        customInitialVelocity.z +=
            velocityStep;


    // --------------------------------------------------------
    // KEEP POSITION INSIDE BOX
    // --------------------------------------------------------

    customInitialPosition.x =
        std::max(
            MinX(box, ball),
            std::min(
                MaxX(box, ball),
                customInitialPosition.x
            )
        );


    customInitialPosition.y =
        std::max(
            MinY(box, ball),
            std::min(
                MaxY(box, ball),
                customInitialPosition.y
            )
        );


    customInitialPosition.z =
        std::max(
            MinZ(box, ball),
            std::min(
                MaxZ(box, ball),
                customInitialPosition.z
            )
        );
}


// ============================================================
// MAIN
// ============================================================

int main()
{
    // ========================================================
    // WINDOW
    // ========================================================

    const int screenWidth =
        1280;


    const int screenHeight =
        720;


    InitWindow(
        screenWidth,
        screenHeight,
        "Physically Based Ball Simulation"
    );


    // IMPORTANT:
    //
    // Display runs independently from physics timestep.
    //
    SetTargetFPS(60);


    // ========================================================
    // CAMERA
    // ========================================================

    Camera3D camera = {};


    camera.position = {
        14.0f,
        10.0f,
        14.0f
    };


    camera.target = {
        0.0f,
        0.0f,
        0.0f
    };


    camera.up = {
        0.0f,
        1.0f,
        0.0f
    };


    camera.fovy =
        45.0f;


    camera.projection =
        CAMERA_PERSPECTIVE;


    // ========================================================
    // BOX
    // ========================================================
    //
    // Larger vertical dimension makes ceiling collisions
    // easier to observe.
    //
    // ========================================================

    Box box;


    box.center = {
        0.0f,
        0.0f,
        0.0f
    };


    box.width =
        10.0f;


    box.height =
        8.0f;


    box.depth =
        10.0f;


    // ========================================================
    // PHYSICS
    // ========================================================

    PhysicsParameters physics;


    // --------------------------------------------------------
    // GRAVITY
    // --------------------------------------------------------

    physics.gravity = {
        0.0f,
        -9.81f,
        0.0f
    };


    // --------------------------------------------------------
    // AIR RESISTANCE
    // --------------------------------------------------------
    //
    // F_drag = -k v
    //
    // Kept relatively small so that several collisions can
    // naturally occur before the ball loses its energy.
    //
    // --------------------------------------------------------

    physics.dragCoefficient =
        0.015f;


    // --------------------------------------------------------
    // WIND
    // --------------------------------------------------------
    //
    // A small constant wind force gives the trajectory a
    // physically motivated horizontal bias.
    //
    // --------------------------------------------------------

    physics.windForce = {
        0.8f,
        0.25f,
        -0.6f
    };


    // --------------------------------------------------------
    // RESTITUTION
    // --------------------------------------------------------
    //
    // e = 1.0 -> perfectly elastic
    // e = 0.0 -> perfectly inelastic
    //
    // 0.88 gives a clearly visible bounce while still losing
    // energy.
    //
    // --------------------------------------------------------

    physics.restitution =
        0.88f;


    // --------------------------------------------------------
    // FRICTION
    // --------------------------------------------------------

    physics.frictionCoefficient =
        0.08f;


    // --------------------------------------------------------
    // RESTING SPEED
    // --------------------------------------------------------

    physics.restingSpeed =
        0.05f;


    // ========================================================
    // SIMULATION
    // ========================================================

    Simulation simulation;


    simulation.ball.radius =
        0.50f;


    simulation.ball.mass =
        1.0f;


    // Default physics timestep.
    //
    // 120 Hz physics
    // 60 Hz display
    //
    simulation.physicsDt =
        1.0f / 120.0f;


    LoadScenario(
        simulation,
        1
    );


    // ========================================================
    // PHYSICS ACCUMULATOR
    // ========================================================

    double accumulator =
        0.0;


    // ========================================================
    // MAIN LOOP
    // ========================================================

    while (!WindowShouldClose())
    {
        // ====================================================
        // REAL DISPLAY FRAME TIME
        // ====================================================

        float frameTime =
            GetFrameTime();


        // Protect against extremely large frame times.
        frameTime =
            std::min(
                frameTime,
                0.10f
            );


        // ====================================================
        // CAMERA
        // ====================================================

        UpdateCamera(
            &camera,
            CAMERA_ORBITAL
        );


        // ====================================================
        // PAUSE
        // ====================================================

        if (IsKeyPressed(KEY_SPACE))
        {
            simulation.paused =
                !simulation.paused;
        }


        // ====================================================
        // RESET
        // ====================================================

        if (IsKeyPressed(KEY_R))
        {
            LoadScenario(
                simulation,
                simulation.scenario
            );


            accumulator =
                0.0;
        }


        // ====================================================
        // SCENARIOS
        // ====================================================

        if (IsKeyPressed(KEY_ONE))
        {
            LoadScenario(
                simulation,
                1
            );

            accumulator = 0.0;
        }


        if (IsKeyPressed(KEY_TWO))
        {
            LoadScenario(
                simulation,
                2
            );

            accumulator = 0.0;
        }


        if (IsKeyPressed(KEY_THREE))
        {
            LoadScenario(
                simulation,
                3
            );

            accumulator = 0.0;
        }


        if (IsKeyPressed(KEY_FOUR))
        {
            LoadScenario(
                simulation,
                4
            );

            accumulator = 0.0;
        }


        if (IsKeyPressed(KEY_FIVE))
        {
            LoadScenario(
                simulation,
                5
            );

            accumulator = 0.0;
        }


        // ====================================================
        // PHYSICS TIMESTEP
        // ====================================================
        //
        // UP:
        //     dt / 2
        //
        // DOWN:
        //     dt * 2
        //
        // Rendering remains at 60 FPS.
        //
        // ====================================================

        if (IsKeyPressed(KEY_UP))
        {
            simulation.physicsDt *=
                0.5f;


            simulation.physicsDt =
                std::max(
                    simulation.physicsDt,
                    1.0f / 1000.0f
                );
        }


        if (IsKeyPressed(KEY_DOWN))
        {
            simulation.physicsDt *=
                2.0f;


            simulation.physicsDt =
                std::min(
                    simulation.physicsDt,
                    1.0f / 15.0f
                );
        }


        // ====================================================
        // CUSTOM CONDITIONS
        // ====================================================

        if (simulation.paused &&
            simulation.scenario == 5)
        {
            UpdateCustomControls(
                box,
                simulation.ball
            );


            if (IsKeyPressed(KEY_ENTER))
            {
                LoadScenario(
                    simulation,
                    5
                );


                accumulator =
                    0.0;
            }
        }


        // ====================================================
        // PHYSICS
        // ====================================================

        if (!simulation.paused)
        {
            accumulator +=
                frameTime;


            accumulator =
                std::min(
                    accumulator,
                    0.25
                );


            while (
                accumulator >=
                simulation.physicsDt)
            {
                bool turbulentWind =
                    simulation.scenario == 5;


                SimulatePhysics(
                    simulation,
                    box,
                    physics,
                    turbulentWind
                );


                accumulator -=
                    simulation.physicsDt;
            }
        }


        // ====================================================
        // DRAW
        // ====================================================

        BeginDrawing();


        ClearBackground(
            RAYWHITE
        );


        // ====================================================
        // 3D
        // ====================================================

        BeginMode3D(
            camera
        );


        // Box
        DrawSimulationBox(
            box
        );


        // Coordinate axes
        DrawAxes();


        // Ground reference
        DrawGrid(
            20,
            1.0f
        );


        // ====================================================
        // BALL
        // ====================================================

        DrawSphere(
            simulation.ball.position,
            simulation.ball.radius,
            RED
        );


        DrawSphereWires(
            simulation.ball.position,
            simulation.ball.radius,
            16,
            16,
            MAROON
        );


        // ====================================================
        // VELOCITY
        // ====================================================

        DrawVelocityVector(
            simulation.ball
        );


        EndMode3D();


        // ====================================================
        // UI
        // ====================================================

        DrawText(
            "PHYSICALLY BASED BALL SIMULATION",
            10,
            30,
            24,
            BLACK
        );


        DrawText(
            "SPACE: Pause / Resume",
            10,
            70,
            18,
            DARKGRAY
        );


        DrawText(
            "R: Reset     1-5: Scenarios",
            10,
            95,
            18,
            DARKGRAY
        );


        DrawText(
            "UP / DOWN: Physics timestep",
            10,
            120,
            18,
            DARKGRAY
        );


        // ----------------------------------------------------
        // Physics timestep
        // ----------------------------------------------------

        std::string dtText =
            "Physics dt: " +
            FormatFloat(
                simulation.physicsDt,
                6
            ) +
            " s";


        DrawText(
            dtText.c_str(),
            10,
            160,
            18,
            BLUE
        );


        // ----------------------------------------------------
        // Simulation time
        // ----------------------------------------------------

        std::string timeText =
            "Simulation time: " +
            FormatFloat(
                static_cast<float>(
                    simulation.simulationTime
                ),
                2
            ) +
            " s";


        DrawText(
            timeText.c_str(),
            10,
            185,
            18,
            BLUE
        );


        // ----------------------------------------------------
        // Speed
        // ----------------------------------------------------

        float speed =
            VecLength(
                simulation.ball.velocity
            );


        std::string speedText =
            "Ball speed: " +
            FormatFloat(
                speed,
                2
            ) +
            " m/s";


        DrawText(
            speedText.c_str(),
            10,
            210,
            18,
            BLUE
        );


        // ----------------------------------------------------
        // Collision count
        // ----------------------------------------------------

        std::string collisionText =
            "Collisions: " +
            std::to_string(
                simulation.collisionCount
            );


        DrawText(
            collisionText.c_str(),
            10,
            235,
            18,
            BLUE
        );


        // ----------------------------------------------------
        // Scenario
        // ----------------------------------------------------

        std::string scenarioText =
            "Scenario: " +
            std::to_string(
                simulation.scenario
            );


        DrawText(
            scenarioText.c_str(),
            10,
            260,
            18,
            DARKBLUE
        );


        // ----------------------------------------------------
        // Wind
        // ----------------------------------------------------

        if (simulation.scenario == 5)
        {
            DrawText(
                "Wind: turbulent spatial/temporal field",
                10,
                285,
                18,
                DARKGREEN
            );
        }
        else
        {
            DrawText(
                "Wind: constant force",
                10,
                285,
                18,
                DARKGREEN
            );
        }


        // ----------------------------------------------------
        // State
        // ----------------------------------------------------

        if (simulation.paused)
        {
            DrawText(
                "PAUSED",
                10,
                325,
                22,
                ORANGE
            );
        }
        else
        {
            DrawText(
                "RUNNING",
                10,
                325,
                22,
                GREEN
            );
        }


        // ====================================================
        // SCENARIO 5 INFORMATION
        // ====================================================

        if (simulation.scenario == 5)
        {
            DrawText(
                "CUSTOM INITIAL CONDITIONS",
                850,
                40,
                18,
                DARKBLUE
            );


            std::string positionText =
                "Position: (" +
                FormatFloat(
                    customInitialPosition.x,
                    2
                ) +
                ", " +
                FormatFloat(
                    customInitialPosition.y,
                    2
                ) +
                ", " +
                FormatFloat(
                    customInitialPosition.z,
                    2
                ) +
                ")";


            DrawText(
                positionText.c_str(),
                850,
                70,
                16,
                DARKGRAY
            );


            std::string velocityText =
                "Velocity: (" +
                FormatFloat(
                    customInitialVelocity.x,
                    2
                ) +
                ", " +
                FormatFloat(
                    customInitialVelocity.y,
                    2
                ) +
                ", " +
                FormatFloat(
                    customInitialVelocity.z,
                    2
                ) +
                ")";


            DrawText(
                velocityText.c_str(),
                850,
                95,
                16,
                DARKGRAY
            );


            DrawText(
                "Position: A/D  W/S  Q/E",
                850,
                130,
                15,
                DARKGRAY
            );


            DrawText(
                "Velocity: J/L  I/K  U/O",
                850,
                155,
                15,
                DARKGRAY
            );


            DrawText(
                "ENTER: Apply custom values",
                850,
                180,
                15,
                DARKGRAY
            );
        }


        // ====================================================
        // POSITION
        // ====================================================

        std::string positionText =
            "Ball position: (" +
            FormatFloat(
                simulation.ball.position.x,
                2
            ) +
            ", " +
            FormatFloat(
                simulation.ball.position.y,
                2
            ) +
            ", " +
            FormatFloat(
                simulation.ball.position.z,
                2
            ) +
            ")";


        DrawText(
            positionText.c_str(),
            10,
            screenHeight - 55,
            16,
            DARKGRAY
        );


        // ====================================================
        // VELOCITY
        // ====================================================

        std::string velocityText =
            "Ball velocity: (" +
            FormatFloat(
                simulation.ball.velocity.x,
                2
            ) +
            ", " +
            FormatFloat(
                simulation.ball.velocity.y,
                2
            ) +
            ", " +
            FormatFloat(
                simulation.ball.velocity.z,
                2
            ) +
            ") m/s";


        DrawText(
            velocityText.c_str(),
            10,
            screenHeight - 30,
            16,
            DARKGRAY
        );


        // ====================================================
        // FPS
        // ====================================================

        std::string fpsText =
            std::to_string(
                GetFPS()
            ) +
            " FPS";


        DrawText(
            fpsText.c_str(),
            screenWidth - 100,
            30,
            18,
            GREEN
        );


        EndDrawing();
    }


    // ========================================================
    // CLEANUP
    // ========================================================

    CloseWindow();


    return 0;
}