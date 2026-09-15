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
// Based on concepts from:
// Donald House & John C. Keyser
// Foundations of Physically Based Modeling and Animation
//
// Features:
// - Explicit Euler integration
// - Gravity
// - Air resistance
// - Constant wind
// - Optional turbulent wind field
// - Fractional timestep collision detection
// - Restitution
// - Coulomb friction
// - Six-sided closed box
// - Multiple scenarios
// - User-adjustable physics timestep
//
// ============================================================


// ============================================================
// VECTOR HELPERS
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
    return std::sqrt(VecDot(a, a));
}

static Vector3 VecNormalize(Vector3 a)
{
    float len = VecLength(a);

    if (len < 0.000001f)
        return {0.0f, 0.0f, 0.0f};

    return VecScale(a, 1.0f / len);
}

static Vector3 VecProject(Vector3 v, Vector3 n)
{
    return VecScale(n, VecDot(v, n));
}

static Vector3 VecReject(Vector3 v, Vector3 n)
{
    return VecSub(v, VecProject(v, n));
}


// ============================================================
// DATA STRUCTURES
// ============================================================

struct Ball
{
    Vector3 position;
    Vector3 velocity;

    float radius;
    float mass;
};

struct Box
{
    Vector3 center;

    float width;
    float height;
    float depth;
};

struct PhysicsParameters
{
    // Gravity
    Vector3 gravity;

    // Linear air resistance:
    // F_drag = -k * v
    float dragCoefficient;

    // Constant wind force
    Vector3 windForce;

    // Collision properties
    float restitution;
    float frictionCoefficient;

    // Resting-contact threshold
    float restingSpeed;
};

struct Collision
{
    bool hit;

    float time;

    // Normal points INTO the box.
    Vector3 normal;
};

struct Simulation
{
    Ball ball;

    double simulationTime;

    unsigned long long collisionCount;

    int scenario;

    bool paused;

    float physicsDt;
};


// ============================================================
// GLOBAL CUSTOM INITIAL CONDITIONS
// ============================================================

Vector3 customInitialPosition = {
    0.0f,
    2.0f,
    0.0f
};

Vector3 customInitialVelocity = {
    4.0f,
    6.0f,
    3.0f
};


// ============================================================
// BOX LIMITS
// ============================================================

static float MinX(const Box& box, const Ball& ball)
{
    return box.center.x -
           box.width * 0.5f +
           ball.radius;
}

static float MaxX(const Box& box, const Ball& ball)
{
    return box.center.x +
           box.width * 0.5f -
           ball.radius;
}

static float MinY(const Box& box, const Ball& ball)
{
    return box.center.y -
           box.height * 0.5f +
           ball.radius;
}

static float MaxY(const Box& box, const Ball& ball)
{
    return box.center.y +
           box.height * 0.5f -
           ball.radius;
}

static float MinZ(const Box& box, const Ball& ball)
{
    return box.center.z -
           box.depth * 0.5f +
           ball.radius;
}

static float MaxZ(const Box& box, const Ball& ball)
{
    return box.center.z +
           box.depth * 0.5f -
           ball.radius;
}


// ============================================================
// WIND FIELD
// ============================================================
//
// Scenario 5 uses a spatially and temporally varying wind.
//
// This provides the creative extension:
// F_wind = F_base + F_turbulent(x,t)
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

    float t = static_cast<float>(time);

    Vector3 turbulent = {
        1.8f * std::sin(0.70f * x + 1.10f * t)
            * std::cos(0.45f * z),

        1.0f * std::sin(0.60f * y + 0.80f * t),

        1.5f * std::cos(0.55f * z + 1.30f * t)
            * std::sin(0.40f * x)
    };

    return VecAdd(physics.windForce, turbulent);
}


// ============================================================
// FORCE CALCULATION
// ============================================================
//
// F_gravity = m*g
//
// F_drag = -k*v
//
// F_wind = constant wind or turbulent wind
//
// a = F_total / m
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
// Prevents the ball from generating thousands of collisions
// when it has settled on a surface.
//
// This is NOT used to replace collision detection.
// Collision detection still uses fractional timesteps.
//
// ============================================================

static void ApplyRestingContactConstraints(
    Ball& ball,
    const Box& box,
    Vector3& acceleration,
    float restingSpeed)
{
    const float epsilon = 0.0005f;

    float minX = MinX(box, ball);
    float maxX = MaxX(box, ball);

    float minY = MinY(box, ball);
    float maxY = MaxY(box, ball);

    float minZ = MinZ(box, ball);
    float maxZ = MaxZ(box, ball);


    // --------------------------------------------------------
    // X MIN WALL
    // --------------------------------------------------------

    if (ball.position.x <= minX + epsilon &&
        ball.velocity.x <= restingSpeed)
    {
        if (acceleration.x < 0.0f)
        {
            ball.position.x = minX;

            if (std::fabs(ball.velocity.x) < restingSpeed ||
                ball.velocity.x < 0.0f)
            {
                ball.velocity.x = 0.0f;
            }

            acceleration.x = 0.0f;
        }
    }


    // --------------------------------------------------------
    // X MAX WALL
    // --------------------------------------------------------

    if (ball.position.x >= maxX - epsilon &&
        ball.velocity.x >= -restingSpeed)
    {
        if (acceleration.x > 0.0f)
        {
            ball.position.x = maxX;

            if (std::fabs(ball.velocity.x) < restingSpeed ||
                ball.velocity.x > 0.0f)
            {
                ball.velocity.x = 0.0f;
            }

            acceleration.x = 0.0f;
        }
    }


    // --------------------------------------------------------
    // Y MIN WALL / FLOOR
    // --------------------------------------------------------

    if (ball.position.y <= minY + epsilon &&
        ball.velocity.y <= restingSpeed)
    {
        if (acceleration.y < 0.0f)
        {
            ball.position.y = minY;

            if (std::fabs(ball.velocity.y) < restingSpeed ||
                ball.velocity.y < 0.0f)
            {
                ball.velocity.y = 0.0f;
            }

            acceleration.y = 0.0f;
        }
    }


    // --------------------------------------------------------
    // Y MAX WALL / CEILING
    // --------------------------------------------------------

    if (ball.position.y >= maxY - epsilon &&
        ball.velocity.y >= -restingSpeed)
    {
        if (acceleration.y > 0.0f)
        {
            ball.position.y = maxY;

            if (std::fabs(ball.velocity.y) < restingSpeed ||
                ball.velocity.y > 0.0f)
            {
                ball.velocity.y = 0.0f;
            }

            acceleration.y = 0.0f;
        }
    }


    // --------------------------------------------------------
    // Z MIN WALL
    // --------------------------------------------------------

    if (ball.position.z <= minZ + epsilon &&
        ball.velocity.z <= restingSpeed)
    {
        if (acceleration.z < 0.0f)
        {
            ball.position.z = minZ;

            if (std::fabs(ball.velocity.z) < restingSpeed ||
                ball.velocity.z < 0.0f)
            {
                ball.velocity.z = 0.0f;
            }

            acceleration.z = 0.0f;
        }
    }


    // --------------------------------------------------------
    // Z MAX WALL
    // --------------------------------------------------------

    if (ball.position.z >= maxZ - epsilon &&
        ball.velocity.z >= -restingSpeed)
    {
        if (acceleration.z > 0.0f)
        {
            ball.position.z = maxZ;

            if (std::fabs(ball.velocity.z) < restingSpeed ||
                ball.velocity.z > 0.0f)
            {
                ball.velocity.z = 0.0f;
            }

            acceleration.z = 0.0f;
        }
    }
}


// ============================================================
// FRACTIONAL TIMESTEP COLLISION DETECTION
// ============================================================
//
// We predict:
//
//     x_predicted = x + v * dt
//
// If the predicted position crosses a wall, calculate:
//
//     t_collision = (wall - x) / v
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
    result.normal = {0.0f, 0.0f, 0.0f};

    float minX = MinX(box, ball);
    float maxX = MaxX(box, ball);

    float minY = MinY(box, ball);
    float maxY = MaxY(box, ball);

    float minZ = MinZ(box, ball);
    float maxZ = MaxZ(box, ball);

    Vector3 predicted = {
        ball.position.x + ball.velocity.x * dt,
        ball.position.y + ball.velocity.y * dt,
        ball.position.z + ball.velocity.z * dt
    };

    const float velocityEpsilon = 0.000001f;


    // --------------------------------------------------------
    // X MIN
    // --------------------------------------------------------

    if (ball.velocity.x < -velocityEpsilon &&
        predicted.x < minX)
    {
        float t =
            (minX - ball.position.x) /
            ball.velocity.x;

        if (t >= 0.0f && t <= dt)
        {
            result.hit = true;
            result.time = t;
            result.normal = {1.0f, 0.0f, 0.0f};
        }
    }


    // --------------------------------------------------------
    // X MAX
    // --------------------------------------------------------

    if (ball.velocity.x > velocityEpsilon &&
        predicted.x > maxX)
    {
        float t =
            (maxX - ball.position.x) /
            ball.velocity.x;

        if (t >= 0.0f && t <= result.time)
        {
            result.hit = true;
            result.time = t;
            result.normal = {-1.0f, 0.0f, 0.0f};
        }
    }


    // --------------------------------------------------------
    // Y MIN / FLOOR
    // --------------------------------------------------------

    if (ball.velocity.y < -velocityEpsilon &&
        predicted.y < minY)
    {
        float t =
            (minY - ball.position.y) /
            ball.velocity.y;

        if (t >= 0.0f && t <= result.time)
        {
            result.hit = true;
            result.time = t;
            result.normal = {0.0f, 1.0f, 0.0f};
        }
    }


    // --------------------------------------------------------
    // Y MAX / CEILING
    // --------------------------------------------------------

    if (ball.velocity.y > velocityEpsilon &&
        predicted.y > maxY)
    {
        float t =
            (maxY - ball.position.y) /
            ball.velocity.y;

        if (t >= 0.0f && t <= result.time)
        {
            result.hit = true;
            result.time = t;
            result.normal = {0.0f, -1.0f, 0.0f};
        }
    }


    // --------------------------------------------------------
    // Z MIN
    // --------------------------------------------------------

    if (ball.velocity.z < -velocityEpsilon &&
        predicted.z < minZ)
    {
        float t =
            (minZ - ball.position.z) /
            ball.velocity.z;

        if (t >= 0.0f && t <= result.time)
        {
            result.hit = true;
            result.time = t;
            result.normal = {0.0f, 0.0f, 1.0f};
        }
    }


    // --------------------------------------------------------
    // Z MAX
    // --------------------------------------------------------

    if (ball.velocity.z > velocityEpsilon &&
        predicted.z > maxZ)
    {
        float t =
            (maxZ - ball.position.z) /
            ball.velocity.z;

        if (t >= 0.0f && t <= result.time)
        {
            result.hit = true;
            result.time = t;
            result.normal = {0.0f, 0.0f, -1.0f};
        }
    }

    return result;
}


// ============================================================
// COLLISION RESPONSE
// ============================================================
//
// Normal impulse:
//
//     Jn = -(1+e) (v.n)
//
// Friction impulse is limited using:
//
//     |Jt| <= mu * Jn
//
// This gives a simple Coulomb friction model.
//
// ============================================================

static void ResolveCollision(
    Ball& ball,
    Vector3 normal,
    const PhysicsParameters& physics,
    unsigned long long& collisionCount)
{
    float normalVelocity =
        VecDot(ball.velocity, normal);

    // Only respond if moving INTO the wall.
    if (normalVelocity >= 0.0f)
        return;


    // --------------------------------------------------------
    // NORMAL IMPULSE
    // --------------------------------------------------------

    float normalImpulseMagnitude =
        -(1.0f + physics.restitution) *
        normalVelocity *
        ball.mass;

    Vector3 normalImpulse =
        VecScale(
            normal,
            normalImpulseMagnitude
        );


    // --------------------------------------------------------
    // TANGENTIAL VELOCITY
    // --------------------------------------------------------

    Vector3 tangentVelocity =
        VecReject(
            ball.velocity,
            normal
        );

    float tangentSpeed =
        VecLength(tangentVelocity);


    Vector3 frictionImpulse = {
        0.0f,
        0.0f,
        0.0f
    };


    // --------------------------------------------------------
    // COULOMB FRICTION
    // --------------------------------------------------------

    if (tangentSpeed > 0.000001f)
    {
        Vector3 tangentDirection =
            VecScale(
                tangentVelocity,
                1.0f / tangentSpeed
            );

        float desiredFrictionImpulse =
            tangentSpeed * ball.mass;

        float maximumFrictionImpulse =
            physics.frictionCoefficient *
            normalImpulseMagnitude;

        float actualFrictionImpulse =
            std::min(
                desiredFrictionImpulse,
                maximumFrictionImpulse
            );

        frictionImpulse =
            VecScale(
                tangentDirection,
                -actualFrictionImpulse
            );
    }


    // --------------------------------------------------------
    // APPLY IMPULSES
    // --------------------------------------------------------

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
}


// ============================================================
// EULER INTEGRATION FOR A PARTIAL TIME
// ============================================================
//
// Explicit Euler:
//
//     x_new = x + v * dt
//     v_new = v + a * dt
//
// ============================================================

static void IntegrateEuler(
    Ball& ball,
    const PhysicsParameters& physics,
    double simulationTime,
    float dt,
    bool turbulentWind)
{
    if (dt <= 0.0f)
        return;

    Vector3 acceleration =
        CalculateAcceleration(
            ball,
            physics,
            simulationTime,
            turbulentWind
        );

    // Basic Euler position update
    ball.position =
        VecAdd(
            ball.position,
            VecScale(
                ball.velocity,
                dt
            )
        );

    // Basic Euler velocity update
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
// SIMULATE PHYSICS
// ============================================================
//
// A single physics timestep may contain more than one
// collision. Therefore the timestep is split into:
//
//     collision fraction
//     collision response
//     remaining fraction
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
           collisionIterations < MAX_COLLISIONS_PER_STEP)
    {
        // ----------------------------------------------------
        // Calculate acceleration at current state
        // ----------------------------------------------------

        Vector3 acceleration =
            CalculateAcceleration(
                simulation.ball,
                physics,
                simulation.simulationTime,
                turbulentWind
            );


        // ----------------------------------------------------
        // Resting contact handling
        // ----------------------------------------------------

        ApplyRestingContactConstraints(
            simulation.ball,
            box,
            acceleration,
            physics.restingSpeed
        );


        // ----------------------------------------------------
        // Find earliest collision during remaining time
        // ----------------------------------------------------

        Collision collision =
            FindEarliestCollision(
                simulation.ball,
                box,
                remainingTime
            );


        // ----------------------------------------------------
        // No collision
        // ----------------------------------------------------

        if (!collision.hit)
        {
            IntegrateEuler(
                simulation.ball,
                physics,
                simulation.simulationTime,
                remainingTime,
                turbulentWind
            );

            simulation.simulationTime += remainingTime;

            remainingTime = 0.0f;

            break;
        }


        // ----------------------------------------------------
        // Collision occurs at fractional timestep
        // ----------------------------------------------------

        float collisionTime =
            std::max(
                0.0f,
                std::min(
                    collision.time,
                    remainingTime
                )
            );


        // ----------------------------------------------------
        // Integrate up to collision
        // ----------------------------------------------------

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


        // ----------------------------------------------------
        // Put ball exactly on collision boundary
        // ----------------------------------------------------

        if (collision.normal.x > 0.5f)
        {
            simulation.ball.position.x =
                MinX(box, simulation.ball);
        }
        else if (collision.normal.x < -0.5f)
        {
            simulation.ball.position.x =
                MaxX(box, simulation.ball);
        }

        if (collision.normal.y > 0.5f)
        {
            simulation.ball.position.y =
                MinY(box, simulation.ball);
        }
        else if (collision.normal.y < -0.5f)
        {
            simulation.ball.position.y =
                MaxY(box, simulation.ball);
        }

        if (collision.normal.z > 0.5f)
        {
            simulation.ball.position.z =
                MinZ(box, simulation.ball);
        }
        else if (collision.normal.z < -0.5f)
        {
            simulation.ball.position.z =
                MaxZ(box, simulation.ball);
        }


        // ----------------------------------------------------
        // Collision response
        // ----------------------------------------------------

        ResolveCollision(
            simulation.ball,
            collision.normal,
            physics,
            simulation.collisionCount
        );


        // ----------------------------------------------------
        // Move an extremely small amount INTO the box.
        //
        // This prevents floating-point precision from
        // detecting the same collision repeatedly.
        // ----------------------------------------------------

        const float positionEpsilon = 0.0001f;

        simulation.ball.position =
            VecAdd(
                simulation.ball.position,
                VecScale(
                    collision.normal,
                    positionEpsilon
                )
            );


        // ----------------------------------------------------
        // Remaining time
        // ----------------------------------------------------

        remainingTime -= collisionTime;

        // Avoid zero-time infinite loops.
        if (collisionTime < 0.000001f)
        {
            remainingTime -= 0.000001f;
        }

        collisionIterations++;
    }


    // --------------------------------------------------------
    // Extremely small numerical protection.
    //
    // This is NOT used as collision detection.
    // The actual collision detection above uses fractional
    // timesteps.
    // --------------------------------------------------------

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
// SCENARIO LOADING
// ============================================================

static void LoadScenario(
    Simulation& simulation,
    int scenario)
{
    simulation.scenario = scenario;

    simulation.simulationTime = 0.0;
    simulation.collisionCount = 0;

    simulation.paused = false;


    switch (scenario)
    {
        // ----------------------------------------------------
        // SCENARIO 1
        // Simple vertical drop
        // ----------------------------------------------------

        case 1:
        {
            simulation.ball.position = {
                0.0f,
                2.0f,
                0.0f
            };

            simulation.ball.velocity = {
                0.0f,
                0.0f,
                0.0f
            };

            break;
        }


        // ----------------------------------------------------
        // SCENARIO 2
        // Diagonal bouncing
        // ----------------------------------------------------

        case 2:
        {
            simulation.ball.position = {
                -3.0f,
                2.0f,
                -2.0f
            };

            simulation.ball.velocity = {
                6.0f,
                4.0f,
                3.5f
            };

            break;
        }


        // ----------------------------------------------------
        // SCENARIO 3
        // Strong 3D motion
        // ----------------------------------------------------

        case 3:
        {
            simulation.ball.position = {
                0.0f,
                2.0f,
                0.0f
            };

            simulation.ball.velocity = {
                7.0f,
                8.0f,
                5.0f
            };

            break;
        }


        // ----------------------------------------------------
        // SCENARIO 4
        // Designed to hit sides and ceiling
        // ----------------------------------------------------

        case 4:
        {
            simulation.ball.position = {
                -3.5f,
                3.5f,
                -2.5f
            };

            simulation.ball.velocity = {
                8.0f,
                7.0f,
                7.0f
            };

            break;
        }


        // ----------------------------------------------------
        // SCENARIO 5
        // User custom / turbulent wind
        // ----------------------------------------------------

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
            simulation.ball.position = {
                0.0f,
                2.0f,
                0.0f
            };

            simulation.ball.velocity = {
                0.0f,
                0.0f,
                0.0f
            };

            simulation.scenario = 1;

            break;
        }
    }
}


// ============================================================
// DRAW BOX
// ============================================================
//
// IMPORTANT:
//
// We intentionally DO NOT draw filled transparent cubes.
//
// Transparent cube faces can still write to the depth buffer
// and hide the red sphere.
//
// Wireframe gives a much clearer visualization for this
// assignment.
//
// ============================================================

static void DrawSimulationBox(const Box& box)
{
    DrawCubeWires(
        box.center,
        box.width,
        box.height,
        box.depth,
        BLUE
    );
}


// ============================================================
// DRAW AXES
// ============================================================

static void DrawAxes()
{
    float axisLength = 7.0f;

    // X axis
    DrawLine3D(
        {-axisLength, 0.0f, 0.0f},
        { axisLength, 0.0f, 0.0f},
        RED
    );

    // Y axis
    DrawLine3D(
        {0.0f, -axisLength, 0.0f},
        {0.0f,  axisLength, 0.0f},
        GREEN
    );

    // Z axis
    DrawLine3D(
        {0.0f, 0.0f, -axisLength},
        {0.0f, 0.0f,  axisLength},
        BLUE
    );
}


// ============================================================
// DRAW VELOCITY VECTOR
// ============================================================

static void DrawVelocityVector(const Ball& ball)
{
    float speed =
        VecLength(ball.velocity);

    if (speed < 0.01f)
        return;

    float vectorScale = 0.35f;

    Vector3 end = {
        ball.position.x +
            ball.velocity.x * vectorScale,

        ball.position.y +
            ball.velocity.y * vectorScale,

        ball.position.z +
            ball.velocity.z * vectorScale
    };

    DrawLine3D(
        ball.position,
        end,
        MAROON
    );

    DrawSphere(
        end,
        0.08f,
        MAROON
    );
}


// ============================================================
// STRING HELPERS
// ============================================================

static std::string FormatFloat(
    float value,
    int precision = 3)
{
    std::ostringstream stream;

    stream << std::fixed
           << std::setprecision(precision)
           << value;

    return stream.str();
}


// ============================================================
// CUSTOM INITIAL CONDITION CONTROLS
// ============================================================
//
// Scenario 5:
//
// Position:
// A / D = X
// W / S = Y
// Q / E = Z
//
// Velocity:
// J / L = VX
// I / K = VY
// U / O = VZ
//
// ENTER = Apply custom conditions
//
// Only active while paused.
//
// ============================================================

static void UpdateCustomControls(
    const Box& box,
    const Ball& ball)
{
    const float positionStep = 0.1f;
    const float velocityStep = 0.25f;

    if (IsKeyDown(KEY_A))
        customInitialPosition.x -= positionStep;

    if (IsKeyDown(KEY_D))
        customInitialPosition.x += positionStep;

    if (IsKeyDown(KEY_W))
        customInitialPosition.y += positionStep;

    if (IsKeyDown(KEY_S))
        customInitialPosition.y -= positionStep;

    if (IsKeyDown(KEY_Q))
        customInitialPosition.z -= positionStep;

    if (IsKeyDown(KEY_E))
        customInitialPosition.z += positionStep;


    if (IsKeyDown(KEY_J))
        customInitialVelocity.x -= velocityStep;

    if (IsKeyDown(KEY_L))
        customInitialVelocity.x += velocityStep;

    if (IsKeyDown(KEY_I))
        customInitialVelocity.y += velocityStep;

    if (IsKeyDown(KEY_K))
        customInitialVelocity.y -= velocityStep;

    if (IsKeyDown(KEY_U))
        customInitialVelocity.z -= velocityStep;

    if (IsKeyDown(KEY_O))
        customInitialVelocity.z += velocityStep;


    // Keep custom position inside the box.
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
    // --------------------------------------------------------
    // WINDOW
    // --------------------------------------------------------

    const int screenWidth = 1280;
    const int screenHeight = 720;

    InitWindow(
        screenWidth,
        screenHeight,
        "Physically Based Ball Simulation"
    );

    SetTargetFPS(60);


    // --------------------------------------------------------
    // CAMERA
    // --------------------------------------------------------

    Camera3D camera = {};

    camera.position = {
        13.0f,
        9.0f,
        13.0f
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

    camera.fovy = 45.0f;

    camera.projection =
        CAMERA_PERSPECTIVE;


    // --------------------------------------------------------
    // BOX
    // --------------------------------------------------------

    Box box;

    box.center = {
        0.0f,
        0.0f,
        0.0f
    };

    box.width = 10.0f;
    box.height = 6.0f;
    box.depth = 8.0f;


    // --------------------------------------------------------
    // PHYSICS PARAMETERS
    // --------------------------------------------------------

    PhysicsParameters physics;

    physics.gravity = {
        0.0f,
        -9.81f,
        0.0f
    };

    // Linear drag coefficient
    physics.dragCoefficient = 0.08f;

    // Constant wind force
    physics.windForce = {
        1.5f,
        0.0f,
        0.8f
    };

    // Coefficient of restitution
    physics.restitution = 0.72f;

    // Coulomb friction coefficient
    physics.frictionCoefficient = 0.25f;

    // Velocity threshold for resting contact
    physics.restingSpeed = 0.08f;


    // --------------------------------------------------------
    // SIMULATION
    // --------------------------------------------------------

    Simulation simulation;

    simulation.ball.radius = 0.5f;
    simulation.ball.mass = 1.0f;

    simulation.physicsDt =
        1.0f / 120.0f;

    LoadScenario(
        simulation,
        1
    );


    // --------------------------------------------------------
    // PHYSICS ACCUMULATOR
    // --------------------------------------------------------
    //
    // Rendering happens at approximately 60 FPS.
    //
    // Physics can run at:
    //
    // 1/30
    // 1/60
    // 1/120
    // 1/240
    //
    // without changing display speed.
    //
    // --------------------------------------------------------

    double accumulator = 0.0;


    // --------------------------------------------------------
    // MAIN LOOP
    // --------------------------------------------------------

    while (!WindowShouldClose())
    {
        // ====================================================
        // FRAME TIME
        // ====================================================

        float frameTime =
            GetFrameTime();

        // Avoid huge physics jumps if the window stalls.
        frameTime =
            std::min(
                frameTime,
                0.1f
            );


        // ====================================================
        // CAMERA
        // ====================================================
        //
        // Mouse controls orbital camera.
        //
        // This keeps the 3D visualization adjustable.
        //
        // ====================================================

        UpdateCamera(
            &camera,
            CAMERA_ORBITAL
        );


        // ====================================================
        // KEYBOARD CONTROLS
        // ====================================================

        // ----------------------------------------------------
        // Pause / Resume
        // ----------------------------------------------------

        if (IsKeyPressed(KEY_SPACE))
        {
            simulation.paused =
                !simulation.paused;
        }


        // ----------------------------------------------------
        // Reset current scenario
        // ----------------------------------------------------

        if (IsKeyPressed(KEY_R))
        {
            LoadScenario(
                simulation,
                simulation.scenario
            );

            accumulator = 0.0;
        }


        // ----------------------------------------------------
        // Scenarios
        // ----------------------------------------------------

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
        // PHYSICS TIMESTEP CONTROL
        // ====================================================
        //
        // UP    -> smaller timestep / more accurate
        // DOWN  -> larger timestep / less accurate
        //
        // Display speed remains based on real frame time.
        //
        // ====================================================

        if (IsKeyPressed(KEY_UP))
        {
            simulation.physicsDt *= 0.5f;

            simulation.physicsDt =
                std::max(
                    simulation.physicsDt,
                    1.0f / 1000.0f
                );
        }

        if (IsKeyPressed(KEY_DOWN))
        {
            simulation.physicsDt *= 2.0f;

            simulation.physicsDt =
                std::min(
                    simulation.physicsDt,
                    1.0f / 15.0f
                );
        }


        // ====================================================
        // CUSTOM CONTROLS
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

                accumulator = 0.0;
            }
        }


        // ====================================================
        // PHYSICS ACCUMULATION
        // ====================================================

        if (!simulation.paused)
        {
            accumulator += frameTime;

            // Limit accumulated time to prevent a spiral
            // after a long pause/window stall.
            accumulator =
                std::min(
                    accumulator,
                    0.25
                );


            while (accumulator >=
                   simulation.physicsDt)
            {
                bool turbulentWind =
                    (simulation.scenario == 5);

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
        // DRAWING
        // ====================================================

        BeginDrawing();

        ClearBackground(RAYWHITE);


        // ----------------------------------------------------
        // 3D WORLD
        // ----------------------------------------------------

        BeginMode3D(camera);


        // Box
        DrawSimulationBox(
            box
        );


        // Axes
        DrawAxes();


        // Ground reference grid
        DrawGrid(
            20,
            1.0f
        );


        // ----------------------------------------------------
        // BALL
        // ----------------------------------------------------
        //
        // Draw AFTER the box.
        //
        // Since the box is wireframe only, the ball cannot be
        // hidden by transparent box faces.
        //
        // ----------------------------------------------------

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


        // ----------------------------------------------------
        // VELOCITY VECTOR
        // ----------------------------------------------------

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
                3
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
        // Ball speed
        // ----------------------------------------------------

        float speed =
            VecLength(
                simulation.ball.velocity
            );

        std::string speedText =
            "Ball speed: " +
            FormatFloat(
                speed,
                3
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
        // Running / paused
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
        // CUSTOM SCENARIO INFORMATION
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


            std::string posText =
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
                posText.c_str(),
                850,
                70,
                16,
                DARKGRAY
            );


            std::string velText =
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
                velText.c_str(),
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
        // CURRENT BALL POSITION / VELOCITY
        // ====================================================

        std::string currentPosition =
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
            currentPosition.c_str(),
            10,
            screenHeight - 55,
            16,
            DARKGRAY
        );


        std::string currentVelocity =
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
            currentVelocity.c_str(),
            10,
            screenHeight - 30,
            16,
            DARKGRAY
        );


        // ----------------------------------------------------
        // FPS
        // ----------------------------------------------------

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