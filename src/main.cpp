#include "raylib.h"

#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>

// ============================================================
// PHYSICALLY BASED BALL SIMULATION
// Assignment 1
//
// Donald House & John C. Keyser
//
// USER-CONTROLLED VERSION
//
// Required:
//   - Gravity
//   - Air resistance
//   - Wind force
//   - Euler integration
//   - Adjustable timestep
//   - Fractional timestep collision detection
//   - Collision response
//   - Friction
//   - Restitution
//   - Six-sided box
//
// Novelty:
//   - Interactive initial-condition setup
//   - User-defined ball radius
//   - User-defined 3D wind force
//   - User-defined initial position
//   - User-defined initial velocity
//   - Simulation does not start until input is submitted
// ============================================================


// ============================================================
// VECTOR FUNCTIONS
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

    return VecScale(a, 1.0f / length);
}


static Vector3 VecProject(Vector3 v, Vector3 n)
{
    return VecScale(
        n,
        VecDot(v, n)
    );
}


static Vector3 VecReject(Vector3 v, Vector3 n)
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
    Vector3 gravity;

    float dragCoefficient;

    Vector3 windForce;

    float restitution;

    float frictionCoefficient;

    float restingSpeed;
};


// ============================================================
// COLLISION
// ============================================================

struct Collision
{
    bool hit;

    float time;

    Vector3 normal;
};


// ============================================================
// SIMULATION STATE
// ============================================================

struct Simulation
{
    Ball ball;

    double simulationTime;

    unsigned long long collisionCount;

    float physicsDt;

    bool paused;

    bool running;
};


// ============================================================
// USER INPUT FIELD
// ============================================================

struct InputField
{
    std::string label;

    std::string value;

    Rectangle rectangle;

    bool active;
};


// ============================================================
// GLOBAL SETUP VALUES
// ============================================================

static std::vector<InputField> inputFields;

static int activeField = 0;

static std::string inputError = "";


// ============================================================
// DEFAULT USER INPUT
// ============================================================
//
// These values are only starting values in the input boxes.
// The simulation does NOT begin automatically.
//
// ============================================================

static void InitializeInputFields()
{
    inputFields.clear();


    inputFields.push_back({
        "Initial Position X",
        "0.0",
        {420, 145, 280, 38},
        false
    });


    inputFields.push_back({
        "Initial Position Y",
        "1.0",
        {420, 195, 280, 38},
        false
    });


    inputFields.push_back({
        "Initial Position Z",
        "0.0",
        {420, 245, 280, 38},
        false
    });


    inputFields.push_back({
        "Initial Velocity X",
        "8.0",
        {420, 315, 280, 38},
        false
    });


    inputFields.push_back({
        "Initial Velocity Y",
        "9.0",
        {420, 365, 280, 38},
        false
    });


    inputFields.push_back({
        "Initial Velocity Z",
        "7.5",
        {420, 415, 280, 38},
        false
    });


    inputFields.push_back({
        "Ball Radius",
        "0.5",
        {420, 485, 280, 38},
        false
    });


    inputFields.push_back({
        "Wind X",
        "0.8",
        {420, 535, 280, 38},
        false
    });


    inputFields.push_back({
        "Wind Y",
        "0.25",
        {420, 585, 280, 38},
        false
    });


    inputFields.push_back({
        "Wind Z",
        "-0.6",
        {420, 635, 280, 38},
        false
    });


    activeField = 0;

    inputFields[0].active = true;

    inputError = "";
}


// ============================================================
// STRING → FLOAT
// ============================================================

static bool ParseFloat(
    const std::string& text,
    float& result)
{
    if (text.empty())
        return false;


    try
    {
        size_t position = 0;

        float value =
            std::stof(
                text,
                &position
            );


        if (position != text.length())
            return false;


        if (!std::isfinite(value))
            return false;


        result = value;

        return true;
    }
    catch (...)
    {
        return false;
    }
}


// ============================================================
// GET INPUT VALUE
// ============================================================

static bool GetInputValue(
    int index,
    float& value)
{
    if (index < 0 ||
        index >= static_cast<int>(
            inputFields.size()))
    {
        return false;
    }


    return ParseFloat(
        inputFields[index].value,
        value
    );
}


// ============================================================
// VALIDATE USER INPUT
// ============================================================

static bool ValidateInput(
    const Box& box,
    float radius)
{
    inputError = "";


    if (radius <= 0.0f)
    {
        inputError =
            "Ball radius must be greater than zero.";

        return false;
    }


    // Keep ball reasonably smaller than the box.
    if (radius >=
        std::min({
            box.width,
            box.height,
            box.depth
        }) / 2.0f)
    {
        inputError =
            "Ball radius is too large for the box.";

        return false;
    }


    float px, py, pz;


    if (!GetInputValue(0, px) ||
        !GetInputValue(1, py) ||
        !GetInputValue(2, pz))
    {
        inputError =
            "Initial position contains invalid values.";

        return false;
    }


    float minX =
        box.center.x -
        box.width / 2.0f +
        radius;


    float maxX =
        box.center.x +
        box.width / 2.0f -
        radius;


    float minY =
        box.center.y -
        box.height / 2.0f +
        radius;


    float maxY =
        box.center.y +
        box.height / 2.0f -
        radius;


    float minZ =
        box.center.z -
        box.depth / 2.0f +
        radius;


    float maxZ =
        box.center.z +
        box.depth / 2.0f -
        radius;


    if (px < minX ||
        px > maxX ||
        py < minY ||
        py > maxY ||
        pz < minZ ||
        pz > maxZ)
    {
        inputError =
            "Initial position must be inside the box.";

        return false;
    }


    // Validate velocity.
    float vx, vy, vz;


    if (!GetInputValue(3, vx) ||
        !GetInputValue(4, vy) ||
        !GetInputValue(5, vz))
    {
        inputError =
            "Initial velocity contains invalid values.";

        return false;
    }


    // Validate wind.
    float wx, wy, wz;


    if (!GetInputValue(7, wx) ||
        !GetInputValue(8, wy) ||
        !GetInputValue(9, wz))
    {
        inputError =
            "Wind force contains invalid values.";

        return false;
    }


    return true;
}


// ============================================================
// CREATE SIMULATION FROM USER INPUT
// ============================================================

static bool CreateSimulation(
    Simulation& simulation,
    const Box& box)
{
    float px, py, pz;

    float vx, vy, vz;

    float radius;

    float wx, wy, wz;


    if (!GetInputValue(0, px) ||
        !GetInputValue(1, py) ||
        !GetInputValue(2, pz) ||

        !GetInputValue(3, vx) ||
        !GetInputValue(4, vy) ||
        !GetInputValue(5, vz) ||

        !GetInputValue(6, radius) ||

        !GetInputValue(7, wx) ||
        !GetInputValue(8, wy) ||
        !GetInputValue(9, wz))
    {
        return false;
    }


    if (!ValidateInput(
            box,
            radius))
    {
        return false;
    }


    simulation.ball.position = {
        px,
        py,
        pz
    };


    simulation.ball.velocity = {
        vx,
        vy,
        vz
    };


    simulation.ball.radius =
        radius;


    simulation.ball.mass =
        1.0f;


    simulation.simulationTime =
        0.0;


    simulation.collisionCount =
        0;


    simulation.physicsDt =
        1.0f / 120.0f;


    simulation.paused =
        false;


    simulation.running =
        true;


    return true;
}


// ============================================================
// PHYSICS PARAMETERS
// ============================================================

static PhysicsParameters CreatePhysicsParameters()
{
    PhysicsParameters physics;


    // Gravity
    physics.gravity = {
        0.0f,
        -9.81f,
        0.0f
    };


    // Linear air resistance
    //
    // F_drag = -k v
    //
    physics.dragCoefficient =
        0.015f;


    // Read wind from user input.
    float wx, wy, wz;


    GetInputValue(7, wx);
    GetInputValue(8, wy);
    GetInputValue(9, wz);


    physics.windForce = {
        wx,
        wy,
        wz
    };


    // Restitution
    physics.restitution =
        0.88f;


    // Friction
    physics.frictionCoefficient =
        0.08f;


    // Resting threshold
    physics.restingSpeed =
        0.05f;


    return physics;
}


// ============================================================
// BOX BOUNDARIES
// ============================================================

static float MinX(
    const Box& box,
    const Ball& ball)
{
    return box.center.x -
           box.width / 2.0f +
           ball.radius;
}


static float MaxX(
    const Box& box,
    const Ball& ball)
{
    return box.center.x +
           box.width / 2.0f -
           ball.radius;
}


static float MinY(
    const Box& box,
    const Ball& ball)
{
    return box.center.y -
           box.height / 2.0f +
           ball.radius;
}


static float MaxY(
    const Box& box,
    const Ball& ball)
{
    return box.center.y +
           box.height / 2.0f -
           ball.radius;
}


static float MinZ(
    const Box& box,
    const Ball& ball)
{
    return box.center.z -
           box.depth / 2.0f +
           ball.radius;
}


static float MaxZ(
    const Box& box,
    const Ball& ball)
{
    return box.center.z +
           box.depth / 2.0f -
           ball.radius;
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
// ============================================================

static Vector3 CalculateAcceleration(
    const Ball& ball,
    const PhysicsParameters& physics)
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


    Vector3 totalForce =
        VecAdd(
            gravityForce,
            VecAdd(
                dragForce,
                physics.windForce
            )
        );


    return VecScale(
        totalForce,
        1.0f / ball.mass
    );
}


// ============================================================
// FRACTIONAL TIMESTEP COLLISION DETECTION
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


    const float eps =
        0.000001f;


    // --------------------------------------------------------
    // X MIN
    // --------------------------------------------------------

    if (ball.velocity.x < -eps &&
        predicted.x < minX)
    {
        float tc =
            (minX - ball.position.x) /
            ball.velocity.x;


        if (tc >= 0.0f &&
            tc <= result.time)
        {
            result.hit = true;

            result.time = tc;

            result.normal = {
                1.0f,
                0.0f,
                0.0f
            };
        }
    }


    // --------------------------------------------------------
    // X MAX
    // --------------------------------------------------------

    if (ball.velocity.x > eps &&
        predicted.x > maxX)
    {
        float tc =
            (maxX - ball.position.x) /
            ball.velocity.x;


        if (tc >= 0.0f &&
            tc <= result.time)
        {
            result.hit = true;

            result.time = tc;

            result.normal = {
                -1.0f,
                0.0f,
                0.0f
            };
        }
    }


    // --------------------------------------------------------
    // FLOOR
    // --------------------------------------------------------

    if (ball.velocity.y < -eps &&
        predicted.y < minY)
    {
        float tc =
            (minY - ball.position.y) /
            ball.velocity.y;


        if (tc >= 0.0f &&
            tc <= result.time)
        {
            result.hit = true;

            result.time = tc;

            result.normal = {
                0.0f,
                1.0f,
                0.0f
            };
        }
    }


    // --------------------------------------------------------
    // CEILING
    // --------------------------------------------------------

    if (ball.velocity.y > eps &&
        predicted.y > maxY)
    {
        float tc =
            (maxY - ball.position.y) /
            ball.velocity.y;


        if (tc >= 0.0f &&
            tc <= result.time)
        {
            result.hit = true;

            result.time = tc;

            result.normal = {
                0.0f,
                -1.0f,
                0.0f
            };
        }
    }


    // --------------------------------------------------------
    // Z MIN
    // --------------------------------------------------------

    if (ball.velocity.z < -eps &&
        predicted.z < minZ)
    {
        float tc =
            (minZ - ball.position.z) /
            ball.velocity.z;


        if (tc >= 0.0f &&
            tc <= result.time)
        {
            result.hit = true;

            result.time = tc;

            result.normal = {
                0.0f,
                0.0f,
                1.0f
            };
        }
    }


    // --------------------------------------------------------
    // Z MAX
    // --------------------------------------------------------

    if (ball.velocity.z > eps &&
        predicted.z > maxZ)
    {
        float tc =
            (maxZ - ball.position.z) /
            ball.velocity.z;


        if (tc >= 0.0f &&
            tc <= result.time)
        {
            result.hit = true;

            result.time = tc;

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
// EULER INTEGRATION
// ============================================================
//
// x_(n+1) = x_n + v_n dt
//
// v_(n+1) = v_n + a_n dt
//
// ============================================================

static void IntegrateEuler(
    Ball& ball,
    const PhysicsParameters& physics,
    float dt)
{
    if (dt <= 0.0f)
        return;


    Vector3 acceleration =
        CalculateAcceleration(
            ball,
            physics
        );


    // Explicit Euler position
    ball.position =
        VecAdd(
            ball.position,
            VecScale(
                ball.velocity,
                dt
            )
        );


    // Explicit Euler velocity
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
// COLLISION RESPONSE
// ============================================================
//
// Normal:
//
//     Jn = -(1 + e)m(v.n)
//
// Tangential/friction:
//
//     |Jt| <= mu |Jn|
//
// ============================================================

static void ResolveCollision(
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


    // Ball is already moving away.
    if (normalVelocity >= 0.0f)
        return;


    // --------------------------------------------------------
    // NORMAL IMPULSE
    // --------------------------------------------------------

    float normalImpulse =
        -(1.0f +
          physics.restitution) *
        normalVelocity *
        ball.mass;


    Vector3 impulseNormal =
        VecScale(
            normal,
            normalImpulse
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
        VecLength(
            tangentVelocity
        );


    Vector3 impulseFriction = {
        0.0f,
        0.0f,
        0.0f
    };


    if (tangentSpeed > 0.000001f)
    {
        Vector3 tangentDirection =
            VecScale(
                tangentVelocity,
                1.0f / tangentSpeed
            );


        float requiredImpulse =
            ball.mass *
            tangentSpeed;


        float maximumImpulse =
            physics.frictionCoefficient *
            normalImpulse;


        float frictionMagnitude =
            std::min(
                requiredImpulse,
                maximumImpulse
            );


        impulseFriction =
            VecScale(
                tangentDirection,
                -frictionMagnitude
            );
    }


    // --------------------------------------------------------
    // TOTAL IMPULSE
    // --------------------------------------------------------

    Vector3 totalImpulse =
        VecAdd(
            impulseNormal,
            impulseFriction
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
// FRACTIONAL TIMESTEP SIMULATION
// ============================================================
//
// If:
//
//     dt = 0.008333
//
// and collision happens at:
//
//     tc = 0.0032
//
// then:
//
//     integrate 0.0032
//     collide
//     integrate remaining 0.005133
//
// ============================================================

static void SimulatePhysics(
    Simulation& simulation,
    const Box& box,
    const PhysicsParameters& physics)
{
    float remaining =
        simulation.physicsDt;


    const int MAX_COLLISIONS =
        8;


    int collisionIterations = 0;


    while (
        remaining > 0.000001f &&
        collisionIterations <
            MAX_COLLISIONS)
    {
        Collision collision =
            FindEarliestCollision(
                simulation.ball,
                box,
                remaining
            );


        // ----------------------------------------------------
        // NO COLLISION
        // ----------------------------------------------------

        if (!collision.hit)
        {
            IntegrateEuler(
                simulation.ball,
                physics,
                remaining
            );


            simulation.simulationTime +=
                remaining;


            remaining = 0.0f;

            break;
        }


        // ----------------------------------------------------
        // FRACTIONAL COLLISION TIME
        // ----------------------------------------------------

        float collisionTime =
            std::max(
                0.0f,
                std::min(
                    collision.time,
                    remaining
                )
            );


        // ----------------------------------------------------
        // INTEGRATE TO COLLISION
        // ----------------------------------------------------

        if (collisionTime >
            0.000001f)
        {
            IntegrateEuler(
                simulation.ball,
                physics,
                collisionTime
            );


            simulation.simulationTime +=
                collisionTime;
        }


        // ----------------------------------------------------
        // PUT BALL EXACTLY ON SURFACE
        // ----------------------------------------------------

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


        // ----------------------------------------------------
        // COLLISION RESPONSE
        // ----------------------------------------------------

        ResolveCollision(
            simulation.ball,
            collision.normal,
            physics,
            simulation.collisionCount
        );


        // Small numerical separation.
        simulation.ball.position =
            VecAdd(
                simulation.ball.position,
                VecScale(
                    collision.normal,
                    0.00001f
                )
            );


        // ----------------------------------------------------
        // REMAINING TIMESTEP
        // ----------------------------------------------------

        remaining -=
            collisionTime;


        // Prevent zero-time loops.
        if (collisionTime <
            0.000001f)
        {
            remaining -=
                0.000001f;
        }


        collisionIterations++;
    }


    // --------------------------------------------------------
    // FINAL SAFETY CLAMP
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
// DRAW BOX
// ============================================================

static void DrawSimulationBox(
    const Box& box)
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
    float length = 7.0f;


    DrawLine3D(
        {-length, 0, 0},
        { length, 0, 0},
        RED
    );


    DrawLine3D(
        {0, -length, 0},
        {0,  length, 0},
        GREEN
    );


    DrawLine3D(
        {0, 0, -length},
        {0, 0,  length},
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


    float scale =
        0.30f;


    Vector3 endpoint =
        VecAdd(
            ball.position,
            VecScale(
                ball.velocity,
                scale
            )
        );


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
        << std::setprecision(
            precision
        )
        << value;


    return stream.str();
}


// ============================================================
// DRAW INPUT SCREEN
// ============================================================

static void DrawInputScreen(
    int screenWidth,
    int screenHeight)
{
    ClearBackground(
        RAYWHITE
    );


    // --------------------------------------------------------
    // TITLE
    // --------------------------------------------------------

    DrawText(
        "PHYSICALLY BASED BALL SIMULATION",
        35,
        25,
        28,
        BLACK
    );


    DrawText(
        "INITIAL CONDITIONS",
        35,
        65,
        22,
        DARKBLUE
    );


    DrawText(
        "Enter the physical initial conditions.",
        35,
        95,
        17,
        DARKGRAY
    );


    DrawText(
        "The simulation will begin only after pressing ENTER.",
        35,
        118,
        17,
        DARKGRAY
    );


    // --------------------------------------------------------
    // INPUT FIELDS
    // --------------------------------------------------------

    for (size_t i = 0;
         i < inputFields.size();
         i++)
    {
        InputField& field =
            inputFields[i];


        DrawText(
            field.label.c_str(),
            80,
            static_cast<int>(
                field.rectangle.y + 8
            ),
            17,
            DARKGRAY
        );


        Color borderColor =
            field.active
                ? DARKBLUE
                : GRAY;


        DrawRectangleLinesEx(
            field.rectangle,
            2,
            borderColor
        );


        DrawText(
            field.value.c_str(),
            static_cast<int>(
                field.rectangle.x + 10
            ),
            static_cast<int>(
                field.rectangle.y + 8
            ),
            18,
            BLACK
        );
    }


    // --------------------------------------------------------
    // GROUP LABELS
    // --------------------------------------------------------

    DrawText(
        "POSITION",
        80,
        125,
        15,
        BLUE
    );


    DrawText(
        "VELOCITY",
        80,
        295,
        15,
        BLUE
    );


    DrawText(
        "BALL",
        80,
        465,
        15,
        BLUE
    );


    DrawText(
        "WIND FORCE",
        80,
        515,
        15,
        BLUE
    );


    // --------------------------------------------------------
    // ERROR
    // --------------------------------------------------------

    if (!inputError.empty())
    {
        DrawText(
            inputError.c_str(),
            740,
            650,
            17,
            RED
        );
    }


    // --------------------------------------------------------
    // INSTRUCTIONS
    // --------------------------------------------------------

    DrawText(
        "TAB / UP / DOWN : Select field",
        750,
        180,
        17,
        DARKGRAY
    );


    DrawText(
        "Type numeric values",
        750,
        210,
        17,
        DARKGRAY
    );


    DrawText(
        "ENTER : Start simulation",
        750,
        240,
        17,
        DARKBLUE
    );


    DrawText(
        "ESC : Exit",
        750,
        270,
        17,
        DARKGRAY
    );


    // --------------------------------------------------------
    // PHYSICS CONSTANTS
    // --------------------------------------------------------

    DrawText(
        "PHYSICS CONSTANTS",
        750,
        340,
        18,
        DARKBLUE
    );


    DrawText(
        "Gravity: (0, -9.81, 0) m/s^2",
        750,
        375,
        16,
        DARKGRAY
    );


    DrawText(
        "Air resistance: linear drag",
        750,
        400,
        16,
        DARKGRAY
    );


    DrawText(
        "Restitution: 0.88",
        750,
        425,
        16,
        DARKGRAY
    );


    DrawText(
        "Friction: 0.08",
        750,
        450,
        16,
        DARKGRAY
    );


    // --------------------------------------------------------
    // EXAMPLE
    // --------------------------------------------------------

    DrawText(
        "TIP",
        750,
        500,
        18,
        DARKGREEN
    );


    DrawText(
        "Try velocity values such as:",
        750,
        530,
        16,
        DARKGRAY
    );


    DrawText(
        "(8, 9, 7.5)",
        750,
        555,
        17,
        DARKGREEN
    );


    DrawText(
        "and wind such as:",
        750,
        580,
        16,
        DARKGRAY
    );


    DrawText(
        "(0.8, 0.25, -0.6)",
        750,
        605,
        17,
        DARKGREEN
    );
}


// ============================================================
// HANDLE INPUT SCREEN
// ============================================================

static bool HandleInputScreen(
    const Box& box)
{
    // --------------------------------------------------------
    // MOUSE SELECTION
    // --------------------------------------------------------

    if (IsMouseButtonPressed(
            MOUSE_BUTTON_LEFT))
    {
        Vector2 mouse =
            GetMousePosition();


        for (size_t i = 0;
             i < inputFields.size();
             i++)
        {
            if (CheckCollisionPointRec(
                    mouse,
                    inputFields[i].rectangle))
            {
                activeField =
                    static_cast<int>(i);


                for (auto& field :
                     inputFields)
                {
                    field.active =
                        false;
                }


                inputFields[i].active =
                    true;


                inputError = "";

                break;
            }
        }
    }


    // --------------------------------------------------------
    // TAB
    // --------------------------------------------------------

    if (IsKeyPressed(KEY_TAB))
    {
        inputFields[activeField].active =
            false;


        activeField++;

        if (activeField >=
            static_cast<int>(
                inputFields.size()))
        {
            activeField = 0;
        }


        inputFields[activeField].active =
            true;
    }


    // --------------------------------------------------------
    // UP / DOWN
    // --------------------------------------------------------

    if (IsKeyPressed(KEY_DOWN))
    {
        inputFields[activeField].active =
            false;


        activeField++;

        if (activeField >=
            static_cast<int>(
                inputFields.size()))
        {
            activeField = 0;
        }


        inputFields[activeField].active =
            true;
    }


    if (IsKeyPressed(KEY_UP))
    {
        inputFields[activeField].active =
            false;


        activeField--;

        if (activeField < 0)
        {
            activeField =
                static_cast<int>(
                    inputFields.size()
                ) - 1;
        }


        inputFields[activeField].active =
            true;
    }


    // --------------------------------------------------------
    // TEXT INPUT
    // --------------------------------------------------------

    int character =
        GetCharPressed();


    while (character > 0)
    {
        // Allow:
        //
        // 0-9
        // -
        // .
        //
        if ((character >= '0' &&
             character <= '9') ||
            character == '-' ||
            character == '.')
        {
            inputFields[
                activeField
            ].value +=
                static_cast<char>(
                    character
                );


            inputError = "";
        }


        character =
            GetCharPressed();
    }


    // --------------------------------------------------------
    // BACKSPACE
    // --------------------------------------------------------

    if (IsKeyPressed(
            KEY_BACKSPACE))
    {
        std::string& value =
            inputFields[
                activeField
            ].value;


        if (!value.empty())
        {
            value.pop_back();
        }


        inputError = "";
    }


    // --------------------------------------------------------
    // START SIMULATION
    // --------------------------------------------------------

    if (IsKeyPressed(KEY_ENTER))
    {
        return true;
    }


    return false;
}


// ============================================================
// DRAW SIMULATION UI
// ============================================================

static void DrawSimulationUI(
    const Simulation& simulation,
    const PhysicsParameters& physics,
    int screenWidth,
    int screenHeight)
{
    // --------------------------------------------------------
    // TITLE
    // --------------------------------------------------------

    DrawText(
        "PHYSICALLY BASED BALL SIMULATION",
        10,
        25,
        24,
        BLACK
    );


    DrawText(
        "SPACE: Pause / Resume",
        10,
        65,
        17,
        DARKGRAY
    );


    DrawText(
        "R: Return to Initial Conditions",
        10,
        90,
        17,
        DARKGRAY
    );


    DrawText(
        "UP / DOWN: Physics timestep",
        10,
        115,
        17,
        DARKGRAY
    );


    // --------------------------------------------------------
    // TIMESTEP
    // --------------------------------------------------------

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
        150,
        17,
        BLUE
    );


    // --------------------------------------------------------
    // SIMULATION TIME
    // --------------------------------------------------------

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
        175,
        17,
        BLUE
    );


    // --------------------------------------------------------
    // SPEED
    // --------------------------------------------------------

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
        200,
        17,
        BLUE
    );


    // --------------------------------------------------------
    // COLLISIONS
    // --------------------------------------------------------

    std::string collisionText =
        "Collisions: " +
        std::to_string(
            simulation.collisionCount
        );


    DrawText(
        collisionText.c_str(),
        10,
        225,
        17,
        BLUE
    );


    // --------------------------------------------------------
    // BALL RADIUS
    // --------------------------------------------------------

    std::string radiusText =
        "Ball radius: " +
        FormatFloat(
            simulation.ball.radius,
            2
        );


    DrawText(
        radiusText.c_str(),
        10,
        250,
        17,
        DARKBLUE
    );


    // --------------------------------------------------------
    // WIND
    // --------------------------------------------------------

    std::string windText =
        "Wind: (" +
        FormatFloat(
            physics.windForce.x,
            2
        ) +
        ", " +
        FormatFloat(
            physics.windForce.y,
            2
        ) +
        ", " +
        FormatFloat(
            physics.windForce.z,
            2
        ) +
        ")";


    DrawText(
        windText.c_str(),
        10,
        275,
        17,
        DARKGREEN
    );


    // --------------------------------------------------------
    // POSITION
    // --------------------------------------------------------

    std::string positionText =
        "Position: (" +
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


    // --------------------------------------------------------
    // VELOCITY
    // --------------------------------------------------------

    std::string velocityText =
        "Velocity: (" +
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


    // --------------------------------------------------------
    // STATE
    // --------------------------------------------------------

    if (simulation.paused)
    {
        DrawText(
            "PAUSED",
            10,
            315,
            22,
            ORANGE
        );
    }
    else
    {
        DrawText(
            "RUNNING",
            10,
            315,
            22,
            GREEN
        );
    }


    // --------------------------------------------------------
    // FPS
    // --------------------------------------------------------

    std::string fpsText =
        std::to_string(
            GetFPS()
        ) +
        " FPS";


    DrawText(
        fpsText.c_str(),
        screenWidth - 100,
        25,
        18,
        GREEN
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


    // Display rate is independent of physics timestep.
    SetTargetFPS(60);


    // ========================================================
    // BOX
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
    // SIMULATION
    // ========================================================

    Simulation simulation;


    simulation.running =
        false;


    simulation.paused =
        false;


    simulation.physicsDt =
        1.0f / 120.0f;


    // ========================================================
    // INPUT SCREEN
    // ========================================================

    InitializeInputFields();


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
        // SETUP SCREEN
        // ====================================================

        if (!simulation.running)
        {
            HandleInputScreen(
                box
            );


            if (IsKeyPressed(
                    KEY_ENTER))
            {
                if (CreateSimulation(
                        simulation,
                        box))
                {
                    accumulator =
                        0.0;
                }
            }


            BeginDrawing();


            DrawInputScreen(
                screenWidth,
                screenHeight
            );


            EndDrawing();


            continue;
        }


        // ====================================================
        // REAL FRAME TIME
        // ====================================================

        float frameTime =
            GetFrameTime();


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

        if (IsKeyPressed(
                KEY_SPACE))
        {
            simulation.paused =
                !simulation.paused;
        }


        // ====================================================
        // RETURN TO INPUT SCREEN
        // ====================================================

        if (IsKeyPressed(KEY_R))
        {
            simulation.running =
                false;


            simulation.paused =
                false;


            accumulator =
                0.0;


            InitializeInputFields();


            continue;
        }


        // ====================================================
        // CHANGE PHYSICS TIMESTEP
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
        // PHYSICS
        // ====================================================

        PhysicsParameters physics =
            CreatePhysicsParameters();


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
                SimulatePhysics(
                    simulation,
                    box,
                    physics
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
        // 3D WORLD
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


        // Reference grid
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
        // VELOCITY VECTOR
        // ====================================================

        DrawVelocityVector(
            simulation.ball
        );


        EndMode3D();


        // ====================================================
        // UI
        // ====================================================

        DrawSimulationUI(
            simulation,
            physics,
            screenWidth,
            screenHeight
        );


        EndDrawing();
    }


    // ========================================================
    // CLEANUP
    // ========================================================

    CloseWindow();


    return 0;
}