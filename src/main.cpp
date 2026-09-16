#include "raylib.h"

#include <cmath>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>


/*
======================================================================

        FOUNDATIONS OF PHYSICALLY BASED MODELING
                    ASSIGNMENT 1

              BOUNCING BALL IN A BOX

======================================================================

MAIN OBJECTIVE

    Simulate a spherical ball moving inside a six-sided box.

    The simulation includes:

        1. Gravity
        2. Wind force
        3. Spatially varying air resistance
        4. Collision detection
        5. Collision response
        6. Friction
        7. Restitution
        8. Euler integration
        9. Fractional timesteps
       10. Adjustable physics timestep


USER INPUT

    The user can specify:

        - Ball radius
        - Ball mass
        - Starting position
        - Starting velocity
        - Box width
        - Box height
        - Box depth


NOVELTY

    The air resistance is spatially varying.

    Instead of having the same drag everywhere, the box contains
    a smooth region where the air is denser.

    The resistance is:

        HIGHER around the central region

        LOWER near the walls

    The drag does NOT stop the ball immediately.

    This allows us to clearly observe the effect of the
    spatially varying force while still allowing the ball
    to bounce around the box.


IMPORTANT ASSIGNMENT REQUIREMENT

    The following parts are intentionally implemented directly:

        - Simulation loop / timestepping
        - Force calculation
        - Euler integration
        - Collision detection and response


RAYLIB

    Raylib is used only for:

        - Window
        - Camera
        - Keyboard input
        - Rendering
        - User interface

======================================================================
*/


// ====================================================================
// BASIC VECTOR OPERATIONS
// ====================================================================

Vector3 Add(Vector3 a, Vector3 b)
{
    return {
        a.x + b.x,
        a.y + b.y,
        a.z + b.z
    };
}


Vector3 Subtract(Vector3 a, Vector3 b)
{
    return {
        a.x - b.x,
        a.y - b.y,
        a.z - b.z
    };
}


Vector3 Multiply(Vector3 a, float value)
{
    return {
        a.x * value,
        a.y * value,
        a.z * value
    };
}


float Dot(Vector3 a, Vector3 b)
{
    return
        a.x * b.x +
        a.y * b.y +
        a.z * b.z;
}


float Length(Vector3 a)
{
    return std::sqrt(
        a.x * a.x +
        a.y * a.y +
        a.z * a.z
    );
}


// ====================================================================
// DATA STRUCTURES
// ====================================================================

struct Ball
{
    Vector3 position;
    Vector3 velocity;

    float radius;
    float mass;
};


struct Box
{
    float width;
    float height;
    float depth;

    Vector3 center;
};


/*
    Parameters controlling the spatial air resistance.

    baseDrag:

        A small amount of drag that exists everywhere.

    centerStrength:

        Additional drag around the central region.

    falloff:

        Controls how quickly the additional drag disappears
        as we move away from the central region.
*/
struct SpatialDragField
{
    float baseDrag;
    float centerStrength;
    float falloff;
};


struct PhysicsParameters
{
    Vector3 gravity;

    Vector3 wind;

    SpatialDragField dragField;

    float restitution;

    float friction;
};


struct Simulation
{
    Ball ball;

    double simulationTime;

    unsigned long long collisionCount;

    float physicsDt;

    bool paused;
    bool running;
};


// ====================================================================
// USER INPUT
// ====================================================================

struct InputField
{
    std::string label;
    std::string value;

    Rectangle rectangle;

    bool active;
};


InputField fields[11];

int activeField = 0;

std::string errorMessage;


// ====================================================================
// INITIAL VALUES
// ====================================================================

void InitializeFields()
{
    /*
        These values are deliberately chosen so that the default
        demonstration produces a clearly visible bouncing path.

        The starting velocity has components in all three axes.

        Therefore the ball does not simply fall vertically onto
        the bottom surface.

        Instead, it travels through the box and can hit:

            left / right
            top / bottom
            front / back
    */

    fields[0] = {
        "Ball Radius",
        "0.45",
        {390, 110, 220, 35},
        false
    };

    fields[1] = {
        "Ball Mass",
        "1.0",
        {390, 155, 220, 35},
        false
    };


    fields[2] = {
        "Position X",
        "-3.0",
        {390, 225, 220, 35},
        false
    };

    fields[3] = {
        "Position Y",
        "1.5",
        {390, 270, 220, 35},
        false
    };

    fields[4] = {
        "Position Z",
        "-3.0",
        {390, 315, 220, 35},
        false
    };


    /*
        Initial velocity.

        X positive:

            moves toward the right wall.

        Y positive:

            moves upward toward the top.

        Z positive:

            moves toward the back wall.

        After the first collisions, the velocity components
        reverse and the ball begins travelling in other directions.
    */

    fields[5] = {
        "Velocity X",
        "6.0",
        {390, 385, 220, 35},
        false
    };

    fields[6] = {
        "Velocity Y",
        "7.0",
        {390, 430, 220, 35},
        false
    };

    fields[7] = {
        "Velocity Z",
        "6.5",
        {390, 475, 220, 35},
        false
    };


    /*
        A relatively large box is used.

        This gives the ball enough distance to travel before
        reaching another surface.
    */

    fields[8] = {
        "Box Width",
        "12.0",
        {390, 545, 220, 35},
        false
    };

    fields[9] = {
        "Box Height",
        "10.0",
        {390, 590, 220, 35},
        false
    };

    fields[10] = {
        "Box Depth",
        "12.0",
        {390, 635, 220, 35},
        false
    };


    activeField = 0;

    fields[0].active = true;

    errorMessage.clear();
}


// ====================================================================
// FLOAT PARSING
// ====================================================================

bool ParseFloat(
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


// ====================================================================
// GET FIELD VALUE
// ====================================================================

bool GetValue(
    int index,
    float& value)
{
    return ParseFloat(
        fields[index].value,
        value
    );
}


// ====================================================================
// VALIDATION
// ====================================================================

bool ValidateInput(
    float radius,
    float mass,
    Vector3 position,
    const Box& box)
{
    errorMessage.clear();


    if (radius <= 0)
    {
        errorMessage =
            "ERROR: Ball radius must be greater than 0.";

        return false;
    }


    if (mass <= 0)
    {
        errorMessage =
            "ERROR: Ball mass must be greater than 0.";

        return false;
    }


    if (box.width <= 0 ||
        box.height <= 0 ||
        box.depth <= 0)
    {
        errorMessage =
            "ERROR: Box dimensions must be greater than 0.";

        return false;
    }


    if (box.width <= radius * 2 ||
        box.height <= radius * 2 ||
        box.depth <= radius * 2)
    {
        errorMessage =
            "ERROR: Ball is too large for the box.";

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


    if (position.x < minX ||
        position.x > maxX ||
        position.y < minY ||
        position.y > maxY ||
        position.z < minZ ||
        position.z > maxZ)
    {
        errorMessage =
            "ERROR: Starting position is outside the box.";

        return false;
    }


    return true;
}


// ====================================================================
// PHYSICS PARAMETERS
// ====================================================================

void CreatePhysics(
    PhysicsParameters& physics)
{
    /*
        Gravity.

        Negative Y means downward because our coordinate system
        uses positive Y as upward.
    */

    physics.gravity = {
        0.0f,
        -9.81f,
        0.0f
    };


    /*
        Wind is represented as an external force.

        It is intentionally not extremely large because we want
        gravity and collision response to remain clearly visible.
    */

    physics.wind = {
        0.35f,
        0.0f,
        -0.25f
    };


    /*
        RESTITUTION

        1.0 = perfectly elastic collision

        0.0 = no bounce

        0.88 means that most of the normal velocity is retained
        after the collision.
    */

    physics.restitution =
        0.88f;


    /*
        FRICTION

        This reduces the velocity parallel to the surface.

        A small value means the ball continues sliding/bouncing
        instead of losing all of its tangential motion.
    */

    physics.friction =
        0.03f;


    /*
        SPATIAL AIR RESISTANCE

        The previous version used a very strong drag coefficient.
        That caused the ball to lose energy too quickly.

        These values are deliberately smaller.

        baseDrag:

            Small resistance everywhere.

        centerStrength:

            Additional resistance in the central region.

        falloff:

            Determines how quickly the extra resistance decreases.
    */

    physics.dragField.baseDrag =
        0.002f;

    physics.dragField.centerStrength =
        0.018f;

    physics.dragField.falloff =
        2.0f;
}


// ====================================================================
// BOX LIMITS
// ====================================================================

float MinimumX(
    const Box& box,
    const Ball& ball)
{
    return
        box.center.x -
        box.width / 2.0f +
        ball.radius;
}


float MaximumX(
    const Box& box,
    const Ball& ball)
{
    return
        box.center.x +
        box.width / 2.0f -
        ball.radius;
}


float MinimumY(
    const Box& box,
    const Ball& ball)
{
    return
        box.center.y -
        box.height / 2.0f +
        ball.radius;
}


float MaximumY(
    const Box& box,
    const Ball& ball)
{
    return
        box.center.y +
        box.height / 2.0f -
        ball.radius;
}


float MinimumZ(
    const Box& box,
    const Ball& ball)
{
    return
        box.center.z -
        box.depth / 2.0f +
        ball.radius;
}


float MaximumZ(
    const Box& box,
    const Ball& ball)
{
    return
        box.center.z +
        box.depth / 2.0f -
        ball.radius;
}


// ====================================================================
// SPATIAL AIR RESISTANCE
// ====================================================================

float CalculateSpatialDrag(
    Vector3 position,
    const Box& box,
    const PhysicsParameters& physics)
{
    /*
        We first measure how far the ball is from the center
        of the box.

        Each axis is normalized by the half-dimension of the box.

        This gives approximately:

            center -> 0

            outer region -> 1
    */

    float halfWidth =
        box.width / 2.0f;

    float halfHeight =
        box.height / 2.0f;

    float halfDepth =
        box.depth / 2.0f;


    float nx =
        (position.x - box.center.x) /
        halfWidth;

    float ny =
        (position.y - box.center.y) /
        halfHeight;

    float nz =
        (position.z - box.center.z) /
        halfDepth;


    /*
        Squared distance from the center.

        We do not need sqrt() because we only need the magnitude
        for the exponential function.

        This produces an ellipsoidal region of stronger resistance.
    */

    float distanceSquared =
        nx * nx +
        ny * ny +
        nz * nz;


    /*
        Gaussian-like falloff.

            exp(-falloff * distanceSquared)

        is approximately:

            1 near the center

            0 far away from the center
    */

    float centerInfluence =
        std::exp(
            -physics.dragField.falloff *
            distanceSquared
        );


    /*
        Finally:

            total drag =
                base drag
                +
                additional central drag
    */

    float drag =
        physics.dragField.baseDrag +
        physics.dragField.centerStrength *
        centerInfluence;


    return drag;
}


/*
======================================================================

                    CODE YOURSELF

                CALCULATION OF FORCES

======================================================================

The acceleration of the ball is calculated from the forces acting
on it.

We have three main forces:

    1. Gravity
    2. Wind
    3. Air resistance

Gravity:

        Fgravity = m * g


Wind:

        Fwind = external wind force


Air resistance:

        Fdrag = -k(x) * v

The negative sign is important.

Air resistance always acts opposite to the direction of velocity.

The coefficient k(x) is NOT constant.

It depends on the ball's position.

Therefore the force depends on:

        position
        velocity

The total force is:

        Ftotal =
            Fgravity +
            Fwind +
            Fdrag


Newton's second law:

        F = m * a

therefore:

        a = F / m

======================================================================
*/


Vector3 CalculateAcceleration(
    const Ball& ball,
    const Box& box,
    const PhysicsParameters& physics)
{
    Vector3 gravityForce =
        Multiply(
            physics.gravity,
            ball.mass
        );


    float dragCoefficient =
        CalculateSpatialDrag(
            ball.position,
            box,
            physics
        );


    Vector3 dragForce =
        Multiply(
            ball.velocity,
            -dragCoefficient
        );


    Vector3 totalForce =
        Add(
            gravityForce,
            physics.wind
        );


    totalForce =
        Add(
            totalForce,
            dragForce
        );


    Vector3 acceleration =
        Multiply(
            totalForce,
            1.0f / ball.mass
        );


    return acceleration;
}


/*
======================================================================

                    CODE YOURSELF

                  EULER INTEGRATION

======================================================================

This assignment specifically asks for basic Euler integration.

Euler integration approximates the continuous equations of motion.

Position:

        x(t + dt) = x(t) + v(t) * dt


Velocity:

        v(t + dt) = v(t) + a(t) * dt


The important point is that dt is user adjustable.

A smaller dt usually produces a more accurate numerical
approximation.

A larger dt is faster computationally but can introduce
more numerical error.

The physics timestep is independent from the rendering frame rate.

======================================================================
*/


void EulerIntegrate(
    Ball& ball,
    const Box& box,
    const PhysicsParameters& physics,
    float dt)
{
    if (dt <= 0.0f)
        return;


    Vector3 acceleration =
        CalculateAcceleration(
            ball,
            box,
            physics
        );


    ball.position =
        Add(
            ball.position,
            Multiply(
                ball.velocity,
                dt
            )
        );


    ball.velocity =
        Add(
            ball.velocity,
            Multiply(
                acceleration,
                dt
            )
        );
}


/*
======================================================================

                CODE YOURSELF

        COLLISION DETECTION AND RESPONSE

======================================================================

The ball is spherical.

Therefore collision with a wall is detected by comparing the
CENTER of the ball with a boundary that has been moved inward
by the radius.

For example, for the left wall:

        x >= boxLeft + radius

For the right wall:

        x <= boxRight - radius


The same idea is applied to all six surfaces.

We have:

        left
        right

        bottom
        top

        front
        back


FRACTIONAL TIMESTEP

Suppose a physics timestep is:

        dt = 0.0083 seconds

but the ball actually reaches a wall after:

        t = 0.003 seconds

We should NOT integrate the entire timestep first.

Instead:

        1. Integrate 0.003 seconds.
        2. Resolve collision.
        3. Integrate the remaining 0.0053 seconds.

This prevents the ball from penetrating deeply through
the wall.

======================================================================
*/


struct Collision
{
    bool hit;

    float time;

    Vector3 normal;
};


// ====================================================================
// COLLISION DETECTION
// ====================================================================

Collision FindCollision(
    const Ball& ball,
    const Box& box,
    float dt)
{
    Collision collision;

    collision.hit = false;

    collision.time = dt;

    collision.normal = {
        0,
        0,
        0
    };


    float minX =
        MinimumX(
            box,
            ball
        );

    float maxX =
        MaximumX(
            box,
            ball
        );


    float minY =
        MinimumY(
            box,
            ball
        );

    float maxY =
        MaximumY(
            box,
            ball
        );


    float minZ =
        MinimumZ(
            box,
            ball
        );

    float maxZ =
        MaximumZ(
            box,
            ball
        );


    Vector3 predicted =
        Add(
            ball.position,
            Multiply(
                ball.velocity,
                dt
            )
        );


    const float epsilon =
        0.000001f;


    // ------------------------------------------------------------
    // X AXIS
    // ------------------------------------------------------------

    if (ball.velocity.x < -epsilon &&
        predicted.x < minX)
    {
        float t =
            (minX - ball.position.x) /
            ball.velocity.x;


        if (t >= 0.0f &&
            t <= collision.time)
        {
            collision.hit = true;

            collision.time = t;

            collision.normal = {
                1,
                0,
                0
            };
        }
    }


    if (ball.velocity.x > epsilon &&
        predicted.x > maxX)
    {
        float t =
            (maxX - ball.position.x) /
            ball.velocity.x;


        if (t >= 0.0f &&
            t <= collision.time)
        {
            collision.hit = true;

            collision.time = t;

            collision.normal = {
                -1,
                0,
                0
            };
        }
    }


    // ------------------------------------------------------------
    // Y AXIS
    // ------------------------------------------------------------

    if (ball.velocity.y < -epsilon &&
        predicted.y < minY)
    {
        float t =
            (minY - ball.position.y) /
            ball.velocity.y;


        if (t >= 0.0f &&
            t <= collision.time)
        {
            collision.hit = true;

            collision.time = t;

            collision.normal = {
                0,
                1,
                0
            };
        }
    }


    if (ball.velocity.y > epsilon &&
        predicted.y > maxY)
    {
        float t =
            (maxY - ball.position.y) /
            ball.velocity.y;


        if (t >= 0.0f &&
            t <= collision.time)
        {
            collision.hit = true;

            collision.time = t;

            collision.normal = {
                0,
                -1,
                0
            };
        }
    }


    // ------------------------------------------------------------
    // Z AXIS
    // ------------------------------------------------------------

    if (ball.velocity.z < -epsilon &&
        predicted.z < minZ)
    {
        float t =
            (minZ - ball.position.z) /
            ball.velocity.z;


        if (t >= 0.0f &&
            t <= collision.time)
        {
            collision.hit = true;

            collision.time = t;

            collision.normal = {
                0,
                0,
                1
            };
        }
    }


    if (ball.velocity.z > epsilon &&
        predicted.z > maxZ)
    {
        float t =
            (maxZ - ball.position.z) /
            ball.velocity.z;


        if (t >= 0.0f &&
            t <= collision.time)
        {
            collision.hit = true;

            collision.time = t;

            collision.normal = {
                0,
                0,
                -1
            };
        }
    }


    return collision;
}


// ====================================================================
// COLLISION RESPONSE
// ====================================================================

void ResolveCollision(
    Ball& ball,
    Vector3 normal,
    const PhysicsParameters& physics,
    unsigned long long& collisionCount)
{
    /*
        Decompose velocity into:

            normal component
            tangential component

        The normal component determines how strongly the ball
        bounces away from the surface.

        The tangential component is affected by friction.
    */

    float normalVelocity =
        Dot(
            ball.velocity,
            normal
        );


    /*
        If the ball is already moving away from the wall,
        there is nothing to bounce.
    */

    if (normalVelocity >= 0.0f)
        return;


    Vector3 normalComponent =
        Multiply(
            normal,
            normalVelocity
        );


    Vector3 tangentialComponent =
        Subtract(
            ball.velocity,
            normalComponent
        );


    /*
        Restitution reverses the normal component.

            v_normal_new =
                -e * v_normal_old

        e = coefficient of restitution.
    */

    Vector3 newNormalComponent =
        Multiply(
            normalComponent,
            -physics.restitution
        );


    /*
        Friction reduces motion parallel to the surface.
    */

    Vector3 newTangentialComponent =
        Multiply(
            tangentialComponent,
            std::max(
                0.0f,
                1.0f - physics.friction
            )
        );


    ball.velocity =
        Add(
            newNormalComponent,
            newTangentialComponent
        );


    collisionCount++;
}


/*
======================================================================

              END OF CODE YOURSELF SECTION

        COLLISION DETECTION AND RESPONSE

======================================================================
*/


// ====================================================================
// CREATE SIMULATION
// ====================================================================

bool CreateSimulation(
    Simulation& simulation,
    Box& box)
{
    float radius;
    float mass;

    float px;
    float py;
    float pz;

    float vx;
    float vy;
    float vz;

    float width;
    float height;
    float depth;


    if (!GetValue(0, radius) ||
        !GetValue(1, mass) ||
        !GetValue(2, px) ||
        !GetValue(3, py) ||
        !GetValue(4, pz) ||
        !GetValue(5, vx) ||
        !GetValue(6, vy) ||
        !GetValue(7, vz) ||
        !GetValue(8, width) ||
        !GetValue(9, height) ||
        !GetValue(10, depth))
    {
        errorMessage =
            "ERROR: Please enter valid numerical values.";

        return false;
    }


    box.width = width;

    box.height = height;

    box.depth = depth;

    box.center = {
        0,
        0,
        0
    };


    Vector3 position = {
        px,
        py,
        pz
    };


    if (!ValidateInput(
            radius,
            mass,
            position,
            box))
    {
        return false;
    }


    simulation.ball.position =
        position;


    simulation.ball.velocity = {
        vx,
        vy,
        vz
    };


    simulation.ball.radius =
        radius;


    simulation.ball.mass =
        mass;


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


/*
======================================================================

                    CODE YOURSELF

             SIMULATION LOOP / TIMESTEPPING

======================================================================

The rendering loop runs at approximately 60 FPS.

The physics timestep is independent of this.

For example:

        rendering:
            approximately 60 frames/sec

        physics:
            120 steps/sec

The accumulator stores real elapsed time.

If enough real time has passed to perform another physics step,
we execute one physics step.

This allows the user to change the physics timestep without
changing the intended rendering speed.

Within each physics timestep, collision detection may divide
the timestep into smaller fractional pieces.

======================================================================
*/


void UpdateSimulation(
    Simulation& simulation,
    const Box& box,
    const PhysicsParameters& physics)
{
    float remaining =
        simulation.physicsDt;


    int iterations = 0;

    const int maximumIterations = 10;


    while (
        remaining > 0.000001f &&
        iterations < maximumIterations)
    {
        Collision collision =
            FindCollision(
                simulation.ball,
                box,
                remaining
            );


        if (!collision.hit)
        {
            EulerIntegrate(
                simulation.ball,
                box,
                physics,
                remaining
            );


            simulation.simulationTime +=
                remaining;


            remaining = 0.0f;

            break;
        }


        float collisionTime =
            std::max(
                0.0f,
                collision.time
            );


        if (collisionTime > 0.000001f)
        {
            EulerIntegrate(
                simulation.ball,
                box,
                physics,
                collisionTime
            );


            simulation.simulationTime +=
                collisionTime;
        }


        /*
            Put the ball exactly on the collision surface.

            This prevents numerical integration from leaving
            the ball slightly inside the wall.
        */

        if (collision.normal.x > 0.5f)
        {
            simulation.ball.position.x =
                MinimumX(
                    box,
                    simulation.ball
                );
        }
        else if (collision.normal.x < -0.5f)
        {
            simulation.ball.position.x =
                MaximumX(
                    box,
                    simulation.ball
                );
        }


        if (collision.normal.y > 0.5f)
        {
            simulation.ball.position.y =
                MinimumY(
                    box,
                    simulation.ball
                );
        }
        else if (collision.normal.y < -0.5f)
        {
            simulation.ball.position.y =
                MaximumY(
                    box,
                    simulation.ball
                );
        }


        if (collision.normal.z > 0.5f)
        {
            simulation.ball.position.z =
                MinimumZ(
                    box,
                    simulation.ball
                );
        }
        else if (collision.normal.z < -0.5f)
        {
            simulation.ball.position.z =
                MaximumZ(
                    box,
                    simulation.ball
                );
        }


        ResolveCollision(
            simulation.ball,
            collision.normal,
            physics,
            simulation.collisionCount
        );


        remaining -=
            collisionTime;


        /*
            Tiny movement away from the wall.

            This prevents floating-point rounding from detecting
            the same collision repeatedly at exactly t = 0.
        */

        simulation.ball.position =
            Add(
                simulation.ball.position,
                Multiply(
                    collision.normal,
                    0.00001f
                )
            );


        if (collisionTime <
            0.000001f)
        {
            remaining -=
                0.000001f;
        }


        iterations++;
    }


    /*
        Final safety clamp.

        Even with fractional timestep collision handling, numerical
        rounding can produce a tiny error. This guarantees that
        the ball remains inside the box.
    */

    simulation.ball.position.x =
        std::max(
            MinimumX(
                box,
                simulation.ball
            ),
            std::min(
                MaximumX(
                    box,
                    simulation.ball
                ),
                simulation.ball.position.x
            )
        );


    simulation.ball.position.y =
        std::max(
            MinimumY(
                box,
                simulation.ball
            ),
            std::min(
                MaximumY(
                    box,
                    simulation.ball
                ),
                simulation.ball.position.y
            )
        );


    simulation.ball.position.z =
        std::max(
            MinimumZ(
                box,
                simulation.ball
            ),
            std::min(
                MaximumZ(
                    box,
                    simulation.ball
                ),
                simulation.ball.position.z
            )
        );
}


/*
======================================================================

            END OF CODE YOURSELF SECTION

              SIMULATION LOOP / TIMESTEPPING

======================================================================
*/


// ====================================================================
// NUMBER FORMATTER
// ====================================================================

std::string Number(
    float value,
    int precision = 2)
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


// ====================================================================
// DRAW BOX
// ====================================================================

void DrawBox(
    const Box& box)
{
    /*
        Only wireframe is drawn.

        This is intentional.

        A solid box would hide the ball from certain camera angles.

        The wireframe makes it obvious that the ball remains inside
        all six boundaries.
    */

    DrawCubeWires(
        box.center,
        box.width,
        box.height,
        box.depth,
        BLUE
    );
}


// ====================================================================
// DRAW BALL
// ====================================================================

void DrawBall(
    const Ball& ball)
{
    /*
        The ball is rendered as a solid red sphere.

        There is NO red velocity dot.

        The velocity dot from the previous version was removed
        because it could be confused with the actual simulated ball.
    */

    DrawSphere(
        ball.position,
        ball.radius,
        RED
    );


    DrawSphereWires(
        ball.position,
        ball.radius,
        16,
        16,
        MAROON
    );
}


// ====================================================================
// DRAW SPATIAL DRAG INDICATOR
// ====================================================================

void DrawDragIndicator(
    const Box& box)
{
    /*
        This is NOT a physical object.

        It only indicates where the drag field is strongest.

        We keep it extremely small so it cannot hide the ball.
    */

    DrawSphere(
        box.center,
        0.08f,
        ORANGE
    );
}


// ====================================================================
// INPUT SCREEN
// ====================================================================

void DrawInputScreen(
    int screenWidth,
    int screenHeight)
{
    ClearBackground(
        RAYWHITE
    );


    DrawText(
        "PHYSICALLY BASED BALL SIMULATION",
        30,
        20,
        27,
        BLACK
    );


    DrawText(
        "INITIAL CONDITIONS",
        30,
        55,
        20,
        DARKBLUE
    );


    DrawText(
        "BALL",
        60,
        85,
        16,
        BLUE
    );


    DrawText(
        "Radius",
        100,
        118,
        16,
        DARKGRAY
    );


    DrawText(
        "Mass",
        100,
        163,
        16,
        DARKGRAY
    );


    DrawText(
        "POSITION",
        60,
        198,
        16,
        BLUE
    );


    DrawText(
        "X",
        100,
        233,
        16,
        DARKGRAY
    );

    DrawText(
        "Y",
        100,
        278,
        16,
        DARKGRAY
    );

    DrawText(
        "Z",
        100,
        323,
        16,
        DARKGRAY
    );


    DrawText(
        "VELOCITY",
        60,
        358,
        16,
        BLUE
    );


    DrawText(
        "X",
        100,
        393,
        16,
        DARKGRAY
    );

    DrawText(
        "Y",
        100,
        438,
        16,
        DARKGRAY
    );

    DrawText(
        "Z",
        100,
        483,
        16,
        DARKGRAY
    );


    DrawText(
        "BOX DIMENSIONS",
        60,
        518,
        16,
        BLUE
    );


    DrawText(
        "Width",
        100,
        553,
        16,
        DARKGRAY
    );

    DrawText(
        "Height",
        100,
        598,
        16,
        DARKGRAY
    );

    DrawText(
        "Depth",
        100,
        643,
        16,
        DARKGRAY
    );


    for (int i = 0; i < 11; i++)
    {
        Color border =
            fields[i].active
                ? DARKBLUE
                : GRAY;


        DrawRectangleLinesEx(
            fields[i].rectangle,
            2,
            border
        );


        DrawText(
            fields[i].value.c_str(),
            static_cast<int>(
                fields[i].rectangle.x + 8
            ),
            static_cast<int>(
                fields[i].rectangle.y + 7
            ),
            17,
            BLACK
        );
    }


    DrawText(
        "ENVIRONMENT",
        700,
        100,
        20,
        DARKBLUE
    );


    DrawText(
        "Gravity",
        700,
        140,
        17,
        DARKGRAY
    );


    DrawText(
        "(0, -9.81, 0) m/s^2",
        830,
        140,
        17,
        DARKGRAY
    );


    DrawText(
        "Wind",
        700,
        180,
        17,
        DARKGRAY
    );


    DrawText(
        "(0.35, 0, -0.25) N",
        830,
        180,
        17,
        DARKGRAY
    );


    DrawText(
        "Spatial drag",
        700,
        220,
        17,
        DARKGRAY
    );


    DrawText(
        "HIGH near center",
        830,
        220,
        17,
        DARKGREEN
    );


    DrawText(
        "LOW near faces",
        830,
        250,
        17,
        DARKGREEN
    );


    DrawText(
        "Restitution",
        700,
        290,
        17,
        DARKGRAY
    );


    DrawText(
        "0.88",
        830,
        290,
        17,
        DARKGRAY
    );


    DrawText(
        "Friction",
        700,
        325,
        17,
        DARKGRAY
    );


    DrawText(
        "0.03",
        830,
        325,
        17,
        DARKGRAY
    );


    DrawText(
        "CONTROLS",
        700,
        390,
        20,
        DARKBLUE
    );


    DrawText(
        "TAB / UP / DOWN",
        700,
        430,
        17,
        DARKGRAY
    );


    DrawText(
        "Select field",
        900,
        430,
        17,
        DARKGRAY
    );


    DrawText(
        "ENTER",
        700,
        465,
        17,
        DARKGRAY
    );


    DrawText(
        "Start simulation",
        900,
        465,
        17,
        DARKGREEN
    );


    DrawText(
        "All values use SI units.",
        700,
        525,
        17,
        DARKGRAY
    );


    DrawText(
        "Ball must fit completely inside box.",
        700,
        555,
        17,
        DARKGRAY
    );


    if (!errorMessage.empty())
    {
        DrawText(
            errorMessage.c_str(),
            700,
            610,
            17,
            RED
        );
    }


    DrawText(
        "ESC = Exit",
        700,
        650,
        17,
        DARKGRAY
    );
}


// ====================================================================
// HANDLE INPUT
// ====================================================================

void HandleInput()
{
    if (IsMouseButtonPressed(
            MOUSE_BUTTON_LEFT))
    {
        Vector2 mouse =
            GetMousePosition();


        for (int i = 0; i < 11; i++)
        {
            if (CheckCollisionPointRec(
                    mouse,
                    fields[i].rectangle))
            {
                for (int j = 0; j < 11; j++)
                    fields[j].active = false;


                activeField =
                    i;


                fields[i].active =
                    true;


                errorMessage.clear();

                break;
            }
        }
    }


    if (IsKeyPressed(KEY_TAB) ||
        IsKeyPressed(KEY_DOWN))
    {
        fields[activeField].active =
            false;


        activeField++;


        if (activeField >= 11)
            activeField = 0;


        fields[activeField].active =
            true;
    }


    if (IsKeyPressed(KEY_UP))
    {
        fields[activeField].active =
            false;


        activeField--;


        if (activeField < 0)
            activeField = 10;


        fields[activeField].active =
            true;
    }


    int character =
        GetCharPressed();


    while (character > 0)
    {
        if ((character >= '0' &&
             character <= '9') ||
            character == '-' ||
            character == '.')
        {
            fields[activeField].value +=
                static_cast<char>(
                    character
                );


            errorMessage.clear();
        }


        character =
            GetCharPressed();
    }


    if (IsKeyPressed(
            KEY_BACKSPACE))
    {
        if (!fields[activeField]
                 .value.empty())
        {
            fields[activeField]
                .value
                .pop_back();
        }


        errorMessage.clear();
    }
}


// ====================================================================
// SIMULATION UI
// ====================================================================

void DrawSimulationUI(
    const Simulation& simulation,
    const PhysicsParameters& physics,
    const Box& box,
    int screenWidth,
    int screenHeight)
{
    DrawRectangle(
        5,
        5,
        345,
        350,
        Fade(
            RAYWHITE,
            0.90f
        )
    );


    DrawText(
        "PHYSICS SIMULATION",
        15,
        15,
        20,
        BLACK
    );


    DrawText(
        "SPACE : Pause / Resume",
        15,
        48,
        16,
        DARKGRAY
    );


    DrawText(
        "R : Initial Conditions",
        15,
        72,
        16,
        DARKGRAY
    );


    DrawText(
        "UP/DOWN : Physics timestep",
        15,
        96,
        16,
        DARKGRAY
    );


    DrawText(
        ("Physics dt: " +
         Number(
             simulation.physicsDt,
             6
         ) +
         " s").c_str(),
        15,
        130,
        16,
        BLUE
    );


    DrawText(
        ("Simulation time: " +
         Number(
             static_cast<float>(
                 simulation.simulationTime
             ),
             2
         ) +
         " s").c_str(),
        15,
        155,
        16,
        BLUE
    );


    float speed =
        Length(
            simulation.ball.velocity
        );


    DrawText(
        ("Speed: " +
         Number(
             speed,
             2
         ) +
         " m/s").c_str(),
        15,
        180,
        16,
        BLUE
    );


    DrawText(
        ("Collisions: " +
         std::to_string(
             simulation.collisionCount
         )).c_str(),
        15,
        205,
        16,
        BLUE
    );


    float localDrag =
        CalculateSpatialDrag(
            simulation.ball.position,
            box,
            physics
        );


    DrawText(
        ("Local air resistance: " +
         Number(
             localDrag,
             4
         )).c_str(),
        15,
        230,
        16,
        DARKGREEN
    );


    DrawText(
        "HIGH center -> LOW faces",
        15,
        255,
        15,
        DARKGREEN
    );


    DrawText(
        ("Radius: " +
         Number(
             simulation.ball.radius
         )).c_str(),
        15,
        285,
        16,
        DARKBLUE
    );


    DrawText(
        ("Mass: " +
         Number(
             simulation.ball.mass
         )).c_str(),
        15,
        310,
        16,
        DARKBLUE
    );


    if (simulation.paused)
    {
        DrawText(
            "PAUSED",
            15,
            335,
            18,
            ORANGE
        );
    }
    else
    {
        DrawText(
            "RUNNING",
            15,
            335,
            18,
            GREEN
        );
    }


    DrawText(
        ("FPS: " +
         std::to_string(
             GetFPS()
         )).c_str(),
        screenWidth - 90,
        15,
        17,
        GREEN
    );


    DrawText(
        ("Position: (" +
         Number(
             simulation.ball.position.x
         ) +
         ", " +
         Number(
             simulation.ball.position.y
         ) +
         ", " +
         Number(
             simulation.ball.position.z
         ) +
         ")").c_str(),
        15,
        screenHeight - 50,
        16,
        DARKGRAY
    );


    DrawText(
        ("Velocity: (" +
         Number(
             simulation.ball.velocity.x
         ) +
         ", " +
         Number(
             simulation.ball.velocity.y
         ) +
         ", " +
         Number(
             simulation.ball.velocity.z
         ) +
         ")").c_str(),
        15,
        screenHeight - 25,
        16,
        DARKGRAY
    );
}


// ====================================================================
// MAIN
// ====================================================================

int main()
{
    const int screenWidth =
        1280;

    const int screenHeight =
        720;


    InitWindow(
        screenWidth,
        screenHeight,
        "Physically Based Ball Simulation"
    );


    SetTargetFPS(60);


    InitializeFields();


    Box box;

    box.width = 12.0f;

    box.height = 10.0f;

    box.depth = 12.0f;

    box.center = {
        0,
        0,
        0
    };


    PhysicsParameters physics;

    CreatePhysics(
        physics
    );


    Simulation simulation;

    simulation.running =
        false;

    simulation.paused =
        false;

    simulation.physicsDt =
        1.0f / 120.0f;


    Camera3D camera = {};


    camera.position = {
        17.0f,
        13.0f,
        17.0f
    };


    camera.target = {
        0,
        0,
        0
    };


    camera.up = {
        0,
        1,
        0
    };


    camera.fovy =
        45.0f;


    camera.projection =
        CAMERA_PERSPECTIVE;


    double accumulator =
        0.0;


    while (!WindowShouldClose())
    {
        // ========================================================
        // INITIAL CONDITION SCREEN
        // ========================================================

        if (!simulation.running)
        {
            HandleInput();


            if (IsKeyPressed(KEY_ENTER))
            {
                if (CreateSimulation(
                        simulation,
                        box))
                {
                    accumulator =
                        0.0;

                    CreatePhysics(
                        physics
                    );
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


        // ========================================================
        // SIMULATION CONTROLS
        // ========================================================

        if (IsKeyPressed(KEY_SPACE))
        {
            simulation.paused =
                !simulation.paused;
        }


        if (IsKeyPressed(KEY_R))
        {
            simulation.running =
                false;

            simulation.paused =
                false;

            accumulator =
                0.0;

            InitializeFields();

            continue;
        }


        // ========================================================
        // PHYSICS TIMESTEP CONTROL
        // ========================================================

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


        // ========================================================
        // FRAME TIME
        // ========================================================

        float frameTime =
            GetFrameTime();


        /*
            Prevent an unusually long frame from producing
            an enormous physics update.
        */

        frameTime =
            std::min(
                frameTime,
                0.10f
            );


        // ========================================================
        // CAMERA
        // ========================================================

        UpdateCamera(
            &camera,
            CAMERA_ORBITAL
        );


        // ========================================================
        // SIMULATION
        // ========================================================

        if (!simulation.paused)
        {
            accumulator +=
                frameTime;


            /*
                Physics updates happen according to physicsDt.

                Rendering continues independently at the target
                frame rate.
            */

            accumulator =
                std::min(
                    accumulator,
                    0.25
                );


            while (
                accumulator >=
                simulation.physicsDt)
            {
                UpdateSimulation(
                    simulation,
                    box,
                    physics
                );


                accumulator -=
                    simulation.physicsDt;
            }
        }


        // ========================================================
        // DRAW
        // ========================================================

        BeginDrawing();


        ClearBackground(
            RAYWHITE
        );


        BeginMode3D(
            camera
        );


        DrawBox(
            box
        );


        /*
            Coordinate axes.

            X = red
            Y = green
            Z = blue
        */

        DrawLine3D(
            {-8, 0, 0},
            {8, 0, 0},
            RED
        );


        DrawLine3D(
            {0, -8, 0},
            {0, 8, 0},
            GREEN
        );


        DrawLine3D(
            {0, 0, -8},
            {0, 0, 8},
            BLUE
        );


        /*
            Small orange point marks the region where the
            spatial air resistance is strongest.

            It is only a visual reference.

            It has NO physical effect on the ball.
        */

        DrawDragIndicator(
            box
        );


        DrawBall(
            simulation.ball
        );


        EndMode3D();


        DrawSimulationUI(
            simulation,
            physics,
            box,
            screenWidth,
            screenHeight
        );


        EndDrawing();
    }


    CloseWindow();


    return 0;
}


/*
======================================================================

                        END OF PROGRAM

======================================================================

WHAT HAS BEEN IMPLEMENTED

    PHYSICS:

        Gravity
        Wind
        Spatially varying air resistance
        Euler integration
        Adjustable physics timestep
        Fractional collision timestep
        Six-sided collision detection
        Collision response
        Restitution
        Friction


    USER INPUT:

        Ball radius
        Ball mass
        Starting position
        Starting velocity
        Box width
        Box height
        Box depth


    VISUALIZATION:

        3D ball
        Six-sided wireframe box
        Camera rotation
        Physics information
        Current drag coefficient
        Position
        Velocity
        Collision count
        Simulation time
        FPS


    NOVELTY:

        Spatial air resistance.

        The drag coefficient changes according to the ball's
        location inside the box.

        The central region has stronger resistance.

        Near the walls, resistance becomes weaker.

        The ball therefore experiences a changing environment
        as it moves through the box.


======================================================================

IMPORTANT PHYSICS EQUATIONS

GRAVITY:

        F = m * g


AIR RESISTANCE:

        F_drag = -k(x) * v


TOTAL FORCE:

        F_total =
            F_gravity +
            F_wind +
            F_drag


NEWTON'S SECOND LAW:

        a = F_total / m


EULER POSITION:

        x_new = x_old + v * dt


EULER VELOCITY:

        v_new = v_old + a * dt


NORMAL COLLISION RESPONSE:

        v_normal_new =
            -restitution * v_normal_old


FRICTION:

        v_tangent_new =
            (1 - friction) * v_tangent_old


======================================================================

WHY THE BALL SHOULD NOT COME TO REST IMMEDIATELY

The spatial drag parameters were intentionally reduced.

Previous:

        centerStrength = 0.30

New:

        centerStrength = 0.018

The previous value was extremely strong relative to the
velocity scale being used.

The new field still produces a measurable difference in
air resistance but does not dominate gravity and collisions.


======================================================================

WHY THE BALL CAN HIT ALL SIX SURFACES

The default initial conditions are approximately:

        position:

            (-3.0, 1.5, -3.0)

        velocity:

            (6.0, 7.0, 6.5)


Therefore it initially moves:

        +X
        +Y
        +Z

This sends it toward three different sides of the box.

After collision:

        X velocity changes sign
        Y velocity changes sign
        Z velocity changes sign

so subsequent motion occurs in the opposite directions.

Gravity continuously modifies Y velocity.

Wind continuously modifies the velocity.

Therefore the trajectory is three-dimensional rather than
a simple vertical bouncing motion.


======================================================================

THE RED DOT HAS BEEN REMOVED

There is only ONE red object representing the ball.

The previous velocity indicator was removed because it could
be confused with the calculated ball position.

The velocity is now displayed numerically in the UI.


======================================================================

END

======================================================================
*/