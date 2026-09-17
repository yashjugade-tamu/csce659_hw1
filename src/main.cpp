#include "raylib.h"
#include <iostream>
#include <string>
#include <cmath>
#include <sstream>
#include <iomanip>

using namespace std;

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


struct Physics
{
    Vector3 gravity;
    Vector3 wind;
    float baseDrag;
    float centreDrag;
    float dragFalloff;
    float restitution;
    float friction;
    float timestep;
};

/*=============================
User input part has been implemented by referring AI suggestions. Most work was done by me*/



struct InputField
{
    string name;
    string value;
    Rectangle rectangle;
    bool selected;
};



// Define global variables.

const int NUMBER_OF_FIELDS = 21;

InputField inputFields[NUMBER_OF_FIELDS];

int selectedField = 0;

string errorMessage = "";

bool simulationStarted = false;

bool paused = false;

//Input from UI

void setupInputFields()
{

    //Ball inputs

    inputFields[0] = {
        "Ball Radius",
        "0.45",
        {350, 70, 220, 30},
        true
    };

    inputFields[1] = {
        "Ball Mass",
        "1.0",
        {350, 105, 220, 30},
        false
    };
    //Position inputs

    inputFields[2] = {
        "Position X",
        "-3.0",
        {350, 160, 220, 30},
        false
    };
    inputFields[3] = {
        "Position Y",
        "1.5",
        {350, 195, 220, 30},
        false
    };
    inputFields[4] = {
        "Position Z",
        "-3.0",
        {350, 230, 220, 30},
        false
    };
    //Velocity inputs

    inputFields[5] = {
        "Velocity X",
        "6.0",
        {350, 285, 220, 30},
        false
    };
    inputFields[6] = {
        "Velocity Y",
        "7.0",
        {350, 320, 220, 30},
        false
    };

    inputFields[7] = {
        "Velocity Z",
        "6.5",
        {350, 355, 220, 30},
        false
    };


    // Box inputs
    inputFields[8] = {
        "Box Width",
        "12.0",
        {350, 410, 220, 30},
        false
    };

    inputFields[9] = {
        "Box Height",
        "10.0",
        {350, 445, 220, 30},
        false
    };
    inputFields[10] = {
        "Box Depth",
        "12.0",
        {350, 480, 220, 30},
        false
    };


    // Wind input
    inputFields[11] = {
        "Wind X",
        "0.35",
        {350, 535, 220, 30},
        false
    };

    inputFields[12] = {
        "Wind Y",
        "0.0",
        {350, 570, 220, 30},
        false
    };

    inputFields[13] = {
        "Wind Z",
        "-0.25",
        {350, 605, 220, 30},
        false
    };
    // Physics inputs

    inputFields[14] = {
        "Restitution",
        "0.88",
        {900, 80, 220, 30},
        false
    };
    inputFields[15] = {
        "Friction",
        "0.03",
        {900, 115, 220, 30},
        false
    };
    inputFields[16] = {
        "Timestep",
        "0.005",
        {900, 150, 220, 30},
        false
    };
    //Spatial based drag inputs
    inputFields[17] = {
        "Base Drag",
        "0.002",
        {900, 210, 220, 30},
        false
    };
    inputFields[18] = {
        "Centre Drag",
        "0.018",
        {900, 245, 220, 30},
        false
    };
    inputFields[19] = {
        "Drag Falloff",
        "2.0",
        {900, 280, 220, 30},
        false
    };


    //Display gravity value, but not editable

    inputFields[20] = {
        "Gravity",
        "-9.81 m/s2",
        {900, 350, 220, 30},
        false
    };
}

bool stringToFloat(string text, float &number)
{
    if (text.length() == 0)
        return false;

    try
    {
        size_t position;
        number =
            stof(
                text,
                &position
            );
        if (position != text.length())
            return false;

        if (!isfinite(number))
            return false;

        return true;
    }
    catch (...)
    {
        return false;
    }
}


//Accept user input and convert to float, return false if invalid

bool getInputValue(int index,float &value)
{
    return stringToFloat(
        inputFields[index].value,
        value
    );
}
/*
=================================
Below part was coded by me as per the instructions in the assignment. AI was not used for this part.
=================================*/

//Validate user input values based on constraints

bool validateInput(Ball &ball, Box &box, Physics &physics)
{
    errorMessage = "";
    if (ball.radius <= 0)
    { errorMessage = "Ball radius must be greater than zero.";
        return false;
    }
    if (ball.mass <= 0)
    {
        errorMessage = "Ball mass must be greater than zero.";
        return false;
    }
    if (box.width <= 0 || box.height <= 0 || box.depth <= 0)
    {
        errorMessage = "All box dimensions must be greater than zero.";
        return false;
    }
    if (physics.timestep <= 0)
    {
        errorMessage = "Timestep must be greater than zero.";
        return false;
    }

    if (physics.timestep > 0.1f)
    {
        errorMessage = "Timestep is too large. Use a value <= 0.1.";
        return false;
    }


    if (physics.restitution < 0 ||
        physics.restitution > 1)
    {
        errorMessage = "Restitution must be between 0 and 1.";
        return false;
    }
    if (physics.friction < 0 ||
        physics.friction > 1)
    {
        errorMessage =
            "Friction must be between 0 and 1.";
        return false;
    }
    if (physics.baseDrag < 0)
    {
        errorMessage ="Base drag cannot be negative.";
        return false;
    }
    if (physics.centreDrag < 0)
    {
        errorMessage = "Centre drag cannot be negative.";
        return false;
    }
    if (physics.dragFalloff <= 0)
    {
        errorMessage =
            "Drag falloff must be greater than zero.";

        return false;
    }


    float minX =-box.width / 2.0f + ball.radius;
    float maxX = box.width / 2.0f - ball.radius;
    float minY =-box.height / 2.0f + ball.radius;

    float maxY = box.height / 2.0f - ball.radius;
    float minZ = -box.depth / 2.0f + ball.radius;

    float maxZ = box.depth / 2.0f - ball.radius;

    if (ball.position.x < minX || ball.position.x > maxX)
    {
        errorMessage = "Starting X position is outside the box.";
        return false;
    }


    if (ball.position.y < minY || ball.position.y > maxY)
    {
        errorMessage = "Starting Y position is outside the box.";
        return false;
    }


    if (ball.position.z < minZ || ball.position.z > maxZ)
    {
        errorMessage = "Starting Z position is outside the box.";
        return false;
    }


    return true;
}


//Create simulation

bool createSimulation(Ball &ball, Box &box, Physics &physics)
{
    float values[20];

    for (int i = 0; i < 20; i++)
    {
        if (!getInputValue(i, values[i]))
        {
            errorMessage =
                "Invalid numerical value. Please check the fields.";

            return false;
        }
    }

    ball.radius = values[0];
    ball.mass = values[1];

    ball.position.x = values[2];
    ball.position.y = values[3];
    ball.position.z = values[4];

    ball.velocity.x = values[5];
    ball.velocity.y = values[6];
    ball.velocity.z = values[7];

    box.width = values[8];
    box.height = values[9];
    box.depth = values[10];

    box.center = {0, 0, 0};

    physics.wind.x = values[11];
    physics.wind.y = values[12];
    physics.wind.z = values[13];

    physics.restitution = values[14];
    physics.friction = values[15];
    physics.timestep = values[16];

    physics.baseDrag = values[17];
    physics.centreDrag = values[18];
    physics.dragFalloff = values[19];

    physics.gravity = {0.0f, -9.81f, 0.0f};


    // Validate everything before starting the simulation.

    if (!validateInput(ball, box, physics))
    {
        return false;
    }


    // Debug information for checking initial conditions.

    cout << "\n========================================\n";
    cout << "[DEBUG] Simulation started\n";
    cout << "========================================\n";

    cout << "[DEBUG] Ball radius : "
         << ball.radius << endl;

    cout << "[DEBUG] Ball mass   : "
         << ball.mass << endl;

    cout << "[DEBUG] Position    : ("
         << ball.position.x << ", "
         << ball.position.y << ", "
         << ball.position.z << ")\n";

    cout << "[DEBUG] Velocity    : ("
         << ball.velocity.x << ", "
         << ball.velocity.y << ", "
         << ball.velocity.z << ")\n";

    cout << "[DEBUG] Box         : "
         << box.width << " x "
         << box.height << " x "
         << box.depth << endl;

    cout << "[DEBUG] Wind        : ("
         << physics.wind.x << ", "
         << physics.wind.y << ", "
         << physics.wind.z << ")\n";

    cout << "[DEBUG] Restitution : "
         << physics.restitution << endl;

    cout << "[DEBUG] Friction    : "
         << physics.friction << endl;

    cout << "[DEBUG] Timestep    : "
         << physics.timestep << endl;

    cout << "[DEBUG] Base drag   : "
         << physics.baseDrag << endl;

    cout << "[DEBUG] Centre drag : "
         << physics.centreDrag << endl;

    cout << "[DEBUG] Falloff     : "
         << physics.dragFalloff << endl;

    cout << "[DEBUG] Gravity     : ("
         << physics.gravity.x << ", "
         << physics.gravity.y << ", "
         << physics.gravity.z << ")\n";

    cout << "========================================\n\n";


    return true;
}



float calculateDrag(Vector3 position,Box &box,Physics &physics)
{
    float dx =(position.x - box.center.x) /(box.width / 2.0f);


    float dy = (position.y - box.center.y) / (box.height / 2.0f);


    float dz =(position.z - box.center.z) /(box.depth / 2.0f);


    float distanceSquared =dx * dx +dy * dy + dz * dz;

    float centreEffect = exp( -physics.dragFalloff *distanceSquared);


    float drag =physics.baseDrag +physics.centreDrag *centreEffect;


    return drag;
}

Vector3 calculateForces(Ball &ball,Box &box,Physics &physics)
{
    Vector3 force = {0,0,0};

    force.x +=ball.mass *physics.gravity.x;

    force.y +=ball.mass *physics.gravity.y;

    force.z +=ball.mass *physics.gravity.z;

    force.x +=physics.wind.x;

    force.y +=physics.wind.y;

    force.z +=physics.wind.z;


    float drag =calculateDrag( ball.position, box, physics );
    force.x += -drag * ball.velocity.x;

    force.y += -drag * ball.velocity.y; 
    force.z += -drag *ball.velocity.z;

    cout << "[DEBUG FORCE] position=("
     << ball.position.x << ", "
     << ball.position.y << ", "
     << ball.position.z << ") "
     << "velocity=("
     << ball.velocity.x << ", "
     << ball.velocity.y << ", "
     << ball.velocity.z << ") "
     << "drag=" << drag
     << endl;

    return force;
}


void eulerIntegration(Ball &ball,Box &box,Physics &physics, float dt)
{
    Vector3 force =calculateForces( ball,box, physics);
    Vector3 acceleration;
    acceleration.x =force.x /ball.mass;
    acceleration.y = force.y / ball.mass;

    acceleration.z =  force.z /ball.mass;



    ball.position.x = ball.position.x + ball.velocity.x * dt;
     ball.position.y = ball.position.y + ball.velocity.y * dt;

    ball.position.z = ball.position.z + ball.velocity.z * dt;

    ball.velocity.x = ball.velocity.x + acceleration.x * dt;

    ball.velocity.y = ball.velocity.y + acceleration.y * dt;

    ball.velocity.z = ball.velocity.z + acceleration.z * dt;
}



struct CollisionInfo
{
    bool collision;
    float time;
    Vector3 normal;
};


CollisionInfo findCollision(Ball &ball,Box &box,float dt)
{
    CollisionInfo result;
    result.collision = false;
    result.time = dt;
    result.normal = { 0,0,0};
    float minX = -box.width / 2.0f +ball.radius;
    float maxX = box.width / 2.0f -ball.radius;
    float minY = -box.height / 2.0f + ball.radius;

    float maxY =  box.height / 2.0f -ball.radius;

    float minZ = -box.depth / 2.0f + ball.radius;
    float maxZ =box.depth / 2.0f - ball.radius;
    Vector3 predicted;
    predicted.x = ball.position.x + ball.velocity.x * dt;

    predicted.y =ball.position.y +ball.velocity.y * dt;

    predicted.z =ball.position.z + ball.velocity.z * dt;

    if (ball.velocity.x < 0 && predicted.x < minX)
    {
        float t = (minX - ball.position.x) / ball.velocity.x;
        if (t >= 0 && t <= result.time)
        {
            result.collision = true;
            result.time = t;
            result.normal = {1,0,0};
        }
    }


//check right wall collision

    if (ball.velocity.x > 0 && predicted.x > maxX)
    {
        float t = (maxX - ball.position.x) / ball.velocity.x;


        if (t >= 0 &&
            t <= result.time)
        {
            result.collision = true;
            result.time = t;

            result.normal = {-1,0,  0};
        }
    }


//Check bottom wall collision

    if (ball.velocity.y < 0 && predicted.y < minY)
    {
        float t =
            (minY - ball.position.y) / ball.velocity.y;


        if (t >= 0 && t <= result.time)
        {
            result.collision = true;
            result.time = t;

            result.normal = {0,1,0};
        }
    }


    //Check top wall collision

    if (ball.velocity.y > 0 &&
        predicted.y > maxY)
    {
        float t =(maxY - ball.position.y) / ball.velocity.y;


        if (t >= 0 &&
            t <= result.time)
        {
            result.collision = true;
            result.time = t;
            result.normal = { 0,-1, 0};
        }
    }

    if (ball.velocity.z < 0 &&
        predicted.z < minZ)
    {
        float t = (minZ - ball.position.z) / ball.velocity.z;


        if (t >= 0 && t <= result.time)
        {
            result.collision = true;
            result.time = t;

            result.normal = { 0, 0, 1};
        }
    }


    //Check back wall collision

    if (ball.velocity.z > 0 &&
        predicted.z > maxZ)
    {
        float t =(maxZ - ball.position.z) /ball.velocity.z;
        if (t >= 0 && t <= result.time)
        {
            result.collision = true;
            result.time = t;
            result.normal = {  0,  0,  -1 };
        }
    }


    return result;
}



void resolveCollision(Ball &ball,Vector3 normal,Physics &physics)
{
    float normalVelocity = ball.velocity.x * normal.x + ball.velocity.y * normal.y +ball.velocity.z * normal.z;


    if (normalVelocity >= 0)
        return;


    Vector3 normalPart;

    normalPart.x = normal.x *    normalVelocity;

    normalPart.y =  normal.y * normalVelocity;

    normalPart.z =  normal.z * normalVelocity;


    Vector3 tangentPart;

    tangentPart.x = ball.velocity.x - normalPart.x;

    tangentPart.y = ball.velocity.y - normalPart.y;

    tangentPart.z = ball.velocity.z -normalPart.z;

    // Restitution

    normalPart.x *=  -physics.restitution;
    normalPart.y *=-physics.restitution;
    normalPart.z *= -physics.restitution;


    // Friction

    tangentPart.x *= (1.0f - physics.friction);

    tangentPart.y *=(1.0f - physics.friction);

    tangentPart.z *= (1.0f - physics.friction);


    ball.velocity.x =  normalPart.x + tangentPart.x;

    ball.velocity.y = normalPart.y + tangentPart.y;

    ball.velocity.z =normalPart.z +  tangentPart.z;
}


void updateSimulation(Ball &ball,Box &box,Physics &physics)
{
    float remainingTime = physics.timestep;


    int iterations = 0;


    while (
        remainingTime > 0.000001f &&iterations < 10)
    {
        CollisionInfo collision =findCollision(  ball, box,remainingTime
            );

        /*if (collision.normal.x > 0.5f)
            cout << "LEFT";

        else if (collision.normal.x < -0.5f)
            cout << "RIGHT";

        else if (collision.normal.y > 0.5f)
            cout << "BOTTOM";

        else if (collision.normal.y < -0.5f)
            cout << "TOP";

        else if (collision.normal.z > 0.5f)
            cout << "FRONT";

        else if (collision.normal.z < -0.5f)
            cout << "BACK";

        cout << endl;*/

        if (!collision.collision)
        {
            eulerIntegration(ball,box,physics,remainingTime);
            remainingTime = 0;

            break;
        }


        float collisionTime =collision.time;


        if (collisionTime > 0.000001f)
        {
            eulerIntegration(ball, box,physics,collisionTime   );
        }


        // Put ball exactly on collision surface

        if (collision.normal.x > 0.5f)
        {
            ball.position.x =-box.width / 2.0f +ball.radius;
        }


        if (collision.normal.x < -0.5f)
        {
            ball.position.x = box.width / 2.0f -ball.radius;
        }


        if (collision.normal.y > 0.5f)
        {
            ball.position.y = -box.height / 2.0f + ball.radius;
        }


        if (collision.normal.y < -0.5f)
        {
            ball.position.y = box.height / 2.0f - ball.radius;
        }


        if (collision.normal.z > 0.5f)
        {
            ball.position.z = -box.depth / 2.0f +ball.radius;
        }


        if (collision.normal.z < -0.5f)
        {
            ball.position.z = box.depth / 2.0f - ball.radius;
        }


        // Collision response

        resolveCollision(ball,collision.normal,physics);


        remainingTime -=collisionTime;


        // Small offset to prevent repeated collision - Suggested by AI

        ball.position.x += collision.normal.x * 0.00001f;

        ball.position.y += collision.normal.y * 0.00001f;
        ball.position.z += collision.normal.z * 0.00001f;
        iterations++;
        if (collisionTime < 0.000001f)
        {
            remainingTime -= 0.000001f;
        }
    }


    // Final safety checks

    float minX =  -box.width / 2.0f +  ball.radius;

    float maxX = box.width / 2.0f -  ball.radius;


    float minY =  -box.height / 2.0f +  ball.radius;

    float maxY =  box.height / 2.0f -  ball.radius;


    float minZ = -box.depth / 2.0f + ball.radius;

    float maxZ =   box.depth / 2.0f -   ball.radius;


    if (ball.position.x < minX)
        ball.position.x = minX;

    if (ball.position.x > maxX)
        ball.position.x = maxX;


    if (ball.position.y < minY)
        ball.position.y = minY;

    if (ball.position.y > maxY)
        ball.position.y = maxY;


    if (ball.position.z < minZ)
        ball.position.z = minZ;

    if (ball.position.z > maxZ)
        ball.position.z = maxZ;
}


string formatNumber(float number, int decimals = 2)
{
    stringstream stream;

    stream << fixed
           << setprecision(decimals)
           << number;

    return stream.str();
}

/*
===================================================
Below portion for the UI has been coded using the help of AI
====================================================*/

void drawInputScreen(
    int screenWidth,
    int screenHeight)
{
    ClearBackground(RAYWHITE);


    DrawText(
        "PHYSICALLY BASED BALL SIMULATION",
        30,
        15,
        27,
        DARKBLUE
    );


    DrawText(
        "USER INPUT / INITIAL CONDITIONS",
        30,
        48,
        18,
        BLACK
    );


    // --------------------------------------------------------
    // LEFT COLUMN
    // --------------------------------------------------------

    DrawText(
        "BALL",
        70,
        70,
        17,
        BLUE
    );

    DrawText(
        "Radius",
        130,
        77,
        15,
        DARKGRAY
    );

    DrawText(
        "Mass",
        130,
        112,
        15,
        DARKGRAY
    );


    DrawText(
        "STARTING POSITION",
        70,
        140,
        17,
        BLUE
    );

    DrawText(
        "X",
        130,
        167,
        15,
        DARKGRAY
    );

    DrawText(
        "Y",
        130,
        202,
        15,
        DARKGRAY
    );

    DrawText(
        "Z",
        130,
        237,
        15,
        DARKGRAY
    );


    DrawText(
        "STARTING VELOCITY",
        70,
        265,
        17,
        BLUE
    );

    DrawText(
        "X",
        130,
        292,
        15,
        DARKGRAY
    );

    DrawText(
        "Y",
        130,
        327,
        15,
        DARKGRAY
    );

    DrawText(
        "Z",
        130,
        362,
        15,
        DARKGRAY
    );


    DrawText(
        "BOX DIMENSIONS",
        70,
        390,
        17,
        BLUE
    );

    DrawText(
        "Width",
        130,
        417,
        15,
        DARKGRAY
    );

    DrawText(
        "Height",
        130,
        452,
        15,
        DARKGRAY
    );

    DrawText(
        "Depth",
        130,
        487,
        15,
        DARKGRAY
    );


    DrawText(
        "WIND",
        70,
        515,
        17,
        BLUE
    );

    DrawText(
        "X",
        130,
        542,
        15,
        DARKGRAY
    );

    DrawText(
        "Y",
        130,
        577,
        15,
        DARKGRAY
    );

    DrawText(
        "Z",
        130,
        612,
        15,
        DARKGRAY
    );


    // --------------------------------------------------------
    // RIGHT COLUMN
    // --------------------------------------------------------

    DrawText(
        "COLLISION / PHYSICS",
        650,
        60,
        20,
        BLACK
    );


    DrawText(
        "Restitution",
        680,
        87,
        15,
        DARKGRAY
    );

    DrawText(
        "Friction",
        680,
        122,
        15,
        DARKGRAY
    );

    DrawText(
        "Timestep (dt)",
        680,
        157,
        15,
        DARKGRAY
    );


    DrawText(
        "SPATIAL AIR RESISTANCE",
        650,
        195,
        20,
        DARKBLUE
    );


    DrawText(
        "Base Drag",
        680,
        217,
        15,
        DARKGRAY
    );

    DrawText(
        "Centre Drag",
        680,
        252,
        15,
        DARKGRAY
    );

    DrawText(
        "Drag Falloff",
        680,
        287,
        15,
        DARKGRAY
    );


    DrawText(
        "FIXED ENVIRONMENT",
        650,
        325,
        20,
        DARKBLUE
    );


    DrawText(
        "Gravity",
        680,
        357,
        15,
        DARKGRAY
    );


    DrawText(
        "Gravity is fixed at -9.81 m/s^2",
        680,
        395,
        15,
        DARKGREEN
    );


    DrawText(
        "Air resistance is weakest near the faces",
        680,
        425,
        15,
        DARKGREEN
    );


    DrawText(
        "and strongest around the centre.",
        680,
        450,
        15,
        DARKGREEN
    );


    DrawText(
        "CONTROLS",
        650,
        495,
        20,
        DARKBLUE
    );


    DrawText(
        "Click field to edit",
        680,
        525,
        15,
        DARKGRAY
    );

    DrawText(
        "TAB / UP / DOWN = change field",
        680,
        550,
        15,
        DARKGRAY
    );

    DrawText(
        "BACKSPACE = delete",
        680,
        575,
        15,
        DARKGRAY
    );

    DrawText(
        "ENTER = start simulation",
        680,
        600,
        15,
        DARKGREEN
    );


    // --------------------------------------------------------
    // INPUT BOXES
    // --------------------------------------------------------

    for (int i = 0; i < NUMBER_OF_FIELDS; i++)
    {
        Color borderColor =
            GRAY;


        if (inputFields[i].selected)
        {
            borderColor =
                DARKBLUE;
        }


        /*
            Gravity is displayed but not editable.
        */

        if (i == 20)
        {
            borderColor =
                LIGHTGRAY;
        }


        DrawRectangleLinesEx(
            inputFields[i].rectangle,
            2,
            borderColor
        );


        DrawText(
            inputFields[i].value.c_str(),
            inputFields[i].rectangle.x + 8,
            inputFields[i].rectangle.y + 6,
            15,
            i == 20 ? DARKGRAY : BLACK
        );
    }


    if (!errorMessage.empty())
    {
        DrawRectangle(
            30,
            650,
            1170,
            45,
            Fade(RED, 0.12f)
        );


        DrawText(
            errorMessage.c_str(),
            50,
            665,
            15,
            RED
        );
    }
}


// ------------------------------------------------------------
// INPUT HANDLING
// ------------------------------------------------------------

void handleInput()
{
    if (IsMouseButtonPressed(
            MOUSE_BUTTON_LEFT))
    {
        Vector2 mouse =
            GetMousePosition();


        for (int i = 0;
             i < NUMBER_OF_FIELDS;
             i++)
        {
            /*
                Gravity is not editable.
            */

            if (i == 20)
                continue;


            if (CheckCollisionPointRec(
                    mouse,
                    inputFields[i].rectangle))
            {
                for (int j = 0;
                     j < NUMBER_OF_FIELDS;
                     j++)
                {
                    inputFields[j].selected =
                        false;
                }


                inputFields[i].selected =
                    true;

                selectedField =
                    i;

                errorMessage =
                    "";

                break;
            }
        }
    }


    if (IsKeyPressed(KEY_TAB) ||
        IsKeyPressed(KEY_DOWN))
    {
        inputFields[selectedField].selected =
            false;


        selectedField++;


        if (selectedField >= 20)
            selectedField = 0;


        inputFields[selectedField].selected =
            true;
    }


    if (IsKeyPressed(KEY_UP))
    {
        inputFields[selectedField].selected =
            false;


        selectedField--;


        if (selectedField < 0)
            selectedField = 19;


        inputFields[selectedField].selected =
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
            inputFields[selectedField]
                .value +=
                (char)character;

            errorMessage = "";
        }


        character =
            GetCharPressed();
    }


    if (IsKeyPressed(KEY_BACKSPACE))
    {
        if (!inputFields[selectedField]
                 .value.empty())
        {
            inputFields[selectedField]
                .value
                .pop_back();
        }


        errorMessage = "";
    }
}


// ------------------------------------------------------------
// DRAW BOX
// ------------------------------------------------------------

void drawBox(Box &box)
{
    /*
        Wireframe allows the ball to remain visible.

        The six sides still exist for simulation purposes.
        We are only choosing not to fill the cube visually.
    */

    DrawCubeWires(
        box.center,
        box.width,
        box.height,
        box.depth,
        BLUE
    );
}


// ------------------------------------------------------------
// DRAW BALL
// ------------------------------------------------------------

void drawBall(Ball &ball)
{
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


// ------------------------------------------------------------
// DRAW SIMULATION INFORMATION
// ------------------------------------------------------------

void drawSimulationInfo(
    Ball &ball,
    Box &box,
    Physics &physics,
    double simulationTime,
    unsigned long long collisions,
    int screenWidth,
    int screenHeight)
{
    DrawRectangle(
        10,
        10,
        380,
        330,
        Fade(RAYWHITE, 0.90f)
    );


    DrawText(
        "PHYSICS SIMULATION",
        20,
        20,
        21,
        DARKBLUE
    );


    DrawText(
        "SPACE = pause/resume",
        20,
        55,
        15,
        DARKGRAY
    );


    DrawText(
        "R = reset / change parameters",
        20,
        80,
        15,
        DARKGRAY
    );


    string dtText =
        "Physics timestep: " +
        formatNumber(
            physics.timestep,
            6
        );


    DrawText(
        dtText.c_str(),
        20,
        115,
        16,
        BLUE
    );


    string timeText =
        "Simulation time: " +
        formatNumber(
            (float)simulationTime,
            2
        ) +
        " s";


    DrawText(
        timeText.c_str(),
        20,
        140,
        16,
        BLUE
    );


    float speed =
        sqrt(
            ball.velocity.x *
            ball.velocity.x +

            ball.velocity.y *
            ball.velocity.y +

            ball.velocity.z *
            ball.velocity.z
        );


    string speedText =
        "Speed: " +
        formatNumber(
            speed,
            2
        ) +
        " m/s";


    DrawText(
        speedText.c_str(),
        20,
        165,
        16,
        BLUE
    );


    string collisionText =
        "Collisions: " +
        to_string(collisions);


    DrawText(
        collisionText.c_str(),
        20,
        190,
        16,
        BLUE
    );


    float currentDrag =
        calculateDrag(
            ball.position,
            box,
            physics
        );


    string dragText =
        "Current spatial drag: " +
        formatNumber(
            currentDrag,
            5
        );


    DrawText(
        dragText.c_str(),
        20,
        215,
        15,
        DARKGREEN
    );


    string windText =
        "Wind: (" +
        formatNumber(physics.wind.x) +
        ", " +
        formatNumber(physics.wind.y) +
        ", " +
        formatNumber(physics.wind.z) +
        ")";


    DrawText(
        windText.c_str(),
        20,
        240,
        15,
        DARKGRAY
    );


    string positionText =
        "Position: (" +
        formatNumber(ball.position.x) +
        ", " +
        formatNumber(ball.position.y) +
        ", " +
        formatNumber(ball.position.z) +
        ")";


    DrawText(
        positionText.c_str(),
        20,
        270,
        14,
        DARKGRAY
    );


    string velocityText =
        "Velocity: (" +
        formatNumber(ball.velocity.x) +
        ", " +
        formatNumber(ball.velocity.y) +
        ", " +
        formatNumber(ball.velocity.z) +
        ")";


    DrawText(
        velocityText.c_str(),
        20,
        295,
        14,
        DARKGRAY
    );


    if (paused)
    {
        DrawText(
            "PAUSED",
            screenWidth - 120,
            20,
            20,
            ORANGE
        );
    }
    else
    {
        DrawText(
            "RUNNING",
            screenWidth - 130,
            20,
            20,
            GREEN
        );
    }


    string fps =
        "FPS: " +
        to_string(GetFPS());


    DrawText(
        fps.c_str(),
        screenWidth - 90,
        screenHeight - 30,
        16,
        DARKGRAY
    );
}


// ------------------------------------------------------------
// MAIN
// ------------------------------------------------------------

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


    setupInputFields();


    Ball ball;

    ball.position = {
        0,
        0,
        0
    };

    ball.velocity = {
        0,
        0,
        0
    };

    ball.radius =
        0.45f;

    ball.mass =
        1.0f;


    Box box;

    box.width =
        12.0f;

    box.height =
        10.0f;

    box.depth =
        12.0f;

    box.center = {
        0,
        0,
        0
    };


    Physics physics;

    physics.gravity = {
        0,
        -9.81f,
        0
    };


    physics.wind = {
        0.35f,
        0,
        -0.25f
    };


    physics.baseDrag =
        0.002f;

    physics.centreDrag =
        0.018f;

    physics.dragFalloff =
        2.0f;


    physics.restitution =
        0.88f;

    physics.friction =
        0.03f;

    physics.timestep =
        0.008333f;


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

    double simulationTime =
        0.0;


    unsigned long long collisionCount =
        0;


    while (!WindowShouldClose())
    {
        // =====================================================
        // INPUT SCREEN
        // =====================================================

        if (!simulationStarted)
        {
            handleInput();


            if (IsKeyPressed(KEY_ENTER))
            {
                if (createSimulation(
                        ball,
                        box,
                        physics))
                {
                    simulationStarted =
                        true;

                    paused =
                        false;

                    accumulator =
                        0.0;

                    simulationTime =
                        0.0;

                    collisionCount =
                        0;
                }
            }


            BeginDrawing();


            drawInputScreen(
                screenWidth,
                screenHeight
            );


            EndDrawing();


            continue;
        }


        // =====================================================
        // SIMULATION CONTROLS
        // =====================================================

        if (IsKeyPressed(KEY_SPACE))
        {
            paused =
                !paused;
        }


        if (IsKeyPressed(KEY_R))
        {
            simulationStarted =
                false;

            paused =
                false;

            accumulator =
                0.0;

            simulationTime =
                0.0;

            collisionCount =
                0;

            setupInputFields();

            continue;
        }


        // =====================================================
        // REAL FRAME TIME
        // =====================================================

        float frameTime =
            GetFrameTime();


        if (frameTime > 0.1f)
            frameTime = 0.1f;


        // =====================================================
        // CAMERA
        // =====================================================

        UpdateCamera(
            &camera,
            CAMERA_ORBITAL
        );


        // =====================================================
        // PHYSICS SIMULATION LOOP
        // =====================================================

        if (!paused)
        {
            accumulator +=
                frameTime;


            int physicsSteps =
                0;


            while (
                accumulator >=
                physics.timestep &&
                physicsSteps < 20)
            {
                Vector3 oldVelocity =
                    ball.velocity;


                updateSimulation(
                    ball,
                    box,
                    physics
                );


                /*
                    Simple collision counter.

                    This is only display information and does
                    not affect the physics.
                */

                float velocityChange =
                    fabs(
                        ball.velocity.x -
                        oldVelocity.x
                    ) +

                    fabs(
                        ball.velocity.y -
                        oldVelocity.y
                    ) +

                    fabs(
                        ball.velocity.z -
                        oldVelocity.z
                    );


                if (velocityChange > 0.1f)
                {
                    collisionCount++;
                }


                simulationTime +=
                    physics.timestep;


                accumulator -=
                    physics.timestep;


                physicsSteps++;
            }
        }


        // =====================================================
        // DRAW
        // =====================================================

        BeginDrawing();


        ClearBackground(
            RAYWHITE
        );


        BeginMode3D(
            camera
        );


        drawBox(
            box
        );


        // Coordinate axes

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


        drawBall(
            ball
        );


        EndMode3D();


        drawSimulationInfo(
            ball,
            box,
            physics,
            simulationTime,
            collisionCount,
            screenWidth,
            screenHeight
        );


        EndDrawing();
    }


    CloseWindow();


    return 0;
}