# Physically Based Ball Simulation

A C++ 3D ball-bouncing simulation for the Physically Based Modeling and Animation assignment.

## Project Structure

```text
Assignment1/
├── include/
│   └── raylib.h
├── lib/
│   └── libraylib.a
├── src/
│   └── main.cpp
├── build/
│   └── PhysicallyBasedBall.exe
└── README.md
```

## Requirements

- Windows
- MinGW / GCC with `g++`
- C++17 or later
- raylib library compiled for MinGW/GCC

## Compile

Run the following command from any directory. Full paths are used so the separate
`build` directory does not cause problems locating the source file:

```powershell
g++ -std=c++17 -o "C:\College\Physics_Based_Modelling\Assignment1\build\PhysicallyBasedBall.exe" "C:\College\Physics_Based_Modelling\Assignment1\src\main.cpp" -I"C:\College\Physics_Based_Modelling\Assignment1\include" -L"C:\College\Physics_Based_Modelling\Assignment1\lib" -lraylib -lopengl32 -lgdi32 -lwinmm
```

## Run

```powershell
& "C:\College\Physics_Based_Modelling\Assignment1\build\PhysicallyBasedBall.exe"
```

Alternatively, navigate to the assignment directory first:

```powershell
cd "C:\College\Physics_Based_Modelling\Assignment1"
```

Then run:

```powershell
.\build\PhysicallyBasedBall.exe
```

## Recompile After Changes

Every time `src\main.cpp` is modified, run the compile command again to create an
updated executable in the `build` directory.

## Notes

- `src/` contains the assignment source code written for the simulation.
- `include/` contains raylib headers.
- `lib/` contains the raylib MinGW static library.
- `build/` contains generated executables and should not contain source code.
- The physics simulation will be implemented separately from the rendering code as the project develops.
