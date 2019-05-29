# Medieval Physics simulation in PhysX using C++

key features:
- Custom Actors (CustomActors.h)
- Ellipsoid convex mesh creation
- Cylinder convex mesh creation
- Prismatic Joints for moving obstacles 
- Connected dynamic actors and joints
- Pure force and physics based object movement (catapult moves by wheels turning and experiencing friction)
- Invisible collision boxes

The brief for this project was loose; implement elements that would be in a medieval fantasy and rugby game/simulator hybrid. For this project, I wanted to simulate the actions of a catapult firing a rugby ball, the catapult moving and experiencing realistic forces (friction etc), realistic looking goal posts (cylinders not cuboids) and have goals that were actually score-able (collision detection).

Circles, cylinders and ellipsoid like shapes are not inherent in PhysX like cubes and spheres are, so I had to get creative with convex hulls to make the cylinders for the wheels on my catapult and the poles for the goal posts. Even more challenging was the rugby ball, where I wrote a function to generate and connect each vertex on the ball, so the rugby ball could be generated with varying numbers of segments (increased segments meant more detailed ball) in a realistic ellipsoid shape.

Once the catapult was setup with joints, stoppers, connected masses etc. I needed to create rugby posts that could check for collisions when the ball flew pass them. Again, invisible collision boxes where not provided, so I wrote my own code to generate the geometry of the box, without rendering the visuals.

uses PhysX SDK 3.3.4
includes visual debugger


