<p align="center">
  <img src="assets/logo.svg" width="88" alt="voxcrowd logo"/>
</p>

# voxcrowd

Pure Dart reciprocal crowd collision avoidance (ORCA / RVO2), Reynolds steering, boid flocking, and tactical formations.

## Features

- **ORCA / RVO2 Reciprocal Avoidance (`OrcaSolver`)**: Linear-program velocity-obstacle half-plane solver in XZ with priority weights, static circle/segment obstacle avoidance, and anti-overlap separation.
- **Steering & Boids (`SteeringBehaviours`, `BoidFlockRule`)**: `seek`, `arrive`, `flee`, `pursue`, `evade`, `wander`, and weighted separation/alignment/cohesion flocking.
- **Tactical Formations (`FormationPattern`)**: `wedge`, `line`, `box`, and `ring` formation slots aligned to leader heading with nearest-agent assignment.
