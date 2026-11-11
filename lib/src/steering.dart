import 'dart:math' as math;

import 'agent.dart';
import 'math.dart';

/// Stateless Reynolds steering behaviours operating on XZ crowd vectors.
final class SteeringBehaviours {
  const SteeringBehaviours._();

  static CrowdVec2 seek(CrowdVec2 pos, CrowdVec2 target, double maxSpeed) {
    final diff = target - pos;
    if (diff.lengthSquared <= 1e-12) return CrowdVec2.zero;
    return diff.normalized * maxSpeed;
  }

  static CrowdVec2 arrive(
    CrowdVec2 pos,
    CrowdVec2 target,
    double maxSpeed, {
    double slowingRadius = 1.5,
    double stopRadius = 0.05,
  }) {
    final diff = target - pos;
    final dist = diff.length;
    if (dist <= stopRadius) return CrowdVec2.zero;
    final ramped = slowingRadius > 1e-6 ? maxSpeed * (dist / slowingRadius) : maxSpeed;
    final speed = math.min(maxSpeed, ramped);
    return (diff / dist) * speed;
  }

  static CrowdVec2 flee(
    CrowdVec2 pos,
    CrowdVec2 threat,
    double maxSpeed, {
    double panicRadius = double.infinity,
  }) {
    final diff = pos - threat;
    final distSq = diff.lengthSquared;
    if (panicRadius.isFinite && distSq > panicRadius * panicRadius) {
      return CrowdVec2.zero;
    }
    if (distSq <= 1e-12) return CrowdVec2.unitZ * maxSpeed;
    return diff.normalized * maxSpeed;
  }

  static CrowdVec2 pursue(
    CrowdVec2 pos,
    CrowdVec2 targetPos,
    CrowdVec2 targetVel,
    double maxSpeed,
  ) {
    final dist = pos.distanceTo(targetPos);
    final t = maxSpeed > 1e-5 ? (dist / maxSpeed).clamp(0.0, 1.5) : 0.0;
    return seek(pos, targetPos + targetVel * t, maxSpeed);
  }

  static CrowdVec2 evade(
    CrowdVec2 pos,
    CrowdVec2 threatPos,
    CrowdVec2 threatVel,
    double maxSpeed, {
    double panicRadius = double.infinity,
  }) {
    final dist = pos.distanceTo(threatPos);
    final t = maxSpeed > 1e-5 ? (dist / maxSpeed).clamp(0.0, 1.5) : 0.0;
    return flee(pos, threatPos + threatVel * t, maxSpeed, panicRadius: panicRadius);
  }

  static CrowdVec2 wander(
    CrowdVec2 currentVel,
    double wanderAngle, {
    double circleDistance = 2.0,
    double circleRadius = 1.0,
    double maxSpeed = 4.0,
  }) {
    final forward =
        currentVel.lengthSquared > 1e-6 ? currentVel.normalized : CrowdVec2.forward;
    final center = forward * circleDistance;
    final offset = CrowdVec2(
      math.cos(wanderAngle) * circleRadius,
      math.sin(wanderAngle) * circleRadius,
    );
    return (center + offset).normalized * maxSpeed;
  }

  /// Tangential whisker steering bias around a circular static obstacle ahead.
  static CrowdVec2 steerAroundCircle(
    CrowdVec2 pos,
    CrowdVec2 desiredVel,
    CrowdVec2 obstacleCenter,
    double obstacleRadius, {
    double clearance = 0.6,
    double lookAhead = 3.5,
  }) {
    final speed = desiredVel.length;
    if (speed <= 1e-6) return CrowdVec2.zero;
    final fwd = desiredVel / speed;
    final toObs = obstacleCenter - pos;
    final proj = toObs.dot(fwd);
    final safeR = obstacleRadius + clearance;
    if (proj < -safeR || proj > lookAhead + safeR) return desiredVel;

    final closest = pos + fwd * math.max(0.0, proj);
    final lateral = closest - obstacleCenter;
    final latDist = lateral.length;
    if (latDist >= safeR) return desiredVel;

    // Pick right-hand side consistently when dead-center.
    final sideDir = latDist > 1e-4 ? (lateral / latDist) : fwd.rightPerp;
    final weight = ((safeR - latDist) / safeR).clamp(0.0, 1.0);
    final steered = (fwd + sideDir * (1.35 * weight)).normalized;
    return steered * speed;
  }
}

/// Configurable Reynolds boid rule set computing separation, alignment, cohesion,
/// and optional target seek into an agent's `preferredVelocity` for ORCA refinement.
final class BoidFlockRule {
  final double separationRadius;
  final double alignmentRadius;
  final double cohesionRadius;
  final double separationWeight;
  final double alignmentWeight;
  final double cohesionWeight;
  final double goalWeight;

  const BoidFlockRule({
    this.separationRadius = 2.2,
    this.alignmentRadius = 5.0,
    this.cohesionRadius = 6.5,
    this.separationWeight = 2.2,
    this.alignmentWeight = 1.0,
    this.cohesionWeight = 0.85,
    this.goalWeight = 1.4,
  });

  CrowdVec2 computePreferredVelocity(
    CrowdAgent self,
    List<CrowdAgent> flock, {
    CrowdVec2? targetGoal,
  }) {
    var separation = CrowdVec2.zero;
    var sepCount = 0;

    var avgVel = CrowdVec2.zero;
    var alignCount = 0;

    var centerOfMass = CrowdVec2.zero;
    var cohCount = 0;

    for (final other in flock) {
      if (identical(other, self) || !other.enabled) continue;
      final diff = self.position - other.position;
      final dist = diff.length;

      if (dist < separationRadius) {
        final push = dist > 1e-5
            ? (diff / dist) * ((separationRadius - dist) / separationRadius)
            : CrowdVec2.unitX;
        separation += push;
        sepCount++;
      }

      if (dist < alignmentRadius) {
        avgVel += other.velocity;
        alignCount++;
      }

      if (dist < cohesionRadius) {
        centerOfMass += other.position;
        cohCount++;
      }
    }

    var combined = CrowdVec2.zero;
    if (sepCount > 0) {
      combined += (separation / sepCount.toDouble()).normalized * (separationWeight * self.maxSpeed);
    }
    if (alignCount > 0) {
      final alignDir = (avgVel / alignCount.toDouble()).normalized;
      combined += alignDir * (alignmentWeight * self.maxSpeed);
    }
    if (cohCount > 0) {
      final com = centerOfMass / cohCount.toDouble();
      combined += SteeringBehaviours.seek(self.position, com, self.maxSpeed) * cohesionWeight;
    }

    final goal = targetGoal ?? self.goal;
    if (goal != null) {
      combined += SteeringBehaviours.arrive(self.position, goal, self.maxSpeed) * goalWeight;
    }

    return combined.clampLength(self.maxSpeed);
  }

  /// Evaluates flocking forces across all enabled members of [flock] in one pass.
  void applyToFlock(List<CrowdAgent> flock, {CrowdVec2? sharedGoal}) {
    final nextPrefs = [
      for (final a in flock)
        a.enabled
            ? computePreferredVelocity(a, flock, targetGoal: sharedGoal)
            : CrowdVec2.zero,
    ];
    for (var i = 0; i < flock.length; i++) {
      if (flock[i].enabled) {
        flock[i].preferredVelocity = nextPrefs[i];
      }
    }
  }
}
