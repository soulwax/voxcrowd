import 'math.dart';

/// Kinematic crowd navigation agent participating in ORCA avoidance and steering.
final class CrowdAgent {
  final String id;
  CrowdVec2 position;
  CrowdVec2 velocity;
  CrowdVec2 preferredVelocity;
  CrowdVec2? goal;

  double radius;
  double maxSpeed;
  double maxAcceleration;
  double neighborDist;
  double timeHorizon;
  double timeHorizonObstacles;

  /// Higher weight yields more readily during reciprocal ORCA encounters.
  /// `0.0` acts as an unyielding VIP/battering-ram agent; `0.5` is balanced symmetric RVO.
  double avoidanceWeight;
  int layerMask;
  bool enabled;

  int activeNeighborCount = 0;
  int activeConstraintCount = 0;
  CrowdVec2 resolvedVelocity = CrowdVec2.zero;

  CrowdAgent({
    required this.id,
    required this.position,
    this.velocity = CrowdVec2.zero,
    this.preferredVelocity = CrowdVec2.zero,
    this.goal,
    this.radius = 0.5,
    this.maxSpeed = 4.0,
    this.maxAcceleration = 16.0,
    this.neighborDist = 6.0,
    this.timeHorizon = 2.5,
    this.timeHorizonObstacles = 1.25,
    this.avoidanceWeight = 0.5,
    this.layerMask = 0x7fffffff,
    this.enabled = true,
  });
}

/// Static circular obstacle in the XZ crowd plane (pillars, trees, boulders).
final class CrowdCircleObstacle {
  final String id;
  CrowdVec2 center;
  double radius;

  CrowdCircleObstacle({
    required this.id,
    required this.center,
    required this.radius,
  });
}

/// Static wall line segment obstacle `[start, end]` in the XZ crowd plane.
final class CrowdSegmentObstacle {
  final String id;
  CrowdVec2 start;
  CrowdVec2 end;

  CrowdSegmentObstacle({
    required this.id,
    required this.start,
    required this.end,
  });

  /// Returns closest point on segment `[start, end]` to [p].
  CrowdVec2 closestPointTo(CrowdVec2 p) {
    final ab = end - start;
    final lenSq = ab.lengthSquared;
    if (lenSq <= 1e-12) return start;
    final t = ((p - start).dot(ab) / lenSq).clamp(0.0, 1.0);
    return start + ab * t;
  }
}
