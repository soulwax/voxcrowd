import 'dart:math' as math;

import 'agent.dart';
import 'math.dart';
import 'steering.dart';

/// Half-plane constraint in 2D velocity space: `det(direction, v - point) >= 0`.
final class OrcaLine {
  final CrowdVec2 point;
  final CrowdVec2 direction;

  const OrcaLine({
    required this.point,
    required this.direction,
  });
}

/// Telemetry summary produced by [OrcaSolver.step].
final class CrowdStepStats {
  final int activeAgents;
  final int totalConstraints;
  final double minPairwiseClearance;
  final double averageSpeed;

  const CrowdStepStats({
    required this.activeAgents,
    required this.totalConstraints,
    required this.minPairwiseClearance,
    required this.averageSpeed,
  });
}

/// Pure-Dart ORCA / RVO2 reciprocal collision avoidance solver on the XZ plane.
final class OrcaSolver {
  static const double _eps = 1e-6;

  final double defaultSlowingRadius;
  final double defaultGoalStopRadius;

  const OrcaSolver({
    this.defaultSlowingRadius = 1.5,
    this.defaultGoalStopRadius = 0.05,
  });

  /// Computes conflict-free velocities for all enabled [agents] and integrates their positions over [dt].
  CrowdStepStats step(
    List<CrowdAgent> agents,
    double dt, {
    List<CrowdCircleObstacle> circleObstacles = const [],
    List<CrowdSegmentObstacle> segmentObstacles = const [],
    bool integratePositions = true,
    bool computeGoalPreferredVelocity = true,
  }) {
    if (dt <= 0.0 || agents.isEmpty) {
      return CrowdStepStats(
        activeAgents: agents.where((a) => a.enabled).length,
        totalConstraints: 0,
        minPairwiseClearance: double.infinity,
        averageSpeed: 0.0,
      );
    }

    final enabledAgents = [
      for (final a in agents)
        if (a.enabled) a,
    ];

    if (computeGoalPreferredVelocity) {
      for (final a in enabledAgents) {
        if (a.goal != null) {
          a.preferredVelocity = SteeringBehaviours.arrive(
            a.position,
            a.goal!,
            a.maxSpeed,
            slowingRadius: defaultSlowingRadius,
            stopRadius: defaultGoalStopRadius,
          );
        }
      }
    }

    var totalConstraints = 0;

    // Solve new velocity for every enabled agent synchronously from current positions & velocities.
    for (var i = 0; i < enabledAgents.length; i++) {
      final agent = enabledAgents[i];
      final orcaLines = <OrcaLine>[];
      var neighborCount = 0;

      // 1. Static circle obstacles (full avoidance responsibility = 1.0).
      for (final circle in circleObstacles) {
        final line = _buildCircleOrAgentConstraint(
          agentPos: agent.position,
          agentVel: agent.velocity,
          agentRadius: agent.radius,
          otherPos: circle.center,
          otherVel: CrowdVec2.zero,
          otherRadius: circle.radius,
          timeHorizon: math.max(dt, agent.timeHorizonObstacles),
          dt: dt,
          responsibility: 1.0,
        );
        if (line != null) orcaLines.add(line);
      }

      // 2. Static wall segment obstacles projected via closest point capsule radius.
      for (final seg in segmentObstacles) {
        final closest = seg.closestPointTo(agent.position);
        final distSq = (agent.position - closest).lengthSquared;
        final maxCheck = agent.neighborDist + agent.radius;
        if (distSq <= maxCheck * maxCheck) {
          final line = _buildCircleOrAgentConstraint(
            agentPos: agent.position,
            agentVel: agent.velocity,
            agentRadius: agent.radius,
            otherPos: closest,
            otherVel: CrowdVec2.zero,
            otherRadius: 0.05,
            timeHorizon: math.max(dt, agent.timeHorizonObstacles),
            dt: dt,
            responsibility: 1.0,
          );
          if (line != null) orcaLines.add(line);
        }
      }

      final numObstLines = orcaLines.length;

      // 3. Dynamic crowd agent neighbors within neighborDist.
      final invTimeHorizon = 1.0 / math.max(dt, agent.timeHorizon);
      for (var j = 0; j < enabledAgents.length; j++) {
        if (i == j) continue;
        final other = enabledAgents[j];
        if ((agent.layerMask & other.layerMask) == 0) continue;

        final relPos = other.position - agent.position;
        final maxDist = agent.neighborDist + agent.radius + other.radius;
        if (relPos.lengthSquared > maxDist * maxDist) continue;

        neighborCount++;
        final weightSum = agent.avoidanceWeight + other.avoidanceWeight;
        final responsibility =
            weightSum > 1e-8 ? (agent.avoidanceWeight / weightSum).clamp(0.0, 1.0) : 0.5;

        final line = _buildCircleOrAgentConstraint(
          agentPos: agent.position,
          agentVel: agent.velocity,
          agentRadius: agent.radius,
          otherPos: other.position,
          otherVel: other.velocity,
          otherRadius: other.radius,
          timeHorizon: 1.0 / invTimeHorizon,
          dt: dt,
          responsibility: responsibility,
        );
        if (line != null) orcaLines.add(line);
      }

      agent.activeNeighborCount = neighborCount;
      agent.activeConstraintCount = orcaLines.length;
      totalConstraints += orcaLines.length;

      // 4. Solve 2D linear program minimizing distance to preferredVelocity.
      final optPref = agent.preferredVelocity.clampLength(agent.maxSpeed);
      final solved = _solveLinearProgram2(orcaLines, agent.maxSpeed, optPref);
      CrowdVec2 candidate = solved.velocity;
      if (solved.failedLine < orcaLines.length) {
        candidate = _solveLinearProgram3(
          orcaLines,
          numObstLines,
          solved.failedLine,
          agent.maxSpeed,
          candidate,
        );
      }

      // Clamp acceleration over dt.
      final maxDeltaV = agent.maxAcceleration * dt;
      final dv = (candidate - agent.velocity).clampLength(maxDeltaV);
      agent.resolvedVelocity = (agent.velocity + dv).clampLength(agent.maxSpeed);
    }

    // Apply resolved velocities & integrate positions.
    var totalSpeed = 0.0;
    for (final a in enabledAgents) {
      a.velocity = a.resolvedVelocity;
      if (integratePositions) {
        a.position = a.position + a.velocity * dt;
      }
      totalSpeed += a.velocity.length;
    }

    // Compute pairwise surface clearance telemetry.
    var minClearance = double.infinity;
    for (var i = 0; i < enabledAgents.length; i++) {
      for (var j = i + 1; j < enabledAgents.length; j++) {
        final a = enabledAgents[i];
        final b = enabledAgents[j];
        final clearance = a.position.distanceTo(b.position) - (a.radius + b.radius);
        if (clearance < minClearance) minClearance = clearance;
      }
    }

    return CrowdStepStats(
      activeAgents: enabledAgents.length,
      totalConstraints: totalConstraints,
      minPairwiseClearance: minClearance,
      averageSpeed: enabledAgents.isEmpty ? 0.0 : totalSpeed / enabledAgents.length,
    );
  }

  OrcaLine? _buildCircleOrAgentConstraint({
    required CrowdVec2 agentPos,
    required CrowdVec2 agentVel,
    required double agentRadius,
    required CrowdVec2 otherPos,
    required CrowdVec2 otherVel,
    required double otherRadius,
    required double timeHorizon,
    required double dt,
    required double responsibility,
  }) {
    final relativePosition = otherPos - agentPos;
    final relativeVelocity = agentVel - otherVel;
    final distSq = relativePosition.lengthSquared;
    final combinedRadius = agentRadius + otherRadius;
    final combinedRadiusSq = combinedRadius * combinedRadius;

    CrowdVec2 u;
    CrowdVec2 lineDirection;

    if (distSq > combinedRadiusSq) {
      final invTimeHorizon = 1.0 / math.max(dt, timeHorizon);
      final w = relativeVelocity - relativePosition * invTimeHorizon;
      final wLenSq = w.lengthSquared;
      final dotProduct1 = w.dot(relativePosition);

      if (dotProduct1 < 0.0 &&
          dotProduct1 * dotProduct1 > combinedRadiusSq * wLenSq) {
        // Project on cut-off circle.
        final wLen = math.sqrt(wLenSq);
        final unitW = wLen > _eps ? (w / wLen) : CrowdVec2.unitX;
        lineDirection = CrowdVec2(unitW.z, -unitW.x);
        u = unitW * (combinedRadius * invTimeHorizon - wLen);
      } else {
        // Project onto velocity obstacle legs.
        final leg = math.sqrt(math.max(0.0, distSq - combinedRadiusSq));
        if (CrowdVec2.det(relativePosition, w) > 0.0) {
          // Project on left leg.
          lineDirection = CrowdVec2(
                relativePosition.x * leg - relativePosition.z * combinedRadius,
                relativePosition.x * combinedRadius + relativePosition.z * leg,
              ) /
              distSq;
        } else {
          // Project on right leg.
          lineDirection = -CrowdVec2(
                relativePosition.x * leg + relativePosition.z * combinedRadius,
                -relativePosition.x * combinedRadius + relativePosition.z * leg,
              ) /
              distSq;
        }
        final dotProduct2 = relativeVelocity.dot(lineDirection);
        u = lineDirection * dotProduct2 - relativeVelocity;
      }
    } else {
      // Currently overlapping: push apart within a single simulation step.
      final invTimeStep = 1.0 / math.max(dt, 1e-4);
      final w = relativeVelocity - relativePosition * invTimeStep;
      final wLen = w.length;
      final unitW = wLen > _eps
          ? (w / wLen)
          : (relativePosition.lengthSquared > _eps
              ? -relativePosition.normalized
              : CrowdVec2.unitX);
      lineDirection = CrowdVec2(unitW.z, -unitW.x);
      u = unitW * (combinedRadius * invTimeStep - wLen);
    }

    return OrcaLine(
      point: agentVel + u * responsibility,
      direction: lineDirection,
    );
  }

  ({bool feasible, CrowdVec2 velocity}) _solveLinearProgram1(
    List<OrcaLine> lines,
    int lineNo,
    double radius,
    CrowdVec2 optVelocity,
    bool directionOpt,
  ) {
    final line = lines[lineNo];
    final dotProduct = line.point.dot(line.direction);
    final discriminant = dotProduct * dotProduct + radius * radius - line.point.lengthSquared;
    if (discriminant < 0.0) {
      return (feasible: false, velocity: CrowdVec2.zero);
    }

    final sqrtDiscriminant = math.sqrt(discriminant);
    var tLeft = -dotProduct - sqrtDiscriminant;
    var tRight = -dotProduct + sqrtDiscriminant;

    for (var i = 0; i < lineNo; i++) {
      final denom = CrowdVec2.det(line.direction, lines[i].direction);
      final num = CrowdVec2.det(lines[i].direction, line.point - lines[i].point);

      if (denom.abs() <= _eps) {
        if (num < 0.0) return (feasible: false, velocity: CrowdVec2.zero);
        continue;
      }

      final t = num / denom;
      if (denom >= 0.0) {
        tRight = math.min(tRight, t);
      } else {
        tLeft = math.max(tLeft, t);
      }

      if (tLeft > tRight) return (feasible: false, velocity: CrowdVec2.zero);
    }

    if (directionOpt) {
      if (optVelocity.dot(line.direction) > 0.0) {
        return (feasible: true, velocity: line.point + line.direction * tRight);
      }
      return (feasible: true, velocity: line.point + line.direction * tLeft);
    }

    final t = line.direction.dot(optVelocity - line.point);
    final clampedT = t.clamp(tLeft, tRight);
    return (feasible: true, velocity: line.point + line.direction * clampedT);
  }

  ({int failedLine, CrowdVec2 velocity}) _solveLinearProgram2(
    List<OrcaLine> lines,
    double radius,
    CrowdVec2 optVelocity, {
    bool directionOpt = false,
  }) {
    var result = directionOpt
        ? optVelocity.normalized * radius
        : optVelocity.clampLength(radius);

    for (var i = 0; i < lines.length; i++) {
      if (CrowdVec2.det(lines[i].direction, lines[i].point - result) > 0.0) {
        final prev = result;
        final lp1 = _solveLinearProgram1(lines, i, radius, optVelocity, directionOpt);
        if (!lp1.feasible) {
          return (failedLine: i, velocity: prev);
        }
        result = lp1.velocity;
      }
    }
    return (failedLine: lines.length, velocity: result);
  }

  CrowdVec2 _solveLinearProgram3(
    List<OrcaLine> lines,
    int numObstLines,
    int beginLine,
    double radius,
    CrowdVec2 initialResult,
  ) {
    var result = initialResult;
    var distance = 0.0;

    for (var i = beginLine; i < lines.length; i++) {
      if (CrowdVec2.det(lines[i].direction, lines[i].point - result) > distance) {
        final projLines = <OrcaLine>[
          for (var k = 0; k < numObstLines; k++) lines[k],
        ];

        for (var j = numObstLines; j < i; j++) {
          final determinant = CrowdVec2.det(lines[i].direction, lines[j].direction);
          CrowdVec2 point;
          if (determinant.abs() <= _eps) {
            if (lines[i].direction.dot(lines[j].direction) > 0.0) {
              continue;
            }
            point = (lines[i].point + lines[j].point) * 0.5;
          } else {
            point = lines[i].point +
                lines[i].direction *
                    (CrowdVec2.det(lines[j].direction, lines[i].point - lines[j].point) /
                        determinant);
          }

          final dir = (lines[j].direction - lines[i].direction).normalized;
          projLines.add(OrcaLine(point: point, direction: dir));
        }

        final tempResult = result;
        final optDir = CrowdVec2(-lines[i].direction.z, lines[i].direction.x);
        final lp2 = _solveLinearProgram2(
          projLines,
          radius,
          optDir,
          directionOpt: true,
        );
        if (lp2.failedLine < projLines.length) {
          result = tempResult;
        } else {
          result = lp2.velocity;
        }
        distance = CrowdVec2.det(lines[i].direction, lines[i].point - result);
      }
    }

    return result;
  }
}
