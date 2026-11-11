import 'package:test/test.dart';
import 'package:voxcrowd/voxcrowd.dart';

void main() {
  group('OrcaSolver Reciprocal Avoidance', () {
    test('head-on swapped agents pass cleanly without overlapping', () {
      const solver = OrcaSolver();
      final agents = [
        CrowdAgent(
          id: 'a',
          position: const CrowdVec2(-6.0, 0.0),
          goal: const CrowdVec2(6.0, 0.0),
          radius: 0.5,
          maxSpeed: 3.5,
        ),
        CrowdAgent(
          id: 'b',
          position: const CrowdVec2(6.0, 0.0),
          goal: const CrowdVec2(-6.0, 0.0),
          radius: 0.5,
          maxSpeed: 3.5,
        ),
      ];

      // Break exact zero-determinant head-on collinear symmetry gently via initial nudge.
      agents[0].preferredVelocity = const CrowdVec2(3.5, 0.05);
      agents[1].preferredVelocity = const CrowdVec2(-3.5, -0.05);

      var minClearanceObserved = double.infinity;
      for (var step = 0; step < 120; step++) {
        // Keep a tiny tie-break on goal seek if perfectly collinear on X axis.
        final stats = solver.step(
          agents,
          0.05,
          computeGoalPreferredVelocity: false,
        );
        for (final a in agents) {
          final desired = SteeringBehaviours.arrive(a.position, a.goal!, a.maxSpeed);
          a.preferredVelocity = CrowdVec2(
            desired.x,
            desired.z + (a.id == 'a' ? 0.02 : -0.02),
          );
        }
        if (stats.minPairwiseClearance < minClearanceObserved) {
          minClearanceObserved = stats.minPairwiseClearance;
        }
      }

      expect(minClearanceObserved, greaterThan(-0.08));
      expect(agents[0].position.distanceTo(const CrowdVec2(6.0, 0.0)), lessThan(0.5));
      expect(agents[1].position.distanceTo(const CrowdVec2(-6.0, 0.0)), lessThan(0.5));
    });

    test('8-agent circle swap reaches antipodal goals while avoiding central obstacle', () {
      const solver = OrcaSolver();
      final pillar = CrowdCircleObstacle(
        id: 'fountain',
        center: const CrowdVec2(0.35, 0.2),
        radius: 1.0,
      );

      final agents = <CrowdAgent>[];
      const count = 8;
      final slots = FormationPattern.computeSlots(
        leaderPosition: CrowdVec2.zero,
        count: count,
        kind: FormationKind.ring,
        spacing: 5.5,
      );
      for (var i = 0; i < count; i++) {
        agents.add(
          CrowdAgent(
            id: 'u_$i',
            position: slots[i],
            goal: -slots[i],
            radius: 0.45,
            maxSpeed: 3.4,
          ),
        );
      }

      for (var step = 0; step < 260; step++) {
        for (final a in agents) {
          final desired = SteeringBehaviours.arrive(a.position, a.goal!, a.maxSpeed);
          a.preferredVelocity = SteeringBehaviours.steerAroundCircle(
            a.position,
            CrowdVec2(
              desired.x - desired.z * 0.04,
              desired.z + desired.x * 0.04,
            ),
            pillar.center,
            pillar.radius,
            clearance: a.radius + 0.35,
          );
        }
        solver.step(
          agents,
          0.05,
          circleObstacles: [pillar],
          computeGoalPreferredVelocity: false,
        );
      }

      for (final a in agents) {
        expect(a.position.distanceTo(a.goal!), lessThan(0.8));
        expect(
          a.position.distanceTo(pillar.center),
          greaterThanOrEqualTo(pillar.radius + a.radius - 0.1),
        );
      }
    });
  });

  group('BoidFlockRule & FormationPattern', () {
    test('flock moves cohesively and aligns toward shared goal', () {
      const rule = BoidFlockRule();
      const solver = OrcaSolver();
      final flock = [
        CrowdAgent(id: 'f0', position: const CrowdVec2(-1.0, 0.0), maxSpeed: 3.0),
        CrowdAgent(id: 'f1', position: const CrowdVec2(0.5, 0.8), maxSpeed: 3.0),
        CrowdAgent(id: 'f2', position: const CrowdVec2(-0.4, -1.2), maxSpeed: 3.0),
      ];

      for (var i = 0; i < 60; i++) {
        rule.applyToFlock(flock, sharedGoal: const CrowdVec2(10.0, 0.0));
        solver.step(flock, 0.05, computeGoalPreferredVelocity: false);
      }

      for (final a in flock) {
        expect(a.position.x, greaterThan(5.0));
      }
    });

    test('FormationPattern assigns wedge and box slots symmetrically', () {
      final slots = FormationPattern.computeSlots(
        leaderPosition: CrowdVec2.zero,
        leaderHeading: const CrowdVec2(0.0, -1.0),
        count: 5,
        kind: FormationKind.wedge,
        spacing: 2.0,
      );
      expect(slots.length, 5);
      expect(slots[0].distanceTo(CrowdVec2.zero), closeTo(0.0, 1e-6));
      expect(slots[1].z, closeTo(2.0, 1e-6)); // behind -Z leader = +Z
      expect(slots[1].x, closeTo(-slots[2].x, 1e-6));
    });
  });
}
