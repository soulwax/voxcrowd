import 'dart:math' as math;

import 'agent.dart';
import 'math.dart';

/// Standard tactical unit formation geometries.
enum FormationKind { wedge, line, box, ring }

/// Generates world XZ formation slots anchored to a leader pose and assigns agents.
final class FormationPattern {
  const FormationPattern._();

  static List<CrowdVec2> computeSlots({
    required CrowdVec2 leaderPosition,
    CrowdVec2 leaderHeading = CrowdVec2.forward,
    required int count,
    FormationKind kind = FormationKind.wedge,
    double spacing = 2.0,
    double offsetBehind = 0.0,
  }) {
    if (count <= 0) return const [];
    final fwd = leaderHeading.lengthSquared > 1e-12
        ? leaderHeading.normalized
        : CrowdVec2.forward;
    final right = fwd.rightPerp;
    final anchor = leaderPosition - fwd * offsetBehind;
    final slots = <CrowdVec2>[];

    switch (kind) {
      case FormationKind.line:
        final centerIdx = (count - 1) * 0.5;
        for (var i = 0; i < count; i++) {
          final lateral = (i - centerIdx) * spacing;
          slots.add(anchor + right * lateral);
        }

      case FormationKind.wedge:
        slots.add(anchor);
        for (var i = 1; i < count; i++) {
          final row = (i + 1) ~/ 2;
          final side = i.isOdd ? -1.0 : 1.0;
          final backDist = row * spacing;
          final sideDist = row * spacing * 0.85 * side;
          slots.add(anchor - fwd * backDist + right * sideDist);
        }

      case FormationKind.box:
        final cols = math.sqrt(count.toDouble()).ceil();
        for (var i = 0; i < count; i++) {
          final row = i ~/ cols;
          final col = i % cols;
          final lateral = (col - (cols - 1) * 0.5) * spacing;
          final backDist = row * spacing;
          slots.add(anchor - fwd * backDist + right * lateral);
        }

      case FormationKind.ring:
        final radius = math.max(spacing, (count * spacing) / (2.0 * math.pi));
        for (var i = 0; i < count; i++) {
          final angle = (i / count) * math.pi * 2.0;
          final localDir = fwd * math.cos(angle) + right * math.sin(angle);
          slots.add(anchor + localDir * radius);
        }
    }

    return slots;
  }

  /// Greedily matches each enabled agent to the nearest unassigned slot and sets `agent.goal`.
  static List<CrowdVec2> assignNearestSlots(
    List<CrowdAgent> agents,
    List<CrowdVec2> slots,
  ) {
    final active = [
      for (final a in agents)
        if (a.enabled) a,
    ];
    if (active.isEmpty || slots.isEmpty) return const [];

    final usedSlots = List<bool>.filled(slots.length, false);
    final assigned = <CrowdVec2>[];

    for (final agent in active) {
      var bestIdx = -1;
      var bestDistSq = double.infinity;
      for (var s = 0; s < slots.length; s++) {
        if (usedSlots[s]) continue;
        final d2 = (agent.position - slots[s]).lengthSquared;
        if (d2 < bestDistSq) {
          bestDistSq = d2;
          bestIdx = s;
        }
      }
      if (bestIdx >= 0) {
        usedSlots[bestIdx] = true;
        agent.goal = slots[bestIdx];
        assigned.add(slots[bestIdx]);
      }
    }

    return assigned;
  }
}
