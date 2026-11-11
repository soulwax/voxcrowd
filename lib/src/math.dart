import 'dart:math' as math;

/// Planar XZ vector used across `voxcrowd` crowd avoidance and steering.
final class CrowdVec2 {
  final double x;
  final double z;

  const CrowdVec2(this.x, this.z);

  static const CrowdVec2 zero = CrowdVec2(0.0, 0.0);
  static const CrowdVec2 unitX = CrowdVec2(1.0, 0.0);
  static const CrowdVec2 unitZ = CrowdVec2(0.0, 1.0);
  static const CrowdVec2 forward = CrowdVec2(0.0, -1.0);

  CrowdVec2 operator +(CrowdVec2 o) => CrowdVec2(x + o.x, z + o.z);
  CrowdVec2 operator -(CrowdVec2 o) => CrowdVec2(x - o.x, z - o.z);
  CrowdVec2 operator -() => CrowdVec2(-x, -z);
  CrowdVec2 operator *(double s) => CrowdVec2(x * s, z * s);
  CrowdVec2 operator /(double s) => CrowdVec2(x / s, z / s);

  double dot(CrowdVec2 o) => x * o.x + z * o.z;

  /// 2D signed determinant / perp-dot product `a.x * b.z - a.z * b.x`.
  static double det(CrowdVec2 a, CrowdVec2 b) => a.x * b.z - a.z * b.x;

  double get lengthSquared => x * x + z * z;
  double get length => math.sqrt(lengthSquared);

  CrowdVec2 get normalized {
    final len = length;
    if (len <= 1e-12) return CrowdVec2.zero;
    return CrowdVec2(x / len, z / len);
  }

  /// Right-handed perpendicular `(z, -x)` in XZ plane.
  CrowdVec2 get rightPerp => CrowdVec2(z, -x);

  /// Left-handed perpendicular `(-z, x)` in XZ plane.
  CrowdVec2 get leftPerp => CrowdVec2(-z, x);

  CrowdVec2 clampLength(double maxLen) {
    final lenSq = lengthSquared;
    if (lenSq <= maxLen * maxLen) return this;
    final len = math.sqrt(lenSq);
    if (len <= 1e-12) return CrowdVec2.zero;
    final scale = maxLen / len;
    return CrowdVec2(x * scale, z * scale);
  }

  double distanceTo(CrowdVec2 o) => (this - o).length;

  static CrowdVec2 lerp(CrowdVec2 a, CrowdVec2 b, double t) =>
      CrowdVec2(a.x + (b.x - a.x) * t, a.z + (b.z - a.z) * t);

  @override
  String toString() =>
      'CrowdVec2(${x.toStringAsFixed(3)}, ${z.toStringAsFixed(3)})';
}
