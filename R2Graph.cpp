#include <cassert>
#include "R2Graph.h"

double R2Point::distanceToLine(const R2Point& p, const R2Vector& v) const {
    R2Vector n = v.normal();
    n.normalize();
    return fabs(n*(*this - p));
}

bool intersectLineSegments(
    const R2Point& p0, const R2Point& p1,   // First line segment
    const R2Point& q0, const R2Point& q1,   // Second line segmant
    R2Point& intersection                   // Result
) {
    R2Point inter;
    if (!intersectStraightLines(
        p0, p1,
        q0, q1,
        inter
    ))
        return false;

    R2Vector n = (p1 - p0).normal();
    R2Vector m = (q1 - q0).normal();
    if (
        ((p0 - q0)*m) * ((p1 - q0)*m) <= 0. &&
        ((q0 - p0)*n) * ((q1 - p0)*n) <= 0.
    ) {
        intersection = inter;
        return true;
    } else {
        return false;
    }
}

bool intersectLineSegmentAndLine(
    const R2Point& p0, const R2Point& p1,   // Line segment
    const R2Point& q, const R2Vector& v,    // Straight line
    R2Point& intersection                   // Result
) {
    R2Point inter;
    if (!intersectStraightLines(
        p0, (p1 - p0),
        q, v,
        inter
    ))
        return false;

    R2Vector n = v.normal();
    if (
        ((p0 - q)*n) * ((p1 - q)*n) <= 0.
    ) {
        intersection = inter;
        return true;
    } else {
        return false;
    }
}

bool intersectStraightLines(
    const R2Point& p, const R2Vector& v,    // First line
    const R2Point& q, const R2Vector& w,    // Second line
    R2Point& intersection                   // Result
) {
    assert(
        v != R2Vector(0., 0.) &&
        w != R2Vector(0., 0.)
    );
    if (
        v == R2Vector(0., 0.) ||
        w == R2Vector(0., 0.)
    )
        return false;
    R2Vector n = v.normal();
    // Point s on the second line:
    //     s = q + w*t
    // Intersection: scalar product equals zero
    // (s - p, n) == 0
    // (q + w*t - p, n) = 0
    // t = (p - q, n) / (w, n)
    double wn = w*n;
    double qpn = (p - q)*n;

    if (fabs(wn) <= R2GRAPH_EPSILON) {
        // Parallel lines
        if (fabs(qpn) <= R2GRAPH_EPSILON) {
            // Equal lines
            intersection = p;
            return true;
        }
        return false;
    }
    double t = qpn / wn;
    intersection = q + w*t;
    return true;
}

double distanceToLine(
    const R2Point& t,                   // Point
    const R2Point& p, const R2Vector& v // Line: (p, v)
) {
    return t.distanceToLine(p, v);
}

bool intersectCircles(
    const R2Point& center1, double r1,  // The first circle
    const R2Point& center2, double r2,  // The second circle
    R2Point intersections[2]            // Two points of intersection
) {
    //                      *
    //                    _/| \_
    //          r1    _/    |      r2
    //          _ /         | h     \_
    //       /              |            \_
    //     *-------------+--+----+----------*
    // center1         d-r2 x    r1        center2
    //
    //     d = center1.distance(center2)
    //
    //   {  r1^2 = x^2 + h^2
    //   {  r2^2 = (d-x)^2 + h^2
    //
    //   r1^2 - r2^2 = x^2 - (d - x)^2
    //   r1^2 - r2^2 = -d^2 + 2*d*x
    //   x = (d^2 + r1^2 - r2^2)/(2*d)
    //   h = sqrt(r1^2 - x^2)

    double d = center1.distance(center2);
    if (d <= R2GRAPH_EPSILON || d > r1 + r2 || r1 + r2 <= R2GRAPH_EPSILON) {
        return false;
    } else if (d <= r1 + r2 && d >= r1 + r2 - R2GRAPH_EPSILON) {
        intersections[0] = center1 + (center2 - center1)*(r1/(r1 + r2));
        intersections[1] = intersections[0];
        return true;
    }
    double rMax = r1; double rMin = r2;
    R2Point maxCenter = center1;
    R2Point minCenter = center2;
    if (r1 < r2) {
        rMax = r2; rMin = r1;
        maxCenter = center2;
        minCenter = center1;
    }
    if (d < rMax - rMin) {
        return false;
    } else if (d <= rMax - rMin) {
        // Circle touch each other
        R2Vector ex = minCenter - maxCenter;
        ex.normalize();
        intersections[0] = maxCenter + ex*rMax;
        intersections[1] = intersections[0];
    }

    double x = (d*d + r1*r1 - r2*r2)/(2.*d);
    assert(x <= r1);
    double h = sqrt(r1*r1 - x*x);
    R2Vector ex = center2 - center1;
    ex.normalize();
    R2Vector ey = ex.normal();
    R2Point px = center1 + ex*x;
    intersections[0] = px + ey*h;
    intersections[1] = px - ey*h;
    return true;
}

bool intersectLineAndCircle(
    const R2Point& p, const R2Vector& v,    // the straight line,
    const R2Point& center, double radius,   // the circle
    R2Point intersections[2]
) {
    double d = center.distanceToLine(p, v);
    if (d > radius) {
        return false;
    } else if (d >= radius - R2GRAPH_EPSILON) {
        // Tangent line
        intersectStraightLines(
            p, v,
            center, v.normal(),
            intersections[0]
        );
        intersections[1] = intersections[0];
        return true;
    }
    R2Point q;
    intersectStraightLines(
        p, v,
        center, v.normal(),
        q
    );
    double x = sqrt(radius*radius - d*d);
    R2Vector ex = v;
    ex.normalize();
    intersections[0] = q - ex*x;
    intersections[1] = q + ex*x;
    return true;
}
