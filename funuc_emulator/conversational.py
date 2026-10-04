"""Conversational programming with an Auto Geometry Engine (A.G.E.).

Solves missing profile geometry (line/arc end points, tangents, arc centres,
intersections) from partial data and generates G-code for conversational
events: Drill, Bolt Hole, Mill, Arc, Pocket, Profile, Conrad (corner radius),
subroutine transforms (repeat/rotate/mirror/scale) and cutter compensation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

Point = Tuple[float, float]
EPS = 1e-9


class AGEError(ValueError):
    """Raised when geometry is under- or over-constrained or has no solution."""


def _f(v: float) -> str:
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if s in ("", "-0") else s


# ---------------------------------------------------------------- geometry
def line_line(p1: Point, a1: float, p2: Point, a2: float) -> Point:
    """Intersection of two lines given point and angle (degrees)."""
    d1 = (math.cos(math.radians(a1)), math.sin(math.radians(a1)))
    d2 = (math.cos(math.radians(a2)), math.sin(math.radians(a2)))
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) < EPS:
        raise AGEError("Lines are parallel; no intersection")
    t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / den
    return (p1[0] + t * d1[0], p1[1] + t * d1[1])


def line_circle(p: Point, angle: float, c: Point, r: float,
                guess: Optional[Point] = None) -> Point:
    """Intersection of a line (point+angle) with a circle.

    Two solutions are resolved with ``guess`` (nearest wins); without a
    guess the one further along the line direction is chosen.
    """
    d = (math.cos(math.radians(angle)), math.sin(math.radians(angle)))
    fx, fy = p[0] - c[0], p[1] - c[1]
    b = fx * d[0] + fy * d[1]
    disc = b * b - (fx * fx + fy * fy - r * r)
    if disc < -EPS:
        raise AGEError("Line does not intersect circle")
    s = math.sqrt(max(disc, 0.0))
    sols = [(p[0] + (-b + k * s) * d[0], p[1] + (-b + k * s) * d[1]) for k in (1, -1)]
    if guess is not None:
        return min(sols, key=lambda q: math.dist(q, guess))
    return sols[0]


def circle_circle(c1: Point, r1: float, c2: Point, r2: float,
                  guess: Optional[Point] = None) -> Point:
    """Intersection of two circles, disambiguated by ``guess``."""
    d = math.dist(c1, c2)
    if d < EPS or d > r1 + r2 + EPS or d < abs(r1 - r2) - EPS:
        raise AGEError("Circles do not intersect")
    a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
    h = math.sqrt(max(r1 * r1 - a * a, 0.0))
    ux, uy = (c2[0] - c1[0]) / d, (c2[1] - c1[1]) / d
    mx, my = c1[0] + a * ux, c1[1] + a * uy
    sols = [(mx - h * uy, my + h * ux), (mx + h * uy, my - h * ux)]
    if guess is not None:
        return min(sols, key=lambda q: math.dist(q, guess))
    return sols[0]


def tangent_point_from_point(p: Point, c: Point, r: float, cw: bool) -> Point:
    """Tangent point on circle (c, r) for a line starting at ``p``.

    ``cw`` is the direction the arc is subsequently traversed.
    """
    d = math.dist(p, c)
    if d < r - EPS:
        raise AGEError("Point is inside circle; no tangent exists")
    base = math.atan2(p[1] - c[1], p[0] - c[0])
    off = math.acos(min(1.0, r / d)) if d > EPS else 0.0
    # CW travel: tangent point lies counter-clockwise of the point-to-centre ray
    ang = base + off if cw else base - off
    return (c[0] + r * math.cos(ang), c[1] + r * math.sin(ang))


def fillet(p0: Point, corner: Point, p1: Point, radius: float):
    """Conrad: round the corner ``p0-corner-p1``.

    Returns (tangent_in, tangent_out, centre, is_cw).
    """
    v0 = (p0[0] - corner[0], p0[1] - corner[1])
    v1 = (p1[0] - corner[0], p1[1] - corner[1])
    l0, l1 = math.hypot(*v0), math.hypot(*v1)
    if l0 < EPS or l1 < EPS:
        raise AGEError("Degenerate corner")
    u0, u1 = (v0[0] / l0, v0[1] / l0), (v1[0] / l1, v1[1] / l1)
    cosang = max(-1.0, min(1.0, u0[0] * u1[0] + u0[1] * u1[1]))
    ang = math.acos(cosang)
    if abs(math.sin(ang)) < EPS:
        raise AGEError("Collinear segments; cannot radius corner")
    t = radius / math.tan(ang / 2)
    if t > l0 + EPS or t > l1 + EPS:
        raise AGEError("Corner radius too large for adjacent segments")
    ta = (corner[0] + u0[0] * t, corner[1] + u0[1] * t)
    tb = (corner[0] + u1[0] * t, corner[1] + u1[1] * t)
    bis = (u0[0] + u1[0], u0[1] + u1[1])
    bl = math.hypot(*bis)
    dist = radius / math.sin(ang / 2)
    c = (corner[0] + bis[0] / bl * dist, corner[1] + bis[1] / bl * dist)
    cross = (corner[0] - p0[0]) * (p1[1] - corner[1]) - (corner[1] - p0[1]) * (p1[0] - corner[0])
    return ta, tb, c, cross < 0


# ---------------------------------------------------------------- A.G.E.
@dataclass
class Element:
    """A profile element. Unknown fields are left as ``None``.

    kind: 'line' or 'arc'. ``guess`` is an approximate end point used to
    resolve ambiguous solutions. ``tangent`` means tangent to the next element.
    """
    kind: str
    x: Optional[float] = None
    y: Optional[float] = None
    angle: Optional[float] = None      # line angle, degrees
    radius: Optional[float] = None     # arc radius
    cw: bool = False                   # arc direction
    cx: Optional[float] = None         # arc centre (absolute)
    cy: Optional[float] = None
    tangent: bool = False              # tangent to next element
    guess: Optional[Point] = None
    # solver status per field: "given" | "calculated" | "guess"
    status: dict = field(default_factory=dict)

    def tangent_ok(self) -> bool:
        return True


def _end_dir(el: Element, start: Point) -> Optional[float]:
    """Travel direction (deg) at the end of a solved element."""
    if el.kind == "line":
        return math.degrees(math.atan2(el.y - start[1], el.x - start[0]))
    a = math.atan2(el.y - el.cy, el.x - el.cx)
    return math.degrees(a + (-math.pi / 2 if el.cw else math.pi / 2))


def solve_profile(start: Point, elements: List[Element],
                  strict: bool = True) -> List[Element]:
    """Fill in missing geometry for a connected chain of lines and arcs.

    Iterates until no more progress is made; raises AGEError listing any
    elements that stay unsolved ("Not Calculated").
    """
    for el in elements:
        for name in ("x", "y", "angle", "radius", "cx", "cy"):
            if getattr(el, name) is not None:
                el.status[name] = "given"
    for _ in range(len(elements) * 4 + 4):
        progress = False
        pos = start
        prev_dir: Optional[float] = None
        if _solve_triples(start, elements):
            progress = True
        for i, el in enumerate(elements):
            before = (el.x, el.y, el.cx, el.cy, el.angle, el.radius)
            nxt = elements[i + 1] if i + 1 < len(elements) else None
            _solve_one(el, pos, prev_dir, nxt)
            if before != (el.x, el.y, el.cx, el.cy, el.angle, el.radius):
                progress = True
            if el.x is None or el.y is None:
                break
            prev_dir = _end_dir(el, pos) if (el.kind == "line" or el.cx is not None) else None
            pos = (el.x, el.y)
        if not progress:
            break
    bad = [i + 1 for i, e in enumerate(elements) if not _complete(e)]
    if bad and strict:
        raise AGEError(f"Not Calculated: element(s) {bad} under-constrained")
    return elements


def _starts(start: Point, elements: List[Element]) -> List[Optional[Point]]:
    out: List[Optional[Point]] = []
    pos: Optional[Point] = start
    for el in elements:
        out.append(pos)
        pos = (el.x, el.y) if el.x is not None and el.y is not None else None
    return out


def tangent_line_two_arcs(c1: Point, r1: float, cw1: bool,
                          c2: Point, r2: float, cw2: bool):
    """Line leaving arc 1 and entering arc 2 tangentially (travel-direction
    aware: external or internal tangent is chosen by the arc directions).

    Returns (point_on_arc1, point_on_arc2).
    """
    s1, s2 = (-1 if cw1 else 1), (-1 if cw2 else 1)
    k = s2 * r2 - s1 * r1
    vx, vy = c2[0] - c1[0], c2[1] - c1[1]
    dist = math.hypot(vx, vy)
    if dist < EPS or abs(k) > dist + EPS:
        raise AGEError("No tangent line exists between arcs")
    theta = math.atan2(vy, vx) - math.asin(max(-1.0, min(1.0, k / dist)))
    nx, ny = -math.sin(theta), math.cos(theta)
    return ((c1[0] - s1 * r1 * nx, c1[1] - s1 * r1 * ny),
            (c2[0] - s2 * r2 * nx, c2[1] - s2 * r2 * ny))


def arc_tangent_two_lines(start: Point, a1: float, end2: Point, a2: float,
                          radius: float):
    """Arc of ``radius`` tangent to line 1 (from ``start`` at angle a1) and
    line 2 (through ``end2`` at angle a2). Returns (t1, t2, centre, cw)."""
    corner = line_line(start, a1, end2, a2)
    return fillet(start, corner, end2, radius)


def _solve_triples(start: Point, els: List[Element]) -> bool:
    """Patterns spanning three elements: arc-line-arc and line-arc-line."""
    changed = False
    starts = _starts(start, els)
    for i in range(len(els) - 2):
        a, b, c = els[i], els[i + 1], els[i + 2]
        sp = starts[i]
        if a.kind == "arc" and b.kind == "line" and c.kind == "arc" and b.tangent_ok() \
                and a.cx is not None and a.cy is not None and a.x is None and a.y is None \
                and c.cx is not None and c.cy is not None and c.radius is not None \
                and sp is not None:
            r1 = a.radius if a.radius is not None else math.dist(sp, (a.cx, a.cy))
            p1, p2 = tangent_line_two_arcs((a.cx, a.cy), r1, a.cw,
                                           (c.cx, c.cy), c.radius, c.cw)
            _set(a, "x", p1[0]); _set(a, "y", p1[1])
            _set(b, "x", p2[0]); _set(b, "y", p2[1])
            _set(a, "radius", r1)
            changed = True
        elif a.kind == "line" and b.kind == "arc" and c.kind == "line" \
                and a.x is None and a.y is None and a.angle is not None \
                and b.radius is not None and b.cx is None and b.x is None \
                and c.angle is not None and c.x is not None and c.y is not None \
                and sp is not None:
            t1, t2, ctr, cw = arc_tangent_two_lines(sp, a.angle, (c.x, c.y), c.angle, b.radius)
            _set(a, "x", t1[0]); _set(a, "y", t1[1])
            _set(b, "x", t2[0]); _set(b, "y", t2[1])
            _set(b, "cx", ctr[0]); _set(b, "cy", ctr[1])
            b.cw = cw
            changed = True
    return changed


def _complete(el: Element) -> bool:
    if el.x is None or el.y is None:
        return False
    return el.kind == "line" or (el.cx is not None and el.cy is not None)


def _set(el, name, val, how="calculated"):
    if getattr(el, name) is None:
        setattr(el, name, val)
        el.status[name] = how


def _solve_one(el: Element, s: Point, prev_dir: Optional[float], nxt: Optional[Element]):
    if el.kind == "line":
        if el.angle is None and prev_dir is not None and el.status.get("tan_prev"):
            _set(el, "angle", prev_dir)
        if el.x is not None and el.y is not None:
            if el.angle is None:
                _set(el, "angle", math.degrees(math.atan2(el.y - s[1], el.x - s[0])))
            return
        if el.angle is not None:
            ca, sa = math.cos(math.radians(el.angle)), math.sin(math.radians(el.angle))
            if el.x is not None and abs(ca) > EPS:
                _set(el, "y", s[1] + (el.x - s[0]) * sa / ca)
            elif el.y is not None and abs(sa) > EPS:
                _set(el, "x", s[0] + (el.y - s[1]) * ca / sa)
            elif nxt is not None:
                _solve_line_via_next(el, s, nxt)
        return
    # ---- arc
    if el.cx is None and el.cy is None and el.radius is not None \
            and el.x is not None and el.y is not None:
        _arc_centre_from_radius(el, s)
    elif el.radius is not None and el.cx is None and el.cy is None \
            and prev_dir is not None and el.status.get("tan_prev"):
        sign = -1 if el.cw else 1
        a = math.radians(prev_dir + sign * 90)
        _set(el, "cx", s[0] + el.radius * math.cos(a))
        _set(el, "cy", s[1] + el.radius * math.sin(a))
    if el.cx is not None and el.cy is not None:
        r = math.dist(s, (el.cx, el.cy))
        _set(el, "radius", r)
        if el.x is None and el.y is None and nxt is not None and nxt.kind == "line" \
                and nxt.angle is not None and el.tangent:
            # end tangent to next line at known angle
            sign = -1 if el.cw else 1
            ang = math.radians(nxt.angle - sign * 90)
            _set(el, "x", el.cx + r * math.cos(ang))
            _set(el, "y", el.cy + r * math.sin(ang))
        elif (el.x is None) != (el.y is None):
            if el.x is not None:
                dy2 = r * r - (el.x - el.cx) ** 2
                if dy2 < -EPS:
                    raise AGEError("Arc end X not on circle")
                ys = [el.cy + k * math.sqrt(max(dy2, 0)) for k in (1, -1)]
                _set(el, "y", min(ys, key=lambda v: abs(v - (el.guess[1] if el.guess else s[1]))))
            else:
                dx2 = r * r - (el.y - el.cy) ** 2
                if dx2 < -EPS:
                    raise AGEError("Arc end Y not on circle")
                xs = [el.cx + k * math.sqrt(max(dx2, 0)) for k in (1, -1)]
                _set(el, "x", min(xs, key=lambda v: abs(v - (el.guess[0] if el.guess else s[0]))))
        elif el.x is None and el.y is None and el.guess is not None and nxt is not None \
                and nxt.kind == "line" and nxt.angle is not None and nxt.x is not None \
                and nxt.y is not None:
            q = line_circle((nxt.x, nxt.y), nxt.angle, (el.cx, el.cy), r, el.guess)
            _set(el, "x", q[0], "calculated")
            _set(el, "y", q[1], "calculated")


def _arc_centre_from_radius(el: Element, s: Point):
    e = (el.x, el.y)
    d = math.dist(s, e)
    r = el.radius
    if d < EPS or d > 2 * r + EPS:
        raise AGEError("Arc radius too small for end points")
    h = math.sqrt(max(r * r - (d / 2) ** 2, 0.0))
    mx, my = (s[0] + e[0]) / 2, (s[1] + e[1]) / 2
    ux, uy = (e[0] - s[0]) / d, (e[1] - s[1]) / d
    # centre to the right of travel for CW, left for CCW (minor arc)
    sgn = -1 if el.cw else 1
    _set(el, "cx", mx - sgn * h * uy)
    _set(el, "cy", my + sgn * h * ux)


def _solve_line_via_next(el: Element, s: Point, nxt: Element):
    """Line with known angle ending at the intersection with the next element."""
    if nxt.kind == "line" and nxt.angle is not None and nxt.x is not None and nxt.y is not None:
        q = line_line(s, el.angle, (nxt.x, nxt.y), nxt.angle)
    elif nxt.kind == "arc" and nxt.cx is not None and nxt.cy is not None and nxt.radius is not None:
        if el.tangent:
            q = tangent_point_from_point(s, (nxt.cx, nxt.cy), nxt.radius, nxt.cw)
        else:
            q = line_circle(s, el.angle, (nxt.cx, nxt.cy), nxt.radius,
                            el.guess or nxt.guess)
    else:
        return
    _set(el, "x", q[0])
    _set(el, "y", q[1])


def tangent_to_previous(el: Element) -> Element:
    """Mark ``el`` as starting tangent to the preceding element."""
    el.status["tan_prev"] = True
    return el


def profile_toolpath(start: Point, elements: List[Element]) -> List[Tuple]:
    """Solved elements -> list of ('line',x,y) / ('arc',x,y,i,j,cw)."""
    path, pos = [], start
    for el in elements:
        if el.kind == "line":
            path.append(("line", el.x, el.y))
        else:
            path.append(("arc", el.x, el.y, el.cx - pos[0], el.cy - pos[1], el.cw))
        pos = (el.x, el.y)
    return path


def conrad(points: List[Point], radius: float, closed: bool = False) -> List[Tuple]:
    """Round every interior corner of a polyline with a single radius input."""
    n = len(points)
    path: List[Tuple] = []
    cur = points[0]
    idx = range(1, n) if not closed else range(0, n)
    for i in idx:
        if not closed and i == n - 1:
            path.append(("line", *points[i]))
            break
        p0, c, p1 = points[(i - 1) % n], points[i], points[(i + 1) % n]
        ta, tb, ctr, cw = fillet(p0, c, p1, radius)
        path.append(("line", *ta))
        path.append(("arc", tb[0], tb[1], ctr[0] - ta[0], ctr[1] - ta[1], cw))
    return path


def chamfer(p0: Point, corner: Point, p1: Point, size: float):
    """Chamfer a corner with equal legs of ``size``; returns (ta, tb)."""
    v0 = (p0[0] - corner[0], p0[1] - corner[1])
    v1 = (p1[0] - corner[0], p1[1] - corner[1])
    l0, l1 = math.hypot(*v0), math.hypot(*v1)
    if l0 < EPS or l1 < EPS or size > min(l0, l1) + EPS:
        raise AGEError("Chamfer too large for adjacent segments")
    return ((corner[0] + v0[0] / l0 * size, corner[1] + v0[1] / l0 * size),
            (corner[0] + v1[0] / l1 * size, corner[1] + v1[1] / l1 * size))


def chamfer_corners(points: List[Point], size: float) -> List[Tuple]:
    """Chamfer every interior corner of an open polyline."""
    path: List[Tuple] = []
    for i in range(1, len(points) - 1):
        ta, tb = chamfer(points[i - 1], points[i], points[i + 1], size)
        path += [("line", *ta), ("line", *tb)]
    path.append(("line", *points[-1]))
    return path


GIVEN, CALCULATED, GUESS, NOT_CALCULATED = "Given", "Calculated", "Guess", "Not Calculated"
STATUS_COLOURS = {GIVEN: "white", CALCULATED: "green", GUESS: "orange",
                  NOT_CALCULATED: "red"}


def field_report(start: Point, elements: List[Element]) -> List[dict]:
    """Solve as far as possible (never raises on under-constraint) and report
    every relevant field with its status for real-time feedback."""
    try:
        solve_profile(start, elements, strict=False)
    except AGEError as exc:
        return [{"element": 0, "field": "error", "value": None,
                 "status": NOT_CALCULATED, "colour": "red", "message": str(exc)}]
    rows = []
    for i, el in enumerate(elements, 1):
        names = ["x", "y"] + (["angle"] if el.kind == "line" else ["radius", "cx", "cy"])
        for n in names:
            v = getattr(el, n)
            if v is not None:
                st = el.status.get(n, GIVEN)
                st = GIVEN if st == "given" else CALCULATED if st == "calculated" else st
            elif n in ("x", "y") and el.guess is not None:
                v, st = el.guess[0 if n == "x" else 1], GUESS
            else:
                st = NOT_CALCULATED
            rows.append({"element": i, "field": n, "value": v, "status": st,
                         "colour": STATUS_COLOURS[st]})
    return rows


def fully_constrained(rows: List[dict]) -> bool:
    return all(r["status"] in (GIVEN, CALCULATED) for r in rows)


def format_report(rows: List[dict]) -> str:
    out = []
    for r in rows:
        v = "--" if r["value"] is None else _f(r["value"])
        out.append(f"{r['element']:>2} {r['field']:<7} {v:>10}  {r['status']}")
    return "\n".join(out)


def parse_elements(text: str) -> List[Element]:
    """Parse conversational element lines, e.g.
    ``line angle=45 x=10`` / ``arc r=5 cw tangent guess=12,3``."""
    els = []
    for raw in text.splitlines():
        parts = raw.split("#")[0].split()
        if not parts:
            continue
        kind = parts[0].lower()
        if kind not in ("line", "arc"):
            raise AGEError(f"Unknown element '{parts[0]}'")
        el = Element(kind)
        keys = {"x": "x", "y": "y", "angle": "angle", "a": "angle", "r": "radius",
                "radius": "radius", "cx": "cx", "cy": "cy", "i": None}
        for tok in parts[1:]:
            t = tok.lower()
            if t in ("cw", "ccw"):
                el.cw = t == "cw"
            elif t == "tangent":
                el.tangent = True
            elif "=" in t:
                k, v = t.split("=", 1)
                try:
                    if k == "guess":
                        gx, gy = v.split(",")
                        el.guess = (float(gx), float(gy))
                    elif keys.get(k):
                        setattr(el, keys[k], float(v))
                    else:
                        raise KeyError
                except (ValueError, KeyError):
                    raise AGEError(f"Bad token '{tok}'")
            else:
                raise AGEError(f"Bad token '{tok}'")
        els.append(el)
    return els


# ---------------------------------------------------------------- G-code
def _path_gcode(path, feed: Optional[float]) -> List[str]:
    out, first = [], True
    for seg in path:
        fw = f" F{_f(feed)}" if (feed and first) else ""
        if seg[0] == "line":
            out.append(f"G01 X{_f(seg[1])} Y{_f(seg[2])}{fw}")
        else:
            g = "G02" if seg[5] else "G03"
            out.append(f"{g} X{_f(seg[1])} Y{_f(seg[2])} I{_f(seg[3])} J{_f(seg[4])}{fw}")
        first = False
    return out


def drill(x, y, z, r=2.0, feed=100.0, peck: Optional[float] = None,
          cycle: Optional[str] = None, dwell: Optional[float] = None) -> List[str]:
    """Single-point drill (G81/G83), tap (G84) or bore (G85/G86)."""
    g = {"drill": "G81", "peck": "G83", "tap": "G84", "bore": "G85"}[
        cycle or ("peck" if peck else "drill")]
    q = f" Q{_f(peck)}" if g == "G83" else ""
    p = f" P{_f(dwell)}" if dwell else ""
    return [f"{g} X{_f(x)} Y{_f(y)} Z{_f(z)} R{_f(r)}{q}{p} F{_f(feed)}", "G80"]


def bolt_hole_points(cx, cy, radius, count, start_angle=0.0, pitch=None) -> List[Point]:
    """Hole centres on a circle; ``pitch`` (deg) defaults to 360/count."""
    if count < 1:
        raise AGEError("Bolt hole count must be >= 1")
    step = pitch if pitch is not None else 360.0 / count
    return [(cx + radius * math.cos(math.radians(start_angle + k * step)),
             cy + radius * math.sin(math.radians(start_angle + k * step)))
            for k in range(count)]


def bolt_hole(cx, cy, radius, count, z, start_angle=0.0, pitch=None, r=2.0,
              feed=100.0) -> List[str]:
    pts = bolt_hole_points(cx, cy, radius, count, start_angle, pitch)
    lines = [f"G81 X{_f(pts[0][0])} Y{_f(pts[0][1])} Z{_f(z)} R{_f(r)} F{_f(feed)}"]
    lines += [f"X{_f(x)} Y{_f(y)}" for x, y in pts[1:]]
    return lines + ["G80"]


def mill(x0, y0, x1, y1, z, feed=200.0) -> List[str]:
    return [f"G00 X{_f(x0)} Y{_f(y0)}", f"G01 Z{_f(z)} F{_f(feed)}",
            f"G01 X{_f(x1)} Y{_f(y1)} F{_f(feed)}"]


def arc(x0, y0, x1, y1, cx, cy, cw, z, feed=200.0) -> List[str]:
    g = "G02" if cw else "G03"
    return [f"G00 X{_f(x0)} Y{_f(y0)}", f"G01 Z{_f(z)} F{_f(feed)}",
            f"{g} X{_f(x1)} Y{_f(y1)} I{_f(cx - x0)} J{_f(cy - y0)} F{_f(feed)}"]


def profile(start: Point, path, z, tool_dia=0.0, side: Optional[str] = None,
            feed=200.0, finish_stock=0.0, finish_feed: Optional[float] = None,
            safe_z=5.0) -> List[str]:
    """Profile with cutter comp (side 'left'/'right') and optional finish pass.

    With finish_stock > 0 a roughing pass is made leaving that stock (via
    D-offset radius increase noted in a comment) then a finish pass.
    """
    code = {"left": "G41", "right": "G42", None: None}[side]
    passes = [("ROUGH", finish_stock), ("FINISH", 0.0)] if finish_stock > 0 else [("", 0.0)]
    out: List[str] = []
    for label, stock in passes:
        if label:
            out.append(f"( {label} PASS, STOCK {_f(stock)}, TOOL DIA {_f(tool_dia + 2 * stock)} )")
        out += [f"G00 X{_f(start[0])} Y{_f(start[1])}", f"G01 Z{_f(z)} F{_f(feed)}"]
        if code:
            out.append(f"{code} D01")
        out += _path_gcode(path, (finish_feed or feed) if label == "FINISH" else feed)
        if code:
            out.append("G40")
        out.append(f"G00 Z{_f(safe_z)}")
    return out


def radial_thinning_factor(ae, tool_dia) -> float:
    """Return the radial chip-thinning factor for a stepover below 50%."""
    if tool_dia <= 0:
        raise AGEError("Tool diameter must be positive")
    if ae <= 0:
        raise AGEError("Radial stepover must be positive")
    ratio = ae / tool_dia
    if ratio >= 0.5:
        return 1.0
    return 2 * math.sqrt(ratio * (1 - ratio))


def adaptive_feed_rate(chip_load, flutes, rpm, ae, tool_dia) -> float:
    """Calculate feed rate with radial chip-thinning compensation."""
    if chip_load <= 0 or flutes <= 0 or rpm <= 0:
        raise AGEError("Chip load, flute count, and spindle speed must be positive")
    return chip_load / radial_thinning_factor(ae, tool_dia) * flutes * rpm


def adaptive_rect_pocket(cx, cy, w, h, z, tool_dia, stepover=0.1,
                         feed=200.0, max_doc=None, safe_z=5.0) -> List[str]:
    """Generate layered rounded-loop roughing paths for a rectangular pocket.

    ``stepover`` is a fraction of tool diameter and must be below 50%.
    ``z`` is the final negative depth and ``max_doc`` limits each depth pass.
    """
    if w <= 0 or h <= 0 or tool_dia <= 0:
        raise AGEError("Pocket dimensions and tool diameter must be positive")
    if not 0 < stepover < 0.5:
        raise AGEError("Adaptive stepover must be between 0 and 50%")
    if feed <= 0 or safe_z <= 0:
        raise AGEError("Feed and safe Z must be positive")
    if z >= 0:
        raise AGEError("Pocket depth must be negative")
    if max_doc is None:
        max_doc = abs(z)
    if max_doc <= 0:
        raise AGEError("Maximum depth of cut must be positive")

    radius = tool_dia / 2
    max_x, max_y = w / 2 - radius, h / 2 - radius
    if max_x < 0 or max_y < 0:
        raise AGEError("Tool too large for pocket")

    step = tool_dia * stepover
    depths = []
    depth = -min(max_doc, abs(z))
    while True:
        depths.append(depth)
        if depth <= z + EPS:
            break
        depth = max(depth - max_doc, z)

    out: List[str] = []
    for depth in depths:
        out += [f"G00 Z{_f(safe_z)}", f"G00 X{_f(cx)} Y{_f(cy)}",
                f"G01 Z{_f(depth)} F{_f(feed)}"]
        # Begin at the centre, then expand the rounded loops by no more than
        # the specified radial engagement until the pocket boundary is reached.
        out.append(f"G01 X{_f(cx)} Y{_f(cy)} F{_f(feed)}")
        offset = min(step, max(max_x, max_y))
        while offset <= max(max_x, max_y) + EPS:
            half_x = min(offset, max_x)
            half_y = min(offset, max_y)
            if half_x > EPS and half_y > EPS:
                out += _rounded_rect_loop(
                    cx, cy, half_x, half_y,
                    min(radius, half_x, half_y), feed,
                )
            if half_x >= max_x - EPS and half_y >= max_y - EPS:
                break
            offset += step
        # Ensure both dimensions reach the wall, including unequal pocket sides.
        if max_x > EPS and max_y > EPS:
            out += _rounded_rect_loop(
                cx, cy, max_x, max_y, min(radius, max_x, max_y), feed,
            )
        out.append(f"G00 Z{_f(safe_z)}")
    return out


def _rounded_rect_loop(cx, cy, half_x, half_y, corner, feed):
    x0, x1 = cx - half_x, cx + half_x
    y0, y1 = cy - half_y, cy + half_y
    return [
        f"G01 X{_f(x0 + corner)} Y{_f(y0)} F{_f(feed)}",
        f"G01 X{_f(x1 - corner)} Y{_f(y0)}",
        f"G03 X{_f(x1)} Y{_f(y0 + corner)} I0 J{_f(corner)}",
        f"G01 X{_f(x1)} Y{_f(y1 - corner)}",
        f"G03 X{_f(x1 - corner)} Y{_f(y1)} I{_f(-corner)} J0",
        f"G01 X{_f(x0 + corner)} Y{_f(y1)}",
        f"G03 X{_f(x0)} Y{_f(y1 - corner)} I0 J{_f(-corner)}",
        f"G01 X{_f(x0)} Y{_f(y0 + corner)}",
        f"G03 X{_f(x0 + corner)} Y{_f(y0)} I{_f(corner)} J0",
    ]


def rect_pocket(cx, cy, w, h, z, tool_dia, stepover=0.5, feed=200.0,
                finish_stock=0.1) -> List[str]:
    """Rectangular pocket: concentric roughing rectangles + finish pass."""
    r = tool_dia / 2
    hw, hh = w / 2 - r - finish_stock, h / 2 - r - finish_stock
    if hw < 0 or hh < 0:
        raise AGEError("Tool too large for pocket")
    step = tool_dia * stepover
    out = [f"G00 X{_f(cx)} Y{_f(cy)}", f"G01 Z{_f(z)} F{_f(feed)}"]
    k = 0.0
    insets = []
    while True:
        a, b = max(hw - k, 0), max(hh - k, 0)
        insets.append((a, b))
        if a == 0 or b == 0:
            break
        k += step
    for a, b in reversed(insets):
        out += _rect_loop(cx, cy, a, b, feed)
    fw, fh = w / 2 - r, h / 2 - r
    out += _rect_loop(cx, cy, fw, fh, feed)
    return out


def _rect_loop(cx, cy, a, b, feed):
    return [f"G01 X{_f(cx - a)} Y{_f(cy - b)} F{_f(feed)}", f"G01 X{_f(cx + a)} Y{_f(cy - b)}",
            f"G01 X{_f(cx + a)} Y{_f(cy + b)}", f"G01 X{_f(cx - a)} Y{_f(cy + b)}",
            f"G01 X{_f(cx - a)} Y{_f(cy - b)}"]


def circ_pocket(cx, cy, dia, z, tool_dia, stepover=0.5, feed=200.0,
                finish_stock=0.1) -> List[str]:
    """Circular pocket: spiral-out full circles + finish pass."""
    r = tool_dia / 2
    rmax = dia / 2 - r
    if rmax < 0:
        raise AGEError("Tool too large for pocket")
    out = [f"G00 X{_f(cx)} Y{_f(cy)}", f"G01 Z{_f(z)} F{_f(feed)}"]
    rr, step = 0.0, tool_dia * stepover
    rough_max = max(rmax - finish_stock, 0.0)
    radii = []
    while rr < rough_max - EPS:
        rr = min(rr + step, rough_max)
        radii.append(rr)
    radii.append(rmax)
    for rad in radii:
        out += [f"G01 X{_f(cx + rad)} Y{_f(cy)} F{_f(feed)}",
                f"G03 X{_f(cx + rad)} Y{_f(cy)} I{_f(-rad)} J0"]
    return out


# ------------------------------------------------------------ subroutines
def transform_points(pts: List[Point], rotate=0.0, mirror: Optional[str] = None,
                     scale=1.0, origin: Point = (0.0, 0.0)) -> List[Point]:
    """Scale, mirror ('x' flips X, 'y' flips Y), then rotate about origin."""
    ca, sa = math.cos(math.radians(rotate)), math.sin(math.radians(rotate))
    out = []
    for x, y in pts:
        x, y = (x - origin[0]) * scale, (y - origin[1]) * scale
        if mirror == "x":
            x = -x
        elif mirror == "y":
            y = -y
        out.append((origin[0] + x * ca - y * sa, origin[1] + x * sa + y * ca))
    return out


def repeat_points(pts: List[Point], count: int, dx=0.0, dy=0.0, rotate=0.0,
                  mirror: Optional[str] = None, scale=1.0,
                  origin: Point = (0.0, 0.0)) -> List[Point]:
    """Repeat a pattern ``count`` times; each copy is offset by (dx,dy) and
    progressively rotated/scaled; ``mirror`` flips alternate copies."""
    out: List[Point] = []
    for k in range(count):
        m = mirror if (mirror and k % 2 == 1) else None
        t = transform_points(pts, rotate * k, m, scale ** k, origin)
        out += [(x + dx * k, y + dy * k) for x, y in t]
    return out


def transform_path(path, rotate=0.0, mirror: Optional[str] = None, scale=1.0,
                   origin: Point = (0.0, 0.0)):
    """Transform a toolpath (as from profile_toolpath/conrad)."""
    res = []
    flip = (mirror in ("x", "y"))
    ca, sa = math.cos(math.radians(rotate)), math.sin(math.radians(rotate))

    def vec(i, j):
        i, j = i * scale, j * scale
        if mirror == "x":
            i = -i
        elif mirror == "y":
            j = -j
        return i * ca - j * sa, i * sa + j * ca

    for seg in path:
        x, y = transform_points([(seg[1], seg[2])], rotate, mirror, scale, origin)[0]
        if seg[0] == "line":
            res.append(("line", x, y))
        else:
            i, j = vec(seg[3], seg[4])
            res.append(("arc", x, y, i, j, (not seg[5]) if flip else seg[5]))
    return res


def path_to_gcode(path, feed=None) -> List[str]:
    return _path_gcode(path, feed)


def program(body: List[str], name="O0001", units="G21") -> str:
    return "\n".join([name, f"G90 G17 {units} G40", "M03 S1000", *body, "M05", "M30", ""])
