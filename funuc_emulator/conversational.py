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


def _end_dir(el: Element, start: Point) -> Optional[float]:
    """Travel direction (deg) at the end of a solved element."""
    if el.kind == "line":
        return math.degrees(math.atan2(el.y - start[1], el.x - start[0]))
    a = math.atan2(el.y - el.cy, el.x - el.cx)
    return math.degrees(a + (-math.pi / 2 if el.cw else math.pi / 2))


def solve_profile(start: Point, elements: List[Element]) -> List[Element]:
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
    if bad:
        raise AGEError(f"Not Calculated: element(s) {bad} under-constrained")
    return elements


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
