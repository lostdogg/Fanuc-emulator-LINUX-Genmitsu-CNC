"""Dependency-free reader for common planar ASCII DXF entities."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Tuple, Union

from .parser import parse_program

Point = Tuple[float, float]
GCODE_HEADER = "G21 G90 G94"


class CadError(ValueError):
    """Raised when a CAD file is malformed or cannot be interpreted."""


@dataclass(frozen=True)
class Line:
    start: Point
    end: Point


@dataclass(frozen=True)
class Arc:
    center: Point
    radius: float
    start_angle: float
    end_angle: float


@dataclass(frozen=True)
class Circle:
    center: Point
    radius: float


@dataclass(frozen=True)
class Polyline:
    points: Tuple[Point, ...]
    closed: bool = False


Entity = Union[Line, Arc, Circle, Polyline]


@dataclass(frozen=True)
class DxfDocument:
    entities: Tuple[Entity, ...]
    units: str
    unit_scale: float


@dataclass(frozen=True)
class HoleFeature:
    center: Point
    diameter: float


@dataclass(frozen=True)
class ProfileFeature:
    points: Tuple[Point, ...]


@dataclass(frozen=True)
class CadFeatures:
    holes: Tuple[HoleFeature, ...]
    profiles: Tuple[ProfileFeature, ...]


_UNITS = {
    0: ("unitless", 1.0),
    1: ("inches", 25.4),
    2: ("feet", 304.8),
    3: ("miles", 1609344.0),
    4: ("millimeters", 1.0),
    5: ("centimeters", 10.0),
    6: ("meters", 1000.0),
}


def parse_dxf(source: str) -> DxfDocument:
    """Read supported entities from the ENTITIES section of ASCII DXF.

    Coordinates are converted to millimeters when the file declares units.
    Supported entity records are LINE, ARC, CIRCLE, and LWPOLYLINE.
    """
    lines = source.lstrip("\ufeff").splitlines()
    while lines and not lines[-1].strip():
        lines.pop()
    if len(lines) % 2:
        raise CadError("DXF must contain complete group-code/value pairs")
    pairs = []
    for i in range(0, len(lines), 2):
        try:
            code = int(lines[i].strip())
        except ValueError as exc:
            raise CadError(f"Invalid DXF group code on line {i + 1}") from exc
        pairs.append((code, lines[i + 1].strip()))

    unit_code = 0
    in_header = False
    for i, (code, value) in enumerate(pairs):
        if code == 0 and value == "SECTION":
            in_header = i + 1 < len(pairs) and pairs[i + 1] == (2, "HEADER")
        elif code == 0 and value == "ENDSEC":
            in_header = False
        elif in_header and code == 9 and value == "$INSUNITS":
            if i + 1 < len(pairs) and pairs[i + 1][0] == 70:
                try:
                    unit_code = int(pairs[i + 1][1])
                except ValueError as exc:
                    raise CadError("Invalid $INSUNITS value") from exc
    units, scale = _UNITS.get(unit_code, ("unitless", 1.0))

    entities: List[Entity] = []
    section = ""
    records: List[Tuple[int, str]] = []

    def consume(record: List[Tuple[int, str]]) -> None:
        if not record:
            return
        kind = record[0][1]
        if kind not in {"LINE", "ARC", "CIRCLE", "LWPOLYLINE"}:
            return
        data = record[1:]

        def one(code, default=None):
            values = [v for c, v in data if c == code]
            return values[0] if values else default

        def require_xy_plane():
            for code, default, expected in (
                (30, "0", 0.0), (31, "0", 0.0), (38, "0", 0.0),
                (210, "0", 0.0), (220, "0", 0.0), (230, "1", 1.0),
            ):
                value = one(code, default)
                try:
                    value = float(value)
                except ValueError as exc:
                    raise CadError(f"{kind} has invalid plane group {code}") from exc
                if not math.isfinite(value) or abs(value - expected) > 1e-9:
                    raise CadError(f"{kind} is not in the supported XY plane")

        def number(code, default=None):
            value = one(code)
            if value is None:
                if default is not None:
                    return default
                raise CadError(f"{kind} is missing required group {code}")
            try:
                value = float(value)
            except ValueError as exc:
                raise CadError(f"{kind} has invalid numeric group {code}") from exc
            if not math.isfinite(value):
                raise CadError(f"{kind} contains a non-finite coordinate")
            return value

        require_xy_plane()
        if kind == "LINE":
            entities.append(Line((number(10) * scale, number(20) * scale),
                                 (number(11) * scale, number(21) * scale)))
        elif kind == "CIRCLE":
            radius = number(40) * scale
            if radius <= 0:
                raise CadError("CIRCLE radius must be positive")
            entities.append(Circle((number(10) * scale, number(20) * scale),
                                   radius))
        elif kind == "ARC":
            radius = number(40) * scale
            if radius <= 0:
                raise CadError("ARC radius must be positive")
            entities.append(Arc(
                (number(10) * scale, number(20) * scale), radius,
                number(50), number(51),
            ))
        else:
            vertices: List[Tuple[Point, float]] = []
            vertex_x = vertex_y = None
            bulge = 0.0
            for code, value in data:
                if code == 10:
                    if vertex_x is not None:
                        if vertex_y is None:
                            raise CadError("LWPOLYLINE vertex is missing its Y coordinate")
                        vertices.append(((vertex_x * scale, vertex_y * scale), bulge))
                    vertex_y = None
                    bulge = 0.0
                    try:
                        vertex_x = float(value)
                    except ValueError as exc:
                        raise CadError("Invalid LWPOLYLINE X coordinate") from exc
                    if not math.isfinite(vertex_x):
                        raise CadError("LWPOLYLINE coordinates must be finite")
                elif code == 20:
                    if vertex_x is None:
                        raise CadError("LWPOLYLINE Y coordinate has no X coordinate")
                    try:
                        vertex_y = float(value)
                    except ValueError as exc:
                        raise CadError("Invalid LWPOLYLINE Y coordinate") from exc
                    if not math.isfinite(vertex_y):
                        raise CadError("LWPOLYLINE coordinates must be finite")
                elif code == 42:
                    if vertex_x is None or vertex_y is None:
                        raise CadError("LWPOLYLINE vertex is missing its Y coordinate")
                    try:
                        bulge = float(value)
                    except ValueError as exc:
                        raise CadError("Invalid LWPOLYLINE bulge") from exc
                    if not math.isfinite(bulge):
                        raise CadError("LWPOLYLINE bulge must be finite")
            if vertex_x is not None:
                if vertex_y is None:
                    raise CadError("LWPOLYLINE vertex is missing its Y coordinate")
                vertices.append(((vertex_x * scale, vertex_y * scale), bulge))
            if len(vertices) < 2:
                raise CadError("LWPOLYLINE must contain at least two vertices")
            try:
                flags = int(one(70, "0"))
            except ValueError as exc:
                raise CadError("Invalid LWPOLYLINE flags") from exc
            closed = bool(flags & 1)
            raw_points: List[Point] = []
            edge_count = len(vertices) if closed else len(vertices) - 1
            for index, (start, bulge) in enumerate(vertices):
                if not raw_points:
                    raw_points.append(start)
                if index >= edge_count:
                    continue
                end = vertices[(index + 1) % len(vertices)][0]
                arc_points = _bulge_segment(start, end, bulge)
                raw_points.extend(
                    arc_points[:-1] if closed and index == edge_count - 1
                    else arc_points
                )
            entities.append(Polyline(tuple(raw_points), closed))

    for code, value in pairs:
        if code == 0 and value == "SECTION":
            section = ""
            records = []
        elif code == 2 and not section:
            section = value
        elif code == 0 and value == "ENDSEC":
            if section == "ENTITIES":
                consume(records)
            section = ""
            records = []
        elif section == "ENTITIES":
            if code == 0:
                consume(records)
                records = [(code, value)]
            else:
                records.append((code, value))
    if section == "ENTITIES":
        consume(records)
    if not entities:
        raise CadError("No supported CAD entities found in DXF")
    return DxfDocument(tuple(entities), units, scale)


def _bulge_segment(start: Point, end: Point, bulge: float) -> List[Point]:
    """Tessellate a DXF bulge arc to a maximum angular step of 15 degrees."""
    theta = 4 * math.atan(bulge)
    if abs(theta) < 1e-12:
        return [end]
    chord = math.dist(start, end)
    if chord < 1e-12:
        raise CadError("LWPOLYLINE bulge has coincident endpoints")
    radius = chord / (2 * math.sin(abs(theta) / 2))
    offset = chord / (2 * math.tan(theta / 2))
    dx, dy = end[0] - start[0], end[1] - start[1]
    center = ((start[0] + end[0]) / 2 - dy / chord * offset,
              (start[1] + end[1]) / 2 + dx / chord * offset)
    start_angle = math.atan2(start[1] - center[1], start[0] - center[0])
    count = max(1, math.ceil(abs(theta) / math.radians(15)))
    return [
        (center[0] + radius * math.cos(start_angle + theta * i / count),
         center[1] + radius * math.sin(start_angle + theta * i / count))
        for i in range(1, count + 1)
    ]


def _arc_points(arc: Arc) -> List[Point]:
    sweep = (arc.end_angle - arc.start_angle) % 360
    if sweep <= 1e-9:
        sweep = 360
    start = math.radians(arc.start_angle % 360)
    sweep = math.radians(sweep)
    count = max(1, math.ceil(sweep / math.radians(15)))
    return [
        (arc.center[0] + arc.radius * math.cos(start + sweep * i / count),
         arc.center[1] + arc.radius * math.sin(start + sweep * i / count))
        for i in range(count + 1)
    ]


def _is_full_arc(arc: Arc) -> bool:
    return (arc.end_angle - arc.start_angle) % 360 <= 1e-9


def extract_features(document: DxfDocument, tolerance: float = 0.001) -> CadFeatures:
    """Recognize circles, full-circle arcs, and closed polyline/line/arc chains.

    Line chains with ambiguous branch junctions are skipped rather than
    arbitrarily choosing a contour.
    """
    if not math.isfinite(tolerance) or tolerance <= 0:
        raise CadError("Chaining tolerance must be positive and finite")
    holes = [HoleFeature(entity.center, entity.radius * 2)
             for entity in document.entities
             if isinstance(entity, Circle) or
             (isinstance(entity, Arc) and _is_full_arc(entity))]
    profiles: List[ProfileFeature] = [
        ProfileFeature(entity.points)
        for entity in document.entities
        if isinstance(entity, Polyline) and entity.closed
    ]
    line_edges = [(entity.start, entity.end)
                  for entity in document.entities if isinstance(entity, Line)]
    for entity in document.entities:
        if isinstance(entity, Arc) and not _is_full_arc(entity):
            points = _arc_points(entity)
            line_edges.extend(zip(points, points[1:]))
    profiles.extend(_closed_line_chains(line_edges, tolerance))
    return CadFeatures(tuple(holes), tuple(profiles))


def _closed_line_chains(edges: List[Tuple[Point, Point]],
                        tolerance: float) -> List[ProfileFeature]:
    remaining = set(range(len(edges)))
    profiles = []
    while remaining:
        first = min(remaining)
        start, end = edges[first]
        chain = [start, end]
        used = {first}
        remaining.remove(first)
        while math.dist(chain[-1], chain[0]) > tolerance:
            matches = []
            for index in remaining:
                a, b = edges[index]
                if math.dist(chain[-1], a) <= tolerance:
                    matches.append((index, b))
                elif math.dist(chain[-1], b) <= tolerance:
                    matches.append((index, a))
            if len(matches) != 1:
                break
            index, next_point = matches[0]
            used.add(index)
            remaining.remove(index)
            chain.append(next_point)
        if math.dist(chain[-1], chain[0]) <= tolerance and len(chain) >= 4:
            chain[-1] = chain[0]
            profiles.append(ProfileFeature(tuple(chain[:-1])))
        else:
            # Do not accidentally combine an open/branching chain with a
            # different profile on a later pass.
            remaining.difference_update(used)
    return profiles


def features_to_gcode(features: CadFeatures, depth: float, feed: float,
                      safe_z: float = 5.0) -> List[str]:
    """Convert extracted holes and closed boundaries to basic milling G-code."""
    if (not all(math.isfinite(v) for v in (depth, feed, safe_z)) or
            depth >= 0 or feed <= 0 or safe_z <= 0):
        raise CadError("Depth must be negative; feed and safe Z must be positive")
    from . import conversational

    out = [GCODE_HEADER]
    for hole in features.holes:
        out.append(
            f"(DRILL CENTER X{hole.center[0]:g} Y{hole.center[1]:g} "
            f"DIAMETER {hole.diameter:g} mm)"
        )
        out.extend(conversational.drill(
            hole.center[0], hole.center[1], depth, safe_z, feed,
        ))
    for profile in features.profiles:
        points = profile.points
        if len(points) < 3:
            continue
        path = [("line", x, y) for x, y in points[1:]]
        path.append(("line", points[0][0], points[0][1]))
        out.extend(conversational.profile(
            points[0], path, depth, feed=feed, safe_z=safe_z,
        ))
    return out


def append_gcode_program(existing: str, generated: str) -> str:
    """Insert generated blocks before an existing program-end marker."""
    lines = existing.splitlines()
    generated_lines = generated.splitlines()
    if generated_lines and generated_lines[0].strip().upper() != GCODE_HEADER:
        generated_lines.insert(0, GCODE_HEADER)

    blocks, _ = parse_program(existing)
    end_blocks = [block.line_number for block in blocks
                  if block.get("M") in (2, 30)]
    if end_blocks:
        insert_at = min(end_blocks) - 1
    else:
        last_nonempty = next(
            (index for index in range(len(lines) - 1, -1, -1)
             if lines[index].strip()),
            None,
        )
        insert_at = (
            last_nonempty
            if last_nonempty is not None and last_nonempty > 0
            and lines[last_nonempty].strip() == "%"
            else len(lines)
        )
    lines[insert_at:insert_at] = generated_lines
    return "\n".join(lines) + ("\n" if lines else "")
