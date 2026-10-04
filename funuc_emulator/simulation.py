"""Deterministic CPU voxel stock-removal simulation for end-mill paths."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from .machine import ToolPathSegment

class SimulationError(ValueError):
    """Raised for invalid stock or tool simulation parameters."""


@dataclass(frozen=True)
class SimulationResult:
    removed_voxels: int
    rapid_collisions: int
    flute_overflows: int
    remaining_voxels: int


class VoxelStock:
    """Dense byte-grid stock with sampled cylindrical end-mill subtraction.

    This CPU reference simulator is intended for visualization and basic
    collision warnings, not machine verification or production control.
    """

    def __init__(self, bounds, voxel_size: float, max_voxels: int = 2_000_000):
        if len(bounds) != 6:
            raise SimulationError("Bounds must be (xmin, xmax, ymin, ymax, zmin, zmax)")
        try:
            self.bounds = tuple(float(v) for v in bounds)
        except (TypeError, ValueError) as exc:
            raise SimulationError("Stock bounds must be numeric") from exc
        if not all(math.isfinite(v) for v in self.bounds):
            raise SimulationError("Stock bounds must be finite")
        self.xmin, self.xmax, self.ymin, self.ymax, self.zmin, self.zmax = self.bounds
        if self.xmin >= self.xmax or self.ymin >= self.ymax or self.zmin >= self.zmax:
            raise SimulationError("Stock bounds must have positive dimensions")
        if not math.isfinite(voxel_size) or voxel_size <= 0:
            raise SimulationError("Voxel size must be positive and finite")
        if not isinstance(max_voxels, int) or max_voxels <= 0:
            raise SimulationError("Voxel limit must be a positive integer")
        self.voxel_size = float(voxel_size)
        self.nx = math.ceil((self.xmax - self.xmin) / self.voxel_size)
        self.ny = math.ceil((self.ymax - self.ymin) / self.voxel_size)
        self.nz = math.ceil((self.zmax - self.zmin) / self.voxel_size)
        voxel_count = self.nx * self.ny * self.nz
        if voxel_count > max_voxels:
            raise SimulationError(
                f"Stock resolution requires {voxel_count} voxels; limit is {max_voxels}"
            )
        self._stock = bytearray(b"\x01") * voxel_count
        self._remaining = voxel_count
        self._column_tops = [self.nz - 1] * (self.nx * self.ny)

    @property
    def remaining_voxels(self) -> int:
        return self._remaining

    def simulate(self, segments: Iterable[ToolPathSegment], tool_dia: float,
                 flute_length: float) -> SimulationResult:
        """Apply tool motion and report removal, rapid contact, and flute overflow."""
        if not math.isfinite(tool_dia) or tool_dia <= 0:
            raise SimulationError("Tool diameter must be positive and finite")
        if not math.isfinite(flute_length) or flute_length <= 0:
            raise SimulationError("Flute length must be positive and finite")
        removed = 0
        rapid_contacts = set()
        overflow_contacts = set()
        radius = tool_dia / 2
        step = self.voxel_size / 2
        last_sample = None
        for segment in segments:
            if segment.motion not in {"rapid", "feed", "arc_cw", "arc_ccw"}:
                raise SimulationError(f"Unsupported motion type: {segment.motion}")
            if not segment.arc_points:
                segment = self._clip_linear_segment(segment, radius, flute_length)
                if segment is None:
                    continue
            points = self._segment_points(segment, step)
            for x, y, z in points:
                sample = (segment.motion, x, y, z)
                if sample == last_sample:
                    continue
                last_sample = sample
                if (x + radius < self.xmin or x - radius > self.xmax or
                        y + radius < self.ymin or y - radius > self.ymax or
                        z > self.zmax or z + flute_length < self.zmin):
                    continue
                i0 = max(0, math.floor((x - radius - self.xmin) / self.voxel_size))
                i1 = min(self.nx - 1, math.floor((x + radius - self.xmin) / self.voxel_size))
                j0 = max(0, math.floor((y - radius - self.ymin) / self.voxel_size))
                j1 = min(self.ny - 1, math.floor((y + radius - self.ymin) / self.voxel_size))
                if i0 > i1 or j0 > j1:
                    continue
                k0 = max(0, math.ceil((z - self.zmin) / self.voxel_size - 0.5))
                k1 = min(
                    self.nz - 1,
                    math.floor((z + flute_length - self.zmin) / self.voxel_size - 0.5),
                )
                if k0 > k1:
                    continue
                for i in range(i0, i1 + 1):
                    vx = self.xmin + (i + 0.5) * self.voxel_size
                    for j in range(j0, j1 + 1):
                        vy = self.ymin + (j + 0.5) * self.voxel_size
                        if (vx - x) ** 2 + (vy - y) ** 2 > radius ** 2:
                            continue
                        column_index = i * self.ny + j
                        column_top = self._column_tops[column_index]
                        if column_top < k0:
                            continue
                        for k in range(k0, min(k1, column_top) + 1):
                            voxel_index = column_index * self.nz + k
                            if not self._stock[voxel_index]:
                                continue
                            if segment.motion == "rapid":
                                rapid_contacts.add(voxel_index)
                                continue
                            self._stock[voxel_index] = 0
                            self._remaining -= 1
                            removed += 1
                            if self._column_tops[column_index] == k:
                                top = k - 1
                                while (top >= 0 and
                                       not self._stock[column_index * self.nz + top]):
                                    top -= 1
                                self._column_tops[column_index] = top
                        if (segment.motion != "rapid" and
                                k0 <= k1 and
                                self._column_tops[column_index] >= k0 and
                                self._column_tops[column_index] > k1):
                            overflow_contacts.add(column_index)
        return SimulationResult(
            removed, len(rapid_contacts), len(overflow_contacts), self._remaining,
        )

    def _clip_linear_segment(self, segment: ToolPathSegment, radius: float,
                             flute_length: float):
        """Clip a linear move to the stock/tool overlap bounds before sampling."""
        start, end = segment.start, segment.end
        if (len(start) != 3 or len(end) != 3 or
                not all(math.isfinite(value) for value in (*start, *end))):
            raise SimulationError("Toolpath coordinates must be finite XYZ points")
        bounds = (
            (self.xmin - radius, self.xmax + radius),
            (self.ymin - radius, self.ymax + radius),
            (self.zmin - flute_length, self.zmax),
        )
        enter, leave = 0.0, 1.0
        for axis, (low, high) in enumerate(bounds):
            origin = start[axis]
            delta = end[axis] - origin
            if abs(delta) < 1e-15:
                if origin < low or origin > high:
                    return None
                continue
            first, last = (low - origin) / delta, (high - origin) / delta
            if first > last:
                first, last = last, first
            enter, leave = max(enter, first), min(leave, last)
            if enter > leave:
                return None
        clipped_start = tuple(start[i] + (end[i] - start[i]) * enter
                              for i in range(3))
        clipped_end = tuple(start[i] + (end[i] - start[i]) * leave
                            for i in range(3))
        return ToolPathSegment(
            segment.motion, clipped_start, clipped_end, segment.arc_points,
        )

    def _segment_points(self, segment: ToolPathSegment, max_step: float):
        start, end = segment.start, segment.end
        if (len(start) != 3 or len(end) != 3 or
                not all(math.isfinite(value) for value in (*start, *end))):
            raise SimulationError("Toolpath coordinates must be finite XYZ points")
        if segment.motion in ("arc_cw", "arc_ccw") and segment.arc_points:
            xy = list(segment.arc_points)
            if any(len(point) != 2 or not all(math.isfinite(v) for v in point)
                   for point in xy):
                raise SimulationError("Arc path points must be finite XY points")
            if math.dist(xy[0], start[:2]) > 1e-9:
                xy.insert(0, start[:2])
            if math.dist(xy[-1], end[:2]) > 1e-9:
                xy.append(end[:2])
            nodes = []
            for index, (x, y) in enumerate(xy):
                fraction = index / max(1, len(xy) - 1)
                nodes.append((x, y, start[2] + (end[2] - start[2]) * fraction))
        else:
            nodes = [start, end]
        points = []
        for a, b in zip(nodes, nodes[1:]):
            distance = math.dist(a, b)
            count = max(1, math.ceil(distance / max_step))
            points.extend(tuple(a[d] + (b[d] - a[d]) * i / count
                                for d in range(3))
                          for i in range(count))
        points.append(tuple(nodes[-1]))
        return points
