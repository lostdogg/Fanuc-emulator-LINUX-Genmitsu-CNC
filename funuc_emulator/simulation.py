"""Deterministic CPU voxel stock-removal simulation for end-mill paths."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Set, Tuple

from .machine import ToolPathSegment

Voxel = Tuple[int, int, int]


class SimulationError(ValueError):
    """Raised for invalid stock or tool simulation parameters."""


@dataclass(frozen=True)
class SimulationResult:
    removed_voxels: int
    rapid_collisions: int
    flute_overflows: int
    remaining_voxels: int


class VoxelStock:
    """Sparse voxel stock with sampled cylindrical end-mill subtraction.

    This CPU reference simulator is intended for visualization and basic
    collision warnings, not machine verification or production control.
    """

    def __init__(self, bounds, voxel_size: float, max_voxels: int = 2_000_000):
        if len(bounds) != 6:
            raise SimulationError("Bounds must be (xmin, xmax, ymin, ymax, zmin, zmax)")
        self.bounds = tuple(float(v) for v in bounds)
        if not all(math.isfinite(v) for v in self.bounds):
            raise SimulationError("Stock bounds must be finite")
        self.xmin, self.xmax, self.ymin, self.ymax, self.zmin, self.zmax = self.bounds
        if self.xmin >= self.xmax or self.ymin >= self.ymax or self.zmin >= self.zmax:
            raise SimulationError("Stock bounds must have positive dimensions")
        if not math.isfinite(voxel_size) or voxel_size <= 0:
            raise SimulationError("Voxel size must be positive and finite")
        self.voxel_size = float(voxel_size)
        self.nx = math.ceil((self.xmax - self.xmin) / self.voxel_size)
        self.ny = math.ceil((self.ymax - self.ymin) / self.voxel_size)
        self.nz = math.ceil((self.zmax - self.zmin) / self.voxel_size)
        voxel_count = self.nx * self.ny * self.nz
        if voxel_count > max_voxels:
            raise SimulationError(
                f"Stock resolution requires {voxel_count} voxels; limit is {max_voxels}"
            )
        self._stock: Set[Voxel] = {
            (i, j, k)
            for i in range(self.nx)
            for j in range(self.ny)
            for k in range(self.nz)
        }

    @property
    def remaining_voxels(self) -> int:
        return len(self._stock)

    def simulate(self, segments: Iterable[ToolPathSegment], tool_dia: float,
                 flute_length: float) -> SimulationResult:
        """Apply tool motion and report removal, rapid contact, and flute overflow."""
        if not math.isfinite(tool_dia) or tool_dia <= 0:
            raise SimulationError("Tool diameter must be positive and finite")
        if not math.isfinite(flute_length) or flute_length <= 0:
            raise SimulationError("Flute length must be positive and finite")
        removed = rapid_collisions = flute_overflows = 0
        radius = tool_dia / 2
        step = self.voxel_size / 2
        radius_cells = math.ceil(radius / self.voxel_size)
        for segment in segments:
            points = self._segment_points(segment, step)
            for x, y, z in points:
                if not (self.xmin <= x <= self.xmax and
                        self.ymin <= y <= self.ymax):
                    continue
                i0 = max(0, math.floor((x - radius - self.xmin) / self.voxel_size))
                i1 = min(self.nx - 1, math.floor((x + radius - self.xmin) / self.voxel_size))
                j0 = max(0, math.floor((y - radius - self.ymin) / self.voxel_size))
                j1 = min(self.ny - 1, math.floor((y + radius - self.ymin) / self.voxel_size))
                if i0 > i1 or j0 > j1:
                    continue
                for i in range(i0, i1 + 1):
                    vx = self.xmin + (i + 0.5) * self.voxel_size
                    for j in range(j0, j1 + 1):
                        vy = self.ymin + (j + 0.5) * self.voxel_size
                        if (vx - x) ** 2 + (vy - y) ** 2 > radius ** 2:
                            continue
                        for k in range(self.nz):
                            voxel = (i, j, k)
                            if voxel not in self._stock:
                                continue
                            vz = self.zmin + (k + 0.5) * self.voxel_size
                            if segment.motion == "rapid":
                                if z <= vz <= z + flute_length:
                                    rapid_collisions += 1
                                continue
                            if z <= vz <= z + flute_length:
                                self._stock.remove(voxel)
                                removed += 1
                            elif vz > z + flute_length:
                                flute_overflows += 1
        return SimulationResult(
            removed, rapid_collisions, flute_overflows, len(self._stock),
        )

    def _segment_points(self, segment: ToolPathSegment, max_step: float):
        start, end = segment.start, segment.end
        if segment.motion in ("arc_cw", "arc_ccw") and segment.arc_points:
            xy = list(segment.arc_points)
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
