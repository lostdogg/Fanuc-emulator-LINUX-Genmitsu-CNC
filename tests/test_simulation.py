import pytest

from funuc_emulator.machine import ToolPathSegment
from funuc_emulator.simulation import SimulationError, VoxelStock


def segment(motion, start=(0, 0, 0), end=(0, 0, 0)):
    return ToolPathSegment(motion, start, end)


def test_feed_motion_removes_voxels_and_tracks_remaining_stock():
    stock = VoxelStock((-2, 2, -2, 2, 0, 2), 1)
    result = stock.simulate([segment("feed")], tool_dia=2, flute_length=2)
    assert result.removed_voxels == 8
    assert result.remaining_voxels == 24
    assert stock.remaining_voxels == 24


def test_rapid_contact_is_reported_without_removal():
    stock = VoxelStock((-2, 2, -2, 2, 0, 2), 1)
    result = stock.simulate([segment("rapid")], tool_dia=2, flute_length=2)
    assert result.rapid_collisions > 0
    assert result.removed_voxels == 0
    assert result.remaining_voxels == 32


def test_flute_overflow_is_reported():
    stock = VoxelStock((-2, 2, -2, 2, 0, 2), 1)
    result = stock.simulate([segment("feed")], tool_dia=2, flute_length=0.5)
    assert result.flute_overflows > 0
    assert result.removed_voxels == 4


def test_tool_engages_stock_when_centerline_is_outside_stock():
    stock = VoxelStock((-2, 2, -2, 2, 0, 2), 1)
    result = stock.simulate(
        [segment("feed", (2.2, 0, 0), (2.2, 0, 0))],
        tool_dia=2, flute_length=2,
    )
    assert result.removed_voxels > 0


def test_tool_above_stock_does_not_report_false_flute_overflow():
    stock = VoxelStock((-2, 2, -2, 2, 0, 2), 1)
    result = stock.simulate(
        [segment("feed", (0, 0, 5), (0, 0, 5))],
        tool_dia=2, flute_length=1,
    )
    assert result.flute_overflows == 0
    assert result.removed_voxels == 0


def test_arc_motion_and_invalid_stock_configuration():
    stock = VoxelStock((-2, 2, -2, 2, 0, 2), 1)
    arc = ToolPathSegment("arc_ccw", (1, 0, 0), (0, 1, 0),
                          [(1, 0), (0.7, 0.7), (0, 1)])
    assert stock.simulate([arc], tool_dia=1, flute_length=1).removed_voxels > 0
    with pytest.raises(SimulationError):
        VoxelStock((0, 100, 0, 100, 0, 100), 0.1)
    with pytest.raises(SimulationError):
        stock.simulate([], tool_dia=0, flute_length=1)
