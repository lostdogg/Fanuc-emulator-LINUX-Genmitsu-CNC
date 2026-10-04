import math
import pytest
from funuc_emulator import conversational as c


def close(a, b, t=1e-6):
    return abs(a - b) < t


def test_line_arc_tangent_solved():
    # line at 0deg from origin, tangent into arc (centre via radius), arc ends at x,y
    els = [c.Element("line", angle=0, x=10),
           c.tangent_to_previous(c.Element("arc", radius=5, x=15, y=5))]
    c.solve_profile((0, 0), els)
    assert close(els[0].y, 0)
    assert close(els[1].cx, 10) and close(els[1].cy, 5)


def test_line_angles_intersect_next_line():
    els = [c.Element("line", angle=45), c.Element("line", angle=-45, x=10, y=0)]
    c.solve_profile((0, 0), els)
    assert close(els[0].x, 5) and close(els[0].y, 5)


def test_under_constrained():
    with pytest.raises(c.AGEError):
        c.solve_profile((0, 0), [c.Element("line")])


def test_guess_resolves_circle_intersection():
    q = c.line_circle((-10, 0), 0, (0, 0), 5, guess=(-5, 0))
    assert close(q[0], -5)
    q = c.line_circle((-10, 0), 0, (0, 0), 5, guess=(5, 0))
    assert close(q[0], 5)


def test_conrad_fillet():
    ta, tb, ctr, cw = c.fillet((10, 0), (0, 0), (0, 10), 2)
    assert close(ta[0], 2) and close(ta[1], 0) and close(tb[1], 2)
    assert close(ctr[0], 2) and close(ctr[1], 2) and cw
    with pytest.raises(c.AGEError):
        c.fillet((10, 0), (0, 0), (0, 10), 20)


def test_bolt_hole_and_gcode():
    pts = c.bolt_hole_points(0, 0, 10, 4)
    assert close(pts[1][0], 0) and close(pts[1][1], 10)
    g = c.bolt_hole(0, 0, 10, 4, -5)
    assert g[0].startswith("G81") and g[-1] == "G80" and len(g) == 5


def test_profile_comp_and_finish():
    path = [("line", 10, 0), ("line", 10, 10)]
    g = c.profile((0, 0), path, -1, 6, "left", finish_stock=0.2)
    assert "G41 D01" in g and g.count("G40") == 2 and any("FINISH" in s for s in g)


def test_pockets():
    assert any(s.startswith("G01") for s in c.rect_pocket(0, 0, 20, 20, -2, 6))
    assert any(s.startswith("G03") for s in c.circ_pocket(0, 0, 20, -2, 6))
    with pytest.raises(c.AGEError):
        c.rect_pocket(0, 0, 4, 4, -1, 6)


def test_transforms():
    p = c.transform_points([(1, 0)], rotate=90)
    assert close(p[0][0], 0) and close(p[0][1], 1)
    assert c.transform_points([(1, 2)], mirror="x")[0] == (-1, 2)
    assert len(c.repeat_points([(0, 0)], 3, dx=5)) == 3
    path = c.transform_path([("arc", 1, 1, 0, 1, True)], mirror="x")
    assert path[0][5] is False
