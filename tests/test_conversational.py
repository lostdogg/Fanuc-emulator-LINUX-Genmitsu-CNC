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


def test_radial_chip_thinning_feed_compensation():
    assert close(c.radial_thinning_factor(0.05, 0.5), 0.6)
    assert close(c.adaptive_feed_rate(0.003, 4, 6000, 0.05, 0.5), 120)
    assert close(c.adaptive_feed_rate(0.003, 4, 6000, 0.1, 0.5), 90)
    assert c.radial_thinning_factor(0.25, 0.5) == 1
    assert c.radial_thinning_factor(0.3, 0.5) == 1
    for args in ((0, 0.5), (0.1, 0), (-0.1, 0.5)):
        with pytest.raises(c.AGEError):
            c.radial_thinning_factor(*args)
    with pytest.raises(c.AGEError):
        c.adaptive_feed_rate(0.003, 0, 6000, 0.05, 0.5)


def test_adaptive_rect_pocket_layers_and_validation():
    g = c.adaptive_rect_pocket(0, 0, 20, 16, -5, 4, stepover=0.1,
                               feed=120, max_doc=2)
    assert sum(line.startswith("G03 X0 Y-0.4 Z-") for line in g) == 3
    assert sum(line.startswith("G03") for line in g) > 3
    assert g[-1] == "G00 Z5"
    with pytest.raises(c.AGEError):
        c.adaptive_rect_pocket(0, 0, 20, 16, -5, 4, stepover=0.5)
    with pytest.raises(c.AGEError):
        c.adaptive_rect_pocket(0, 0, 3, 16, -5, 4)
    with pytest.raises(c.AGEError):
        c.adaptive_rect_pocket(0, 0, 4, 16, -5, 4)


def test_transforms():
    p = c.transform_points([(1, 0)], rotate=90)
    assert close(p[0][0], 0) and close(p[0][1], 1)
    assert c.transform_points([(1, 2)], mirror="x")[0] == (-1, 2)
    assert len(c.repeat_points([(0, 0)], 3, dx=5)) == 3
    path = c.transform_path([("arc", 1, 1, 0, 1, True)], mirror="x")
    assert path[0][5] is False


def test_tangent_line_between_two_arcs():
    # two CCW arcs of same radius: external tangent is parallel to centre line
    p1, p2 = c.tangent_line_two_arcs((0, 0), 5, False, (20, 0), 5, False)
    assert close(p1[1], -5) and close(p2[1], -5)
    els = [c.Element("arc", cx=0, cy=0, radius=5, cw=False),
           c.Element("line"),
           c.Element("arc", cx=20, cy=0, radius=5, cw=False, x=20, y=5)]
    c.solve_profile((5, 0), els, strict=False)
    assert close(els[1].x, 20) and close(els[1].y, -5)


def test_arc_tangent_two_lines():
    els = [c.Element("line", angle=0), c.Element("arc", radius=2),
           c.Element("line", angle=90, x=10, y=10)]
    c.solve_profile((0, 0), els)
    assert close(els[0].x, 8) and close(els[1].x, 10) and close(els[1].y, 2)
    assert close(els[1].cx, 8) and close(els[1].cy, 2)


def test_chamfer_and_report():
    assert c.chamfer((10, 0), (0, 0), (0, 10), 2) == ((2.0, 0.0), (0.0, 2.0))
    els = c.parse_elements("line angle=45\nline angle=-45 x=10 y=0\nline")
    rows = c.field_report((0, 0), els)
    assert not c.fully_constrained(rows)
    assert any(r["status"] == "Calculated" and r["colour"] == "green" for r in rows)
    assert any(r["status"] == "Not Calculated" for r in rows)
    els = c.parse_elements("line angle=0 guess=5,0")
    rows = c.field_report((0, 0), els)
    assert any(r["status"] == "Guess" for r in rows)
    with pytest.raises(c.AGEError):
        c.parse_elements("blob")
