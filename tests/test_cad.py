import pytest

from funuc_emulator import cad


def dxf(*records, units=4):
    pairs = ["0", "SECTION", "2", "HEADER", "9", "$INSUNITS",
             "70", str(units), "0", "ENDSEC", "0", "SECTION",
             "2", "ENTITIES"]
    for kind, data in records:
        pairs.extend(("0", kind))
        for code, value in data:
            pairs.extend((str(code), str(value)))
    pairs.extend(("0", "ENDSEC", "0", "EOF"))
    return "\n".join(pairs)


def test_parse_ascii_dxf_entities_and_unit_conversion():
    source = dxf(
        ("CIRCLE", [(10, 1), (20, 2), (40, 0.5)]),
        ("ARC", [(10, 0), (20, 0), (40, 2), (50, 0), (51, 90)]),
        ("LWPOLYLINE", [(70, 1), (10, 0), (20, 0), (10, 1), (20, 0),
                        (10, 1), (20, 1), (10, 0), (20, 1)]),
        units=1,
    )
    doc = cad.parse_dxf(source)
    assert doc.units == "inches"
    assert doc.entities[0] == cad.Circle((25.4, 50.8), 12.7)
    assert isinstance(doc.entities[1], cad.Arc)
    assert doc.entities[2].closed


def test_lwpolyline_bulges_are_tessellated_for_profile_generation():
    source = dxf(
        ("LWPOLYLINE", [(70, 1), (10, 0), (20, 0), (42, 0.41421356237),
                        (10, 1), (20, 0), (10, 1), (20, 1), (10, 0), (20, 1)]),
    )
    polyline = cad.parse_dxf(source).entities[0]
    assert isinstance(polyline, cad.Polyline)
    assert polyline.closed
    assert len(polyline.points) > 4
    assert any(y < 0 for _, y in polyline.points)


def test_arc_entities_chain_with_lines_into_profile():
    source = dxf(
        ("ARC", [(10, 0), (20, 0), (40, 1), (50, 0), (51, 180)]),
        ("LINE", [(10, -1), (20, 0), (11, -1), (21, -1)]),
        ("LINE", [(10, -1), (20, -1), (11, 1), (21, -1)]),
        ("LINE", [(10, 1), (20, -1), (11, 1), (21, 0)]),
    )
    features = cad.extract_features(cad.parse_dxf(source))
    assert len(features.profiles) == 1
    assert len(features.profiles[0].points) > 4


def test_full_circle_arc_is_recognized_as_hole():
    source = dxf(("ARC", [(10, 2), (20, 3), (40, 1), (50, 0), (51, 360)]))
    features = cad.extract_features(cad.parse_dxf(source))
    assert features.holes == (cad.HoleFeature((2, 3), 2),)
    assert not features.profiles


def test_extract_line_chains_and_generate_conversational_gcode():
    source = dxf(
        ("LINE", [(10, 0), (20, 0), (11, 10), (21, 0)]),
        ("LINE", [(10, 10), (20, 10), (11, 0), (21, 10)]),
        ("LINE", [(10, 10), (20, 0), (11, 10), (21, 10)]),
        ("LINE", [(10, 0), (20, 10), (11, 0), (21, 0)]),
        ("CIRCLE", [(10, 5), (20, 5), (40, 1)]),
    )
    features = cad.extract_features(cad.parse_dxf(source))
    assert len(features.holes) == 1
    assert len(features.profiles) == 1
    code = cad.features_to_gcode(features, -2, 100)
    assert any(line.startswith("G81 X5 Y5 Z-2") for line in code)
    assert any("DIAMETER 2 mm" in line for line in code)
    assert any(line.startswith("G01 X") for line in code)
    assert code[0] == "G21 G90 G94"


def test_dxf_rejects_invalid_or_unsupported_geometry():
    with pytest.raises(cad.CadError):
        cad.parse_dxf("0\nSECTION\n2\nENTITIES\n0\nEOF\n")
    with pytest.raises(cad.CadError):
        cad.parse_dxf(dxf(("CIRCLE", [(10, 0), (20, 0), (40, -1)])))
    with pytest.raises(cad.CadError):
        cad.parse_dxf(dxf(("LINE", [(10, 0), (20, 0), (30, 1),
                                    (11, 1), (21, 0)])))
    with pytest.raises(cad.CadError):
        cad.features_to_gcode(cad.CadFeatures((), ()), 0, 100)


def test_extract_chains_respects_tolerance():
    source = dxf(
        ("LINE", [(10, 0), (20, 0), (11, 1), (21, 0)]),
        ("LINE", [(10, 1.0005), (20, 0), (11, 1), (21, 1)]),
        ("LINE", [(10, 1), (20, 1), (11, 0), (21, 1)]),
        ("LINE", [(10, 0), (20, 1), (11, 0), (21, 0)]),
    )
    assert len(cad.extract_features(cad.parse_dxf(source), tolerance=0.001).profiles) == 1
    assert not cad.extract_features(cad.parse_dxf(source), tolerance=0.0001).profiles


def test_appending_generated_gcode_preserves_program_end_order():
    existing = "%\nO1\nG21 G90 G94\nG00 X0\nM30\n%"
    generated = "G21 G90 G94\nG81 X1 Y2 Z-3 R5 F100\nG80\n"
    combined = cad.append_gcode_program(existing, generated)
    assert combined.index("G81") < combined.index("M30")
    assert combined.count("G21 G90 G94") == 1
    assert combined.rstrip().endswith("%")
