"""@brief 海图 XML、WKT 和坐标转换测试。

@author susheng
@date 2026-08-28
"""

import math
from pathlib import Path

import pytest

from river_chart.parser import ChartParseError, load_chart, parse_wkt


# [功能与联系] 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。
def test_parse_polygon_and_hole() -> None:
    geometry = parse_wkt("POLYGON ((0 0, 5 0, 5 5, 0 0), (1 1, 2 1, 1 1))")
    assert geometry.geometry_type == "POLYGON"
    assert len(geometry.parts) == 2
    assert geometry.parts[0][1] == (5.0, 0.0)


# [功能与联系] 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。
def test_parse_multi_geometries() -> None:
    geometry = parse_wkt("MULTIPOLYGON (((0 0, 1 0, 0 0)), ((2 2, 3 2, 2 2)))")
    assert geometry.geometry_type == "MULTIPOLYGON"
    assert len(geometry.parts) == 2


# [功能与联系] 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。
def test_parse_bad_wkt_is_actionable() -> None:
    with pytest.raises(ChartParseError, match="invalid|coordinate|expected|unterminated"):
        parse_wkt("POLYGON (not-a-coordinate)")


# [功能与联系] 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。
def test_load_chart_sanitizes_empty_tags_and_converts_projected_coordinates(tmp_path: Path) -> None:
    source = tmp_path / "chart.xml"
    source.write_text(
        """<?xml version=\"1.0\"?>
<S57ChartData totalMaps=\"1\">
  <Map index=\"0\">
    <Metadata><DSID_DSNM>TEST.000</DSID_DSNM></Metadata>
    <Bounds><MinX>100</MinX><MinY>20</MinY><MaxX>101</MaxX><MaxY>21</MaxY></Bounds>
    <Features><FeatureGroup acronym=\"DEPARE\"><Feature>
      <RCID>1</RCID><ID>2</ID><Name>DEPARE</Name>
      <Attributes><></></Attributes>
      <Geometry><Geometries><Polygon>POLYGON ((0 0, 10 0, 0 10, 0 0))</Polygon></Geometries></Geometry>
    </Feature></FeatureGroup></Features>
  </Map>
</S57ChartData>""",
        encoding="utf-8",
    )
    chart = load_chart(source, origin_longitude=100.0, origin_latitude=20.0)
    assert chart.map_count == 1
    assert chart.feature_count == 1
    assert chart.source_is_projected is False
    first_point = chart.features[0].geometry[0].parts[0][0]
    assert first_point[0] == pytest.approx(-10_460_610, abs=100)
    assert first_point[1] == pytest.approx(-2_226_390, abs=100)


# [功能与联系] 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。
def test_load_chart_uses_map_bounds_as_default_origin(tmp_path: Path) -> None:
    source = tmp_path / "chart.xml"
    source.write_text(
        """<S57ChartData><Map><Bounds><MinX>120</MinX><MinY>30</MinY><MaxX>122</MaxX><MaxY>32</MaxY></Bounds>
<Features><FeatureGroup acronym=\"TEST\"><Feature><Geometry><Geometries>
<Polygon>POLYGON ((121 31, 121.1 31, 121 31.1, 121 31))</Polygon>
</Geometries></Geometry></Feature></FeatureGroup></Features></Map></S57ChartData>""",
        encoding="utf-8",
    )
    chart = load_chart(source)
    assert chart.origin_longitude == 121.0
    assert chart.origin_latitude == 31.0
    assert chart.features[0].geometry[0].parts[0][0] == pytest.approx((0.0, 0.0))


# [功能与联系] 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。
def test_load_chart_retains_s57_feature_type(tmp_path: Path) -> None:
    source = tmp_path / "chart.xml"
    source.write_text(
        """<S57ChartData><Map><Bounds><MinX>120</MinX><MinY>30</MinY><MaxX>122</MaxX><MaxY>32</MaxY></Bounds>
<Features><FeatureGroup acronym="LIGHTS"><Feature><Type>P</Type><Geometry><Geometries>
<Polygon>POLYGON ((121 31))</Polygon>
</Geometries></Geometry></Feature></FeatureGroup></Features></Map></S57ChartData>""",
        encoding="utf-8",
    )
    chart = load_chart(source)
    assert chart.features[0].feature_type == "P"
    assert chart.features[0].geometry[0].parts[0] == ((0.0, 0.0),)


# [功能与联系] 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。
def test_load_chart_converts_inverted_web_mercator_to_local_meters(tmp_path: Path) -> None:
    source = tmp_path / "chart.xml"
    # 100 E / 30 N and 101 E / 30 N, encoded as Web Mercator with -Y.
    source.write_text(
        """<S57ChartData><Map><Bounds><MinX>100</MinX><MinY>30</MinY><MaxX>101</MaxX><MaxY>30</MaxY></Bounds>
<Features><FeatureGroup acronym="DEPCNT"><Feature><Type>L</Type><Geometry><Geometries>
<Polygon>POLYGON ((11131949.079327 -3503549.843504, 11243268.570120 -3503549.843504))</Polygon>
</Geometries></Geometry></Feature></FeatureGroup></Features></Map></S57ChartData>""",
        encoding="utf-8",
    )
    chart = load_chart(source, origin_longitude=100.0, origin_latitude=30.0)
    points = chart.features[0].geometry[0].parts[0]
    assert chart.source_is_projected is True
    assert points[0] == pytest.approx((0.0, 0.0), abs=0.01)
    assert points[1] == pytest.approx(
        (111_319.49079327358 * math.cos(math.radians(30.0)), 0.0), abs=0.01
    )


# [功能与联系] 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。
def test_rejects_invalid_scale(tmp_path: Path) -> None:
    source = tmp_path / "chart.xml"
    source.write_text(
        """<S57ChartData><Map><Bounds><MinX>1</MinX><MinY>1</MinY><MaxX>2</MaxX><MaxY>2</MaxY></Bounds>
<Features/></Map></S57ChartData>""",
        encoding="utf-8",
    )
    with pytest.raises(ChartParseError, match="scale"):
        load_chart(source, scale=0.0)
