"""@brief 海图节点 Marker 构造测试。

@author susheng
@date 2026-08-28
"""

import rclpy
import pytest
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker

from river_chart.node import RiverChartNode
from river_chart.parser import Chart, ChartFeature, Geometry


@pytest.fixture

def chart_node(monkeypatch):
    """Create a node without loading the workstation's chart fixture."""

    monkeypatch.setattr(RiverChartNode, "_publish_chart", lambda self: None)
    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init()
    node = RiverChartNode()
    # Rendering unit tests exercise arbitrary S-57 groups. The normal node
    # default intentionally filters them for the concise operational view.
    node.set_parameters([
        Parameter("focus_navigation_features", value=False),
        Parameter("point_size", value=2.0),
    ])
    try:
        yield node
    finally:
        node.destroy_node()
        if owns_context and rclpy.ok():
            rclpy.shutdown()

def test_point_feature_wrapped_as_polygon_publishes_rviz_points(chart_node: RiverChartNode) -> None:
    """S-57 point objects must not disappear when exported as POLYGON WKT."""

    feature = ChartFeature(
        map_index=0,
        group="LIGHTS",
        rcid="1",
        feature_id="2",
        name="LIGHTS",
        feature_type="P",
        geometry=(Geometry("POLYGON", (((12.5, -7.25),),)),),
    )
    chart = Chart(
        features=(feature,),
        map_count=1,
        bounds=(108.0, 30.0, 109.0, 31.0),
        metadata=(),
        origin_longitude=108.5,
        origin_latitude=30.5,
        scale=1.0,
        source_is_projected=True,
    )

    markers = chart_node._build_markers(chart)

    assert len(markers.markers) == 2
    marker = markers.markers[1]
    assert marker.action == Marker.ADD
    assert marker.type == Marker.POINTS
    assert marker.ns == "s57/LIGHTS"
    assert marker.scale.x == pytest.approx(2.0)
    assert marker.scale.y == pytest.approx(2.0)
    assert [(point.x, point.y, point.z) for point in marker.points] == [
        pytest.approx((12.5, -7.25, 0.0))
    ]

def test_line_feature_drops_exporter_added_closing_vertex(chart_node: RiverChartNode) -> None:
    """S-57 line features exported as polygons must remain open in RViz."""

    feature = ChartFeature(
        map_index=0,
        group="DEPCNT",
        rcid="1",
        feature_id="2",
        name="DEPCNT",
        feature_type="L",
        geometry=(
            Geometry(
                "POLYGON",
                (((0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 0.0)),),
            ),
        ),
    )
    chart = Chart(
        features=(feature,),
        map_count=1,
        bounds=(108.0, 30.0, 109.0, 31.0),
        metadata=(),
        origin_longitude=108.5,
        origin_latitude=30.5,
        scale=1.0,
        source_is_projected=True,
    )

    markers = chart_node._build_markers(chart)

    assert len(markers.markers) == 2
    marker = markers.markers[1]
    assert marker.type == Marker.LINE_STRIP
    assert [(point.x, point.y) for point in marker.points] == [
        (0.0, 0.0),
        (10.0, 0.0),
        (10.0, 5.0),
    ]

def test_lane_data_preserves_tsslpt_orientation(chart_node: RiverChartNode) -> None:
    """规划用静态消息必须保留 Marker 中没有的 ORIENT 属性。"""
    import json

    feature = ChartFeature(
        map_index=0,
        group="TSSLPT",
        rcid="7",
        feature_id="8",
        name="TSSLPT",
        feature_type="A",
        geometry=(Geometry("POLYGON", (((0, 0), (5, 0), (5, 4), (0, 0)),)),),
        attributes={"ORIENT": "327"},
    )
    chart = Chart(
        features=(feature,), map_count=1, bounds=(0, 0, 1, 1), metadata=(),
        origin_longitude=0, origin_latitude=0, scale=1, source_is_projected=True,
    )
    payload = json.loads(chart_node._lane_data_json(chart))
    assert payload["lanes"][0]["orient_deg"] == 327.0
    assert payload["lanes"][0]["points"][1] == [5.0, 0.0]


def test_planning_fills_lane_interior_without_erasing_land(chart_node):
    feature = ChartFeature(map_index=0, group="TSSLPT", rcid="1", feature_id="1", name="TSSLPT", feature_type="A",
        geometry=(Geometry("POLYGON", (((0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0), (0.0, 0.0)),)),), attributes={"ORIENT":"90"})
    land = ChartFeature(map_index=0, group="LNDARE", rcid="2", feature_id="2", name="LNDARE", feature_type="A",
        geometry=(Geometry("POLYGON", (((45.0, 45.0), (55.0, 45.0), (55.0, 55.0), (45.0, 55.0), (45.0, 45.0)),)),), attributes={})
    chart = Chart(features=(feature,land), map_count=1, bounds=(0.0,0.0,100.0,100.0), metadata=(),
        origin_longitude=0, origin_latitude=0, scale=1, source_is_projected=True)
    chart_node.set_parameters([Parameter("planning_resolution",value=1.0),Parameter("channel_corridor_width",value=2.0)])
    static,cost=chart_node._planning_grid(chart)
    def value(grid,x,y):
        ix=int((x-grid.info.origin.position.x)/grid.info.resolution)
        iy=int((y-grid.info.origin.position.y)/grid.info.resolution)
        return grid.data[iy*grid.info.width+ix]
    assert value(cost,30,30)==0, "Lane interior retained outside penalty"
    assert value(cost,50,50)==100 and value(static,50,50)==100, "Lane erased island"
