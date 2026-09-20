"""@brief 解析 GDAL/S-57 XML 海图导出文件。

The export used by this project contains ordinary WKT geometry, but a few
empty attribute values are serialized as ``<></>``.  ``xml.etree`` quite
reasonably rejects that spelling, so the input is normalized before parsing.
No GDAL or Shapely dependency is required.

@author susheng
@date 2026-08-28
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import re
from typing import Any, Iterator, Mapping, Sequence
import xml.etree.ElementTree as ET


Point = tuple[float, float]

_NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
_WKT_TOKEN = re.compile(rf"\s*(\(|\)|,|{_NUMBER}|[A-Za-z_][A-Za-z_0-9]*)")
_INVALID_EMPTY_TAG = re.compile(r"<\s*>\s*</\s*>")
_EARTH_RADIUS_M = 6378137.0
_DEG_TO_RAD = math.pi / 180.0
_METERS_PER_DEGREE = 111_319.49079327358


class ChartParseError(ValueError):
    """Raised when a chart cannot be decoded."""


@dataclass(frozen=True)
class Geometry:
    """A WKT geometry represented as one or more point paths.

    ``parts`` contains paths for lines and rings for polygons.  A point is
    represented by a one-point path.  Polygon holes are retained as separate
    rings; consumers that need a filled polygon can use the first ring as the
    exterior and subsequent rings as holes.
    """

    geometry_type: str
    parts: tuple[tuple[Point, ...], ...]


@dataclass
class ChartFeature:
    """A chart feature and its transformed geometry."""

    map_index: int
    group: str
    rcid: str
    feature_id: str
    name: str
    feature_type: str
    geometry: tuple[Geometry, ...]
    attributes: Mapping[str, str] = field(default_factory=dict)


@dataclass
class Chart:
    """Decoded chart data in a local ENU-like frame, measured in metres."""

    features: tuple[ChartFeature, ...]
    map_count: int
    bounds: tuple[float, float, float, float]
    metadata: tuple[Mapping[str, str], ...]
    origin_longitude: float
    origin_latitude: float
    scale: float
    source_is_projected: bool
    source_path: str = ""

    @property
    # [功能与联系] 返回海图要素类别去重列表，供元数据和显示筛选。
    def groups(self) -> tuple[str, ...]:
        """Return feature groups in first-seen order."""

        return tuple(dict.fromkeys(feature.group for feature in self.features))

    @property
    # [功能与联系] 返回已解析有效要素数，供元数据和日志。
    def feature_count(self) -> int:
        return len(self.features)

    # [功能与联系] 序列化地图来源、原点、scale和要素统计，用于锁存诊断话题。
    def metadata_json(self) -> str:
        """Serialize useful chart information for a latched ROS topic."""

        return json.dumps(
            {
                "source": self.source_path,
                "map_count": self.map_count,
                "feature_count": self.feature_count,
                "groups": list(self.groups),
                "bounds_lon_lat": self.bounds,
                "origin_longitude": self.origin_longitude,
                "origin_latitude": self.origin_latitude,
                "scale": self.scale,
                "source_is_projected": self.source_is_projected,
            },
            ensure_ascii=False,
            sort_keys=True,
        )


# [功能与联系] 将WKT字符串切分为几何类型、括号与数字token，供递归坐标解析。
def _tokenize_wkt(value: str) -> list[str]:
    tokens: list[str] = []
    position = 0
    while position < len(value):
        match = _WKT_TOKEN.match(value, position)
        if match is None:
            # Whitespace at the end is harmless; any other character means
            # this is not WKT that the fallback parser can safely interpret.
            if value[position:].strip():
                raise ChartParseError(f"invalid WKT near {value[position:position + 32]!r}")
            break
        tokens.append(match.group(1))
        position = match.end()
    return tokens


# [功能与联系] 递归读取括号坐标结构及数字序列，供parse_wkt处理多段/多环。
def _parse_group(tokens: Sequence[str], index: int = 0) -> tuple[Any, int]:
    if index >= len(tokens) or tokens[index] != "(":
        raise ChartParseError("WKT coordinate group must start with '('")
    index += 1
    values: list[Any] = []
    while index < len(tokens) and tokens[index] != ")":
        if tokens[index] == "(":
            child, index = _parse_group(tokens, index)
            values.append(child)
        else:
            if index + 1 >= len(tokens):
                raise ChartParseError("truncated WKT coordinate")
            try:
                x = float(tokens[index])
                y = float(tokens[index + 1])
            except ValueError as exc:
                raise ChartParseError("WKT coordinate is not numeric") from exc
            index += 2
            # S-57 exports 2D coordinates.  Accept an optional Z/M pair and
            # ignore it so common 3D WKT remains displayable.
            if index < len(tokens) and tokens[index] not in {",", ")", "("}:
                index += 1
            if index < len(tokens) and tokens[index] not in {",", ")", "("}:
                index += 1
            values.append((x, y))
        if index < len(tokens) and tokens[index] == ",":
            index += 1
        elif index < len(tokens) and tokens[index] != ")":
            raise ChartParseError("expected ',' or ')' in WKT")
    if index >= len(tokens) or tokens[index] != ")":
        raise ChartParseError("unterminated WKT coordinate group")
    return values, index + 1


# [功能与联系] 验证并转换二维坐标对，供WKT标准化。
def _as_point(value: Any) -> Point:
    if not isinstance(value, tuple) or len(value) != 2:
        raise ChartParseError("expected a WKT point")
    return float(value[0]), float(value[1])


# [功能与联系] 将坐标序列标准化为不可变点列表，供Geometry几何parts构建。
def _as_path(value: Any) -> tuple[Point, ...]:
    if not isinstance(value, list):
        raise ChartParseError("expected a WKT point path")
    return tuple(_as_point(point) for point in value)


# [功能与联系] 解析POINT/LINESTRING/POLYGON及多几何WKT，将要素标准化；格式异常抛ChartParseError。
def parse_wkt(value: str | None) -> Geometry:
    """Parse a WKT string without requiring Shapely.

    The parser intentionally supports the geometry families emitted by the
    XML exporter: ``POINT``, ``MULTIPOINT``, ``LINESTRING``,
    ``MULTILINESTRING``, ``POLYGON`` and ``MULTIPOLYGON``.  Empty geometries
    are represented by an empty ``parts`` tuple.
    """

    text = (value or "").strip()
    if not text:
        raise ChartParseError("empty WKT geometry")
    match = re.match(r"^([A-Za-z]+)\s*(?:ZM|Z|M)?\s*(.*)$", text, re.IGNORECASE | re.DOTALL)
    if match is None:
        raise ChartParseError(f"invalid WKT geometry {text[:40]!r}")
    geometry_type = match.group(1).upper()
    body = match.group(2).strip()
    if body.upper() == "EMPTY":
        return Geometry(geometry_type, ())
    tokens = _tokenize_wkt(body)
    parsed, index = _parse_group(tokens)
    if index != len(tokens):
        raise ChartParseError("unexpected tokens after WKT geometry")

    if geometry_type == "POINT":
        return Geometry(geometry_type, ((_as_point(parsed[0]),),) if parsed else ())
    if geometry_type == "MULTIPOINT":
        parts = []
        for item in parsed:
            parts.append((_as_point(item[0]),) if isinstance(item, list) else (_as_point(item),))
        return Geometry(geometry_type, tuple(parts))
    if geometry_type == "LINESTRING":
        return Geometry(geometry_type, (_as_path(parsed),))
    if geometry_type == "MULTILINESTRING":
        return Geometry(geometry_type, tuple(_as_path(path) for path in parsed))
    if geometry_type == "POLYGON":
        return Geometry(geometry_type, tuple(_as_path(ring) for ring in parsed))
    if geometry_type == "MULTIPOLYGON":
        return Geometry(
            geometry_type,
            tuple(_as_path(ring) for polygon in parsed for ring in polygon),
        )
    raise ChartParseError(f"unsupported WKT geometry type {geometry_type!r}")


# [功能与联系] 将Mercator northing还原为纬度，处理源导出坐标系。
def _inverse_mercator_latitude(northing: float) -> float:
    """Return latitude in radians for an EPSG:3857 northing."""

    return math.atan(math.sinh(northing / _EARTH_RADIUS_M))


# [功能与联系] 从地图元数据求全局经纬度范围，用于默认参考原点。
def _global_bounds(maps: Sequence[ET.Element]) -> tuple[float, float, float, float]:
    values: list[tuple[float, float, float, float]] = []
    for map_element in maps:
        bounds = map_element.find("Bounds")
        if bounds is None:
            continue
        try:
            values.append(
                (
                    float(bounds.findtext("MinX", "nan")),
                    float(bounds.findtext("MinY", "nan")),
                    float(bounds.findtext("MaxX", "nan")),
                    float(bounds.findtext("MaxY", "nan")),
                )
            )
        except ValueError:
            continue
    if not values or any(not math.isfinite(v) for row in values for v in row):
        raise ChartParseError("chart contains no valid geographic Bounds")
    return (
        min(row[0] for row in values),
        min(row[1] for row in values),
        max(row[2] for row in values),
        max(row[3] for row in values),
    )


# [功能与联系] 遍历XML要素几何字段，向load_chart提供待解析WKT。
def _iter_wkt_elements(feature_element: ET.Element) -> Iterator[tuple[str, str]]:
    geometry = feature_element.find("Geometry")
    if geometry is None:
        return
    geometries = geometry.find("Geometries")
    if geometries is None:
        return
    for element in geometries:
        text = (element.text or "").strip()
        if text:
            yield element.tag.upper(), text


# [功能与联系] 对Geometry的每个点应用投影函数，输出局部坐标。
def _transform_geometry(geometry: Geometry, transform) -> Geometry:
    return Geometry(
        geometry.geometry_type,
        tuple(tuple(transform(x, y) for x, y in part) for part in geometry.parts),
    )


# [功能与联系] 修复特定非法空标签、解析XML/WKT并按源投影转换局部坐标；scale实际除坐标值，影响整链路几何尺寸。
def load_chart(
    path: str | Path,
    *,
    origin_longitude: float | None = None,
    origin_latitude: float | None = None,
    scale: float = 1.0,
) -> Chart:
    """Load an exported chart and convert every geometry to local metres.

    Projected coordinates are recognized by their magnitude.  The supplied
    exporter uses Web-Mercator X with a negated Y, which is converted back to
    longitude/latitude before applying a local east/north approximation in
    physical metres.  Longitude/latitude WKT is also accepted.  ``scale``
    divides the resulting local coordinates and is useful when a consumer
    wants a compact visualization.
    """

    source_path = str(Path(path).expanduser())
    try:
        raw = Path(source_path).read_text(encoding="utf-8")
    except OSError as exc:
        raise ChartParseError(f"cannot read chart {source_path!r}: {exc}") from exc
    raw = _INVALID_EMPTY_TAG.sub("<Empty></Empty>", raw)
    try:
        root = ET.fromstring(raw)
    except ET.ParseError as exc:
        raise ChartParseError(f"invalid XML in {source_path!r}: {exc}") from exc
    maps = list(root.findall("Map"))
    if not maps:
        raise ChartParseError("XML chart contains no Map elements")
    bounds = _global_bounds(maps)
    auto_lon = (bounds[0] + bounds[2]) / 2.0
    auto_lat = (bounds[1] + bounds[3]) / 2.0
    longitude = auto_lon if origin_longitude is None or not math.isfinite(origin_longitude) else float(origin_longitude)
    latitude = auto_lat if origin_latitude is None or not math.isfinite(origin_latitude) else float(origin_latitude)
    scale = float(scale)
    if not math.isfinite(scale) or scale <= 0.0:
        raise ChartParseError("scale must be a finite number greater than zero")

    sample_point: Point | None = None
    for map_element in maps:
        for feature_element in map_element.findall("./Features/FeatureGroup/Feature"):
            for _, text in _iter_wkt_elements(feature_element):
                try:
                    parsed = parse_wkt(text)
                except ChartParseError:
                    continue
                if parsed.parts and parsed.parts[0]:
                    sample_point = parsed.parts[0][0]
                    break
            if sample_point is not None:
                break
        if sample_point is not None:
            break
    source_is_projected = (
        sample_point is not None
        and max(abs(sample_point[0]), abs(sample_point[1])) > 1000.0
    )
    if source_is_projected:
        origin_longitude_radians = longitude * _DEG_TO_RAD
        origin_latitude_radians = latitude * _DEG_TO_RAD
        cos_origin_latitude = math.cos(origin_latitude_radians)

        # [功能与联系] 将源坐标转换为局部平面坐标并除以scale；load_chart按源投影选择此闭包，不能只改变Marker显示而忽略规划坐标。
        def transform(x: float, y: float) -> Point:
            # The supplied exporter stores the EPSG:3857 northing as -Y.
            longitude_radians = x / _EARTH_RADIUS_M
            latitude_radians = _inverse_mercator_latitude(-y)
            return (
                _EARTH_RADIUS_M
                * (longitude_radians - origin_longitude_radians)
                * cos_origin_latitude
                / scale,
                _EARTH_RADIUS_M * (latitude_radians - origin_latitude_radians) / scale,
            )

    else:
        cos_lat = math.cos(latitude * _DEG_TO_RAD)

        # [功能与联系] 将源坐标转换为局部平面坐标并除以scale；load_chart按源投影选择此闭包，不能只改变Marker显示而忽略规划坐标。
        def transform(x: float, y: float) -> Point:
            return (
                (x - longitude) * _METERS_PER_DEGREE * cos_lat / scale,
                (y - latitude) * _METERS_PER_DEGREE / scale,
            )

    features: list[ChartFeature] = []
    metadata: list[Mapping[str, str]] = []
    for map_index, map_element in enumerate(maps):
        map_metadata = {
            item.tag: (item.text or "").strip()
            for item in map_element.findall("./Metadata/*")
            if item.tag != "Empty"
        }
        metadata.append(map_metadata)
        for group in map_element.findall("./Features/FeatureGroup"):
            group_name = group.attrib.get("acronym") or group.findtext("Name") or "Generic"
            for feature_element in group.findall("Feature"):
                geometries: list[Geometry] = []
                for _, wkt in _iter_wkt_elements(feature_element):
                    try:
                        geometries.append(_transform_geometry(parse_wkt(wkt), transform))
                    except ChartParseError:
                        # A malformed individual feature should not hide the
                        # remaining 783 valid objects in a chart.
                        continue
                if not geometries:
                    continue
                attributes = {
                    item.tag: (item.text or "").strip()
                    for item in feature_element.findall("./Attributes/*")
                    if item.tag != "Empty"
                }
                features.append(
                    ChartFeature(
                        map_index=map_index,
                        group=group_name,
                        rcid=feature_element.findtext("RCID", ""),
                        feature_id=feature_element.findtext("ID", ""),
                        name=feature_element.findtext("Name", group_name),
                        feature_type=feature_element.findtext("Type", "").upper(),
                        geometry=tuple(geometries),
                        attributes=attributes,
                    )
                )
    return Chart(
        features=tuple(features),
        map_count=len(maps),
        bounds=bounds,
        metadata=tuple(metadata),
        origin_longitude=longitude,
        origin_latitude=latitude,
        scale=scale,
        source_is_projected=source_is_projected,
        source_path=source_path,
    )
