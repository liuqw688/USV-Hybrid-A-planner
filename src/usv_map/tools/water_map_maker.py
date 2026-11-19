#!/usr/bin/env python3
"""下载天地图影像、分割水域并导出 S-57 风格 XML 海图."""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
import urllib.parse
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml


EARTH_RADIUS_M = 6_378_137.0
TILE_SIZE = 256
DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "water_map.yaml"


def mercator_x(longitude_deg: float) -> float:
    """
    将经度转换为 Web Mercator 东向坐标.

    :param longitude_deg: 经度，单位 deg。

    :return: 投影东向坐标，单位 m。
    """
    return EARTH_RADIUS_M * math.radians(longitude_deg)


def mercator_y(latitude_deg: float) -> float:
    """
    将纬度裁剪至 Web Mercator 有效范围后转换为北向坐标.

    :param latitude_deg: 纬度，单位 deg；限制在约 ±85.0511°。

    :return: 投影北向坐标，单位 m。
    """
    latitude_deg = max(-85.05112878, min(85.05112878, latitude_deg))
    return EARTH_RADIUS_M * math.log(math.tan(math.pi / 4.0 + math.radians(latitude_deg) / 2.0))


def inverse_mercator(x: float, y: float) -> tuple[float, float]:
    """
    将 Web Mercator 米制坐标转换为经纬度.

    :param x: 投影东向坐标，单位 m。
    :param y: 投影北向坐标，单位 m。

    :return: (经度, 纬度)，单位 deg。
    """
    longitude = math.degrees(x / EARTH_RADIUS_M)
    latitude = math.degrees(2.0 * math.atan(math.exp(y / EARTH_RADIUS_M)) - math.pi / 2.0)
    return longitude, latitude


def projected_bounds(config: dict[str, Any]) -> tuple[float, float, float, float]:
    """
    按中心及地面半径计算投影包围盒，并补偿纬度放大率.

    :param config: 含中心经纬度(rad)及 radius_m(m)的配置字典。

    :return: (min_x, min_y, max_x, max_y)，Web Mercator 坐标，单位 m。

    :raises ValueError: 中心纬度不在 (-85°, 85°) 内或半径非正。
    """
    latitude = math.degrees(float(config["center_latitude_rad"]))
    longitude = math.degrees(float(config["center_longitude_rad"]))
    radius = float(config["radius_m"])
    if not -85.0 < latitude < 85.0:
        raise ValueError("中心纬度超出 Web Mercator 支持范围")
    if radius <= 0.0:
        raise ValueError("radius_m 必须大于 0")

    # Mercator 放大率为 1/cos(latitude)，换算后仍得到近似地面米制距离。
    half_extent = radius / math.cos(math.radians(latitude))
    center_x, center_y = mercator_x(longitude), mercator_y(latitude)
    return (
        center_x - half_extent,
        center_y - half_extent,
        center_x + half_extent,
        center_y + half_extent,
    )


def world_pixel(x: float, y: float, zoom: int) -> tuple[float, float]:
    """
    将投影坐标转换为指定缩放级别的全局像素坐标.

    :param x: Web Mercator 东向坐标，单位 m。
    :param y: Web Mercator 北向坐标，单位 m。
    :param zoom: 非负的瓦片缩放级别。

    :return: (像素列, 像素行)，图像原点位于左上角，允许小数。
    """
    world_size = TILE_SIZE * (1 << zoom)
    extent = math.pi * EARTH_RADIUS_M
    return (
        (x + extent) / (2.0 * extent) * world_size,
        (extent - y) / (2.0 * extent) * world_size,
    )


def resolve_path(value: str, base_dir: Path) -> Path:
    """
    展开用户目录，并将相对路径解析到给定基准目录.

    :param value: 待解析路径。
    :param base_dir: 相对路径的基准目录。

    :return: 解析后的路径对象。
    """
    path = Path(value).expanduser()
    return path if path.is_absolute() else (base_dir / path).resolve()


def read_config(path: Path) -> tuple[dict[str, Any], Path]:
    """
    读取水域制图配置及资源路径的基准目录.

    :param path: YAML 配置文件路径。

    :return: (配置字典, 配置文件上两级目录)。

    :raises ValueError: YAML 顶层非字典。
    :raises OSError: 配置文件无法读取。
    :raises yaml.YAMLError: YAML 语法无效。
    """
    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    if not isinstance(config, dict):
        raise ValueError("配置文件顶层必须是 YAML 字典")
    return config, path.resolve().parent.parent


def get_tianditu_tile(
    session: Any,
    cache_dir: Path,
    layer: str,
    zoom: int,
    tile_x: int,
    tile_y: int,
    token: str,
    request_delay: float,
) -> np.ndarray:
    """
    优先读取缓存，否则下载并校验单张天地图瓦片.

    :param session: HTTP 会话对象。
    :param cache_dir: 瓦片缓存目录。
    :param layer: WMTS 图层名。
    :param zoom: 瓦片缩放级别。
    :param tile_x: 瓦片列号。
    :param tile_y: 瓦片行号。
    :param token: 服务访问密钥；异常信息不得输出包含密钥的请求 URL。
    :param request_delay: 下载成功后的请求间隔，单位 s；非正值不等待。

    :return: 256×256 的 BGR 图像。

    :raises RuntimeError: 下载失败、HTTP 状态异常或图像无效。
    """
    cache_path = cache_dir / f"{layer}_{zoom}_{tile_x}_{tile_y}.tile"
    if cache_path.is_file():
        encoded = cache_path.read_bytes()
    else:
        query = urllib.parse.urlencode({
            "SERVICE": "WMTS",
            "REQUEST": "GetTile",
            "VERSION": "1.0.0",
            "LAYER": layer.removesuffix("_w"),
            "STYLE": "default",
            "TILEMATRIXSET": "w",
            "FORMAT": "tiles",
            "TILEMATRIX": str(zoom),
            "TILEROW": str(tile_y),
            "TILECOL": str(tile_x),
            "tk": token,
        })
        url = f"https://t0.tianditu.gov.cn/{layer}/wmts?{query}"
        try:
            response = session.get(url, timeout=30)
        except Exception as error:  # 异常包含带密钥的 URL，仅输出异常类型。
            raise RuntimeError(
                f"下载瓦片 {zoom}/{tile_x}/{tile_y} 失败（{type(error).__name__}）"
            ) from None
        if response.status_code != 200:
            raise RuntimeError(
                f"下载瓦片 {zoom}/{tile_x}/{tile_y} 失败，HTTP {response.status_code}；"
                "请检查 config 中的 tianditu_api_key 和天地图服务状态"
            )
        encoded = response.content
        cache_dir.mkdir(parents=True, exist_ok=True)
        cache_path.write_bytes(encoded)
        if request_delay > 0.0:
            time.sleep(request_delay)

    image = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != (TILE_SIZE, TILE_SIZE):
        cache_path.unlink(missing_ok=True)
        raise RuntimeError(f"瓦片 {zoom}/{tile_x}/{tile_y} 不是有效的 256×256 图像")
    return image


def download_mosaic(config: dict[str, Any], base_dir: Path, layer_override: str | None = None) -> tuple[np.ndarray, tuple[float, float, float, float]]:
    """
    下载目标范围内的瓦片，拼接并裁剪为底图.

    :param config: 包含中心、半径、缩放级别、密钥及瓦片数量限制的配置。
    :param base_dir: 缓存等相对路径的基准目录。
    :param layer_override: 可选图层覆盖值；为空时使用配置图层。

    :return: (BGR 底图, 投影包围盒)，包围盒顺序为 min_x/min_y/max_x/max_y，单位 m。

    :raises ValueError: 密钥缺失、图层或范围无效，或瓦片数超过配置上限。
    :raises RuntimeError: 缺少 requests，或瓦片下载、解码失败。
    """
    token = str(config.get("tianditu_api_key") or "").strip()
    if not token:
        token = os.environ.get("TIANDITU_TK", "").strip()
    if not token:
        raise ValueError("请在 config/water_map.yaml 中设置 tianditu_api_key")

    zoom = int(config["zoom"])
    if not 1 <= zoom <= 18:
        raise ValueError("zoom 必须在 1 到 18 之间")
    layer = str(layer_override or config.get("layer", "img_w"))
    if not layer.endswith("_w"):
        raise ValueError("天地图 WMTS 图层名应使用 img_w 等 XYZ 瓦片图层")

    min_x, min_y, max_x, max_y = projected_bounds(config)
    left_px, bottom_px = world_pixel(min_x, min_y, zoom)
    right_px, top_px = world_pixel(max_x, max_y, zoom)
    pixel_left, pixel_right = math.floor(left_px), math.ceil(right_px)
    pixel_top, pixel_bottom = math.floor(top_px), math.ceil(bottom_px)
    tile_count = 1 << zoom
    first_tile_x = max(0, pixel_left // TILE_SIZE)
    last_tile_x = min(tile_count - 1, (pixel_right - 1) // TILE_SIZE)
    first_tile_y = max(0, pixel_top // TILE_SIZE)
    last_tile_y = min(tile_count - 1, (pixel_bottom - 1) // TILE_SIZE)
    if first_tile_x > last_tile_x or first_tile_y > last_tile_y:
        raise ValueError("目标范围超出瓦片地图边界")
    tile_total = (last_tile_x - first_tile_x + 1) * (last_tile_y - first_tile_y + 1)
    maximum_tiles = max(1, int(config.get("maximum_tiles", 128)))
    if tile_total > maximum_tiles:
        raise ValueError(
            f"本次需要 {tile_total} 张瓦片，超过 maximum_tiles={maximum_tiles}；"
            "请降低 zoom 或缩小 radius_m"
        )

    cache_dir = resolve_path(str(config.get("tile_cache", "cache/tianditu")), base_dir)
    cache_dir.mkdir(parents=True, exist_ok=True)
    try:
        import requests
    except ImportError as error:
        raise RuntimeError("缺少 requests，请安装 requirements.txt") from error

    mosaic = np.zeros((
        (last_tile_y - first_tile_y + 1) * TILE_SIZE,
        (last_tile_x - first_tile_x + 1) * TILE_SIZE,
        3,
    ), dtype=np.uint8)
    with requests.Session() as session:
        session.headers.update({"User-Agent": "river-chart-water-map-maker/1.0"})
        for tile_y in range(first_tile_y, last_tile_y + 1):
            for tile_x in range(first_tile_x, last_tile_x + 1):
                tile = get_tianditu_tile(
                    session,
                    cache_dir,
                    layer,
                    zoom,
                    tile_x,
                    tile_y,
                    token,
                    float(config.get("request_delay_s", 0.15)),
                )
                offset_x = (tile_x - first_tile_x) * TILE_SIZE
                offset_y = (tile_y - first_tile_y) * TILE_SIZE
                mosaic[offset_y:offset_y + TILE_SIZE, offset_x:offset_x + TILE_SIZE] = tile

    crop_left = pixel_left - first_tile_x * TILE_SIZE
    crop_top = pixel_top - first_tile_y * TILE_SIZE
    image = mosaic[crop_top:crop_top + pixel_bottom - pixel_top,
                   crop_left:crop_left + pixel_right - pixel_left].copy()
    resolution = 2.0 * math.pi * EARTH_RADIUS_M / (TILE_SIZE * (1 << zoom))
    projected_x0 = -math.pi * EARTH_RADIUS_M + pixel_left * resolution
    projected_y1 = math.pi * EARTH_RADIUS_M - pixel_top * resolution
    bounds = (
        projected_x0,
        projected_y1 - image.shape[0] * resolution,
        projected_x0 + image.shape[1] * resolution,
        projected_y1,
    )
    return image, bounds


def load_image(config: dict[str, Any], base_dir: Path) -> tuple[np.ndarray, tuple[float, float, float, float]]:
    """
    从配置指定的本地图像读取底图，未指定时下载瓦片.

    :param config: 影像来源与地理范围配置。
    :param base_dir: 本地相对路径的基准目录。

    :return: (BGR 底图, Web Mercator 包围盒)，坐标单位 m。

    :raises ValueError: 本地图像无法读取或地理范围配置无效。
    :raises RuntimeError: 瓦片下载或解码失败。
    """
    source = str(config.get("source_image", "")).strip()
    if not source:
        return download_mosaic(config, base_dir)
    image_path = resolve_path(source, base_dir)
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"无法读取本地图像：{image_path}")
    return image, projected_bounds(config)


def water_mask_from_image(image: np.ndarray, config: dict[str, Any]) -> np.ndarray:
    """
    按 HSV 阈值分割水域，并进行闭运算和小连通域过滤.

    :param image: uint8 BGR 底图。
    :param config: HSV 阈值、形态学核尺寸及最小水域面积配置。

    :return: uint8 二值掩膜，255 表示水体，0 表示其他区域。

    :raises ValueError: HSV 阈值不是三个分量。
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.array(config.get("water_hsv_lower", [85, 25, 25]), dtype=np.uint8)
    upper = np.array(config.get("water_hsv_upper", [140, 255, 245]), dtype=np.uint8)
    if lower.shape != (3,) or upper.shape != (3,):
        raise ValueError("water_hsv_lower/upper 必须是 [H, S, V] 三个值")
    mask = cv2.inRange(hsv, lower, upper)

    kernel_size = max(1, int(config.get("morphology_kernel", 3)))
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    min_area = max(0, int(config.get("minimum_water_area_px", 8)))
    if min_area > 1:
        _, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        keep = stats[:, cv2.CC_STAT_AREA] >= min_area
        keep[0] = False
        mask = np.where(keep[labels], 255, 0).astype(np.uint8)
    return mask


def load_water_mask(path: Path, expected_shape: tuple[int, int]) -> np.ndarray:
    """
    读取并按 128 阈值二值化人工水域掩膜.

    :param path: 灰度掩膜文件路径。
    :param expected_shape: 期望的 (高度, 宽度)，单位 px。

    :return: uint8 二值掩膜，255 表示水体。

    :raises ValueError: 图像无法读取或尺寸与底图不一致。
    """
    mask = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise ValueError(f"无法读取手工水体掩膜：{path}")
    if mask.shape != expected_shape:
        raise ValueError(
            f"水体掩膜尺寸 {mask.shape[1]}×{mask.shape[0]} 与底图 "
            f"{expected_shape[1]}×{expected_shape[0]} 不一致"
        )
    return np.where(mask >= 128, 255, 0).astype(np.uint8)


def make_preview(image: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """
    在底图上叠加水域及陆地颜色，生成分割检查图.

    :param image: BGR 底图。
    :param mask: 同尺寸二值掩膜，非零表示水体。

    :return: BGR 预览图；输入底图保持不变。
    """
    preview = image.copy()
    water = mask != 0
    land = ~water
    water_color = np.array([220, 150, 40], dtype=np.uint8)  # BGR 水域青蓝色。
    land_color = np.array([50, 145, 220], dtype=np.uint8)  # BGR 陆地暖橙色。
    preview[water] = (0.55 * preview[water] + 0.45 * water_color).astype(np.uint8)
    preview[land] = (0.75 * preview[land] + 0.25 * land_color).astype(np.uint8)
    return preview


def contour_rings(mask: np.ndarray, minimum_area: int, simplify_px: float) -> list[list[np.ndarray]]:
    """
    提取水域外轮廓及孔洞，并按面积过滤、按像素精度简化.

    :param mask: uint8 二值掩膜。
    :param minimum_area: 最小保留面积，单位 px²。
    :param simplify_px: 轮廓简化误差，单位 px。

    :return: 多边形列表；每个多边形先存外环，再存孔洞，环为 N×2 像素坐标。
    """
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return []
    hierarchy = hierarchy[0]
    result: list[list[np.ndarray]] = []
    for index, item in enumerate(hierarchy):
        if item[3] != -1 or abs(cv2.contourArea(contours[index])) < minimum_area:
            continue
        outer = cv2.approxPolyDP(contours[index], simplify_px, True)
        if len(outer) < 3:
            continue
        rings = [outer.reshape(-1, 2)]
        child = item[2]
        while child != -1:
            if abs(cv2.contourArea(contours[child])) >= minimum_area:
                hole = cv2.approxPolyDP(contours[child], simplify_px, True)
                if len(hole) >= 3:
                    rings.append(hole.reshape(-1, 2))
            child = hierarchy[child][0]
        result.append(rings)
    return result


def pixel_to_chart(point: np.ndarray, bounds: tuple[float, float, float, float], width: int,
                   height: int) -> tuple[float, float]:
    """
    将像素中心转换为海图投影坐标，纵坐标采用南向为正.

    :param point: (列, 行)像素坐标。
    :param bounds: Web Mercator 的 min_x/min_y/max_x/max_y，单位 m。
    :param width: 图像宽度，须为正数，单位 px。
    :param height: 图像高度，须为正数，单位 px。

    :return: (东向坐标, 负北向坐标)，单位 m。
    """
    min_x, min_y, max_x, max_y = bounds
    x = min_x + (float(point[0]) + 0.5) * (max_x - min_x) / width
    northing = max_y - (float(point[1]) + 0.5) * (max_y - min_y) / height
    return x, -northing


def wkt_polygon(rings: list[np.ndarray], bounds: tuple[float, float, float, float],
                width: int, height: int) -> str:
    """
    将像素轮廓转换为带显式闭合点的海图 WKT 多边形.

    :param rings: 非空外环及可选孔洞，每个环至少含一个像素点。
    :param bounds: Web Mercator 包围盒，单位 m。
    :param width: 图像宽度，须为正数，单位 px。
    :param height: 图像高度，须为正数，单位 px。

    :return: 三位小数米制坐标的 POLYGON 字符串，纵轴采用南向为正。
    """
    encoded_rings = []
    for ring in rings:
        points = [pixel_to_chart(point, bounds, width, height) for point in ring]
        points.append(points[0])
        encoded_rings.append("(" + ", ".join(f"{x:.3f} {y:.3f}" for x, y in points) + ")")
    return "POLYGON (" + ", ".join(encoded_rings) + ")"


def append_feature(group: ET.Element, feature_id: int, acronym: str,
                   rings: list[np.ndarray], bounds: tuple[float, float, float, float],
                   width: int, height: int) -> None:
    """
    向图层追加一个面积要素，写入标识、包围盒及 WKT 几何.

    :param group: 待修改的 XML FeatureGroup 节点。
    :param feature_id: 要素标识。
    :param acronym: 要素类别，DEPARE 会额外写入深度属性。
    :param rings: 非空像素外环及可选孔洞。
    :param bounds: Web Mercator 包围盒，单位 m。
    :param width: 图像宽度，须为正数，单位 px。
    :param height: 图像高度，须为正数，单位 px。
    """
    feature = ET.SubElement(group, "Feature")
    ET.SubElement(feature, "RCID").text = str(feature_id)
    ET.SubElement(feature, "ID").text = str(feature_id)
    ET.SubElement(feature, "Name").text = acronym
    ET.SubElement(feature, "Type").text = "A"
    ET.SubElement(feature, "TableName").text = "O"
    attributes = ET.SubElement(feature, "Attributes")
    if acronym == "DEPARE":
        ET.SubElement(attributes, "DRVAL1").text = "0"
        ET.SubElement(attributes, "DRVAL2").text = "0"

    geometry = ET.SubElement(feature, "Geometry")
    projected_points = [
        pixel_to_chart(point, bounds, width, height)
        for ring in rings for point in ring
    ]
    min_x = min(point[0] for point in projected_points)
    min_y = min(point[1] for point in projected_points)
    max_x = max(point[0] for point in projected_points)
    max_y = max(point[1] for point in projected_points)
    ET.SubElement(geometry, "BoundingBox").text = (
        f"{min_x:.3f},{min_y:.3f},{max_x:.3f},{max_y:.3f}"
    )
    geometries = ET.SubElement(geometry, "Geometries", {"level": "0"})
    ET.SubElement(geometries, "Polygon").text = wkt_polygon(rings, bounds, width, height)


def geographic_bounds(projected: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """
    将投影包围盒转换为经纬度包围盒.

    :param projected: Web Mercator 的 min_x/min_y/max_x/max_y，单位 m。

    :return: (最小经度, 最小纬度, 最大经度, 最大纬度)，单位 deg。
    """
    min_x, min_y, max_x, max_y = projected
    min_lon, min_lat = inverse_mercator(min_x, min_y)
    max_lon, max_lat = inverse_mercator(max_x, max_y)
    return min_lon, min_lat, max_lon, max_lat


def write_chart(mask: np.ndarray, bounds: tuple[float, float, float, float],
                config: dict[str, Any], output_path: Path) -> tuple[int, int]:
    """
    从二值掩膜生成水域、陆地面积要素并写入 XML 海图.

    :param mask: uint8 二值掩膜，255 表示水体，0 表示陆地。
    :param bounds: Web Mercator 包围盒，单位 m。
    :param config: 面积过滤、轮廓简化及海图比例尺配置。
    :param output_path: 输出 XML 路径，缺失的父目录会自动创建。

    :return: (水域多边形数量, 陆地多边形数量)。

    :raises OSError: 输出目录或文件无法写入。
    """
    height, width = mask.shape
    minimum_area = max(0, int(config.get("minimum_region_area_px", 8)))
    simplify_px = max(0.0, float(config.get("simplify_px", 1.5)))
    water_polygons = contour_rings(mask, minimum_area, simplify_px)
    land_polygons = contour_rings(cv2.bitwise_not(mask), minimum_area, simplify_px)

    root = ET.Element("S57ChartData", {
        "version": "1.0",
        "totalMaps": "1",
    })
    map_element = ET.SubElement(root, "Map", {"index": "0"})
    metadata = ET.SubElement(map_element, "Metadata")
    ET.SubElement(metadata, "DSID_DSNM").text = output_path.name
    ET.SubElement(metadata, "DSID_COMT").text = (
        "Tianditu image tiles; water segmentation requires visual review"
    )
    ET.SubElement(metadata, "DSPM_CSCL").text = str(config.get("chart_scale", 5000))
    min_lon, min_lat, max_lon, max_lat = geographic_bounds(bounds)
    chart_bounds = ET.SubElement(map_element, "Bounds")
    for tag, value in (
        ("MinX", min_lon), ("MinY", min_lat), ("MaxX", max_lon), ("MaxY", max_lat)
    ):
        ET.SubElement(chart_bounds, tag).text = f"{value:.9f}"

    features = ET.SubElement(map_element, "Features")
    feature_id = 1
    for acronym, polygons in (("DEPARE", water_polygons), ("LNDARE", land_polygons)):
        group = ET.SubElement(features, "FeatureGroup", {
            "acronym": acronym,
            "priority": "1" if acronym == "DEPARE" else "2",
            "count": str(len(polygons)),
        })
        for polygon in polygons:
            append_feature(
                group,
                feature_id,
                acronym,
                polygon,
                bounds,
                width,
                height,
            )
            feature_id += 1

    ET.indent(root, space="  ")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output_path, encoding="utf-8", xml_declaration=True)
    return len(water_polygons), len(land_polygons)


def run(config_path: Path) -> None:
    """
    执行底图获取、水域分割和海图导出，并保存分割检查图.

    :param config_path: YAML 配置文件路径。

    :raises ValueError: 配置、影像或掩膜无效。
    :raises RuntimeError: 下载失败或图像输出失败。
    :raises OSError: 文件无法读取或写入。
    """
    config, base_dir = read_config(config_path)
    image, bounds = load_image(config, base_dir)
    output_dir = resolve_path(str(config.get("output_dir", "resource/generated")), base_dir)
    stem = str(config.get("output_name", "reservoir_test"))
    if not stem or Path(stem).name != stem:
        raise ValueError("output_name 只能是文件名，不能包含目录")
    image_path = output_dir / f"{stem}_image.png"
    mask_path = output_dir / f"{stem}_water_mask.png"
    preview_path = output_dir / f"{stem}_preview.png"
    xml_path = output_dir / f"{stem}.xml"

    manual_mask = str(config.get("water_mask", "")).strip()
    if manual_mask:
        water_mask = load_water_mask(resolve_path(manual_mask, base_dir), image.shape[:2])
    else:
        water_mask = water_mask_from_image(image, config)

    output_dir.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(image_path), image):
        raise RuntimeError(f"无法写入底图：{image_path}")
    if not cv2.imwrite(str(mask_path), water_mask):
        raise RuntimeError(f"无法写入水体掩膜：{mask_path}")
    if not cv2.imwrite(str(preview_path), make_preview(image, water_mask)):
        raise RuntimeError(f"无法写入分割预览：{preview_path}")

    water_count, land_count = write_chart(water_mask, bounds, config, xml_path)
    print(f"影像尺寸：{image.shape[1]}×{image.shape[0]} 像素")
    print(f"水域面：{water_count}，不可通行区域：{land_count}")
    print(f"XML 海图：{xml_path}")
    print(f"分割预览：{preview_path}")
    print(f"水体掩膜：{mask_path}（白色=水体，黑色=不可通行）")
    print("影像来源：天地图；用于测试时请保留天地图来源标注。")


def main() -> int:
    """解析命令行并执行制图；成功返回 0，已处理的失败返回 1."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="YAML 配置文件")
    args = parser.parse_args()
    try:
        run(args.config.expanduser().resolve())
    except (OSError, ValueError, RuntimeError, yaml.YAMLError) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
