#!/usr/bin/env python3
"""合并生成的水库图层，输出统一的 S-57 风格 XML 海图."""

from __future__ import annotations

import argparse
import copy
import re
import xml.etree.ElementTree as ET
from pathlib import Path


def read_xml(path: Path) -> ET.Element:
    """
    读取海图 XML，并将导出器生成的空标签替换为有效占位节点.

    :param path: XML 文件路径。

    :return: XML 根节点。

    :raises OSError: 文件无法读取。
    :raises ET.ParseError: 修正空标签后仍存在 XML 语法错误。
    """
    text = path.read_text(encoding="utf-8")
    text = re.sub(r"<\s*>\s*</\s*>", "<Empty></Empty>", text)
    return ET.fromstring(text)


def map_element(root: ET.Element, path: Path) -> ET.Element:
    """
    读取且要求根节点下恰好有一个 Map.

    :param root: XML 根节点。
    :param path: 用于异常定位的来源文件路径。

    :return: 唯一的 Map 节点。

    :raises ValueError: Map 节点数量不等于 1。
    """
    maps = root.findall("Map")
    if len(maps) != 1:
        raise ValueError(f"{path} must contain exactly one Map element")
    return maps[0]


def reference_map(root: ET.Element, path: Path) -> ET.Element:
    """
    读取参考海图的首个 Map，供元数据复用.

    :param root: 参考 XML 根节点。
    :param path: 用于异常定位的来源文件路径。

    :return: 首个 Map 节点。

    :raises ValueError: 缺少 Map 节点。
    """
    map_node = root.find("Map")
    if map_node is None:
        raise ValueError(f"{path} has no Map element")
    return map_node


def group_map(map_node: ET.Element) -> dict[str, ET.Element]:
    """
    按 acronym 建立要素组索引，跳过未命名组.

    :param map_node: 海图 Map 节点。

    :return: acronym 到原始 FeatureGroup 节点的映射；重名时保留最后一个。

    :raises ValueError: 缺少 Features 节点。
    """
    features = map_node.find("Features")
    if features is None:
        raise ValueError("chart Map is missing Features")
    return {
        group.get("acronym", ""): group
        for group in features.findall("FeatureGroup")
        if group.get("acronym")
    }


def append_group(target: ET.Element, source: ET.Element, acronym: str) -> None:
    """
    深拷贝指定要素组并追加到目标 Features，避免共享可变节点.

    :param target: 目标 Features 节点。
    :param source: 来源 Map 或 Features 节点。
    :param acronym: 待复制要素组的类别名。

    :raises ValueError: 来源中缺少指定要素组。
    """
    features = source if source.tag == "Features" else source.find("Features")
    group = features.find(f"FeatureGroup[@acronym='{acronym}']") if features is not None else None
    if group is None:
        raise ValueError(f"source chart is missing FeatureGroup {acronym}")
    target.append(copy.deepcopy(group))


def update_counts(features: ET.Element) -> None:
    """
    将各 FeatureGroup 的 count 属性更新为实际 Feature 子节点数量.

    :param features: 待原地更新的 Features 节点。
    """
    for group in features.findall("FeatureGroup"):
        group.set("count", str(len(group.findall("Feature"))))


def main() -> None:
    """合并参考元数据、水域、航道及泊位图层，并按命令行路径写出海图."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--water", type=Path, required=True)
    parser.add_argument("--fairway", type=Path, required=True)
    parser.add_argument("--berths", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    reference_root = read_xml(args.reference)
    reference_metadata_map = reference_map(reference_root, args.reference)
    water_map = map_element(read_xml(args.water), args.water)
    fairway_map = map_element(read_xml(args.fairway), args.fairway)
    berths_map = map_element(read_xml(args.berths), args.berths)

    source_groups = group_map(water_map)
    fairway_groups = group_map(fairway_map)
    berth_groups = group_map(berths_map)
    for required in ("DEPARE", "LNDARE"):
        if required not in source_groups:
            raise ValueError(f"water chart is missing FeatureGroup {required}")
    for required in ("TSSLPT", "TSSBND", "TSELNE"):
        if required not in fairway_groups:
            raise ValueError(f"fairway chart is missing FeatureGroup {required}")
    if "BERTHS" not in berth_groups:
        raise ValueError("berth chart is missing FeatureGroup BERTHS")

    root_attributes = {"version": reference_root.get("version", "1.0"), "totalMaps": "1"}
    if reference_root.get("exportTime"):
        root_attributes["exportTime"] = reference_root.get("exportTime")
    root = ET.Element("S57ChartData", root_attributes)
    chart_map = ET.SubElement(root, "Map", {"index": "0"})

    metadata = reference_metadata_map.find("Metadata")
    if metadata is None:
        raise ValueError(f"reference chart {args.reference} is missing Metadata")
    metadata = copy.deepcopy(metadata)
    dataset_name = metadata.find("DSID_DSNM")
    if dataset_name is None:
        dataset_name = ET.SubElement(metadata, "DSID_DSNM")
    dataset_name.text = args.output.name
    comment = metadata.find("DSID_COMT")
    if comment is not None:
        comment.text = "Reservoir generated chart: vector water/land, fairway, berths and reefs"
    chart_map.append(metadata)

    bounds = water_map.find("Bounds")
    if bounds is None:
        raise ValueError(f"water chart {args.water} is missing Bounds")
    chart_map.append(copy.deepcopy(bounds))

    features = ET.SubElement(chart_map, "Features")
    append_group(features, water_map, "DEPARE")
    append_group(features, water_map, "LNDARE")
    append_group(features, fairway_map, "TSSLPT")
    append_group(features, fairway_map, "TSSBND")
    append_group(features, fairway_map, "TSELNE")
    append_group(features, berths_map, "BERTHS")
    update_counts(features)
    total_features = sum(len(group.findall("Feature")) for group in features.findall("FeatureGroup"))
    ET.SubElement(features, "TotalFeatures").text = str(total_features)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(args.output, encoding="utf-8", xml_declaration=True)
    print(f"wrote {args.output} ({total_features} features)")


if __name__ == "__main__":
    main()
