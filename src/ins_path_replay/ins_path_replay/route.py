"""WGS84 ENU projection and equal-arc sampling, independent of ROS."""
import bisect
import csv
from dataclasses import dataclass
import math
from pathlib import Path


@dataclass(frozen=True)
class Route:
    raw: tuple
    points: tuple
    headings: tuple
    sample_arcs: tuple
    metadata: dict


def ecef(lat, lon, altitude):
    a, e2 = 6378137.0, 6.6943799901413165e-3
    n = a / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
    return ((n + altitude) * math.cos(lat) * math.cos(lon),
            (n + altitude) * math.cos(lat) * math.sin(lon),
            (n * (1.0 - e2) + altitude) * math.sin(lat))


def local_enu(lat, lon, altitude, origin):
    """Inputs radians/metres; x east, y north, z up."""
    lat0, lon0, alt0 = origin
    xyz, ref = ecef(lat, lon, altitude), ecef(lat0, lon0, alt0)
    dx, dy, dz = (a - b for a, b in zip(xyz, ref))
    return (-math.sin(lon0) * dx + math.cos(lon0) * dy,
            -math.sin(lat0) * math.cos(lon0) * dx
            - math.sin(lat0) * math.sin(lon0) * dy + math.cos(lat0) * dz,
            math.cos(lat0) * math.cos(lon0) * dx
            + math.cos(lat0) * math.sin(lon0) * dy + math.sin(lat0) * dz)


def sample_equal_arc(points, count=30):
    if count < 2:
        raise ValueError('point_count must be at least 2')
    clean, arcs = [], []
    for x, y in points:
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError('Non-finite projected coordinate')
        if not clean:
            clean.append((x, y)); arcs.append(0.0)
        else:
            ds = math.hypot(x - clean[-1][0], y - clean[-1][1])
            if ds > 1e-8:
                clean.append((x, y)); arcs.append(arcs[-1] + ds)
    if len(clean) < 2:
        raise ValueError('Recording contains no nonzero travel')
    samples, sample_arcs = [], []
    for i in range(count):
        s = arcs[-1] * i / (count - 1)
        j = min(len(clean) - 1, max(1, bisect.bisect_left(arcs, s)))
        alpha = (s - arcs[j - 1]) / (arcs[j] - arcs[j - 1])
        samples.append(tuple(a + alpha * (b - a) for a, b in zip(clean[j-1], clean[j])))
        sample_arcs.append(s)
    samples[0], samples[-1] = clean[0], clean[-1]
    headings = [math.atan2(b[1]-a[1], b[0]-a[0]) for a, b in zip(samples, samples[1:])]
    if any(math.dist(a, b) < 1e-6 for a, b in zip(samples, samples[1:])):
        raise ValueError('Adjacent resampled points coincide; heading is undefined')
    headings.append(headings[-1])
    return tuple(samples), tuple(headings), tuple(sample_arcs)


def load_route(csv_path, count=30, latitude_column='INS_LAT', longitude_column='INS_LON',
               altitude_column='ins_altitude', angle_unit='radians'):
    if angle_unit not in ('radians', 'degrees'):
        raise ValueError('angle_unit must be radians or degrees (never guessed)')
    raw, origin, times, jumps = [], None, [], []
    factor = math.pi / 180 if angle_unit == 'degrees' else 1.0
    with open(csv_path, newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream)
        for column in (latitude_column, longitude_column, altitude_column):
            if column not in (reader.fieldnames or []):
                raise ValueError(f'Missing CSV column: {column}')
        for row_number, row in enumerate(reader, 2):
            try:
                lat, lon = float(row[latitude_column])*factor, float(row[longitude_column])*factor
                alt = float(row[altitude_column])
                if not all(map(math.isfinite, (lat, lon, alt))) or abs(lat) >= math.pi/2 or abs(lon) > math.pi:
                    raise ValueError('invalid latitude, longitude or altitude')
                if origin is None:
                    origin = (lat, lon, alt)
                east, north, _ = local_enu(lat, lon, alt, origin)
                if raw:
                    jumps.append(math.dist(raw[-1], (east, north)))
                raw.append((east, north))
                if row.get('time'):
                    times.append(float(row['time']))
            except (ValueError, TypeError) as error:
                raise ValueError(f'CSV row {row_number}: {error}') from error
    points, headings, arcs = sample_equal_arc(raw, count)
    return Route(tuple(raw), points, headings, arcs, {
        'csv_path': str(Path(csv_path).resolve()), 'rows': len(raw), 'point_count': count,
        'latitude_column': latitude_column, 'longitude_column': longitude_column,
        'altitude_column': altitude_column, 'angle_unit': angle_unit,
        'origin_latitude_deg': math.degrees(origin[0]),
        'origin_longitude_deg': math.degrees(origin[1]), 'origin_altitude_m': origin[2],
        'raw_length_m': arcs[-1], 'sample_arc_spacing_m': arcs[-1]/(count-1),
        'sparse_length_m': sum(math.dist(a,b) for a,b in zip(points, points[1:])),
        'recording_duration_s': times[-1]-times[0] if times else None,
        'maximum_raw_step_m': max(jumps), 'raw_steps_over_2m': sum(d > 2 for d in jumps),
        'frame_convention': 'WGS84 ENU tangent at first CSV row; x east, y north',
        'last_heading': 'same as incoming final segment; no subsequent point exists',
        'preprocessing': 'no smoothing or outlier removal; duplicate positions add zero arc',
    })


def distance_to_polyline(x, y, points):
    best = float('inf')
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx, dy = bx-ax, by-ay
        t = max(0.0, min(1.0, ((x-ax)*dx+(y-ay)*dy)/max(1e-16, dx*dx+dy*dy)))
        best = min(best, math.hypot(x-ax-t*dx, y-ay-t*dy))
    return best


def decimate(points, spacing=0.5):
    result = [points[0]]
    for p in points[1:-1]:
        if math.dist(p, result[-1]) >= spacing:
            result.append(p)
    result.append(points[-1])
    return result
