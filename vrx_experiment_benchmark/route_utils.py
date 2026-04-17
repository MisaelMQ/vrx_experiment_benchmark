import math
import yaml
from typing import Dict, List, Tuple, Any


# Mode state enumerations
MODE_TO_CODE = {
    "start": 0,
    "transit": 1,
    "finish": 2,
}

CODE_TO_MODE = {v: k for k, v in MODE_TO_CODE.items()}


EARTH_RADIUS_M = 6378137.0


# Convert geodetic coordinates to local Cartesian (XY)
def geodetic_to_local_xy(
    lat_deg: float,
    lon_deg: float,
    lat0_deg: float,
    lon0_deg: float
) -> Tuple[float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)

    dlat = lat - lat0
    dlon = lon - lon0

    x = EARTH_RADIUS_M * dlon * math.cos(lat0)
    y = EARTH_RADIUS_M * dlat
    return x, y


# Wrap angle to range [-pi, pi]
def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


# Unwrap angle using a previous reference
def unwrap_angle(new_angle: float, prev_angle: float) -> float:
    delta = wrap_angle(new_angle - prev_angle)
    return prev_angle + delta


# Extract yaw angle from a quaternion
def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


# Load route data directly from YAML
def load_route_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# Compute the geometric orientation of a segment
def segment_yaw(x0: float, y0: float, x1: float, y1: float) -> float:
    return math.atan2(y1 - y0, x1 - x0)


# Project waypoint geodetics to a local reference origin
def convert_waypoint_latlon_to_xy(
    waypoint: Dict[str, Any],
    origin_lat: float,
    origin_lon: float
) -> Dict[str, Any]:
    if "lat" not in waypoint or "lon" not in waypoint:
        raise KeyError("Each waypoint must contain 'lat' and 'lon'")

    x, y = geodetic_to_local_xy(
        float(waypoint["lat"]),
        float(waypoint["lon"]),
        origin_lat,
        origin_lon,
    )

    wp = dict(waypoint)
    wp["x"] = float(x)
    wp["y"] = float(y)
    return wp


# Process an entire route calculating XY coordinates and segment orientations
def compute_local_route_from_latlon(
    route_data: Dict[str, Any],
    origin_lat: float,
    origin_lon: float
) -> Dict[str, Any]:
    if "waypoints" not in route_data:
        raise KeyError("Route YAML must contain 'waypoints'")

    local_route = {
        "route_id": route_data.get("route_id", "unknown_route"),
        "frame_id": route_data.get("frame_id", "map_local"),
        "origin_lat": float(origin_lat),
        "origin_lon": float(origin_lon),
        "waypoints": [],
    }

    waypoints = route_data["waypoints"]
    if len(waypoints) == 0:
        return local_route

    local_waypoints: List[Dict[str, Any]] = [
        convert_waypoint_latlon_to_xy(wp, origin_lat, origin_lon) for wp in waypoints
    ]

    for i, wp in enumerate(local_waypoints):
        wp_local = dict(wp)

        if i < len(local_waypoints) - 1:
            nxt = local_waypoints[i + 1]
            wp_local["path_yaw"] = segment_yaw(
                wp_local["x"], wp_local["y"], nxt["x"], nxt["y"]
            )
        elif len(local_waypoints) >= 2:
            prev = local_waypoints[i - 1]
            wp_local["path_yaw"] = segment_yaw(
                prev["x"], prev["y"], wp_local["x"], wp_local["y"]
            )
        else:
            wp_local["path_yaw"] = 0.0

        local_route["waypoints"].append(wp_local)

    return local_route


# Compute 2D Euclidean distance
def distance_xy(x0: float, y0: float, x1: float, y1: float) -> float:
    return math.hypot(x1 - x0, y1 - y0)