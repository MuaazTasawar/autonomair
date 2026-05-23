import math


class FormationController:
    """
    Calculates target GPS positions for each drone in a formation
    relative to a lead drone's position and heading.

    Supported formations:
        - line      : drones spaced in a straight line behind the lead
        - triangle  : lead at front, two wingmen behind
        - v_shape   : classic V formation
        - circle    : drones evenly spaced around a central point
    """

    FORMATIONS = ["line", "triangle", "v_shape", "circle"]

    def __init__(self, config):
        self.spacing = config.get("formation_spacing", 10)  # metres between drones

    # ------------------------------------------------------------------ #
    #  Public API                                                          #
    # ------------------------------------------------------------------ #

    def get_positions(self, formation, lead_lat, lead_lon, lead_alt, heading_deg, num_drones):
        """
        Returns a list of (lat, lon, alt) tuples for each drone
        including the lead drone at index 0.

        Args:
            formation   : one of FORMATIONS
            lead_lat    : lead drone latitude
            lead_lon    : lead drone longitude
            lead_alt    : altitude for all drones (metres)
            heading_deg : lead drone heading in degrees (0 = North)
            num_drones  : total number of drones including lead

        Returns:
            list of dicts: [{"lat": ..., "lon": ..., "alt": ...}, ...]
        """
        if formation not in self.FORMATIONS:
            raise ValueError(f"[FormationController] Unknown formation '{formation}'. "
                             f"Choose from {self.FORMATIONS}")

        method = getattr(self, f"_formation_{formation}")
        offsets = method(num_drones)  # list of (north_m, east_m) offsets

        positions = []
        for north_m, east_m in offsets:
            # Rotate offsets by heading so formation faces direction of travel
            rotated_n, rotated_e = self._rotate(north_m, east_m, heading_deg)
            lat, lon = self._offset_position(lead_lat, lead_lon, rotated_n, rotated_e)
            positions.append({"lat": lat, "lon": lon, "alt": lead_alt})

        return positions

    # ------------------------------------------------------------------ #
    #  Formation definitions (offsets in metres, NED frame)               #
    # ------------------------------------------------------------------ #

    def _formation_line(self, num_drones):
        """
        Lead drone at front, others spaced directly behind.
        Offsets: (north, east)
        """
        offsets = [(0, 0)]  # lead
        for i in range(1, num_drones):
            offsets.append((-self.spacing * i, 0))
        return offsets

    def _formation_triangle(self, num_drones):
        """
        Lead at apex, wingmen spread behind left and right.
        Works best with 3 drones.
        """
        offsets = [(0, 0)]  # lead
        left_right = [
            (-self.spacing, -self.spacing),
            (-self.spacing,  self.spacing),
        ]
        for i in range(1, num_drones):
            if i - 1 < len(left_right):
                offsets.append(left_right[i - 1])
            else:
                # Extra drones go in a second row
                row = (i - 1) // 2 + 1
                side = -self.spacing if (i % 2 == 0) else self.spacing
                offsets.append((-self.spacing * (row + 1), side))
        return offsets

    def _formation_v_shape(self, num_drones):
        """
        Classic V — lead at front, drones spread diagonally behind.
        """
        offsets = [(0, 0)]  # lead
        for i in range(1, num_drones):
            side = i // 2 + 1
            direction = -1 if i % 2 == 0 else 1
            offsets.append((-self.spacing * side, self.spacing * side * direction))
        return offsets

    def _formation_circle(self, num_drones):
        """
        All drones equally spaced around a circle.
        Lead drone is at the northernmost point.
        """
        radius = self.spacing
        offsets = []
        for i in range(num_drones):
            angle = (2 * math.pi / num_drones) * i - math.pi / 2
            north = radius * math.cos(angle)
            east = radius * math.sin(angle)
            offsets.append((north, east))
        return offsets

    # ------------------------------------------------------------------ #
    #  Geometry utilities                                                  #
    # ------------------------------------------------------------------ #

    def _rotate(self, north, east, heading_deg):
        """Rotate a NED offset vector by a heading angle."""
        angle = math.radians(heading_deg)
        rotated_n = north * math.cos(angle) - east * math.sin(angle)
        rotated_e = north * math.sin(angle) + east * math.cos(angle)
        return rotated_n, rotated_e

    def _offset_position(self, lat, lon, north_m, east_m):
        """Convert metre offsets to GPS coordinates."""
        new_lat = lat + (north_m / 111320)
        new_lon = lon + (east_m / (111320 * math.cos(math.radians(lat))))
        return new_lat, new_lon