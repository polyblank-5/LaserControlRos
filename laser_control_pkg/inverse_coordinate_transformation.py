import yaml
import math

from typing import TypedDict, Literal


class ConstantsBase:
    # Coordinate limits
    x_min: int
    x_max: int
    y_min: int
    y_max: int
    z_min: int
    z_max: int

    # Right lower connection point
    Frx: int
    Frz: int
    Fry0: int
    Fry_min: int

    # Left lower connection point
    Flx: int
    Flz: int
    Fly0: int
    Fly_min: int

    # Upper motor center coordinates
    Moy: int
    Moz: int
    Fox: int

    # Upper connection null angle
    Mo0: int

    # Moving coordinate system - right lower connection point
    dmrx: float
    dmry: float
    dmrz: float

    # Moving coordinate system - left lower connection point
    dmlx: float
    dmly: float
    dmlz: float

    # Moving coordinate system - upper connection point
    dmox: int
    dmoy: float
    dmoz: float

    # Lengths
    ls: int
    lo: int
    lH: int

    # Moving coordinate system - laser focal point
    dLx: int
    dLy: int
    dLz: int

    # Laser field dimensions
    bLaserfeld: int
    lLaserfeld: int
    l0Laserfeld: int


class Constants(ConstantsBase):
    def __init__(self, config_path: str = "constants.yaml"):
        """
        Initialize the constants by loading them from a YAML file.
        Args:
            config_path (str): Path to the YAML configuration file.
        """
        # Load YAML file
        with open(config_path, "r") as file:
            data:dict = yaml.safe_load(file)

        # Assign constants as attributes of the class
        for key, value in data.items():
            setattr(self, key, value)

    def __repr__(self) -> str:
        """String representation of all constants for debugging."""
        return "\n".join(f"{key}: {value}" for key, value in self.__dict__.items())

class InverseCoordinateTransformation():
    def __init__(self, constants:Constants):
        self.C = constants
    def inv_kin_km(self, kmx, kmy, kmz = 0):
        """Inverse Kinematics calculation."""
        kmx = max(min(kmx, self.C.x_max), self.C.x_min)
        kmy = max(min(kmy, self.C.y_max), self.C.y_min)
        kmz = max(min(kmz, self.C.z_max), self.C.z_min)
        
        achswerte = [
            self.inv_kin_rechts(kmx, kmy, kmz),
            self.inv_kin_links(kmx, kmy, kmz),
            self.inv_kin_oben(kmx, kmy, kmz)
        ]
        return achswerte

    def inv_kin_oben(self, kmx, kmy, kmz):
        """Calculate the elevation angle."""
        moy_moy = -self.C.Moy + (kmy + self.C.dmoy)
        moz_moz = -self.C.Moz + (kmz + self.C.dmoz)
        distance = math.sqrt(moy_moy**2 + moz_moz**2)
        innenwinkel = math.acos((self.C.lH**2 + distance**2 - (self.C.lo**2 - (kmx + self.C.dmox - self.C.Fox)**2)) / (2 * self.C.lH * distance))
        innenwinkel_deg = math.degrees(innenwinkel)
        winkel_beta = math.asin(moy_moy / distance)
        winkel_beta_deg = math.degrees(winkel_beta)
        hoehenwinkel_deg = innenwinkel_deg + winkel_beta_deg

        # ATB_IGUS Convention
        hoehenwinkel_deg = self.C.Mo0 - hoehenwinkel_deg
        return hoehenwinkel_deg

    def inv_kin_rechts(self, kmx, kmy, kmz):
        """Inverse Kinematics for the right axis."""
        achse_rechts = kmy + self.C.dmry - math.sqrt(self.C.ls**2 - (self.C.Frx - (kmx + self.C.dmrx))**2 - (self.C.Frz - (kmz + self.C.dmrz))**2)

        # ATB_IGUS Convention
        achse_rechts = self.C.Fry0 - achse_rechts
        return achse_rechts

    def inv_kin_links(self, kmx, kmy, kmz):
        """Inverse Kinematics for the left axis."""
        achse_links = kmy + self.C.dmly - math.sqrt(self.C.ls**2 - (self.C.Flx - (kmx + self.C.dmlx))**2 - (self.C.Flz - (kmz + self.C.dmlz))**2)

        # ATB_IGUS Convention
        achse_links = self.C.Fly0 - achse_links
        return achse_links