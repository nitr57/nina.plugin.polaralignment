import argparse
import json
import math
from pathlib import Path

import numpy as np
import scipy
from astropy import units as u
from astropy.coordinates import AltAz
from astropy.coordinates import EarthLocation
from astropy.coordinates import ICRS
from astropy.coordinates import SkyCoord
from astropy.time import Time
from astropy.utils import iers


# Keep the generator deterministic and offline-friendly.
iers.conf.auto_download = False
iers.conf.auto_max_age = None


PRESSURE_HPA = 1005.0
TEMPERATURE_C = 7.0
RELATIVE_HUMIDITY = 0.8
WAVELENGTH_MICRON = 0.574
MEASUREMENT_ROTATION_DEGREES = 30.0


def normalize_degrees(value):
    value = value % 360.0
    if value < 0:
        value += 360.0
    return value


def plugin_vector_from_altaz(azimuth_degrees, altitude_degrees):
    # The plugin uses x=north, y=west, z=up.
    azimuth_radians = math.radians(azimuth_degrees)
    altitude_radians = math.radians(altitude_degrees)
    cosine_altitude = math.cos(altitude_radians)
    return np.array([
        math.cos(azimuth_radians) * cosine_altitude,
        -math.sin(azimuth_radians) * cosine_altitude,
        math.sin(altitude_radians),
    ], dtype=float)


def altaz_from_plugin_vector(vector):
    x, y, z = vector
    horizontal_length = math.hypot(x, y)
    azimuth_degrees = math.degrees(math.atan2(-y, x))
    if azimuth_degrees < 0:
        azimuth_degrees += 360.0

    altitude_degrees = math.degrees(math.atan2(z, horizontal_length))
    return azimuth_degrees, altitude_degrees


def rodrigues(vector, axis, angle_degrees):
    axis = np.array(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    vector = np.array(vector, dtype=float)
    angle_radians = math.radians(angle_degrees)
    return vector * math.cos(angle_radians) + np.cross(axis, vector) * math.sin(angle_radians) + axis * np.dot(axis, vector) * (1.0 - math.cos(angle_radians))


def create_location(latitude_degrees, longitude_degrees, elevation_meters):
    return EarthLocation.from_geodetic(lat=latitude_degrees * u.deg,
                                       lon=longitude_degrees * u.deg,
                                       height=elevation_meters * u.m)


def create_altaz(observation_time, location, azimuth_degrees, altitude_degrees):
    return AltAz(obstime=observation_time,
                 location=location,
                 az=azimuth_degrees * u.deg,
                 alt=altitude_degrees * u.deg,
                 pressure=PRESSURE_HPA * u.hPa,
                 temperature=TEMPERATURE_C * u.deg_C,
                 relative_humidity=RELATIVE_HUMIDITY,
                 obswl=WAVELENGTH_MICRON * u.micron)


def icrs_from_plugin_vector(vector, observation_time, location):
    azimuth_degrees, altitude_degrees = altaz_from_plugin_vector(vector)
    altaz = create_altaz(observation_time, location, azimuth_degrees, altitude_degrees)
    return SkyCoord(altaz).transform_to(ICRS())


def plugin_vector_from_icrs(icrs_coordinate, observation_time, location):
    altaz = icrs_coordinate.transform_to(AltAz(obstime=observation_time,
                                               location=location,
                                               pressure=PRESSURE_HPA * u.hPa,
                                               temperature=TEMPERATURE_C * u.deg_C,
                                               relative_humidity=RELATIVE_HUMIDITY,
                                               obswl=WAVELENGTH_MICRON * u.micron))
    return plugin_vector_from_altaz(altaz.az.deg, altaz.alt.deg), altaz.az.deg, altaz.alt.deg


def mount_axis_vector(latitude_degrees, initial_azimuth_error_degrees, initial_altitude_error_degrees):
    if latitude_degrees >= 0:
        axis_azimuth_degrees = initial_azimuth_error_degrees
        axis_altitude_degrees = abs(latitude_degrees) + initial_altitude_error_degrees
    else:
        axis_azimuth_degrees = normalize_degrees(180.0 + initial_azimuth_error_degrees)
        axis_altitude_degrees = abs(latitude_degrees) - initial_altitude_error_degrees

    return plugin_vector_from_altaz(axis_azimuth_degrees, axis_altitude_degrees)


def coordinate_fixture(icrs_coordinate):
    return {
        "rightAscensionDegrees": float(icrs_coordinate.ra.deg),
        "declinationDegrees": float(icrs_coordinate.dec.deg),
    }


def generate_site_scenarios(site_name,
                            latitude_degrees,
                            longitude_degrees,
                            elevation_meters,
                            reference_time_utc,
                            observation_time_utc,
                            initial_azimuth_error_degrees,
                            initial_altitude_error_degrees,
                            residual_azimuth_error_degrees,
                            residual_altitude_error_degrees,
                            reference_altitudes_degrees,
                            reference_azimuths_degrees):
    location = create_location(latitude_degrees, longitude_degrees, elevation_meters)
    reference_time = Time(reference_time_utc, scale="utc")
    observation_time = Time(observation_time_utc, scale="utc")
    axis_vector = mount_axis_vector(latitude_degrees, initial_azimuth_error_degrees, initial_altitude_error_degrees)

    scenarios = []
    for reference_altitude_degrees in reference_altitudes_degrees:
        for reference_azimuth_degrees in reference_azimuths_degrees:
            # The third measurement point is the initial reference frame used by the continuous phase.
            third_vector = plugin_vector_from_altaz(reference_azimuth_degrees, reference_altitude_degrees)
            second_vector = rodrigues(third_vector, axis_vector, MEASUREMENT_ROTATION_DEGREES)
            first_vector = rodrigues(second_vector, axis_vector, MEASUREMENT_ROTATION_DEGREES)
            first_azimuth_degrees, first_altitude_degrees = altaz_from_plugin_vector(first_vector)
            second_azimuth_degrees, second_altitude_degrees = altaz_from_plugin_vector(second_vector)
            third_azimuth_degrees, third_altitude_degrees = altaz_from_plugin_vector(third_vector)

            first_coordinate = icrs_from_plugin_vector(first_vector, reference_time, location)
            second_coordinate = icrs_from_plugin_vector(second_vector, reference_time, location)
            third_coordinate = icrs_from_plugin_vector(third_vector, reference_time, location)

            reference_vector_now, _, _ = plugin_vector_from_icrs(third_coordinate, observation_time, location)

            delta_azimuth_degrees = initial_azimuth_error_degrees - residual_azimuth_error_degrees
            delta_altitude_degrees = initial_altitude_error_degrees - residual_altitude_error_degrees

            azimuth_adjusted_vector = rodrigues(reference_vector_now, np.array([0.0, 0.0, 1.0]), delta_azimuth_degrees)
            altitude_axis = rodrigues(np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0]), delta_azimuth_degrees)
            corrected_vector = rodrigues(azimuth_adjusted_vector, altitude_axis, delta_altitude_degrees)

            current_coordinate = icrs_from_plugin_vector(corrected_vector, observation_time, location)
            current_azimuth_degrees, current_altitude_degrees = altaz_from_plugin_vector(corrected_vector)

            scenarios.append({
                "name": f"{site_name}_Alt{reference_altitude_degrees:02.0f}_Az{reference_azimuth_degrees:03.0f}",
                "siteName": site_name,
                "referenceTimeUtc": reference_time.to_datetime(timezone=None).replace(tzinfo=None).isoformat() + "Z",
                "observationTimeUtc": observation_time.to_datetime(timezone=None).replace(tzinfo=None).isoformat() + "Z",
                "latitudeDegrees": latitude_degrees,
                "longitudeDegrees": longitude_degrees,
                "elevationMeters": elevation_meters,
                "initialAzimuthErrorDegrees": initial_azimuth_error_degrees,
                "initialAltitudeErrorDegrees": initial_altitude_error_degrees,
                "residualAzimuthErrorDegrees": residual_azimuth_error_degrees,
                "residualAltitudeErrorDegrees": residual_altitude_error_degrees,
                "firstAzimuthDegrees": first_azimuth_degrees,
                "firstAltitudeDegrees": first_altitude_degrees,
                "secondAzimuthDegrees": second_azimuth_degrees,
                "secondAltitudeDegrees": second_altitude_degrees,
                "referenceAzimuthDegrees": third_azimuth_degrees,
                "referenceAltitudeDegrees": third_altitude_degrees,
                "currentAzimuthDegrees": current_azimuth_degrees,
                "currentAltitudeDegrees": current_altitude_degrees,
                "minimumScenarioAltitudeDegrees": min(first_altitude_degrees, second_altitude_degrees, third_altitude_degrees, current_altitude_degrees),
                "firstPoint": coordinate_fixture(first_coordinate),
                "secondPoint": coordinate_fixture(second_coordinate),
                "thirdPoint": coordinate_fixture(third_coordinate),
                "currentPoint": coordinate_fixture(current_coordinate),
            })

    return scenarios


def generate_manifest():
    reference_altitudes_degrees = [1.0, 2.0, 3.0, 4.0, 5.0, 10.0, 20.0, 35.0, 50.0, 65.0, 80.0]
    reference_azimuths_degrees = list(range(0, 360, 30))

    scenarios = []
    scenarios.extend(generate_site_scenarios(site_name="North",
                                             latitude_degrees=48.0,
                                             longitude_degrees=7.0,
                                             elevation_meters=250.0,
                                             reference_time_utc="2024-10-01T21:00:00",
                                             observation_time_utc="2024-10-01T21:09:00",
                                             initial_azimuth_error_degrees=1.2,
                                             initial_altitude_error_degrees=-0.7,
                                             residual_azimuth_error_degrees=0.18,
                                             residual_altitude_error_degrees=-0.11,
                                             reference_altitudes_degrees=reference_altitudes_degrees,
                                             reference_azimuths_degrees=reference_azimuths_degrees))
    scenarios.extend(generate_site_scenarios(site_name="South",
                                             latitude_degrees=-33.0,
                                             longitude_degrees=151.0,
                                             elevation_meters=40.0,
                                             reference_time_utc="2024-11-01T10:30:00",
                                             observation_time_utc="2024-11-01T10:44:00",
                                             initial_azimuth_error_degrees=-0.9,
                                             initial_altitude_error_degrees=0.6,
                                             residual_azimuth_error_degrees=0.25,
                                             residual_altitude_error_degrees=-0.15,
                                             reference_altitudes_degrees=reference_altitudes_degrees,
                                             reference_azimuths_degrees=reference_azimuths_degrees))

    return {
        "generator": "tools/generate_continuous_oracle_sweep.py",
        "astropyVersion": "7.2.0",
        "numpyVersion": np.__version__,
        "scipyVersion": scipy.__version__,
        "pressureHPa": PRESSURE_HPA,
        "temperatureC": TEMPERATURE_C,
        "relativeHumidity": RELATIVE_HUMIDITY,
        "wavelengthMicron": WAVELENGTH_MICRON,
        "measurementRotationDegrees": MEASUREMENT_ROTATION_DEGREES,
        "scenarioCount": len(scenarios),
        "scenarios": scenarios,
    }


def main():
    parser = argparse.ArgumentParser(description="Generate Astropy-based oracle sweep fixtures for the TPPA continuous estimator tests.")
    parser.add_argument("output", type=Path, help="Output JSON path")
    args = parser.parse_args()

    manifest = generate_manifest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"Wrote {manifest['scenarioCount']} scenarios to {args.output}")


if __name__ == "__main__":
    main()
