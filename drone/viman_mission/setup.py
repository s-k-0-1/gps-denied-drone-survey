from glob import glob

from setuptools import setup

package_name = "viman_mission"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Maintainer",
    maintainer_email="maintainer@example.com",
    description="Autonomous RTAB-Map VIO mission stack for GPS-denied flight",
    license="MIT",
    entry_points={
        "console_scripts": [
            "auto_mission = viman_mission.auto_mission:main",
            "vision_bridge = viman_mission.vision_bridge:main",
            "rtabmap_trigger = viman_mission.rtabmap_trigger:main",
            "vio_gate = viman_mission.vio_gate:main",
            "mission_director = viman_mission.mission_director:main",
            "square_mission = viman_mission.square_mission:main",
            "survey_mission = viman_mission.survey_mission:main",
            "survey_boundary_director = viman_mission.survey_boundary_director:main",
            "rs_pipeline = viman_mission.rs_pipeline:main",
            "precision_land = viman_mission.precision_land:main",
            "whycode_mission = viman_mission.whycode_mission:main",
            "whycon_detector = viman_mission.whycon_detector:main",
            "whycode_detector = viman_mission.whycode_detector:main",
            "corner_survey_mission = viman_mission.corner_survey_mission:main",
            "yellow_boundary_detector = viman_mission.yellow_boundary_detector:main",
            "boundary_test_auto = viman_mission.boundary_test_auto:main",
            "corner1_test_auto = viman_mission.corner1_test_auto:main",
            "hsv_calibrate = viman_mission.hsv_calibrate:main",
            "boundary_guard = viman_mission.boundary_guard:main",
        ],
    },
)
