"""
Scenario registry for S1/S3 only.
"""

from scenarios.s1_inevitable_dynamics import build_s1
from scenarios.s3_bad_weighting import build_s3


def get_scenario(name: str):
    name = name.lower()
    if name == "s1":
        return build_s1()
    if name == "s3":
        return build_s3()
    raise ValueError("Unknown scenario. Use s1 or s3.")
