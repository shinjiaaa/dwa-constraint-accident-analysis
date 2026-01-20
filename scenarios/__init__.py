"""
Scenario registry for S1/S2 only.

S1: Inevitable collision due to dynamics (feasible = ∅)
    Tests "inevitability" (불가피성) - whether safe alternatives existed

S2: Feasible exists but wrong decision due to cost structure (feasible ≠ ∅)
    Tests "choice error" (선택오류) - why a risky action was chosen despite safer alternatives
"""

from scenarios.s1_inevitable_dynamics import build_s1
from scenarios.s2_bad_decision import build_s2


def get_scenario(name: str):
    name = name.lower()
    if name == "s1":
        return build_s1()
    if name == "s2":
        return build_s2()
    raise ValueError("Unknown scenario. Use s1 or s2.")
