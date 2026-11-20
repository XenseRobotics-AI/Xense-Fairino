"""Whole-body QP for the local model, including its optional wrist guard."""
from vr_fairino_ik import FairinoQPIK


class FairinoDemoIK(FairinoQPIK):
    demo_only = True
