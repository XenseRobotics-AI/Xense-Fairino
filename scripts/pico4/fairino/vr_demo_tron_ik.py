"""Model-only TRON2 adapter; optional wrist opt-out never marks real requests."""
from vr_tron_ik import TronIK


class TronDemoIK(TronIK):
    demo_only = True
