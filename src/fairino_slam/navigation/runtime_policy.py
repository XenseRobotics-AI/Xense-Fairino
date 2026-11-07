"""Shared limits for deployed navigation helpers (no ROS or desktop dependencies)."""

# Maximum age of the last navigation velocity command before latching a stop.
COMMAND_TIMEOUT = 0.6

# Combined map-to-base TF must be strictly younger than this limit.
TF_MAX_AGE = 1.0

# Read-only preflight waits and path previews; never resend a motion goal.
TF_READY_WAIT = 3.0
PREVIEW_MAX_ATTEMPTS = 3
PREVIEW_RETRY_DELAY = 0.5
PREVIEW_TIMEOUT = 40.0
