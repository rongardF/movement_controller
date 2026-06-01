

class AbortPlanningError(Exception):
    """Exception raised to signal that the planning process was aborted by the requester (e.g. due to a cancel request)."""
    pass