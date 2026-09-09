from abc import ABC, abstractmethod

from endtools.model.dispensing_metrics_dto import DispensingMetricsDTO


class Controller(ABC):
    """Abstract base class for controllers."""

    @property
    @abstractmethod
    def is_dispensing(self) -> bool:
        """Return whether the controller is currently dispensing."""
        pass

    @abstractmethod
    def start_dispensing(self):
        """Start the controller."""
        pass

    @abstractmethod
    def stop_dispensing(self) -> DispensingMetricsDTO:
        """Stop the controller."""
        pass

    @abstractmethod
    def setup(self, config):
        """Set up the controller with the given configuration."""
        pass

    @abstractmethod
    def teardown(self):
        """Tear down the controller."""
        pass