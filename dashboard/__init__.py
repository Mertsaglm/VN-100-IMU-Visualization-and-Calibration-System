"""
dashboard — real-time VN-100 visualization (pyqtgraph + PySide6).

Built on top of the pyvn100 library; the data source can be SimTransport
(no hardware) or SerialTransport (real STM32/VN-100).
"""
from .app import DashboardWindow, run

__all__ = ["DashboardWindow", "run"]
