"""
Reports module for fillhead device.

This module provides report generation functionality for press operations,
generating beautiful interactive HTML reports with Plotly charts.
"""

from .press_report import generate_press_report, PressReportGenerator

__all__ = ['generate_press_report', 'PressReportGenerator']

