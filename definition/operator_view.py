"""
Fillhead Operator View - Simplified operator interface for fillhead.

This provides a streamlined view focused on essential fill/injection operation:
- Large PASS/FAIL display
- Job/serial tracking
- Cycle time
- Injection metrics: volume dispensed, injection cycle count
- Vacuum/heater status indicators
- Press telemetry: force_motor_torque, current_pos, joules, endpoint, startpoint

Telemetry parsing uses FILLHEAD_TELEM: prefix; events use FILLHEAD_ prefix.
"""

import tkinter as tk

# Protocol prefixes for fillhead telemetry and events
FILLHEAD_TELEM_PREFIX = "FILLHEAD_TELEM: "
FILLHEAD_EVENT_PREFIX = "FILLHEAD_EVENT: "
from tkinter import ttk
import sys
import os
import time
import platform
import subprocess


def show_onscreen_keyboard():
    """Show the on-screen keyboard (Mac only)."""
    if platform.system() == "Darwin":
        try:
            # Use AppleScript to show the keyboard viewer from the input menu
            subprocess.Popen([
                "osascript", "-e",
                'tell application "System Events" to tell process "SystemUIServer" to click (menu bar item 1 of menu bar 1 whose description contains "text input")'
            ])
        except Exception as e:
            print(f"[FILLHEAD OPERATOR VIEW] Could not open on-screen keyboard: {e}")

# Adjust path for standalone execution
if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(os.path.dirname(current_dir))
    sys.path.insert(0, parent_dir)

from src import theme
try:
    from src.stats import get_stats, format_duration, format_cycle_time, format_yield
    HAS_STATS = True
except ImportError:
    HAS_STATS = False


def show_stats_window(parent):
    """Show a popup window with cycle statistics."""
    if not HAS_STATS:
        popup = tk.Toplevel(parent)
        popup.title("Cycle Statistics")
        popup.configure(bg=theme.BG_COLOR)
        tk.Label(popup, text="Stats module not available", font=(theme.FONT_FAMILY, 14),
                 bg=theme.BG_COLOR, fg=theme.FG_COLOR).pack(pady=20, padx=20)
        tk.Button(popup, text="Close", command=popup.destroy).pack(pady=10)
        return

    stats = get_stats()
    all_stats = stats.get_all_stats()

    # Create popup window
    popup = tk.Toplevel(parent)
    popup.title("Cycle Statistics")
    popup.configure(bg=theme.BG_COLOR)
    popup.geometry("500x600")
    popup.resizable(False, False)

    # Center on parent
    popup.transient(parent)
    popup.grab_set()

    # Title
    title = tk.Label(
        popup,
        text="Cycle Statistics",
        font=(theme.FONT_FAMILY, 24, 'bold'),
        foreground=theme.PRIMARY_ACCENT,
        bg=theme.BG_COLOR
    )
    title.pack(pady=(20, 20))

    # Stats container
    stats_frame = tk.Frame(popup, bg=theme.BG_COLOR)
    stats_frame.pack(fill=tk.BOTH, expand=True, padx=30, pady=10)

    def add_stat_row(frame, label, value, row):
        """Add a label-value row to the stats display."""
        lbl = tk.Label(
            frame,
            text=label,
            font=(theme.FONT_FAMILY, 14),
            foreground=theme.COMMENT_COLOR,
            bg=theme.BG_COLOR,
            anchor='w'
        )
        lbl.grid(row=row, column=0, sticky='w', pady=5)

        val = tk.Label(
            frame,
            text=str(value),
            font=(theme.FONT_FAMILY, 14, 'bold'),
            foreground=theme.FG_COLOR,
            bg=theme.BG_COLOR,
            anchor='e'
        )
        val.grid(row=row, column=1, sticky='e', pady=5, padx=(20, 0))

    # Configure grid columns
    stats_frame.columnconfigure(0, weight=1)
    stats_frame.columnconfigure(1, weight=1)

    row = 0

    # Section: Operations
    section1 = tk.Label(stats_frame, text="── Operations ──", font=(theme.FONT_FAMILY, 12, 'bold'),
                        foreground=theme.SECONDARY_ACCENT, bg=theme.BG_COLOR)
    section1.grid(row=row, column=0, columnspan=2, pady=(10, 5), sticky='w')
    row += 1

    add_stat_row(stats_frame, "Since Host Boot:", str(all_stats['operations_since_boot']), row)
    row += 1
    add_stat_row(stats_frame, "Total:", str(all_stats['operations_total']), row)
    row += 1

    # Section: Cycle Times
    section2 = tk.Label(stats_frame, text="── Cycle Times ──", font=(theme.FONT_FAMILY, 12, 'bold'),
                        foreground=theme.SECONDARY_ACCENT, bg=theme.BG_COLOR)
    section2.grid(row=row, column=0, columnspan=2, pady=(15, 5), sticky='w')
    row += 1

    add_stat_row(stats_frame, "Last Cycle:", format_cycle_time(all_stats['last_cycle_time']), row)
    row += 1
    # This Job Avg
    this_job_label = f"This Job Avg ({all_stats['current_job']}):" if all_stats['current_job'] else "This Job Avg:"
    add_stat_row(stats_frame, this_job_label, format_cycle_time(all_stats['job_average_cycle_time']), row)
    row += 1
    # Last Job Avg (only show if we have a last job)
    if all_stats['last_job']:
        last_job_label = f"Last Job Avg ({all_stats['last_job']}):"
        add_stat_row(stats_frame, last_job_label, format_cycle_time(all_stats['last_job_average_cycle_time']), row)
        row += 1
    # Only show "Last 100" if we have at least 1 sample
    if all_stats['sample_size_100'] > 0:
        add_stat_row(stats_frame, f"Avg (Last {all_stats['sample_size_100']}):",
                     format_cycle_time(all_stats['average_cycle_time_100']), row)
        row += 1
    # Only show "Last 1000" if we have at least 100 samples
    if all_stats['sample_size_1000'] >= 100:
        add_stat_row(stats_frame, f"Avg (Last {all_stats['sample_size_1000']}):",
                     format_cycle_time(all_stats['average_cycle_time_1000']), row)
        row += 1
    add_stat_row(stats_frame, "Avg (Total):", format_cycle_time(all_stats['average_cycle_time_total']), row)
    row += 1

    # Section: Yield
    section3 = tk.Label(stats_frame, text="── Yield ──", font=(theme.FONT_FAMILY, 12, 'bold'),
                        foreground=theme.SECONDARY_ACCENT, bg=theme.BG_COLOR)
    section3.grid(row=row, column=0, columnspan=2, pady=(15, 5), sticky='w')
    row += 1

    # This Job
    this_job_yield_label = f"This Job ({all_stats['current_job']}):" if all_stats['current_job'] else "This Job:"
    add_stat_row(stats_frame, this_job_yield_label, format_yield(all_stats['yield_job']), row)
    row += 1
    # Last Job (only show if we have a last job)
    if all_stats['last_job']:
        last_job_yield_label = f"Last Job ({all_stats['last_job']}):"
        add_stat_row(stats_frame, last_job_yield_label, format_yield(all_stats['yield_last_job']), row)
        row += 1
    # Only show "Last 100" if we have at least 1 sample
    if all_stats['sample_size_100'] > 0:
        add_stat_row(stats_frame, f"Last {all_stats['sample_size_100']}:", format_yield(all_stats['yield_100']), row)
        row += 1
    # Only show "Last 1000" if we have at least 100 samples
    if all_stats['sample_size_1000'] >= 100:
        add_stat_row(stats_frame, f"Last {all_stats['sample_size_1000']}:", format_yield(all_stats['yield_1000']), row)
        row += 1
    add_stat_row(stats_frame, "Total:", format_yield(all_stats['yield_total']), row)
    row += 1

    # Close button
    close_btn = tk.Button(
        popup,
        text="Close",
        command=popup.destroy,
        font=(theme.FONT_FAMILY, 14, 'bold'),
        bg=theme.WIDGET_BG,
        fg=theme.FG_COLOR,
        activebackground='#444444',
        activeforeground=theme.FG_COLOR,
        relief='raised',
        borderwidth=2,
        padx=30,
        pady=8,
        cursor='hand2',
        highlightthickness=0,  # Fix for macOS color rendering
        highlightbackground=theme.BG_COLOR
    )
    close_btn.pack(pady=(10, 20))

    # Center window on screen
    popup.update_idletasks()
    width = popup.winfo_width()
    height = popup.winfo_height()
    x = (popup.winfo_screenwidth() // 2) - (width // 2)
    y = (popup.winfo_screenheight() // 2) - (height // 2)
    popup.geometry(f'{width}x{height}+{x}+{y}')


def create_operator_view(parent, shared_gui_refs, view_id=None):
    """
    Create fillhead operator view content directly in parent frame.

    Args:
        parent: Parent frame to pack content into (should be already packed)
        shared_gui_refs: Shared GUI references dictionary
        view_id: Optional view ID to determine which view to create
    """
    print(f"[FILLHEAD OPERATOR_VIEW] create_operator_view called with view_id={view_id}")
    print(f"[FILLHEAD OPERATOR_VIEW] parent={parent}")

    # Route to appropriate view creator based on view_id
    if view_id == 'fill_operator_view':
        print(f"[FILLHEAD OPERATOR_VIEW] Creating fill operator view")
        create_fill_operator_view(parent, shared_gui_refs)
    elif view_id == 'injection_operator_view':
        print(f"[FILLHEAD OPERATOR_VIEW] Creating injection operator view")
        create_injection_operator_view(parent, shared_gui_refs)
    else:
        print(f"[FILLHEAD OPERATOR_VIEW] Creating fill operator view (default)")
        # Default: fill operator view
        create_fill_operator_view(parent, shared_gui_refs)


def create_fill_operator_view(parent, shared_gui_refs):
    """
    Create simplified fillhead operator view with job/serial tracking, PASS/FAIL indicator,
    injection metrics (volume dispensed, cycle count), vacuum/heater status, and press telemetry.
    Uses FILLHEAD_TELEM: prefix for telemetry and FILLHEAD_ prefix for events.

    Args:
        parent: Parent frame to pack content into (should be already packed)
        shared_gui_refs: Shared GUI references dictionary
    """
    print(f"[FILLHEAD OPERATOR VIEW] Creating fill operator view")
    print(f"[FILLHEAD OPERATOR VIEW] parent={parent}, exists={parent.winfo_exists()}")

    # Get serial manager from shared refs
    serial_manager = shared_gui_refs.get('serial_manager')

    # Ensure fillhead telemetry vars exist (FILLHEAD_TELEM: parsed fields)
    _ensure_fillhead_vars(shared_gui_refs)

    # PASS/FAIL Indicator Section (BIG and at the top)
    pass_fail_var = tk.StringVar(value='READY')
    pass_fail_label = tk.Label(
        parent,
        textvariable=pass_fail_var,
        font=(theme.FONT_FAMILY, 180, 'bold'),
        foreground=theme.COMMENT_COLOR,
        bg=theme.BG_COLOR
    )
    pass_fail_label.pack(pady=(40, 20))

    # Error/Warning display (centered, below PASS/FAIL) - only shows when there's an error/warning
    error_warning_var = tk.StringVar(value='')
    error_warning_label = tk.Label(
        parent,
        textvariable=error_warning_var,
        font=(theme.FONT_FAMILY, 24),
        foreground=theme.ERROR_RED,
        bg=theme.BG_COLOR
    )
    error_warning_label.pack(pady=(0, 40))

    # Separator after PASS/FAIL and error display
    separator_top = tk.Frame(parent, bg=theme.SECONDARY_ACCENT, height=3)
    separator_top.pack(fill=tk.X, pady=(0, 40), padx=100)

    # Input fields container - single grid for proper vertical alignment
    inputs_frame = tk.Frame(parent, bg=theme.BG_COLOR)
    inputs_frame.pack(pady=15)

    # Fixed label width for alignment (in characters)
    label_width = 15
    # Fixed button dimensions
    button_width = 14
    entry_width = 25
    # Common height styling for entries and buttons - make them taller
    entry_ipady = 18  # Internal padding to match button height
    button_ipady = 12  # Internal padding for buttons

    # Job Number Row
    job_label = tk.Label(
        inputs_frame,
        text="Job Number:",
        font=(theme.FONT_FAMILY, 24, 'bold'),
        foreground='white',
        bg=theme.BG_COLOR,
        anchor='e',
        width=label_width
    )
    job_label.grid(row=0, column=0, padx=(0, 20), pady=15, sticky='e')

    job_var = tk.StringVar(value=serial_manager.get_job() if serial_manager else '')
    job_entry = tk.Entry(
        inputs_frame,
        textvariable=job_var,
        font=(theme.FONT_FAMILY, 24),
        width=entry_width,
        bg=theme.WIDGET_BG,
        fg=theme.FG_COLOR,
        insertbackground=theme.PRIMARY_ACCENT,
        relief='solid',
        borderwidth=2,
        highlightthickness=2,
        highlightbackground='#555555',
        highlightcolor=theme.PRIMARY_ACCENT
    )
    job_entry.grid(row=0, column=1, padx=(0, 20), pady=15, ipady=entry_ipady, sticky='ew')

    # Show on-screen keyboard when entry is focused (Mac only)
    if platform.system() == "Darwin":
        job_entry.bind('<FocusIn>', lambda e: show_onscreen_keyboard())

    # Scanner target button for Job
    job_scanner_btn = tk.Button(
        inputs_frame,
        text="Scan Here",
        font=(theme.FONT_FAMILY, 20, 'bold'),
        bg=theme.WIDGET_BG,
        fg=theme.FG_COLOR,
        activebackground='#444444',
        activeforeground=theme.FG_COLOR,
        relief='raised',
        borderwidth=2,
        padx=25,
        cursor='hand2',
        width=button_width,
        highlightthickness=0,
        highlightbackground=theme.BG_COLOR
    )
    job_scanner_btn.grid(row=0, column=2, pady=15, ipady=button_ipady, sticky='nsew')

    def on_job_changed(*args):
        if serial_manager:
            serial_manager.set_job(job_var.get())
    job_var.trace_add('write', on_job_changed)

    # Serial Number Row
    serial_label = tk.Label(
        inputs_frame,
        text="Serial Number:",
        font=(theme.FONT_FAMILY, 24, 'bold'),
        foreground='white',
        bg=theme.BG_COLOR,
        anchor='e',
        width=label_width
    )
    serial_label.grid(row=1, column=0, padx=(0, 20), pady=15, sticky='e')

    serial_var = tk.StringVar(value=serial_manager.get_serial() if serial_manager else '')
    serial_entry = tk.Entry(
        inputs_frame,
        textvariable=serial_var,
        font=(theme.FONT_FAMILY, 24),
        width=entry_width,
        bg=theme.WIDGET_BG,
        fg=theme.FG_COLOR,
        insertbackground=theme.PRIMARY_ACCENT,
        relief='solid',
        borderwidth=2,
        highlightthickness=2,
        highlightbackground='#555555',
        highlightcolor=theme.PRIMARY_ACCENT
    )
    serial_entry.grid(row=1, column=1, padx=(0, 20), pady=15, ipady=entry_ipady, sticky='ew')

    # Show on-screen keyboard when entry is focused (Mac only)
    if platform.system() == "Darwin":
        serial_entry.bind('<FocusIn>', lambda e: show_onscreen_keyboard())

    # Scanner target button for Serial
    serial_scanner_btn = tk.Button(
        inputs_frame,
        text="Scan Here",
        font=(theme.FONT_FAMILY, 20, 'bold'),
        bg=theme.WIDGET_BG,
        fg=theme.FG_COLOR,
        activebackground='#444444',
        activeforeground=theme.FG_COLOR,
        relief='raised',
        borderwidth=2,
        padx=25,
        cursor='hand2',
        width=button_width,
        highlightthickness=0,
        highlightbackground=theme.BG_COLOR
    )
    serial_scanner_btn.grid(row=1, column=2, pady=15, ipady=button_ipady, sticky='nsew')

    # Update button appearances based on scanner target
    def update_scanner_buttons():
        current_target = serial_manager.get_scanner_target() if serial_manager else 'job'
        print(f"[FILLHEAD OPERATOR VIEW] Updating button states, current target: {current_target}")
        if current_target == 'job':
            job_scanner_btn.config(
                bg=theme.SUCCESS_GREEN,
                fg='white',
                activebackground=theme.SUCCESS_GREEN,
                activeforeground='white',
                relief='sunken'
            )
            serial_scanner_btn.config(
                bg=theme.WIDGET_BG,
                fg=theme.FG_COLOR,
                activebackground='#444444',
                activeforeground=theme.FG_COLOR,
                relief='raised'
            )
        else:  # serial
            job_scanner_btn.config(
                bg=theme.WIDGET_BG,
                fg=theme.FG_COLOR,
                activebackground='#444444',
                activeforeground=theme.FG_COLOR,
                relief='raised'
            )
            serial_scanner_btn.config(
                bg=theme.SUCCESS_GREEN,
                fg='white',
                activebackground=theme.SUCCESS_GREEN,
                activeforeground='white',
                relief='sunken'
            )

    # Scanner target button handlers
    def set_scanner_to_job():
        print(f"[FILLHEAD OPERATOR VIEW] Job button clicked")
        if serial_manager:
            serial_manager.set_scanner_target('job')
            update_scanner_buttons()

    def set_scanner_to_serial():
        print(f"[FILLHEAD OPERATOR VIEW] Serial button clicked")
        if serial_manager:
            serial_manager.set_scanner_target('serial')
            update_scanner_buttons()

    job_scanner_btn.config(command=set_scanner_to_job)
    serial_scanner_btn.config(command=set_scanner_to_serial)
    update_scanner_buttons()

    def on_serial_changed(*args):
        if serial_manager:
            serial_manager.set_serial(serial_var.get())
    serial_var.trace_add('write', on_serial_changed)

    # Metrics section: injection volume, cycle count, vacuum/heater, press telemetry
    metrics_frame = tk.Frame(parent, bg=theme.BG_COLOR)
    metrics_frame.pack(pady=(20, 10))

    # Injection metrics row
    inj_row = tk.Frame(metrics_frame, bg=theme.BG_COLOR)
    inj_row.pack(pady=5)
    tk.Label(inj_row, text="Volume Dispensed:", font=(theme.FONT_FAMILY, 16, 'bold'),
             foreground=theme.COMMENT_COLOR, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 10))
    vol_dispensed_var = shared_gui_refs.get('fillhead_injection_cumulative_ml_var',
                                            shared_gui_refs.get('fillhead_inject_cumulative_ml_var',
                                            shared_gui_refs.get('total_dispensed_var', tk.StringVar(value='--- ml'))))
    tk.Label(inj_row, textvariable=vol_dispensed_var, font=(theme.FONT_FAMILY, 16),
             foreground=theme.PRIMARY_ACCENT, bg=theme.BG_COLOR).pack(side=tk.LEFT)
    tk.Label(inj_row, text="  |  Cycle:", font=(theme.FONT_FAMILY, 16, 'bold'),
             foreground=theme.COMMENT_COLOR, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(20, 10))
    if 'fillhead_injection_cycle_count_var' not in shared_gui_refs:
        shared_gui_refs['fillhead_injection_cycle_count_var'] = tk.StringVar(value='0')
    inj_cycle_var = shared_gui_refs['fillhead_injection_cycle_count_var']
    tk.Label(inj_row, textvariable=inj_cycle_var, font=(theme.FONT_FAMILY, 16),
             foreground=theme.PRIMARY_ACCENT, bg=theme.BG_COLOR).pack(side=tk.LEFT)

    # Vacuum/heater status row
    vac_heater_row = tk.Frame(metrics_frame, bg=theme.BG_COLOR)
    vac_heater_row.pack(pady=5)
    vac_var = shared_gui_refs.get('fillhead_vacuum_state_var', tk.StringVar(value='---'))
    heater_var = shared_gui_refs.get('fillhead_heater_state_var', tk.StringVar(value='---'))
    tk.Label(vac_heater_row, text="Vacuum:", font=(theme.FONT_FAMILY, 14),
             foreground=theme.COMMENT_COLOR, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 5))
    vac_label = tk.Label(vac_heater_row, textvariable=vac_var, font=(theme.FONT_FAMILY, 14),
                         foreground=theme.FG_COLOR, bg=theme.BG_COLOR)
    vac_label.pack(side=tk.LEFT, padx=(0, 20))
    tk.Label(vac_heater_row, text="Heater:", font=(theme.FONT_FAMILY, 14),
             foreground=theme.COMMENT_COLOR, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 5))
    heater_label = tk.Label(vac_heater_row, textvariable=heater_var, font=(theme.FONT_FAMILY, 14),
                            foreground=theme.FG_COLOR, bg=theme.BG_COLOR)
    heater_label.pack(side=tk.LEFT)

    def vac_heater_tracer(*args):
        try:
            vac_state = vac_var.get().upper()
            heater_state = heater_var.get().upper()
            vac_color = theme.SUCCESS_GREEN if 'ON' in vac_state or 'ACTIVE' in vac_state else theme.COMMENT_COLOR
            heater_color = theme.SUCCESS_GREEN if 'ON' in heater_state or 'ACTIVE' in heater_state else theme.COMMENT_COLOR
            vac_label.config(foreground=vac_color)
            heater_label.config(foreground=heater_color)
        except tk.TclError:
            pass
    vac_var.trace_add('write', vac_heater_tracer)
    heater_var.trace_add('write', vac_heater_tracer)
    vac_heater_tracer()

    # Press telemetry row: force_motor_torque, current_pos, joules, endpoint, startpoint
    telemetry_row = tk.Frame(metrics_frame, bg=theme.BG_COLOR)
    telemetry_row.pack(pady=5)
    force_var = shared_gui_refs.get('fillhead_force_motor_torque_var',
                                     shared_gui_refs.get('fillhead_force_var', tk.StringVar(value='---')))
    pos_var = shared_gui_refs.get('fillhead_current_pos_var', tk.StringVar(value='---'))
    joules_var = shared_gui_refs.get('fillhead_joules_var', tk.StringVar(value='---'))
    endpoint_var = shared_gui_refs.get('fillhead_endpoint_var', tk.StringVar(value='---'))
    startpoint_var = shared_gui_refs.get('fillhead_startpoint_var', tk.StringVar(value='---'))
    tk.Label(telemetry_row, text="Force:", font=(theme.FONT_FAMILY, 12),
             foreground=theme.COMMENT_COLOR, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 5))
    tk.Label(telemetry_row, textvariable=force_var, font=(theme.FONT_FAMILY, 12),
             foreground=theme.PRIMARY_ACCENT, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 15))
    tk.Label(telemetry_row, text="Pos:", font=(theme.FONT_FAMILY, 12),
             foreground=theme.COMMENT_COLOR, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 5))
    tk.Label(telemetry_row, textvariable=pos_var, font=(theme.FONT_FAMILY, 12),
             foreground=theme.PRIMARY_ACCENT, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 15))
    tk.Label(telemetry_row, text="Joules:", font=(theme.FONT_FAMILY, 12),
             foreground=theme.COMMENT_COLOR, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 5))
    tk.Label(telemetry_row, textvariable=joules_var, font=(theme.FONT_FAMILY, 12),
             foreground=theme.PRIMARY_ACCENT, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 15))
    tk.Label(telemetry_row, text="End:", font=(theme.FONT_FAMILY, 12),
             foreground=theme.COMMENT_COLOR, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 5))
    tk.Label(telemetry_row, textvariable=endpoint_var, font=(theme.FONT_FAMILY, 12),
             foreground=theme.PRIMARY_ACCENT, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 15))
    tk.Label(telemetry_row, text="Start:", font=(theme.FONT_FAMILY, 12),
             foreground=theme.COMMENT_COLOR, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 5))
    tk.Label(telemetry_row, textvariable=startpoint_var, font=(theme.FONT_FAMILY, 12),
             foreground=theme.PRIMARY_ACCENT, bg=theme.BG_COLOR).pack(side=tk.LEFT)

    # Cycle time display (centered, below metrics)
    cycle_time_frame = tk.Frame(parent, bg=theme.BG_COLOR)
    cycle_time_frame.pack(pady=(30, 10))

    cycle_time_title = tk.Label(
        cycle_time_frame,
        text="Cycle Time:",
        font=(theme.FONT_FAMILY, 20, 'bold'),
        foreground=theme.COMMENT_COLOR,
        bg=theme.BG_COLOR
    )
    cycle_time_title.pack(side=tk.LEFT, padx=(0, 10))

    cycle_time_var = tk.StringVar(value='--:--')
    cycle_time_label = tk.Label(
        cycle_time_frame,
        textvariable=cycle_time_var,
        font=(theme.FONT_FAMILY, 20),
        foreground=theme.PRIMARY_ACCENT,
        bg=theme.BG_COLOR
    )
    cycle_time_label.pack(side=tk.LEFT)

    # Track cycle start time and injection cycle count
    cycle_start_time = [None]
    injection_cycle_count = [0]  # Incremented on each PASS

    # Track previous script state to detect transitions
    prev_running = [False]
    prev_held = [False]

    def update_pass_fail():
        """Update PASS/FAIL indicator, cycle time, injection count, and status based on script state.
        Parses FILLHEAD_ prefixed events and FILLHEAD_TELEM: telemetry from status.
        """
        try:
            script_runner = shared_gui_refs.get('script_runner')

            if script_runner is None:
                is_running = False
                is_held = False
                had_errors = False
            else:
                is_running = script_runner.is_running
                is_held = script_runner.is_held
                had_errors = getattr(script_runner, 'had_errors', False)

            # Detect state transitions
            just_started = not prev_running[0] and is_running
            just_stopped = prev_running[0] and not is_running
            just_held = not prev_held[0] and is_held

            # Track cycle time
            if just_started:
                cycle_start_time[0] = time.time()
                cycle_time_var.set('00:00')
                cycle_time_label.config(foreground=theme.BUSY_BLUE)
            elif is_running and not is_held and cycle_start_time[0] is not None:
                elapsed = time.time() - cycle_start_time[0]
                mins = int(elapsed // 60)
                secs = int(elapsed % 60)
                cycle_time_var.set(f'{mins:02d}:{secs:02d}')
            elif (just_stopped or just_held) and cycle_start_time[0] is not None:
                elapsed = time.time() - cycle_start_time[0]
                mins = int(elapsed // 60)
                secs = int(elapsed % 60)
                cycle_time_var.set(f'{mins:02d}:{secs:02d}')
                if had_errors or is_held:
                    cycle_time_label.config(foreground=theme.ERROR_RED)
                else:
                    cycle_time_label.config(foreground=theme.SUCCESS_GREEN)

            if is_running and not is_held:
                pass_fail_var.set('RUNNING')
                pass_fail_label.config(foreground=theme.BUSY_BLUE)
            elif is_held or just_held:
                pass_fail_var.set('FAIL')
                pass_fail_label.config(foreground=theme.ERROR_RED)
            elif just_stopped:
                if had_errors:
                    pass_fail_var.set('FAIL')
                    pass_fail_label.config(foreground=theme.ERROR_RED)
                else:
                    pass_fail_var.set('PASS')
                    pass_fail_label.config(foreground=theme.SUCCESS_GREEN)
                    injection_cycle_count[0] += 1
                    inj_cycle_var.set(str(injection_cycle_count[0]))
            elif not is_running and not is_held:
                if pass_fail_var.get() not in ['PASS', 'FAIL']:
                    pass_fail_var.set('READY')
                    pass_fail_label.config(foreground=theme.COMMENT_COLOR)

            prev_running[0] = is_running
            prev_held[0] = is_held

            # Update error/warning from status - parse FILLHEAD_ prefixed errors
            main_status_var = shared_gui_refs.get('status_var_fillhead') or shared_gui_refs.get('status_var')
            if main_status_var:
                status_text = main_status_var.get()

                # Check if reset was performed
                if 'reset' in status_text.lower() and ('complete' in status_text.lower() or 'DONE' in status_text):
                    pass_fail_var.set('READY')
                    pass_fail_label.config(foreground=theme.COMMENT_COLOR)
                    error_warning_var.set('')
                    cycle_time_var.set('--:--')
                    cycle_time_label.config(foreground=theme.PRIMARY_ACCENT)
                    cycle_start_time[0] = None
                # Parse FILLHEAD_ERROR:, FILLHEAD_WARNING:, or generic ERROR:/WARNING:
                elif 'FILLHEAD_ERROR:' in status_text or 'FILLHEAD_WARNING:' in status_text or '_ERROR:' in status_text or '_WARNING:' in status_text or 'ERROR:' in status_text or 'WARNING:' in status_text:
                    display_text = status_text
                    for prefix in ('FILLHEAD_ERROR:', 'FILLHEAD_WARNING:', '_ERROR:', '_WARNING:', 'ERROR:', 'WARNING:'):
                        idx = display_text.find(prefix)
                        if idx >= 0:
                            display_text = display_text[idx + len(prefix):].strip()
                            break
                    error_warning_var.set(display_text)
                    error_warning_label.config(foreground=theme.ERROR_RED)
                elif pass_fail_var.get() == 'READY':
                    error_warning_var.set('')

        except Exception as e:
            print(f"[FILLHEAD OPERATOR VIEW] Error updating PASS/FAIL: {e}")

    # Poll script state periodically (every 100ms)
    def poll_state():
        try:
            update_pass_fail()
        except Exception as e:
            print(f"[FILLHEAD OPERATOR VIEW] Polling error: {e}")
        if parent.winfo_exists():
            parent.after(100, poll_state)

    # Initialize injection cycle count display
    inj_cycle_var.set(str(injection_cycle_count[0]))

    # Stats button (if available)
    if HAS_STATS:
        stats_btn = tk.Button(
            parent,
            text="Cycle Statistics",
            command=lambda: show_stats_window(parent),
            font=(theme.FONT_FAMILY, 14),
            bg=theme.WIDGET_BG,
            fg=theme.FG_COLOR,
            activebackground='#444444',
            activeforeground=theme.FG_COLOR,
            relief='raised',
            borderwidth=2,
            padx=20,
            pady=6,
            cursor='hand2',
            highlightthickness=0,
            highlightbackground=theme.BG_COLOR
        )
        stats_btn.pack(pady=(10, 20))

    print(f"[FILLHEAD OPERATOR VIEW] Starting PASS/FAIL polling")
    poll_state()

    print(f"[FILLHEAD OPERATOR VIEW] Content created directly in parent frame")
    print(f"[FILLHEAD OPERATOR VIEW] Parent has {len(parent.winfo_children())} children")


def _ensure_fillhead_vars(shared_gui_refs):
    """Ensure fillhead telemetry variables exist (FILLHEAD_TELEM: parsed fields)."""
    vars_to_ensure = [
        'fillhead_injection_cumulative_ml_var', 'fillhead_injection_active_ml_var',
        'fillhead_injection_cycle_count_var', 'fillhead_vacuum_state_var',
        'fillhead_heater_state_var', 'fillhead_force_motor_torque_var',
        'fillhead_current_pos_var', 'fillhead_joules_var',
        'fillhead_endpoint_var', 'fillhead_startpoint_var',
        'fillhead_force_var', 'fillhead_inject_cumulative_ml_var',
        'total_dispensed_var'
    ]
    for name in vars_to_ensure:
        if name not in shared_gui_refs:
            if 'torque' in name or 'pos' in name or 'joules' in name or 'ml' in name:
                shared_gui_refs[name] = tk.StringVar(value='---')
            elif 'cycle_count' in name:
                shared_gui_refs[name] = tk.StringVar(value='0')
            else:
                shared_gui_refs[name] = tk.StringVar(value='---')


def create_injection_operator_view(parent, shared_gui_refs):
    """
    Create simplified injection operator view with volume dispensed, cycle count,
    vacuum/heater status, and PASS/FAIL.

    Args:
        parent: Parent frame to pack content into (should be already packed)
        shared_gui_refs: Shared GUI references dictionary
    """
    print(f"[FILLHEAD OPERATOR VIEW] Creating injection operator view")

    _ensure_fillhead_vars(shared_gui_refs)

    # PASS/FAIL
    pass_fail_var = tk.StringVar(value='READY')
    pass_fail_label = tk.Label(
        parent,
        textvariable=pass_fail_var,
        font=(theme.FONT_FAMILY, 120, 'bold'),
        foreground=theme.COMMENT_COLOR,
        bg=theme.BG_COLOR
    )
    pass_fail_label.pack(pady=(30, 20))

    # Volume dispensed (large)
    vol_frame = tk.Frame(parent, bg=theme.BG_COLOR)
    vol_frame.pack(pady=15)
    tk.Label(vol_frame, text="Volume Dispensed:", font=(theme.FONT_FAMILY, 20, 'bold'),
             foreground=theme.COMMENT_COLOR, bg=theme.BG_COLOR).pack()
    vol_var = shared_gui_refs.get('fillhead_injection_cumulative_ml_var',
                                   shared_gui_refs.get('fillhead_inject_cumulative_ml_var',
                                   shared_gui_refs.get('total_dispensed_var', tk.StringVar(value='--- ml'))))
    tk.Label(vol_frame, textvariable=vol_var, font=(theme.FONT_FAMILY, 36, 'bold'),
             foreground=theme.PRIMARY_ACCENT, bg=theme.BG_COLOR).pack()

    # Injection cycle count
    cycle_frame = tk.Frame(parent, bg=theme.BG_COLOR)
    cycle_frame.pack(pady=10)
    tk.Label(cycle_frame, text="Injection Cycles:", font=(theme.FONT_FAMILY, 18, 'bold'),
             foreground=theme.COMMENT_COLOR, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 10))
    inj_cycle_var = shared_gui_refs.get('fillhead_injection_cycle_count_var', tk.StringVar(value='0'))
    tk.Label(cycle_frame, textvariable=inj_cycle_var, font=(theme.FONT_FAMILY, 18),
             foreground=theme.PRIMARY_ACCENT, bg=theme.BG_COLOR).pack(side=tk.LEFT)

    # Vacuum/heater status
    status_frame = tk.Frame(parent, bg=theme.BG_COLOR)
    status_frame.pack(pady=15)
    vac_var = shared_gui_refs.get('fillhead_vacuum_state_var', tk.StringVar(value='---'))
    heater_var = shared_gui_refs.get('fillhead_heater_state_var', tk.StringVar(value='---'))
    tk.Label(status_frame, text="Vacuum:", font=(theme.FONT_FAMILY, 14),
             foreground=theme.COMMENT_COLOR, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 5))
    tk.Label(status_frame, textvariable=vac_var, font=(theme.FONT_FAMILY, 14),
             foreground=theme.FG_COLOR, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 25))
    tk.Label(status_frame, text="Heater:", font=(theme.FONT_FAMILY, 14),
             foreground=theme.COMMENT_COLOR, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 5))
    tk.Label(status_frame, textvariable=heater_var, font=(theme.FONT_FAMILY, 14),
             foreground=theme.FG_COLOR, bg=theme.BG_COLOR).pack(side=tk.LEFT)

    # Job/serial (compact)
    serial_manager = shared_gui_refs.get('serial_manager')
    job_var = tk.StringVar(value=serial_manager.get_job() if serial_manager else '')
    serial_var = tk.StringVar(value=serial_manager.get_serial() if serial_manager else '')
    job_serial_frame = tk.Frame(parent, bg=theme.BG_COLOR)
    job_serial_frame.pack(pady=10)
    tk.Label(job_serial_frame, text="Job:", font=(theme.FONT_FAMILY, 12),
             foreground=theme.COMMENT_COLOR, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 5))
    tk.Label(job_serial_frame, textvariable=job_var, font=(theme.FONT_FAMILY, 12),
             foreground=theme.FG_COLOR, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 20))
    tk.Label(job_serial_frame, text="Serial:", font=(theme.FONT_FAMILY, 12),
             foreground=theme.COMMENT_COLOR, bg=theme.BG_COLOR).pack(side=tk.LEFT, padx=(0, 5))
    tk.Label(job_serial_frame, textvariable=serial_var, font=(theme.FONT_FAMILY, 12),
             foreground=theme.FG_COLOR, bg=theme.BG_COLOR).pack(side=tk.LEFT)


def create_generic_operator_view(parent, shared_gui_refs):
    """
    Create generic fillhead operator view content (placeholder for future use).

    Args:
        parent: Parent frame to pack content into (should be already packed)
        shared_gui_refs: Shared GUI references dictionary
    """
    tk.Label(
        parent,
        text="Generic Operator View\n(Use 'fill operator view' or 'injection operator view')",
        font=(theme.FONT_FAMILY, 20, 'bold'),
        foreground='#B0A3D4',  # Lavender
        bg=theme.BG_COLOR
    ).pack(expand=True, pady=50)


if __name__ == "__main__":
    # Test the operator view
    root = tk.Tk()
    root.title("Fillhead Operator View Test")
    root.geometry("800x600")
    root.configure(bg=theme.BG_COLOR)

    # Create test shared GUI refs
    test_refs = {}

    # Create test container
    container = tk.Frame(root, bg=theme.BG_COLOR)
    container.pack(fill=tk.BOTH, expand=True)

    # Create fill operator view
    create_fill_operator_view(container, test_refs)

    root.mainloop()
