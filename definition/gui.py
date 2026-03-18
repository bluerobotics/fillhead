import tkinter as tk
from tkinter import ttk
import sys
import os

if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(os.path.dirname(current_dir))
    sys.path.insert(0, parent_dir)

from src import theme

# --- GUI Helper Functions ---

def draw_vertical_text(canvas, x, y, text, font, fill, anchor="center"):
    """Draw text vertically without using the angle parameter (for compatibility with older Tk)."""
    char_height = 12
    if isinstance(font, tuple) and len(font) >= 2:
        try:
            font_size = int(font[1])
            char_height = max(font_size + 2, 10)
        except (ValueError, TypeError):
            pass
    total_height = len(text) * char_height
    start_y = y - total_height // 2 if anchor == "center" else y
    for i, char in enumerate(text):
        char_y = start_y + (i * char_height)
        canvas.create_text(x, char_y, text=char, font=font, fill=fill, anchor="center")

def make_homed_tracer(var, label_to_color):
    """Changes a label's color based on 'homed' status."""
    def tracer(*args):
        try:
            state = var.get().lower()
            if state == 'homed':
                label_to_color.config(foreground=theme.SUCCESS_GREEN)
            else:
                label_to_color.config(foreground=theme.ERROR_RED)
        except tk.TclError:
            pass
    return tracer

def make_torque_tracer(double_var, string_var):
    """Updates a string variable with a percentage from a double variable."""
    def tracer(*args):
        try:
            val_raw = double_var.get()
            val_float = float(str(val_raw).split()[0])
            string_var.set(f"{int(val_float)}%")
        except (tk.TclError, ValueError, IndexError):
            string_var.set("ERR")
    return tracer

def make_state_tracer(var, label_to_color):
    """Changes a label's color based on general device state."""
    def tracer(*args):
        try:
            state = var.get().upper()
            color = theme.FG_COLOR
            if "STANDBY" in state or "IDLE" in state:
                color = theme.SUCCESS_GREEN
            elif "BUSY" in state or "ACTIVE" in state or "HOMING" in state or "MOVING" in state or "PRESSING" in state:
                color = theme.BUSY_BLUE
            elif "ERROR" in state:
                color = theme.ERROR_RED
            elif "DISABLED" in state:
                color = theme.ERROR_RED
            elif "ENABLED" in state:
                color = theme.SUCCESS_GREEN
            label_to_color.config(foreground=color)
        except tk.TclError:
            pass
    return tracer

def make_on_off_tracer(var, *labels_with_colors):
    """Changes label colors for ON/OFF states."""
    def tracer(*args):
        try:
            state = var.get().upper()
            is_on = "ON" in state or "ACTIVE" in state
            for label, on_color, off_color in labels_with_colors:
                color = on_color if is_on else off_color
                label.config(foreground=color)
        except tk.TclError:
            pass
    return tracer

def make_heater_value_tracer(var, label_to_color):
    def tracer(*args):
        try:
            value_str = var.get().split()[0]
            temp = float(value_str)
            color = theme.SUCCESS_GREEN if 0 <= temp <= 200 else theme.FG_COLOR
            label_to_color.config(foreground=color)
        except (ValueError, IndexError, tk.TclError):
            label_to_color.config(foreground=theme.FG_COLOR)
    return tracer

def make_vacuum_value_tracer(var, label_to_color):
    def tracer(*args):
        try:
            value_str = var.get().split()[0]
            pressure = float(value_str)
            color = theme.SUCCESS_GREEN if -15 <= pressure <= 1 else theme.FG_COLOR
            label_to_color.config(foreground=color)
        except (ValueError, IndexError, tk.TclError):
            label_to_color.config(foreground=theme.FG_COLOR)
    return tracer

def make_force_tracer(var, label_to_color):
    """Changes color based on force magnitude (in kg)."""
    def tracer(*args):
        try:
            try:
                value_str = var.get().split()[0]
                force = float(value_str)
                if force < 100:
                    color = theme.FG_COLOR
                elif force < 500:
                    color = theme.SUCCESS_GREEN
                elif force < 800:
                    color = theme.WARNING_YELLOW
                else:
                    color = theme.ERROR_RED
                label_to_color.config(foreground=color)
            except (ValueError, IndexError):
                label_to_color.config(foreground=theme.FG_COLOR)
        except tk.TclError:
            pass
    return tracer

def make_unit_stripper(source_var, dest_var, unit_to_strip):
    """Strips a unit suffix from a source variable and updates a destination variable."""
    def tracer(*args):
        value = source_var.get()
        if value.endswith(unit_to_strip):
            value = value[:-len(unit_to_strip)].strip()
        dest_var.set(value)
    return tracer

def create_torque_widget(parent, torque_dv, height):
    """Creates a vertical torque meter widget."""
    torque_frame = ttk.Frame(parent, height=height, width=40, style='TFrame')
    torque_frame.pack_propagate(False)
    torque_sv = tk.StringVar()
    torque_frame.tracer = make_torque_tracer(torque_dv, torque_sv)
    torque_dv.trace_add('write', torque_frame.tracer)
    pbar = ttk.Progressbar(torque_frame, variable=torque_dv, maximum=100, orient=tk.VERTICAL, style='Card.Vertical.TProgressbar')
    pbar.pack(fill=tk.BOTH, expand=True)
    label = ttk.Label(torque_frame, textvariable=torque_sv, font=theme.FONT_SMALL, anchor='center', style='Subtle.TLabel')
    label.place(relx=0.5, rely=0.5, anchor='center')
    torque_frame.tracer()
    return torque_frame

def create_device_frame(parent, title, state_var, conn_var):
    """Creates the main bordered frame for a device panel."""
    outer_container = ttk.Frame(parent, style='CardBorder.TFrame', padding=1)
    container = ttk.Frame(outer_container, style='Card.TFrame', padding=10)
    container.pack(fill='x', expand=True)

    header_frame = ttk.Frame(container, style='Card.TFrame')
    header_frame.pack(fill='x', expand=True, anchor='n')

    title_label = ttk.Label(header_frame, text=title.lower(), font=theme.FONT_LARGE_BOLD, foreground=theme.DEVICE_COLOR, style='Subtle.TLabel')
    title_label.pack(side=tk.LEFT, padx=(0, 5))

    ip_label = ttk.Label(header_frame, text="", font=theme.FONT_SMALL, style='Subtle.TLabel')
    ip_label.pack(side=tk.LEFT, anchor='sw', pady=(0, 2))

    state_label = ttk.Label(header_frame, textvariable=state_var, font=theme.FONT_BOLD, style='Subtle.TLabel')
    state_label.pack(side=tk.RIGHT)
    state_label.tracer = make_state_tracer(state_var, state_label)
    state_var.trace_add('write', state_label.tracer)
    state_label.tracer()

    def conn_tracer(*args):
        full_status = conn_var.get()
        is_connected = "Connected" in full_status
        ip_address = ""
        if is_connected:
            try:
                ip_address = full_status.split('(')[1].split(')')[0]
            except IndexError:
                ip_address = "?.?.?.?"
        ip_label.config(text=ip_address)
    header_frame.conn_tracer = conn_tracer
    conn_var.trace_add("write", header_frame.conn_tracer)
    header_frame.conn_tracer()
    outer_container.ip_label = ip_label

    content_frame = ttk.Frame(container, style='Card.TFrame')
    content_frame.pack(fill='x', expand=True, pady=(5, 0))
    return outer_container, content_frame

def get_gui_variable_names():
    """Returns a list of tkinter variable names required by this GUI module."""
    return [
        # Existing fillhead variables
        'fillhead_main_state_var', 'status_var_fillhead',
        'fillhead_injector_state_var', 'fillhead_machine_steps_var', 'fillhead_homed0_var',
        'fillhead_cartridge_steps_var', 'fillhead_homed1_var',
        'fillhead_inject_cumulative_ml_var', 'fillhead_inject_active_ml_var',
        'fillhead_torque0_var', 'fillhead_torque1_var',
        'fillhead_inj_valve_pos_var', 'fillhead_inj_valve_homed_var', 'fillhead_torque2_var',
        'fillhead_inj_valve_state_var',
        'fillhead_vac_valve_pos_var', 'fillhead_vac_valve_homed_var', 'fillhead_torque3_var',
        'fillhead_vac_valve_state_var',
        'fillhead_vacuum_state_var', 'fillhead_vacuum_psig_var',
        'fillhead_heater_state_var', 'fillhead_temp_c_var', 'fillhead_heater_display_var',
        'pid_setpoint_var',
        'total_dispensed_var',
        'injection_target_ml_var',
        'cycle_dispensed_var',
        # Press / force variables (from telemetry)
        'fillhead_force_load_cell_var', 'fillhead_force_motor_torque_var',
        'fillhead_force_var', 'fillhead_force_limit_var', 'fillhead_force_source_var',
        'fillhead_current_pos_var', 'fillhead_retract_pos_var',
        'fillhead_target_pos_var', 'fillhead_homed_var',
        'fillhead_enabled0_var', 'fillhead_enabled1_var',
        'fillhead_torque_avg_var', 'fillhead_joules_var',
        'fillhead_endpoint_var', 'fillhead_startpoint_var',
        'fillhead_enabled_combined_var',
        'fillhead_home_sensor_m0_var', 'fillhead_home_sensor_m1_var',
        'fillhead_polarity_var',
    ]


# --- Main GUI Creation Function ---

def create_gui_components(parent, shared_gui_refs):
    """Creates the Fillhead status panel with press and injection subsystems."""

    for var_name in get_gui_variable_names():
        if var_name.endswith('_var'):
            if 'torque' in var_name:
                shared_gui_refs.setdefault(var_name, tk.DoubleVar(value=0.0))
            else:
                shared_gui_refs.setdefault(var_name, tk.StringVar(value='---'))

    # --- Computed variable tracers ---

    def update_enabled_combined(*args):
        try:
            e0 = shared_gui_refs['fillhead_enabled0_var'].get()
            e1 = shared_gui_refs['fillhead_enabled1_var'].get()
            if e0 == "enabled" and e1 == "enabled":
                shared_gui_refs['fillhead_enabled_combined_var'].set("enabled")
            else:
                shared_gui_refs['fillhead_enabled_combined_var'].set("disabled")
        except Exception:
            shared_gui_refs['fillhead_enabled_combined_var'].set("---")

    shared_gui_refs['fillhead_enabled0_var'].trace_add('write', update_enabled_combined)
    shared_gui_refs['fillhead_enabled1_var'].trace_add('write', update_enabled_combined)

    def update_force_display(*args):
        try:
            source = shared_gui_refs['fillhead_force_source_var'].get()
            if source == "load_cell":
                force_value = shared_gui_refs['fillhead_force_load_cell_var'].get()
                shared_gui_refs['fillhead_force_var'].set(force_value)
            elif source == "motor_torque":
                force_value = shared_gui_refs['fillhead_force_motor_torque_var'].get()
                shared_gui_refs['fillhead_force_var'].set(force_value)
            else:
                shared_gui_refs['fillhead_force_var'].set("---")
        except Exception:
            shared_gui_refs['fillhead_force_var'].set("---")

    shared_gui_refs['fillhead_force_source_var'].trace_add('write', update_force_display)
    shared_gui_refs['fillhead_force_load_cell_var'].trace_add('write', update_force_display)
    shared_gui_refs['fillhead_force_motor_torque_var'].trace_add('write', update_force_display)

    def update_heater_display(*args):
        try:
            temp_val = shared_gui_refs['fillhead_temp_c_var'].get().split()[0]
            heater_state = shared_gui_refs['fillhead_heater_state_var'].get().upper()
            is_on = "ON" in heater_state or "ACTIVE" in heater_state
            if is_on:
                setpoint_val = shared_gui_refs['pid_setpoint_var'].get()
            else:
                setpoint_val = "---"
            shared_gui_refs['fillhead_heater_display_var'].set(f"{temp_val} / {setpoint_val} °C")
        except (IndexError, ValueError, tk.TclError):
            shared_gui_refs['fillhead_heater_display_var'].set("--- / --- °C")

    shared_gui_refs['fillhead_temp_c_var'].trace_add("write", update_heater_display)
    shared_gui_refs['pid_setpoint_var'].trace_add("write", update_heater_display)
    shared_gui_refs['fillhead_heater_state_var'].trace_add("write", update_heater_display)

    def update_total_dispensed(*args):
        try:
            total_val = shared_gui_refs['fillhead_inject_cumulative_ml_var'].get().split()[0]
            shared_gui_refs['total_dispensed_var'].set(f"{total_val} ml")
        except (IndexError, ValueError, tk.TclError):
            shared_gui_refs['total_dispensed_var'].set("--- ml")

    shared_gui_refs['fillhead_inject_cumulative_ml_var'].trace_add("write", update_total_dispensed)

    def update_cycle_dispensed(*args):
        try:
            active_val = shared_gui_refs['fillhead_inject_active_ml_var'].get().split()[0]
            target_val = shared_gui_refs['injection_target_ml_var'].get()
            if target_val == '---':
                shared_gui_refs['cycle_dispensed_var'].set(f"{active_val} / --- ml")
            else:
                shared_gui_refs['cycle_dispensed_var'].set(f"{active_val} / {float(target_val):.2f} ml")
        except (IndexError, ValueError, tk.TclError):
            shared_gui_refs['cycle_dispensed_var'].set("--- / --- ml")

    shared_gui_refs['fillhead_inject_active_ml_var'].trace_add("write", update_cycle_dispensed)
    shared_gui_refs['injection_target_ml_var'].trace_add("write", update_cycle_dispensed)

    # --- Fonts ---
    font_large = theme.FONT_LARGE_BOLD
    font_medium = theme.FONT_BOLD
    font_small = theme.FONT_NORMAL
    font_small_readout = ("JetBrains Mono", 12, "bold")
    font_injector_readout = ("JetBrains Mono", 12, "bold")
    small_bar_height = 20

    # === Device Frame ===
    fillhead_outer_container, fillhead_content = create_device_frame(
        parent, "Fillhead",
        shared_gui_refs['fillhead_main_state_var'],
        shared_gui_refs['status_var_fillhead']
    )

    ip_label = getattr(fillhead_outer_container, 'ip_label', None)
    if ip_label is not None:
        trace_info = shared_gui_refs['status_var_fillhead'].trace_info()
        for trace in trace_info:
            if 'write' in trace[0]:
                try:
                    shared_gui_refs['status_var_fillhead'].trace_remove('write', trace[1])
                except Exception:
                    pass

        def fillhead_conn_tracer(*args):
            try:
                full_status = shared_gui_refs['status_var_fillhead'].get()
                if '(' in full_status and ')' in full_status:
                    try:
                        address = full_status.split('(')[1].split(')')[0]
                        if 'SIM' in full_status.upper() or 'SIMULATOR' in full_status.upper():
                            ip_label.config(text="[Simulator]", foreground=theme.WARNING_YELLOW)
                        else:
                            ip_label.config(text=f"@ {address}", foreground=theme.SUCCESS_GREEN)
                    except (IndexError, AttributeError):
                        ip_label.config(text="", foreground=theme.COMMENT_COLOR)
                else:
                    ip_label.config(text="", foreground=theme.COMMENT_COLOR)
            except tk.TclError:
                pass

        shared_gui_refs['status_var_fillhead'].trace_add('write', fillhead_conn_tracer)
        fillhead_conn_tracer()
        parent.after(100, fillhead_conn_tracer)

    # =====================================================================
    # PRESS SECTION (ported from Pressboi)
    # =====================================================================
    press_container = ttk.Frame(fillhead_content, style='CardBorder.TFrame')
    press_container.pack(anchor='w', pady=(10, 5), fill='x')
    press_grid = ttk.Frame(press_container, style='Card.TFrame', padding=5)
    press_grid.pack(fill='both', expand=True, padx=1, pady=1)

    torque_widget = create_torque_widget(press_grid, shared_gui_refs['fillhead_torque_avg_var'], 120)
    torque_widget.pack(side=tk.RIGHT, padx=(20, 0))

    left_side = ttk.Frame(press_grid, style='Card.TFrame')
    left_side.pack(side=tk.LEFT, fill='both', expand=True)

    # Force row
    force_row = ttk.Frame(left_side, style='Card.TFrame')
    force_row.pack(fill='x', pady=(0, 3))

    ttk.Label(force_row, text="Force:", font=font_medium, style='Subtle.TLabel').pack(side=tk.LEFT, padx=(0, 10))
    force_value_label = ttk.Label(force_row, textvariable=shared_gui_refs['fillhead_force_var'],
                                   font=font_large, foreground=theme.PRIMARY_ACCENT, style='Subtle.TLabel', anchor='e')
    force_value_label.pack(side=tk.LEFT)

    ttk.Label(force_row, text=" / ", font=font_medium, foreground=theme.COMMENT_COLOR, style='Subtle.TLabel').pack(side=tk.LEFT)
    ttk.Label(force_row, textvariable=shared_gui_refs['fillhead_force_limit_var'],
              font=font_medium, foreground=theme.COMMENT_COLOR, style='Subtle.TLabel', anchor='e').pack(side=tk.LEFT)

    # Force source indicator
    force_source_row = ttk.Frame(left_side, style='Card.TFrame')
    force_source_row.pack(fill='x', pady=(0, 5))

    force_source_label = ttk.Label(force_source_row, text="", font=font_small,
                                    foreground=theme.COMMENT_COLOR, style='Subtle.TLabel')
    force_source_label.pack(side=tk.LEFT)

    def update_force_source(*args):
        try:
            source = shared_gui_refs['fillhead_force_source_var'].get()
            if source == "load_cell":
                force_source_label.config(text="[Load Cell]", foreground=theme.SUCCESS_GREEN)
            elif source == "motor_torque":
                force_source_label.config(text="[Motor Torque]", foreground=theme.WARNING_YELLOW)
            else:
                force_source_label.config(text="[---]", foreground=theme.COMMENT_COLOR)
        except tk.TclError:
            pass
        except Exception:
            try:
                force_source_label.config(text="[---]", foreground=theme.COMMENT_COLOR)
            except tk.TclError:
                pass

    shared_gui_refs['fillhead_force_source_var'].trace_add('write', update_force_source)
    update_force_source()

    force_tracer = make_force_tracer(shared_gui_refs['fillhead_force_var'], force_value_label)
    shared_gui_refs['fillhead_force_var'].trace_add('write', force_tracer)
    force_tracer()

    # Position info
    pos_grid = ttk.Frame(left_side, style='Card.TFrame')
    pos_grid.pack(fill='x')
    pos_grid.grid_columnconfigure(1, weight=1)

    # Current → Target position
    pos_row = ttk.Frame(pos_grid, style='Card.TFrame')
    pos_row.grid(row=0, column=0, columnspan=4, sticky='w', pady=(0, 2))

    pos_label = ttk.Label(pos_row, text="Position:", font=font_medium, style='Subtle.TLabel')
    pos_label.pack(side=tk.LEFT, padx=(0, 10))

    current_pos_display = tk.StringVar(value='0.00')
    current_stripper = make_unit_stripper(shared_gui_refs['fillhead_current_pos_var'], current_pos_display, ' mm')
    shared_gui_refs['fillhead_current_pos_var'].trace_add('write', current_stripper)
    current_stripper()

    current_pos_label = ttk.Label(pos_row, textvariable=current_pos_display,
                                   font=font_large, foreground=theme.SUCCESS_GREEN, style='Subtle.TLabel', anchor='e')
    current_pos_label.pack(side=tk.LEFT)

    ttk.Label(pos_row, text=" → ", font=font_medium, foreground=theme.COMMENT_COLOR, style='Subtle.TLabel').pack(side=tk.LEFT)

    target_pos_display = tk.StringVar(value='0.00')
    target_stripper = make_unit_stripper(shared_gui_refs['fillhead_target_pos_var'], target_pos_display, ' mm')
    shared_gui_refs['fillhead_target_pos_var'].trace_add('write', target_stripper)
    target_stripper()

    target_pos_label = ttk.Label(pos_row, textvariable=target_pos_display,
                                  font=font_large, foreground=theme.WARNING_YELLOW, style='Subtle.TLabel', anchor='e')
    target_pos_label.pack(side=tk.LEFT)

    def position_color_tracer(*args):
        try:
            homed_state = shared_gui_refs['fillhead_homed_var'].get()
            main_state = shared_gui_refs['fillhead_main_state_var'].get().upper()

            if homed_state not in ('homed', 'Homed'):
                pos_label.config(foreground=theme.ERROR_RED)
                current_pos_label.config(foreground=theme.ERROR_RED)
                target_pos_label.config(foreground=theme.ERROR_RED)
                return

            current_str = current_pos_display.get()
            target_str = target_pos_display.get()
            current_pos = float(current_str) if current_str not in ('---', '') else 0.0
            target_pos = float(target_str) if target_str not in ('---', '') else 0.0

            at_target = abs(current_pos - target_pos) < 0.5
            is_moving = 'MOVING' in main_state or 'HOMING' in main_state

            pos_label.config(foreground=theme.FG_COLOR)
            if is_moving:
                current_pos_label.config(foreground=theme.BUSY_BLUE)
                target_pos_label.config(foreground=theme.WARNING_YELLOW)
            elif at_target:
                current_pos_label.config(foreground=theme.SUCCESS_GREEN)
                target_pos_label.config(foreground=theme.SUCCESS_GREEN)
            else:
                current_pos_label.config(foreground=theme.SUCCESS_GREEN)
                target_pos_label.config(foreground=theme.WARNING_YELLOW)
        except Exception:
            pass

    shared_gui_refs['fillhead_homed_var'].trace_add('write', position_color_tracer)
    shared_gui_refs['fillhead_main_state_var'].trace_add('write', position_color_tracer)
    shared_gui_refs['fillhead_current_pos_var'].trace_add('write', position_color_tracer)
    shared_gui_refs['fillhead_target_pos_var'].trace_add('write', position_color_tracer)
    position_color_tracer()

    # Retract position
    retract_row = ttk.Frame(pos_grid, style='Card.TFrame')
    retract_row.grid(row=1, column=0, columnspan=4, sticky='w', pady=(0, 2))

    ttk.Label(retract_row, text="Retract Position:", font=font_small, style='Subtle.TLabel').pack(side=tk.LEFT, padx=(0, 10))

    retract_pos_display = tk.StringVar(value='0.00')
    retract_stripper = make_unit_stripper(shared_gui_refs['fillhead_retract_pos_var'], retract_pos_display, ' mm')
    shared_gui_refs['fillhead_retract_pos_var'].trace_add('write', retract_stripper)
    retract_stripper()

    ttk.Label(retract_row, textvariable=retract_pos_display,
              font=font_small, style='Subtle.TLabel', anchor='e').pack(side=tk.LEFT)

    # Homed + Motors status row
    status_row = ttk.Frame(pos_grid, style='Card.TFrame')
    status_row.grid(row=2, column=0, columnspan=4, sticky='w', pady=(3, 2))

    homed_value_label = ttk.Label(status_row, textvariable=shared_gui_refs['fillhead_homed_var'],
                                   font=font_small, style='Subtle.TLabel')
    homed_value_label.pack(side=tk.LEFT, padx=(0, 10))
    homed_tracer = make_homed_tracer(shared_gui_refs['fillhead_homed_var'], homed_value_label)
    shared_gui_refs['fillhead_homed_var'].trace_add('write', homed_tracer)
    homed_tracer()

    # Polarity indicator
    polarity_label = ttk.Label(status_row, textvariable=shared_gui_refs['fillhead_polarity_var'],
                                font=font_small, foreground=theme.COMMENT_COLOR, style='Subtle.TLabel')
    polarity_label.pack(side=tk.LEFT, padx=(0, 10))

    def polarity_tracer(*args):
        try:
            pol = shared_gui_refs['fillhead_polarity_var'].get().lower()
            if pol == "inverted":
                polarity_label.config(foreground=theme.WARNING_YELLOW)
            else:
                polarity_label.config(foreground=theme.COMMENT_COLOR)
        except tk.TclError:
            pass
    shared_gui_refs['fillhead_polarity_var'].trace_add('write', polarity_tracer)
    polarity_tracer()

    # Home sensor indicators
    ttk.Label(status_row, text="M0:", font=font_small, foreground=theme.COMMENT_COLOR, style='Subtle.TLabel').pack(side=tk.LEFT, padx=(0, 2))
    m0_label = ttk.Label(status_row, textvariable=shared_gui_refs['fillhead_home_sensor_m0_var'],
                          font=font_small, style='Subtle.TLabel')
    m0_label.pack(side=tk.LEFT, padx=(0, 8))

    def m0_tracer(*args):
        try:
            state = shared_gui_refs['fillhead_home_sensor_m0_var'].get().lower()
            m0_label.config(foreground=theme.SUCCESS_GREEN if state == "active" else theme.COMMENT_COLOR)
        except tk.TclError:
            pass
    shared_gui_refs['fillhead_home_sensor_m0_var'].trace_add('write', m0_tracer)
    m0_tracer()

    ttk.Label(status_row, text="M1:", font=font_small, foreground=theme.COMMENT_COLOR, style='Subtle.TLabel').pack(side=tk.LEFT, padx=(0, 2))
    m1_label = ttk.Label(status_row, textvariable=shared_gui_refs['fillhead_home_sensor_m1_var'],
                          font=font_small, style='Subtle.TLabel')
    m1_label.pack(side=tk.LEFT, padx=(0, 10))

    def m1_tracer(*args):
        try:
            state = shared_gui_refs['fillhead_home_sensor_m1_var'].get().lower()
            m1_label.config(foreground=theme.SUCCESS_GREEN if state == "active" else theme.COMMENT_COLOR)
        except tk.TclError:
            pass
    shared_gui_refs['fillhead_home_sensor_m1_var'].trace_add('write', m1_tracer)
    m1_tracer()

    ttk.Label(status_row, text="Motors:", font=font_small, style='Subtle.TLabel').pack(side=tk.LEFT, padx=(0, 5))
    enabled_value_label = ttk.Label(status_row, textvariable=shared_gui_refs['fillhead_enabled_combined_var'],
                                     font=font_small, style='Subtle.TLabel')
    enabled_value_label.pack(side=tk.LEFT)
    enabled_tracer = make_state_tracer(shared_gui_refs['fillhead_enabled_combined_var'], enabled_value_label)
    shared_gui_refs['fillhead_enabled_combined_var'].trace_add('write', enabled_tracer)
    enabled_tracer()

    # Energy (joules) display
    joules_row = ttk.Frame(pos_grid, style='Card.TFrame')
    joules_row.grid(row=3, column=0, columnspan=4, sticky='w', pady=(0, 2))

    ttk.Label(joules_row, text="Energy:", font=font_small, style='Subtle.TLabel').pack(side=tk.LEFT, padx=(0, 10))

    joules_display = tk.StringVar(value='0.000')
    joules_stripper = make_unit_stripper(shared_gui_refs['fillhead_joules_var'], joules_display, ' J')
    shared_gui_refs['fillhead_joules_var'].trace_add('write', joules_stripper)
    joules_stripper()

    ttk.Label(joules_row, textvariable=joules_display,
              font=font_small, foreground=theme.PRIMARY_ACCENT, style='Subtle.TLabel', anchor='e').pack(side=tk.LEFT)
    ttk.Label(joules_row, text=" J", font=font_small, foreground=theme.COMMENT_COLOR, style='Subtle.TLabel').pack(side=tk.LEFT)

    # Endpoint / Startpoint display
    point_row = ttk.Frame(pos_grid, style='Card.TFrame')
    point_row.grid(row=4, column=0, columnspan=4, sticky='w', pady=(0, 2))

    ttk.Label(point_row, text="Endpoint:", font=font_small, style='Subtle.TLabel').pack(side=tk.LEFT, padx=(0, 10))

    endpoint_display = tk.StringVar(value='0.00')
    endpoint_stripper = make_unit_stripper(shared_gui_refs['fillhead_endpoint_var'], endpoint_display, ' mm')
    shared_gui_refs['fillhead_endpoint_var'].trace_add('write', endpoint_stripper)
    endpoint_stripper()

    ttk.Label(point_row, textvariable=endpoint_display,
              font=font_small, foreground=theme.PRIMARY_ACCENT, style='Subtle.TLabel', anchor='e').pack(side=tk.LEFT)
    ttk.Label(point_row, text=" mm", font=font_small, foreground=theme.COMMENT_COLOR, style='Subtle.TLabel').pack(side=tk.LEFT)

    ttk.Label(point_row, text="   Start:", font=font_small, style='Subtle.TLabel').pack(side=tk.LEFT, padx=(0, 10))

    startpoint_display = tk.StringVar(value='0.00')
    startpoint_stripper = make_unit_stripper(shared_gui_refs['fillhead_startpoint_var'], startpoint_display, ' mm')
    shared_gui_refs['fillhead_startpoint_var'].trace_add('write', startpoint_stripper)
    startpoint_stripper()

    ttk.Label(point_row, textvariable=startpoint_display,
              font=font_small, foreground=theme.PRIMARY_ACCENT, style='Subtle.TLabel', anchor='e').pack(side=tk.LEFT)
    ttk.Label(point_row, text=" mm", font=font_small, foreground=theme.COMMENT_COLOR, style='Subtle.TLabel').pack(side=tk.LEFT)

    ttk.Separator(fillhead_content, orient='horizontal').pack(fill='x', pady=8, padx=10)

    # =====================================================================
    # INJECTOR SECTION (existing)
    # =====================================================================
    injector_container = ttk.Frame(fillhead_content, style='CardBorder.TFrame')
    injector_container.pack(anchor='w', pady=(0, 5), fill='x')
    content_grid = ttk.Frame(injector_container, style='Card.TFrame', padding=5)
    content_grid.pack(fill='both', expand=True, padx=1, pady=1)
    content_grid.grid_columnconfigure(1, weight=1)
    content_grid.grid_columnconfigure(3, weight=1)

    inj_frame = ttk.Frame(content_grid, style='Card.TFrame')
    inj_frame.grid(row=0, column=0, columnspan=2, rowspan=2, sticky='nsew')
    ttk.Label(inj_frame, text="Injector:", font=font_injector_readout, style='Subtle.TLabel').pack(anchor='w')
    injector_state_label = ttk.Label(inj_frame, textvariable=shared_gui_refs['fillhead_injector_state_var'], font=font_injector_readout, style='Subtle.TLabel')
    injector_state_label.pack(anchor='w')
    injector_state_label.tracer = make_state_tracer(shared_gui_refs['fillhead_injector_state_var'], injector_state_label)
    shared_gui_refs['fillhead_injector_state_var'].trace_add('write', injector_state_label.tracer)
    injector_state_label.tracer()

    machine_label = ttk.Label(content_grid, text="Machine:", font=font_injector_readout, style='Subtle.TLabel')
    machine_label.grid(row=0, column=2, sticky='w', padx=(10, 5))
    ttk.Label(content_grid, textvariable=shared_gui_refs['fillhead_machine_steps_var'], font=font_injector_readout, style='Subtle.TLabel', anchor='e').grid(row=0, column=3, sticky='ew')
    machine_homed_var = shared_gui_refs['fillhead_homed0_var']
    machine_label.tracer = make_homed_tracer(machine_homed_var, machine_label)
    machine_homed_var.trace_add('write', machine_label.tracer)
    machine_label.tracer()

    cartridge_label = ttk.Label(content_grid, text="Cartridge:", font=font_injector_readout, style='Subtle.TLabel')
    cartridge_label.grid(row=1, column=2, sticky='w', padx=(10, 5))
    ttk.Label(content_grid, textvariable=shared_gui_refs['fillhead_cartridge_steps_var'], font=font_injector_readout, style='Subtle.TLabel', anchor='e').grid(row=1, column=3, sticky='ew')
    cartridge_homed_var = shared_gui_refs['fillhead_homed1_var']
    cartridge_label.tracer = make_homed_tracer(cartridge_homed_var, cartridge_label)
    cartridge_homed_var.trace_add('write', cartridge_label.tracer)
    cartridge_label.tracer()

    total_disp_frame = ttk.Frame(content_grid, style='Card.TFrame')
    total_disp_frame.grid(row=2, column=0, columnspan=4, sticky='ew')
    total_disp_frame.grid_columnconfigure(1, weight=1)
    ttk.Label(total_disp_frame, text="Total Dispensed:", font=font_injector_readout, style='Subtle.TLabel').grid(row=0, column=0, sticky='w')
    ttk.Label(total_disp_frame, textvariable=shared_gui_refs['fillhead_inject_cumulative_ml_var'], font=font_injector_readout, style='Subtle.TLabel', anchor='e').grid(row=0, column=1, sticky='ew')

    cycle_disp_frame = ttk.Frame(content_grid, style='Card.TFrame')
    cycle_disp_frame.grid(row=3, column=0, columnspan=4, sticky='ew')
    cycle_disp_frame.grid_columnconfigure(1, weight=1)
    ttk.Label(cycle_disp_frame, text="Cycle Dispensed:", font=font_injector_readout, style='Subtle.TLabel').grid(row=0, column=0, sticky='w')
    ttk.Label(cycle_disp_frame, textvariable=shared_gui_refs['fillhead_inject_active_ml_var'], foreground=theme.SUCCESS_GREEN, font=font_injector_readout, style='Subtle.TLabel', anchor='e').grid(row=0, column=1, sticky='ew')

    injector_torque_widget = create_torque_widget(content_grid, shared_gui_refs['fillhead_torque0_var'], 115)
    injector_torque_widget.grid(row=0, column=4, rowspan=4, sticky='ns', padx=(10, 0))

    ttk.Separator(fillhead_content, orient='horizontal').pack(fill='x', pady=8, padx=10)

    # =====================================================================
    # VALVE AXES SECTION (existing)
    # =====================================================================
    pinch_axes_data = [
        {'label': 'Inj Valve', 'pos_var': 'fillhead_inj_valve_pos_var', 'homed_var': 'fillhead_inj_valve_homed_var', 'torque_var': 'fillhead_torque2_var', 'state_var': 'fillhead_inj_valve_state_var'},
        {'label': 'Vac Valve', 'pos_var': 'fillhead_vac_valve_pos_var', 'homed_var': 'fillhead_vac_valve_homed_var', 'torque_var': 'fillhead_torque3_var', 'state_var': 'fillhead_vac_valve_state_var'},
    ]
    for axis_info in pinch_axes_data:
        axis_frame = ttk.Frame(fillhead_content, style='Card.TFrame')
        axis_frame.pack(anchor='w', pady=2, fill='x')
        axis_frame.grid_columnconfigure(2, weight=1)
        axis_label = ttk.Label(axis_frame, text=f"{axis_info['label']}:", anchor='w', font=font_small_readout, style='Subtle.TLabel')
        axis_label.grid(row=0, column=0, sticky='w', padx=(0, 5))
        ttk.Label(axis_frame, textvariable=shared_gui_refs[axis_info['state_var']], font=font_small_readout, style='Subtle.TLabel').grid(row=0, column=1, sticky='w', padx=(0, 10))
        ttk.Label(axis_frame, textvariable=shared_gui_refs[axis_info['pos_var']], font=font_small_readout, anchor='e', style='Subtle.TLabel').grid(row=0, column=2, sticky='ew', padx=(0, 10))
        axis_torque_widget = create_torque_widget(axis_frame, shared_gui_refs[axis_info['torque_var']], small_bar_height)
        axis_torque_widget.grid(row=0, column=3, rowspan=1, sticky='ns', padx=(10, 0))
        homed_var = shared_gui_refs[axis_info['homed_var']]
        axis_label.tracer = make_homed_tracer(homed_var, axis_label)
        homed_var.trace_add('write', axis_label.tracer)
        axis_label.tracer()

    ttk.Separator(fillhead_content, orient='horizontal').pack(fill='x', pady=8, padx=10)

    # =====================================================================
    # VACUUM & HEATER SECTION (existing)
    # =====================================================================
    vac_frame = ttk.Frame(fillhead_content, style='Card.TFrame')
    vac_frame.pack(anchor='w', pady=2, fill='x')
    vac_frame.grid_columnconfigure(2, weight=1)
    vac_label = ttk.Label(vac_frame, text="Vacuum:", anchor='w', font=font_small_readout, style='Subtle.TLabel')
    vac_label.grid(row=0, column=0, sticky='w', padx=(0, 5))
    vac_status_label = ttk.Label(vac_frame, textvariable=shared_gui_refs['fillhead_vacuum_state_var'], font=font_small_readout, style='Subtle.TLabel')
    vac_status_label.grid(row=0, column=1, sticky='w', padx=(0, 10))
    vac_status_tracer = make_on_off_tracer(shared_gui_refs['fillhead_vacuum_state_var'], (vac_status_label, theme.SUCCESS_GREEN, theme.COMMENT_COLOR))
    shared_gui_refs['fillhead_vacuum_state_var'].trace_add('write', vac_status_tracer)
    vac_status_tracer()
    vac_value_tracer = make_vacuum_value_tracer(shared_gui_refs['fillhead_vacuum_psig_var'], vac_label)
    shared_gui_refs['fillhead_vacuum_psig_var'].trace_add('write', vac_value_tracer)
    vac_value_tracer()
    ttk.Label(vac_frame, textvariable=shared_gui_refs['fillhead_vacuum_psig_var'], font=font_small_readout, foreground=theme.PRIMARY_ACCENT, anchor='e', style='Subtle.TLabel').grid(row=0, column=2, sticky='ew', padx=(0, 10))

    heater_frame = ttk.Frame(fillhead_content, style='Card.TFrame')
    heater_frame.pack(anchor='w', pady=2, fill='x')
    heater_frame.grid_columnconfigure(2, weight=1)
    heater_label = ttk.Label(heater_frame, text="Heater:", anchor='w', font=font_small_readout, style='Subtle.TLabel')
    heater_label.grid(row=0, column=0, sticky='w', padx=(0, 5))
    heater_status_label = ttk.Label(heater_frame, textvariable=shared_gui_refs['fillhead_heater_state_var'], font=font_small_readout, style='Subtle.TLabel')
    heater_status_label.grid(row=0, column=1, sticky='w', padx=(0, 10))
    heater_status_tracer = make_on_off_tracer(shared_gui_refs['fillhead_heater_state_var'], (heater_status_label, theme.SUCCESS_GREEN, theme.COMMENT_COLOR))
    shared_gui_refs['fillhead_heater_state_var'].trace_add('write', heater_status_tracer)
    heater_status_tracer()
    heater_value_tracer = make_heater_value_tracer(shared_gui_refs['fillhead_temp_c_var'], heater_label)
    shared_gui_refs['fillhead_temp_c_var'].trace_add('write', heater_value_tracer)
    heater_value_tracer()
    ttk.Label(heater_frame, textvariable=shared_gui_refs['fillhead_heater_display_var'], font=font_small_readout, foreground=theme.WARNING_YELLOW, anchor='e', style='Subtle.TLabel').grid(row=0, column=2, sticky='ew', padx=(0, 10))

    # =====================================================================
    # FORCE VS POSITION GRAPH (ported from Pressboi)
    # =====================================================================
    graph_container = ttk.Frame(fillhead_content, style='Card.TFrame', padding=10)
    graph_container.pack(fill='both', expand=True, pady=(10, 0))

    ttk.Label(graph_container, text="Force vs Position",
              font=font_medium, foreground=theme.PRIMARY_ACCENT,
              style='Subtle.TLabel').pack(anchor='w', pady=(0, 5))

    graph_canvas = tk.Canvas(graph_container,
                            width=300, height=180,
                            bg=theme.WIDGET_BG,
                            highlightthickness=1,
                            highlightbackground=theme.COMMENT_COLOR)
    graph_canvas.pack(fill='both', expand=True)

    graph_data = []
    max_points = 500

    def update_graph(*args):
        try:
            pos_str = shared_gui_refs['fillhead_current_pos_var'].get()
            force_str = shared_gui_refs['fillhead_force_var'].get()
            pos = float(pos_str.split()[0]) if pos_str != '---' else None
            force = float(force_str.split()[0]) if force_str != '---' else None
            if pos is not None and force is not None:
                graph_data.append((pos, force))
                if len(graph_data) > max_points:
                    graph_data.pop(0)
                draw_graph()
        except (ValueError, IndexError, AttributeError, tk.TclError):
            pass

    def draw_graph():
        try:
            graph_canvas.delete("all")
            if len(graph_data) < 2:
                graph_canvas.create_text(200, 100,
                                        text="Waiting for data...",
                                        fill=theme.COMMENT_COLOR,
                                        font=font_small)
                return

            width = graph_canvas.winfo_width()
            height = graph_canvas.winfo_height()
            if width <= 1:
                width = 400
            if height <= 1:
                height = 200

            margin_left = 50
            margin_right = 20
            margin_top = 20
            margin_bottom = 40
            plot_width = width - margin_left - margin_right
            plot_height = height - margin_top - margin_bottom

            positions = [p for p, f in graph_data]
            forces = [f for p, f in graph_data]
            min_pos = min(positions)
            max_pos = max(positions)
            min_force = min(forces)
            max_force = max(forces)

            pos_range = max_pos - min_pos if max_pos != min_pos else 1
            force_range = max_force - min_force if max_force != min_force else 1
            min_pos -= pos_range * 0.1
            max_pos += pos_range * 0.1
            min_force -= force_range * 0.1
            max_force += force_range * 0.1

            graph_canvas.create_line(margin_left, margin_top,
                                    margin_left, height - margin_bottom,
                                    fill=theme.COMMENT_COLOR, width=2)
            graph_canvas.create_line(margin_left, height - margin_bottom,
                                    width - margin_right, height - margin_bottom,
                                    fill=theme.COMMENT_COLOR, width=2)

            graph_canvas.create_text(width // 2, height - 10,
                                    text="Position (mm)",
                                    fill=theme.FG_COLOR,
                                    font=theme.FONT_SMALL)
            draw_vertical_text(graph_canvas, 15, height // 2,
                              "Force (kg)",
                              theme.FONT_SMALL,
                              theme.FG_COLOR,
                              anchor="center")

            for i in range(5):
                y = margin_top + (plot_height * i / 4)
                force_val = max_force - (force_range * i / 4)
                graph_canvas.create_text(margin_left - 5, y,
                                        text=f"{force_val:.0f}",
                                        fill=theme.COMMENT_COLOR,
                                        font=theme.FONT_SMALL,
                                        anchor='e')

            for i in range(5):
                x = margin_left + (plot_width * i / 4)
                pos_val = min_pos + (pos_range * i / 4)
                graph_canvas.create_text(x, height - margin_bottom + 5,
                                        text=f"{pos_val:.0f}",
                                        fill=theme.COMMENT_COLOR,
                                        font=theme.FONT_SMALL,
                                        anchor='n')

            points = []
            for pos, force in graph_data:
                x = margin_left + ((pos - min_pos) / pos_range) * plot_width
                y = height - margin_bottom - ((force - min_force) / force_range) * plot_height
                points.append((x, y))

            for i in range(len(points) - 1):
                x1, y1 = points[i]
                x2, y2 = points[i + 1]
                graph_canvas.create_line(x1, y1, x2, y2,
                                        fill=theme.PRIMARY_ACCENT,
                                        width=2)

            for x, y in points:
                graph_canvas.create_oval(x - 2, y - 2, x + 2, y + 2,
                                        fill=theme.SUCCESS_GREEN,
                                        outline=theme.SUCCESS_GREEN)
        except tk.TclError:
            pass

    button_frame = ttk.Frame(graph_container, style='Card.TFrame')
    button_frame.pack(fill='x', pady=(5, 0))

    def clear_graph():
        graph_data.clear()
        draw_graph()

    ttk.Button(button_frame, text="Clear Graph",
              command=clear_graph).pack(side=tk.RIGHT)

    shared_gui_refs['fillhead_current_pos_var'].trace_add('write', update_graph)
    shared_gui_refs['fillhead_force_var'].trace_add('write', update_graph)
    graph_canvas.bind('<Configure>', lambda e: draw_graph())

    return fillhead_outer_container
