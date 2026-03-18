"""
Fillhead Device Simulator
Handles fillhead-specific command simulation and state updates.
Superset of Pressboi — includes press, injection, valve, heater, and vacuum simulation.
"""
import time
import random
import socket
import threading
import argparse
import json
import os


def handle_command(device_sim, command, args, gui_address):
    """
    Handle fillhead-specific commands.

    Args:
        device_sim: Reference to the DeviceSimulator instance
        command: Command string (e.g., "home", "move_abs")
        args: List of command arguments
        gui_address: Tuple of (ip, port) for GUI

    Returns:
        True if command was fully handled (async response sent by sim),
        False to let the framework send a generic FILLHEAD_DONE response.
    """
    cmd_lower = command.lower()

    # ── Press commands ────────────────────────────────────────────────

    if cmd_lower == "home":
        device_sim.set_state('MAIN_STATE', 'HOMING')
        device_sim.command_queue.append(
            (simulate_press_homing, (device_sim, 2.0, gui_address, command)))
        return True

    elif cmd_lower == "move_abs":
        target = float(args[0]) if len(args) > 0 else 0.0
        speed = float(args[1]) if len(args) > 1 else 6.25
        force_limit = float(args[2]) if len(args) > 2 else 1000.0
        device_sim.state['target_pos'] = target
        device_sim.state['force_limit'] = force_limit
        cur = device_sim.state.get('current_pos', 0.0)
        duration = abs(target - cur) / speed if speed > 0 else 2.0
        device_sim.set_state('MAIN_STATE', 'MOVING')
        device_sim.command_queue.append(
            (simulate_press_move,
             (device_sim, target, max(duration, 0.5), gui_address, command)))
        return True

    elif cmd_lower == "move_inc":
        distance = float(args[0]) if len(args) > 0 else 0.0
        speed = float(args[1]) if len(args) > 1 else 6.25
        force_limit = float(args[2]) if len(args) > 2 else 1000.0
        cur = device_sim.state.get('current_pos', 0.0)
        target = cur + distance
        device_sim.state['target_pos'] = target
        device_sim.state['force_limit'] = force_limit
        duration = abs(distance) / speed if speed > 0 else 2.0
        device_sim.set_state('MAIN_STATE', 'MOVING')
        device_sim.command_queue.append(
            (simulate_press_move,
             (device_sim, target, max(duration, 0.5), gui_address, command)))
        return True

    elif cmd_lower == "retract":
        target = device_sim.state.get('retract_pos', 0.0)
        speed = float(args[0]) if args else 25.0
        cur = device_sim.state.get('current_pos', 0.0)
        duration = abs(target - cur) / speed if speed > 0 else 2.0
        device_sim.state['target_pos'] = target
        device_sim.set_state('MAIN_STATE', 'MOVING')
        device_sim.command_queue.append(
            (simulate_press_move,
             (device_sim, target, max(duration, 0.5), gui_address, command)))
        return True

    elif cmd_lower == "set_retract":
        device_sim.state['retract_pos'] = float(args[0]) if len(args) > 0 else 0.0
        if len(args) > 1:
            device_sim.state['retract_speed'] = float(args[1])
        return False

    elif cmd_lower == "pause":
        if device_sim.state.get('MAIN_STATE') == 'MOVING':
            device_sim.set_state('MAIN_STATE', 'PAUSED')
        return False

    elif cmd_lower == "resume":
        if device_sim.state.get('MAIN_STATE') == 'PAUSED':
            device_sim.set_state('MAIN_STATE', 'MOVING')
        return False

    elif cmd_lower == "cancel":
        device_sim.set_state('MAIN_STATE', 'STANDBY')
        return False

    # ── Injection commands ────────────────────────────────────────────

    elif cmd_lower == "inject":
        vol = float(args[0]) if args else 0.0
        device_sim.state['inj_tgt_ml'] = vol
        device_sim.set_state('MAIN_STATE', 'INJECTING')
        device_sim.command_queue.append(
            (simulate_injection, (device_sim, vol, 2.0, gui_address, command)))
        return True

    elif cmd_lower == "set_cartridge_ml_per_mm":
        ratio = float(args[0]) if args else 5.2732
        device_sim.state['cartridge_ml_per_mm'] = ratio
        return False

    elif cmd_lower == "jog_move":
        distance = float(args[0]) if args else 0.0
        device_sim.state['inj_mach_mm'] = (
            device_sim.state.get('inj_mach_mm', 0.0) + distance)
        return False

    elif cmd_lower in ("machine_home", "machine_home_move"):
        device_sim.set_state('MAIN_STATE', 'HOMING')
        device_sim.command_queue.append(
            (simulate_injector_homing,
             (device_sim, 'machine', 2.0, gui_address, command)))
        return True

    elif cmd_lower in ("cartridge_home", "cartridge_home_move"):
        device_sim.set_state('MAIN_STATE', 'HOMING')
        device_sim.command_queue.append(
            (simulate_injector_homing,
             (device_sim, 'cartridge', 2.0, gui_address, command)))
        return True

    elif cmd_lower == "move_to_cartridge_home":
        device_sim.state['inj_cart_mm'] = 0.0
        return False

    elif cmd_lower == "move_to_cartridge_retract":
        distance = float(args[0]) if args else 5.0
        device_sim.state['inj_cart_mm'] = distance
        return False

    elif cmd_lower == "pause_injection":
        return False

    elif cmd_lower == "resume_injection":
        return False

    elif cmd_lower == "cancel_injection":
        device_sim.set_state('MAIN_STATE', 'STANDBY')
        device_sim.state['inj_tgt_ml'] = 0
        return False

    # ── Valve commands ────────────────────────────────────────────────

    elif cmd_lower.startswith("injection_valve_"):
        action = cmd_lower[len("injection_valve_"):]
        if action == 'home':
            device_sim.set_state('MAIN_STATE', 'HOMING')
            device_sim.command_queue.append(
                (simulate_valve_homing,
                 (device_sim, 'inj_valve', 1.5, gui_address, command)))
            return True
        elif action == 'open':
            device_sim.state['inj_valve_st'] = 'Open'
        elif action == 'close':
            device_sim.state['inj_valve_st'] = 'Closed'
        elif action == 'jog':
            d = float(args[0]) if args else 0.0
            device_sim.state['inj_valve_pos'] = (
                device_sim.state.get('inj_valve_pos', 0.0) + d)
        elif action.startswith('home_on_boot'):
            val = args[0] if args else 'true'
            device_sim.state['inj_valve_home_on_boot'] = (val == 'true')
        return False

    elif cmd_lower.startswith("vacuum_valve_"):
        action = cmd_lower[len("vacuum_valve_"):]
        if action == 'home':
            device_sim.set_state('MAIN_STATE', 'HOMING')
            device_sim.command_queue.append(
                (simulate_valve_homing,
                 (device_sim, 'vac_valve', 1.5, gui_address, command)))
            return True
        elif action == 'open':
            device_sim.state['vac_valve_st'] = 'Open'
        elif action == 'close':
            device_sim.state['vac_valve_st'] = 'Closed'
        elif action == 'jog':
            d = float(args[0]) if args else 0.0
            device_sim.state['vac_valve_pos'] = (
                device_sim.state.get('vac_valve_pos', 0.0) + d)
        elif action.startswith('home_on_boot'):
            val = args[0] if args else 'true'
            device_sim.state['vac_valve_home_on_boot'] = (val == 'true')
        return False

    # ── Heater commands ───────────────────────────────────────────────

    elif cmd_lower == "heater_on":
        device_sim.state['h_st'] = 1
        if args:
            device_sim.state['h_sp'] = float(args[0])
        return False

    elif cmd_lower == "heater_off":
        device_sim.state['h_st'] = 0
        return False

    elif cmd_lower == "set_heater_gains":
        if len(args) >= 3:
            device_sim.state['h_kp'] = float(args[0])
            device_sim.state['h_ki'] = float(args[1])
            device_sim.state['h_kd'] = float(args[2])
        return False

    elif cmd_lower == "set_heater_setpoint":
        if args:
            device_sim.state['h_sp'] = float(args[0])
        return False

    # ── Vacuum commands ───────────────────────────────────────────────

    elif cmd_lower == "vacuum_on":
        device_sim.state['vac_st'] = 1
        if args:
            device_sim.state['vac_tgt'] = float(args[0])
        return False

    elif cmd_lower == "vacuum_off":
        device_sim.state['vac_st'] = 0
        return False

    elif cmd_lower == "vacuum_leak_test":
        device_sim.set_state('MAIN_STATE', 'LEAK_TESTING')
        device_sim.command_queue.append(
            (simulate_leak_test, (device_sim, 3.0, gui_address, command)))
        return True

    elif cmd_lower == "set_vacuum_target":
        if args:
            device_sim.state['vac_tgt'] = float(args[0])
        return False

    elif cmd_lower == "set_vacuum_timeout_s":
        if args:
            device_sim.state['vac_timeout'] = float(args[0])
        return False

    elif cmd_lower == "set_leak_test_delta":
        if args:
            device_sim.state['leak_delta'] = float(args[0])
        return False

    elif cmd_lower == "set_leak_test_duration_s":
        if args:
            device_sim.state['leak_duration'] = float(args[0])
        return False

    # ── System commands ───────────────────────────────────────────────

    elif cmd_lower in ("reset", "clear_errors"):
        device_sim.set_state('MAIN_STATE', 'STANDBY')
        return False

    elif cmd_lower == "enable":
        device_sim.state['motors_enabled'] = 1
        return False

    elif cmd_lower == "disable":
        device_sim.state['motors_enabled'] = 0
        return False

    elif cmd_lower == "abort":
        device_sim.set_state('MAIN_STATE', 'STANDBY')
        return False

    elif cmd_lower == "discover_device":
        msg = "DISCOVERY_RESPONSE: DEVICE_ID=fillhead PORT=8888 FW=1.0.0"
        device_sim.sock.sendto(msg.encode(), gui_address)
        return True

    elif cmd_lower == "test_watchdog":
        return False

    elif cmd_lower == "reboot_bootloader":
        msg = f"FILLHEAD_INFO: {command} Rebooting to bootloader..."
        device_sim.sock.sendto(msg.encode(), gui_address)
        return True

    elif cmd_lower == "dump_nvm":
        device_sim.sock.sendto(
            f"FILLHEAD_INFO: {command} NVM contents: (simulated)".encode(),
            gui_address)
        device_sim.sock.sendto(
            f"FILLHEAD_DONE: {command}".encode(), gui_address)
        return True

    elif cmd_lower == "reset_nvm":
        device_sim.sock.sendto(
            f"FILLHEAD_INFO: {command} NVM reset to factory defaults".encode(),
            gui_address)
        device_sim.sock.sendto(
            f"FILLHEAD_DONE: {command}".encode(), gui_address)
        return True

    elif cmd_lower == "dump_error_log":
        device_sim.sock.sendto(
            f"FILLHEAD_INFO: {command} Error log: (empty)".encode(),
            gui_address)
        device_sim.sock.sendto(
            f"FILLHEAD_DONE: {command}".encode(), gui_address)
        return True

    # ── Force / config commands ───────────────────────────────────────

    elif cmd_lower == "set_force_mode":
        if args:
            device_sim.state['force_mode'] = args[0]
        return False

    elif cmd_lower == "set_force_offset":
        if args:
            device_sim.state['force_offset'] = float(args[0])
        return False

    elif cmd_lower == "set_force_zero":
        device_sim.state['force_load_cell'] = 0.0
        device_sim.state['force_motor_torque'] = 0.0
        device_sim.sock.sendto(
            f"FILLHEAD_INFO: {command} Force zeroed".encode(), gui_address)
        return False

    elif cmd_lower == "set_force_scale":
        if args:
            device_sim.state['force_scale'] = float(args[0])
        return False

    elif cmd_lower == "set_strain_cal":
        return False

    elif cmd_lower == "set_polarity":
        if args:
            device_sim.state['polarity'] = args[0]
        return False

    elif cmd_lower == "home_on_boot":
        if args:
            device_sim.state['home_on_boot'] = args[0]
        return False

    elif cmd_lower == "set_press_threshold":
        if args:
            device_sim.state['press_threshold'] = float(args[0])
        return False

    return False


# ═══════════════════════════════════════════════════════════════════════
# Simulation helpers
# ═══════════════════════════════════════════════════════════════════════

def simulate_press_homing(device_sim, duration, gui_address, command):
    """Simulates press axis homing (sets position to zero)."""
    time.sleep(duration)
    if device_sim._stop_event.is_set():
        return

    device_sim.state['current_pos'] = 0.0
    device_sim.state['homed'] = 1
    device_sim.state['force_load_cell'] = 0.0
    device_sim.state['force_motor_torque'] = 0.0
    device_sim.state['force_source'] = "load_cell"
    device_sim.state['joules'] = 0.0
    device_sim.state['home_sensor_m0'] = 1
    device_sim.state['home_sensor_m1'] = 1
    device_sim.set_state('MAIN_STATE', 'STANDBY')
    device_sim.sock.sendto(
        f"FILLHEAD_DONE: {command}".encode(), gui_address)
    print(f"[fillhead] Press homing complete")


def simulate_press_move(device_sim, target, duration, gui_address, command):
    """Simulates press move with force and joule accumulation."""
    start_time = time.time()
    start_pos = device_sim.state.get('current_pos', 0.0)

    device_sim.state['joules'] = 0.0
    device_sim.state['startpoint'] = 0.0
    prev_pos = start_pos
    threshold_crossed = False

    while time.time() - start_time < duration:
        if device_sim._stop_event.is_set():
            return
        if device_sim.state.get('MAIN_STATE') == 'PAUSED':
            time.sleep(0.05)
            continue

        elapsed = time.time() - start_time
        progress = min(elapsed / duration, 1.0)
        current_pos = start_pos + (target - start_pos) * progress
        device_sim.state['current_pos'] = current_pos

        force_kg = random.uniform(50, 300)
        device_sim.state['force_load_cell'] = force_kg
        device_sim.state['force_motor_torque'] = force_kg
        device_sim.state['force_source'] = "load_cell"
        device_sim.state['torque_avg'] = random.uniform(20, 60)

        threshold = device_sim.state.get('press_threshold', 2.0)
        if not threshold_crossed and force_kg >= threshold:
            device_sim.state['startpoint'] = current_pos
            threshold_crossed = True

        distance_mm = abs(current_pos - prev_pos)
        device_sim.state['joules'] += force_kg * distance_mm * 0.00981
        prev_pos = current_pos

        time.sleep(0.05)

    device_sim.state['current_pos'] = target
    device_sim.state['endpoint'] = target
    device_sim.state['target_pos'] = target
    device_sim.state['force_load_cell'] = 0.0
    device_sim.state['force_motor_torque'] = 0.0
    device_sim.state['torque_avg'] = 0.0
    device_sim.set_state('MAIN_STATE', 'STANDBY')
    device_sim.sock.sendto(
        f"FILLHEAD_DONE: {command}".encode(), gui_address)
    print(f"[fillhead] Move complete to {target}, "
          f"Energy: {device_sim.state['joules']:.1f} J")


def simulate_injector_homing(device_sim, component, duration,
                             gui_address, command):
    """Simulates injector motor homing (machine or cartridge)."""
    time.sleep(duration)
    if device_sim._stop_event.is_set():
        return

    if component == 'machine':
        device_sim.state['inj_h_mach'] = 1
        device_sim.state['inj_mach_mm'] = 0.0
    elif component == 'cartridge':
        device_sim.state['inj_h_cart'] = 1
        device_sim.state['inj_cart_mm'] = 0.0

    device_sim.state['inj_st'] = 'Standby'
    device_sim.set_state('MAIN_STATE', 'STANDBY')
    device_sim.sock.sendto(
        f"FILLHEAD_DONE: {command}".encode(), gui_address)
    print(f"[fillhead] Injector {component} homing complete")


def simulate_valve_homing(device_sim, valve_type, duration,
                          gui_address, command):
    """Simulates pinch-valve homing."""
    time.sleep(duration)
    if device_sim._stop_event.is_set():
        return

    device_sim.state[f'{valve_type}_homed'] = 1
    device_sim.state[f'{valve_type}_pos'] = 0.0
    device_sim.state[f'{valve_type}_st'] = 'Open'
    device_sim.set_state('MAIN_STATE', 'STANDBY')
    device_sim.sock.sendto(
        f"FILLHEAD_DONE: {command}".encode(), gui_address)
    print(f"[fillhead] {valve_type} homing complete")


def simulate_injection(device_sim, volume, duration, gui_address, command):
    """Simulates the injection dispensing process."""
    start_time = time.time()
    start_vol = device_sim.state.get('inj_active_ml', 0.0)

    while time.time() - start_time < duration:
        elapsed = time.time() - start_time
        progress = elapsed / duration
        device_sim.state['inj_active_ml'] = start_vol + volume * progress
        time.sleep(0.05)
        if device_sim._stop_event.is_set():
            return

    device_sim.state['inj_active_ml'] = 0
    device_sim.state['inj_cumulative_ml'] = (
        device_sim.state.get('inj_cumulative_ml', 0.0) + volume)
    device_sim.state['inj_tgt_ml'] = 0
    device_sim.set_state('MAIN_STATE', 'STANDBY')
    device_sim.sock.sendto(
        f"FILLHEAD_DONE: {command}".encode(), gui_address)
    print(f"[fillhead] Injection complete: {volume:.2f} ml")


def simulate_leak_test(device_sim, duration, gui_address, command):
    """Simulates a vacuum leak test (always passes in sim)."""
    time.sleep(duration)
    if device_sim._stop_event.is_set():
        return

    device_sim.set_state('MAIN_STATE', 'STANDBY')
    device_sim.sock.sendto(
        f"FILLHEAD_DONE: {command} PASSED".encode(), gui_address)
    print(f"[fillhead] Leak test passed")


# ═══════════════════════════════════════════════════════════════════════
# Periodic state update
# ═══════════════════════════════════════════════════════════════════════

def update_state(device_sim):
    """Update fillhead dynamic state (called periodically by framework)."""
    # Heater PID simulation
    if device_sim.state.get('h_st') == 1:
        error = (device_sim.state.get('h_sp', 70)
                 - device_sim.state.get('h_pv', 25))
        device_sim.state['h_op'] = min(100, max(0, error * 10))
        device_sim.state['h_pv'] += (
            device_sim.state.get('h_op', 0) * 0.01 - 0.05)
    else:
        device_sim.state['h_op'] = 0
        if device_sim.state.get('h_pv', 0) > 25:
            device_sim.state['h_pv'] -= 0.1

    # Home-sensor state: active when close to position 0
    pos = device_sim.state.get('current_pos', 0.0)
    device_sim.state['home_sensor_m0'] = 1 if pos < 0.5 else 0
    device_sim.state['home_sensor_m1'] = 1 if pos < 0.5 else 0

    # Pinch valve home sensors: active when valve is at open (homed) position
    inj_pos = device_sim.state.get('inj_valve_pos', 0.0)
    vac_pos = device_sim.state.get('vac_valve_pos', 0.0)
    device_sim.state['inj_valve_home_sensor'] = 1 if abs(inj_pos) < 0.5 else 0
    device_sim.state['vac_valve_home_sensor'] = 1 if abs(vac_pos) < 0.5 else 0


# ═══════════════════════════════════════════════════════════════════════
# Standalone runner
# ═══════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    DEFAULT_STATE = {
        'MAIN_STATE': 'STANDBY',
        'current_pos': 0.0,
        'target_pos': 0.0,
        'retract_pos': 0.0,
        'retract_speed': 25.0,
        'endpoint': 0.0,
        'startpoint': 0.0,
        'press_threshold': 2.0,
        'homed': 0,
        'home_sensor_m0': 0,
        'home_sensor_m1': 0,
        'force_load_cell': 0.0,
        'force_motor_torque': 0.0,
        'force_limit': 1000.0,
        'force_source': 'load_cell',
        'force_mode': 'load_cell',
        'force_offset': 0.0,
        'force_scale': 1.0,
        'joules': 0.0,
        'torque_avg': 0.0,
        'polarity': 'normal',
        'home_on_boot': 'false',
        'motors_enabled': 1,
        'inj_mach_mm': 0.0,
        'inj_cart_mm': 0.0,
        'inj_h_mach': 0,
        'inj_h_cart': 0,
        'inj_st': 'Standby',
        'inj_active_ml': 0.0,
        'inj_cumulative_ml': 0.0,
        'inj_tgt_ml': 0.0,
        'cartridge_ml_per_mm': 5.2732,
        'inj_valve_st': 'Not Homed',
        'inj_valve_homed': 0,
        'inj_valve_pos': 0.0,
        'inj_valve_home_sensor': 0,
        'inj_valve_home_on_boot': True,
        'vac_valve_st': 'Not Homed',
        'vac_valve_homed': 0,
        'vac_valve_pos': 0.0,
        'vac_valve_home_sensor': 0,
        'vac_valve_home_on_boot': True,
        'h_st': 0,
        'h_sp': 70.0,
        'h_pv': 25.0,
        'h_op': 0.0,
        'vac_st': 0,
        'vac_tgt': -12.0,
    }

    class StandaloneSimulator:
        """Minimal DeviceSimulator stand-in for standalone testing."""

        def __init__(self, port):
            self.state = dict(DEFAULT_STATE)
            self.command_queue = []
            self._stop_event = threading.Event()
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(('0.0.0.0', port))
            self.port = port

        def set_state(self, key, value):
            self.state[key] = value
            print(f"  [{key}] = {value}")

        def _worker(self):
            """Process queued async commands."""
            while not self._stop_event.is_set():
                if self.command_queue:
                    func, func_args = self.command_queue.pop(0)
                    func(*func_args)
                else:
                    time.sleep(0.05)

        def _telemetry_loop(self, interval):
            """Periodically call update_state and print telemetry."""
            while not self._stop_event.is_set():
                update_state(self)
                time.sleep(interval)

        def run(self):
            print(f"[fillhead-sim] Listening on UDP :{self.port}")
            worker = threading.Thread(target=self._worker, daemon=True)
            worker.start()
            telem = threading.Thread(
                target=self._telemetry_loop, args=(0.5,), daemon=True)
            telem.start()

            try:
                while True:
                    data, addr = self.sock.recvfrom(4096)
                    msg = data.decode().strip()
                    if not msg:
                        continue
                    parts = msg.split()
                    cmd = parts[0]
                    cmd_args = parts[1:]
                    print(f"<< {msg}  (from {addr})")

                    handled = handle_command(self, cmd, cmd_args, addr)
                    if not handled:
                        reply = f"FILLHEAD_DONE: {cmd}"
                        self.sock.sendto(reply.encode(), addr)
                        print(f">> {reply}")
            except KeyboardInterrupt:
                print("\n[fillhead-sim] Shutting down")
                self._stop_event.set()

    parser = argparse.ArgumentParser(
        description="Fillhead standalone device simulator")
    parser.add_argument(
        '-p', '--port', type=int, default=8888,
        help="UDP port to listen on (default: 8888)")
    cli_args = parser.parse_args()

    sim = StandaloneSimulator(cli_args.port)
    sim.run()
