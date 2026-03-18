"""
Press Report Generator

Generates beautiful interactive HTML reports for press operations using Plotly.
Analyzes CSV log data and produces comprehensive reports with pass/fail determination.
"""

import csv
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Any, Tuple

# Try to import jinja2, provide helpful error if not available
try:
    from jinja2 import Environment, FileSystemLoader, select_autoescape
except ImportError:
    raise ImportError(
        "Jinja2 is required for report generation. "
        "Install it with: pip install jinja2"
    )

class PressReportGenerator:
    """
    Generates HTML press operation reports from CSV log data.
    
    Features:
    - Interactive Plotly force vs distance charts
    - Pass/fail determination based on configurable thresholds
    - Energy visualization
    - Raw data table
    - Beautiful dark-themed responsive design
    """
    
    def __init__(self, template_dir: Optional[Path] = None):
        """
        Initialize the report generator.
        
        Args:
            template_dir: Directory containing report templates.
                         Defaults to templates/ in the same directory as this file.
        """
        if template_dir is None:
            template_dir = Path(__file__).parent / 'templates'
        
        self.template_dir = Path(template_dir)
        
        # Setup Jinja2 environment
        self.env = Environment(
            loader=FileSystemLoader(str(self.template_dir)),
            autoescape=select_autoescape(['html', 'xml'])
        )
        
        # Load report definition
        definition_path = Path(__file__).parent / 'press_report.json'
        if definition_path.exists():
            with open(definition_path, 'r') as f:
                self.definition = json.load(f)
        else:
            self.definition = {}
    
    def parse_csv_log(self, csv_path: str) -> Dict[str, Any]:
        """
        Parse a CSV log file and extract relevant data.
        
        Args:
            csv_path: Path to the CSV log file
            
        Returns:
            Dictionary containing parsed data:
            - times: List of elapsed times in seconds
            - positions: List of positions in mm
            - forces: List of force values in kg
            - energies: List of energy values in J (if available)
            - start_time: Datetime of first data point
            - end_time: Datetime of last data point
        """
        times = []
        positions = []
        forces = []
        energies = []
        start_time = None
        end_time = None
        
        # Retry logic for Windows file handle timing issues
        # File may not be immediately available after being closed by data logger
        max_retries = 5
        retry_delay = 0.3  # 300ms between retries
        last_error = None
        last_error_type = None
        f = None
        temp_file_path = None  # Track temp file for cleanup
        
        for attempt in range(max_retries):
            try:
                f = open(csv_path, 'r', newline='')
                break  # Success!
            except FileNotFoundError as e:
                last_error = e
                last_error_type = "FileNotFoundError"
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
            except PermissionError as e:
                last_error = e
                last_error_type = "PermissionError"
                # On Windows, try copy-to-temp fallback for locked files
                # This works because Windows allows copying files that are open for writing
                if attempt == max_retries - 1:
                    try:
                        import shutil
                        import tempfile
                        # Create a temp copy
                        temp_dir = tempfile.gettempdir()
                        temp_file_path = os.path.join(temp_dir, f"report_temp_{os.path.basename(csv_path)}")
                        print(f"[REPORT] File locked, attempting copy-to-temp: {temp_file_path}")
                        shutil.copy2(csv_path, temp_file_path)
                        f = open(temp_file_path, 'r', newline='')
                        print(f"[REPORT] Successfully opened temp copy of locked file")
                        break  # Success via temp copy!
                    except Exception as copy_error:
                        print(f"[REPORT] Copy-to-temp fallback failed: {copy_error}")
                        # Keep the original PermissionError
                else:
                    time.sleep(retry_delay)
            except OSError as e:
                last_error = e
                last_error_type = "OSError"
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
        
        if f is None:
            # Preserve the original error type in the message for accurate diagnostics
            error_msg = f"Could not open CSV after {max_retries} attempts: {csv_path} ({last_error_type}: {last_error})"
            # Re-raise the same type of exception that actually occurred
            if last_error_type == "PermissionError":
                raise PermissionError(error_msg) from last_error
            elif last_error_type == "OSError":
                raise OSError(error_msg) from last_error
            else:
                raise FileNotFoundError(error_msg) from last_error
        
        with f:
            reader = csv.DictReader(f)
            headers = reader.fieldnames or []
            
            # Find column names (handle different naming conventions)
            pos_col = None
            force_col = None
            energy_col = None
            time_col = None
            date_col = None
            
            detected_force_mode = None
            for header in headers:
                header_lower = header.lower()
                if 'current_pos' in header_lower or 'position' in header_lower:
                    pos_col = header
                elif 'force_load_cell' in header_lower:
                    force_col = header
                    detected_force_mode = 'Load Cell'
                elif 'force_motor_torque' in header_lower and force_col is None:
                    force_col = header
                    detected_force_mode = 'Motor Torque'
                elif 'joules' in header_lower or 'energy' in header_lower:
                    energy_col = header
                elif header_lower == 'elapsed_s' or 'elapsed' in header_lower:
                    time_col = header
                elif header_lower == 'date':
                    date_col = header
            
            if not pos_col or not force_col:
                raise ValueError(
                    f"CSV file missing required columns. "
                    f"Found: {headers}. "
                    f"Need position and force columns."
                )
            
            for row in reader:
                try:
                    # Parse time
                    if time_col and row.get(time_col):
                        times.append(float(row[time_col]))
                    else:
                        times.append(len(times) * 0.02)  # Assume 50Hz if no time column
                    
                    # Parse position
                    pos_val = row.get(pos_col, '0')
                    if pos_val:
                        positions.append(float(pos_val))
                    
                    # Parse force
                    force_val = row.get(force_col, '0')
                    if force_val:
                        forces.append(float(force_val))
                    
                    # Parse energy (optional)
                    if energy_col:
                        energy_val = row.get(energy_col, '')
                        if energy_val:
                            energies.append(float(energy_val))
                    
                    # Track start/end dates
                    if date_col and row.get(date_col):
                        try:
                            date_str = row[date_col]
                            time_str = row.get('time_ms', '00:00:00.000')
                            dt = datetime.strptime(f"{date_str} {time_str}", "%Y-%m-%d %H:%M:%S.%f")
                            if start_time is None:
                                start_time = dt
                            end_time = dt
                        except (ValueError, TypeError):
                            pass
                            
                except (ValueError, TypeError) as e:
                    # Skip rows with invalid data
                    continue
        
        # Ensure all lists have same length
        min_len = min(len(times), len(positions), len(forces))
        times = times[:min_len]
        positions = positions[:min_len]
        forces = forces[:min_len]
        if energies:
            energies = energies[:min(len(energies), min_len)]
        
        # Clean up temp file if we used one
        if temp_file_path and os.path.exists(temp_file_path):
            try:
                os.remove(temp_file_path)
            except Exception:
                pass  # Ignore cleanup errors
        
        return {
            'times': times,
            'positions': positions,
            'forces': forces,
            'energies': energies,
            'start_time': start_time,
            'end_time': end_time,
            'has_energy_data': len(energies) > 0,
            'detected_force_mode': detected_force_mode
        }
    
    def calculate_metrics(self, data: Dict[str, Any], 
                          force_min: Optional[float] = None,
                          force_max: Optional[float] = None,
                          endpoint_min: Optional[float] = None,
                          endpoint_max: Optional[float] = None,
                          energy_min: Optional[float] = None,
                          energy_max: Optional[float] = None) -> Dict[str, Any]:
        """
        Calculate metrics from parsed log data.
        
        Args:
            data: Parsed CSV data from parse_csv_log()
            force_min: Minimum force threshold for pass
            force_max: Maximum force threshold for pass
            endpoint_min: Minimum endpoint threshold for pass
            endpoint_max: Maximum endpoint threshold for pass
            energy_min: Minimum energy threshold for pass
            energy_max: Maximum energy threshold for pass
            
        Returns:
            Dictionary of calculated metrics
        """
        positions = data['positions']
        forces = data['forces']
        energies = data['energies']
        times = data['times']
        
        if not positions or not forces:
            return {
                'peak_force': 0,
                'peak_force_position': 0,
                'start_position': 0,
                'endpoint': 0,
                'energy': 0,
                'duration': 0,
                'data_points': 0
            }
        
        # Find peak force and its position
        peak_force = max(forces)
        peak_idx = forces.index(peak_force)
        peak_force_position = positions[peak_idx]
        
        # Start and end positions
        start_position = positions[0]
        endpoint = positions[-1]
        
        # Energy (use logged value if available, otherwise calculate)
        if energies:
            energy = energies[-1]  # Final accumulated energy
        else:
            # Calculate energy as integral of force over distance (simplified)
            energy = 0.0
            for i in range(1, len(positions)):
                dx = abs(positions[i] - positions[i-1]) / 1000.0  # mm to m
                avg_force = (forces[i] + forces[i-1]) / 2 * 9.81  # kg to N
                energy += avg_force * dx
        
        # Duration
        duration = times[-1] - times[0] if times else 0
        
        # Pass/fail calculations
        peak_force_pass = None
        if force_min is not None or force_max is not None:
            peak_force_pass = True
            if force_min is not None and peak_force < force_min:
                peak_force_pass = False
            if force_max is not None and peak_force > force_max:
                peak_force_pass = False
        
        endpoint_pass = None
        if endpoint_min is not None or endpoint_max is not None:
            endpoint_pass = True
            if endpoint_min is not None and endpoint < endpoint_min:
                endpoint_pass = False
            if endpoint_max is not None and endpoint > endpoint_max:
                endpoint_pass = False
        
        energy_pass = None
        if energy_min is not None or energy_max is not None:
            energy_pass = True
            if energy_min is not None and energy < energy_min:
                energy_pass = False
            if energy_max is not None and energy > energy_max:
                energy_pass = False
        
        # Overall pass/fail
        overall_pass = None
        pass_fail_reason = "No thresholds defined"
        
        checks = [
            (peak_force_pass, "force"),
            (endpoint_pass, "endpoint"),
            (energy_pass, "energy")
        ]
        
        defined_checks = [(p, n) for p, n in checks if p is not None]
        
        if defined_checks:
            failed = [n for p, n in defined_checks if not p]
            if failed:
                overall_pass = False
                pass_fail_reason = f"Failed: {', '.join(failed)}"
            else:
                overall_pass = True
                pass_fail_reason = "All thresholds met"
        
        # Energy visualization scaling
        # Scale the bar so thresholds are centered, not at extremes
        energy_percent = 50  # Default to middle
        energy_min_percent = 0
        energy_max_percent = 100
        energy_scale_max = energy  # The value that represents 100% on the bar
        
        if energy_min is not None and energy_max is not None:
            # Calculate a scale that puts thresholds at ~25% and ~75%
            # Scale max = max(energy, energy_max) + padding
            threshold_range = energy_max - energy_min
            padding = threshold_range * 0.5  # 50% padding above and below
            energy_scale_min = max(0, energy_min - padding)
            energy_scale_max = max(energy, energy_max + padding)
            scale_range = energy_scale_max - energy_scale_min
            
            if scale_range > 0:
                energy_percent = ((energy - energy_scale_min) / scale_range) * 100
                energy_min_percent = ((energy_min - energy_scale_min) / scale_range) * 100
                energy_max_percent = ((energy_max - energy_scale_min) / scale_range) * 100
        elif energy_max is not None and energy_max > 0:
            # Only max defined - add 20% headroom
            energy_scale_max = max(energy, energy_max) * 1.2
            energy_percent = (energy / energy_scale_max) * 100
            energy_max_percent = (energy_max / energy_scale_max) * 100
        elif energy > 0:
            energy_percent = 50  # If no max, show as middle
        
        return {
            'peak_force': peak_force,
            'peak_force_position': peak_force_position,
            'peak_force_pass': peak_force_pass,
            'start_position': start_position,
            'endpoint': endpoint,
            'endpoint_pass': endpoint_pass,
            'energy': energy,
            'energy_pass': energy_pass,
            'energy_percent': energy_percent,
            'energy_min_percent': energy_min_percent,
            'energy_max_percent': energy_max_percent,
            'duration': duration,
            'duration_str': f"{duration:.2f}s",
            'data_points': len(positions),
            'overall_pass': overall_pass,
            'pass_fail_reason': pass_fail_reason
        }
    
    def generate_report(self,
                        csv_path: str,
                        output_path: Optional[str] = None,
                        serial_number: str = "N/A",
                        device_name: str = "Fillhead",
                        firmware_version: str = "Unknown",
                        force_mode: str = "Unknown",
                        app_version: str = "Unknown",
                        job_number: str = "N/A",
                        op_number: str = "N/A",
                        title: str = "Press Operation Report",
                        force_min: Optional[float] = None,
                        force_max: Optional[float] = None,
                        endpoint_min: Optional[float] = None,
                        endpoint_max: Optional[float] = None,
                        energy_min: Optional[float] = None,
                        energy_max: Optional[float] = None,
                        press_startpoint: Optional[float] = None,
                        press_threshold: Optional[float] = None,
                        telemetry_endpoint: Optional[float] = None) -> Tuple[bool, str, Optional[str]]:
        """
        Generate a complete HTML report from a CSV log file.
        
        Args:
            csv_path: Path to the CSV log file
            output_path: Output path for the HTML report (auto-generated if None)
            serial_number: Serial number for the report
            device_name: Name of the device
            firmware_version: Firmware version string
            force_mode: Force sensing mode (load_cell or motor_torque)
            app_version: Application version
            job_number: Job number/identifier
            op_number: Operation number
            title: Report title
            force_min/max: Force thresholds for pass/fail
            endpoint_min/max: Endpoint thresholds for pass/fail
            energy_min/max: Energy thresholds for pass/fail
            
        Returns:
            Tuple of (success: bool, message: str, output_path: Optional[str])
        """
        try:
            # Parse CSV data
            data = self.parse_csv_log(csv_path)
            
            if not data['positions']:
                return (False, "No valid data found in CSV file", None)
            
            # Calculate metrics
            metrics = self.calculate_metrics(
                data,
                force_min=force_min,
                force_max=force_max,
                endpoint_min=endpoint_min,
                endpoint_max=endpoint_max,
                energy_min=energy_min,
                energy_max=energy_max
            )
            
            # Find the press range: from startpoint to endpoint (farthest position)
            # If startpoint is provided, filter data from that point onwards
            positions = data['positions']
            forces = data['forces']
            times = data['times']
            energies = data['energies']
            
            # Find peak force index
            peak_force = max(forces) if forces else 0
            peak_idx = forces.index(peak_force) if forces else 0
            
            # Find endpoint index (farthest position traveled)
            max_position = max(positions) if positions else 0
            endpoint_idx = positions.index(max_position) if positions else 0
            
            # Find startpoint index (where position crosses press_startpoint)
            start_idx = 0
            if press_startpoint is not None and press_startpoint > 0:
                for i, pos in enumerate(positions):
                    if pos >= press_startpoint:
                        start_idx = i
                        break
            
            # Clip data from startpoint to endpoint (farthest position reached)
            end_idx = max(endpoint_idx + 1, start_idx + 1)  # Include endpoint
            
            chart_positions = positions[start_idx:end_idx]
            chart_forces = forces[start_idx:end_idx]
            chart_times = times[start_idx:end_idx]
            chart_energies = energies[start_idx:end_idx] if energies else []
            
            # Prepare chart data as JSON
            chart_data = json.dumps({
                'positions': chart_positions,
                'forces': chart_forces,
                'times': chart_times,
                'energies': chart_energies,
                'full_positions': positions,
                'full_forces': forces
            })
            
            # Prepare raw data for table
            raw_data = []
            for i in range(len(data['positions'])):
                row = {
                    'time': data['times'][i],
                    'position': data['positions'][i],
                    'force': data['forces'][i],
                    'energy': data['energies'][i] if i < len(data['energies']) else 0
                }
                raw_data.append(row)
            
            # Prepare template context
            now = datetime.now()
            # Use detected force mode if not provided
            effective_force_mode = force_mode
            if force_mode in ('Unknown', 'N/A', None) and data.get('detected_force_mode'):
                effective_force_mode = data['detected_force_mode']
            
            # Use telemetry endpoint if available, and recalculate pass/fail
            print(f"[REPORT DEBUG] telemetry_endpoint={telemetry_endpoint}, metrics['endpoint']={metrics['endpoint']}")
            final_endpoint = telemetry_endpoint if telemetry_endpoint is not None else metrics['endpoint']
            print(f"[REPORT DEBUG] final_endpoint={final_endpoint}")
            final_endpoint_pass = None
            if endpoint_min is not None or endpoint_max is not None:
                final_endpoint_pass = True
                if endpoint_min is not None and final_endpoint < endpoint_min:
                    final_endpoint_pass = False
                if endpoint_max is not None and final_endpoint > endpoint_max:
                    final_endpoint_pass = False
            
            # Recalculate overall pass with corrected endpoint
            overall_pass = metrics['overall_pass']
            pass_fail_reason = metrics['pass_fail_reason']
            if final_endpoint_pass is not None and final_endpoint_pass != metrics['endpoint_pass']:
                # Endpoint pass/fail changed, recalculate overall
                checks = [
                    (metrics['peak_force_pass'], "force"),
                    (final_endpoint_pass, "endpoint"),
                    (metrics['energy_pass'], "energy")
                ]
                defined_checks = [(p, n) for p, n in checks if p is not None]
                if defined_checks:
                    failed = [n for p, n in defined_checks if not p]
                    if failed:
                        overall_pass = False
                        pass_fail_reason = f"Failed: {', '.join(failed)}"
                    else:
                        overall_pass = True
                        pass_fail_reason = "All checks passed"
            
            context = {
                'title': title,
                'serial_number': serial_number,
                'device_name': device_name,
                'firmware_version': firmware_version,
                'force_mode': effective_force_mode or 'N/A',
                'app_version': app_version,
                'job_number': job_number,
                'op_number': op_number,
                'date': data['start_time'].strftime("%Y-%m-%d") if data['start_time'] else now.strftime("%Y-%m-%d"),
                'time': data['start_time'].strftime("%H:%M:%S") if data['start_time'] else now.strftime("%H:%M:%S"),
                'duration': metrics['duration_str'],
                'generation_timestamp': now.strftime("%Y-%m-%d %H:%M:%S"),
                
                # Metrics
                'peak_force': metrics['peak_force'],
                'peak_force_pass': metrics['peak_force_pass'],
                'start_position': metrics['start_position'],
                # Use telemetry endpoint if available (more accurate than CSV last position)
                'endpoint': final_endpoint,
                'endpoint_pass': final_endpoint_pass,
                'energy': metrics['energy'],
                'energy_pass': metrics['energy_pass'],
                'energy_percent': metrics['energy_percent'],
                'energy_min_percent': metrics['energy_min_percent'],
                'energy_max_percent': metrics['energy_max_percent'],
                'overall_pass': overall_pass,
                'pass_fail_reason': pass_fail_reason,
                'data_points': metrics['data_points'],
                
                # Thresholds
                'force_min': force_min,
                'force_max': force_max,
                'endpoint_min': endpoint_min,
                'endpoint_max': endpoint_max,
                'energy_min': energy_min,
                'energy_max': energy_max,
                
                # Press startpoint and threshold
                'press_startpoint': press_startpoint,
                'press_threshold': press_threshold,
                
                # Chart data
                'chart_data': chart_data,
                
                # Raw data
                'raw_data': raw_data,
                'has_energy_data': data['has_energy_data']
            }
            
            # Render template
            template = self.env.get_template('press_report.html')
            html_content = template.render(**context)
            
            # Generate output path if not provided
            if output_path is None:
                csv_name = Path(csv_path).stem
                output_path = str(Path(csv_path).parent / f"{csv_name}_report.html")
            
            # Write output
            try:
                # Ensure output directory exists
                output_dir = Path(output_path).parent
                output_dir.mkdir(parents=True, exist_ok=True)
                
                with open(output_path, 'w', encoding='utf-8') as f:
                    f.write(html_content)
            except FileNotFoundError as e:
                return (False, f"Cannot write report - output path not found: {output_path} - {e}", None)
            except PermissionError as e:
                return (False, f"Cannot write report - permission denied: {output_path} - {e}", None)
            except OSError as e:
                return (False, f"Cannot write report - OS error: {output_path} - {e}", None)
            
            return (True, f"Report generated successfully: {output_path}", output_path)
            
        except FileNotFoundError as e:
            return (False, f"CSV file not found: {csv_path} - {e}", None)
        except PermissionError as e:
            return (False, f"CSV file access denied (file may be locked by another process): {csv_path} - {e}", None)
        except OSError as e:
            return (False, f"OS error reading CSV file: {csv_path} - {e}", None)
        except Exception as e:
            import traceback
            traceback.print_exc()
            return (False, f"Error generating report: {str(e)}", None)


def generate_press_report(csv_path: str,
                          output_path: Optional[str] = None,
                          serial_number: str = "N/A",
                          device_name: str = "Fillhead",
                          firmware_version: str = "Unknown",
                          force_mode: str = "Unknown",
                          app_version: str = "Unknown",
                          job_number: str = "N/A",
                          op_number: str = "N/A",
                          title: str = "Press Operation Report",
                          force_min: Optional[float] = None,
                          force_max: Optional[float] = None,
                          endpoint_min: Optional[float] = None,
                          endpoint_max: Optional[float] = None,
                          energy_min: Optional[float] = None,
                          energy_max: Optional[float] = None,
                          press_startpoint: Optional[float] = None,
                          press_threshold: Optional[float] = None,
                          telemetry_endpoint: Optional[float] = None) -> Tuple[bool, str, Optional[str]]:
    """
    Convenience function to generate a press report.
    
    This is the main entry point for report generation from scripts.
    See PressReportGenerator.generate_report() for full documentation.
    
    Returns:
        Tuple of (success: bool, message: str, output_path: Optional[str])
    """
    generator = PressReportGenerator()
    return generator.generate_report(
        csv_path=csv_path,
        output_path=output_path,
        serial_number=serial_number,
        device_name=device_name,
        firmware_version=firmware_version,
        force_mode=force_mode,
        app_version=app_version,
        job_number=job_number,
        op_number=op_number,
        title=title,
        force_min=force_min,
        force_max=force_max,
        endpoint_min=endpoint_min,
        endpoint_max=endpoint_max,
        energy_min=energy_min,
        energy_max=energy_max,
        press_startpoint=press_startpoint,
        press_threshold=press_threshold,
        telemetry_endpoint=telemetry_endpoint
    )


# CLI for testing
if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Generate press operation report')
    parser.add_argument('csv_file', help='Path to CSV log file')
    parser.add_argument('-o', '--output', help='Output HTML file path')
    parser.add_argument('-s', '--serial', default='TEST-001', help='Serial number')
    parser.add_argument('--force-min', type=float, help='Minimum force threshold')
    parser.add_argument('--force-max', type=float, help='Maximum force threshold')
    parser.add_argument('--endpoint-min', type=float, help='Minimum endpoint threshold')
    parser.add_argument('--endpoint-max', type=float, help='Maximum endpoint threshold')
    parser.add_argument('--energy-min', type=float, help='Minimum energy threshold')
    parser.add_argument('--energy-max', type=float, help='Maximum energy threshold')
    
    args = parser.parse_args()
    
    success, message, output = generate_press_report(
        csv_path=args.csv_file,
        output_path=args.output,
        serial_number=args.serial,
        force_min=args.force_min,
        force_max=args.force_max,
        endpoint_min=args.endpoint_min,
        endpoint_max=args.endpoint_max,
        energy_min=args.energy_min,
        energy_max=args.energy_max
    )
    
    print(message)
    sys.exit(0 if success else 1)

