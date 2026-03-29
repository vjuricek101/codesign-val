#from enum import verify
import logging
import re
import os
import copy
import shutil
from math import sqrt
import subprocess
import threading
import time

import logging

logger = logging.getLogger(__name__)

import networkx as nx

from openroad_interface import def_generator
from . import estimation as est
from . import scale_lef_files as scale_lef
from openroad_interface.lib_cell_generator import LibCellGenerator
from . import macro_maker as make_macros
from src import sim_util
## This is the area between the die area and the core area.
DIE_CORE_BUFFER_SIZE = 50


DEBUG = False
def log_info(msg):
    if DEBUG:
        logger.info(msg)
def log_warning(msg):
    if DEBUG:
        logger.warning(msg)

MAX_TRACTABLE_AREA_DBU = 1e20

TARGET_UTILIZATION = 0.5

# Minimum displacement (μm) for a macro to be considered "actively placed" by CT
# vs. left at the initial.plc position. Used by _extract_ct_placed_macros.
CT_MIN_PLACEMENT_DISPLACEMENT = 5.0


def _extract_ct_placed_macros(
    out_plc_path: str,
    init_plc_path: str,
    filtered_plc_path: str,
    min_displacement: float = 5.0
) -> int:
    """Compare CT output PLC vs initial PLC; keep only macros that moved.

    Macros CT didn't place remain at their initial.plc positions.  We detect
    active placement by comparing (x, y) in each file.  Any macro whose center
    moved by more than `min_displacement` μm is written to `filtered_plc_path`.

    Returns the number of actively-placed macros written.
    """
    def _parse_plc(path):
        positions = {}
        with open(path, 'r') as f:
            for line in f:
                s = line.strip()
                if s.startswith('#') or not s:
                    continue
                parts = s.split()
                if len(parts) >= 3:
                    try:
                        idx = parts[0]
                        x, y = float(parts[1]), float(parts[2])
                        positions[idx] = (x, y, line)
                    except ValueError:
                        pass
        return positions

    init_pos = _parse_plc(init_plc_path)
    out_pos  = _parse_plc(out_plc_path)

    # Copy header lines from output PLC
    header_lines = []
    with open(out_plc_path, 'r') as f:
        for line in f:
            if line.strip().startswith('#') or not line.strip():
                header_lines.append(line)
            else:
                break  # stop at first data line

    n_placed = 0
    data_lines = []
    for idx, (ox, oy, orig_line) in out_pos.items():
        if idx in init_pos:
            ix, iy, _ = init_pos[idx]
            dist = ((ox - ix)**2 + (oy - iy)**2) ** 0.5
            if dist >= min_displacement:
                data_lines.append(orig_line)
                n_placed += 1
        else:
            # macro not in initial.plc → always include
            data_lines.append(orig_line)
            n_placed += 1

    with open(filtered_plc_path, 'w') as f:
        f.writelines(header_lines)
        f.writelines(data_lines)

    return n_placed


# When True, monitor the OpenROAD log and terminate immediately on any
# line containing "ERROR" (case-insensitive). Set to False to disable.
OPENROAD_ABORT_ON_LOG_ERROR = True
OPENROAD_ERROR_TEXT = "error"

class OpenRoadRun:
    def __init__(self, cfg, codesign_root_dir, tmp_dir, run_openroad, circuit_model, subdirectory=None, custom_lef_files_to_include=None, top_level=True, memory_models=None):
        """
        Initialize the OpenRoadRun with configuration and root directory.

        :param cfg: top level codesign config file
        :param codesign_root_dir: root directory of codesign (where src and test are)
        :param tmp_dir: temporary directory for OpenROAD run
        :param run_openroad: flag to run OpenROAD or use previous results
        :param circuit_model: circuit model configuration
        :param subdirectory: subdirectory for hierarchical runs
        :param custom_lef_files_to_include: custom LEF files to include
        :param top_level: flag indicating if this is the top level of hierarchy
        :param memory_models: dict of memory name -> MemoryModel instances
        """
        self.cfg = cfg
        self.codesign_root_dir = codesign_root_dir
        self.tmp_dir = tmp_dir
        self.run_openroad = run_openroad
        self.directory = os.path.join(self.codesign_root_dir, f"{self.tmp_dir}/pd")
        self.subdirectory = subdirectory
        self.custom_lef_files_to_include = custom_lef_files_to_include
        self.top_level = top_level
        self.memory_models = memory_models or {}

        ## results will be placed here. This is necessary for running the flow hierarchically.
        if subdirectory is not None:
            self.directory = os.path.join(self.directory, subdirectory)

        self.circuit_model = circuit_model

        self.component_to_function = {
            "Mult16": "Mult16",
            "Add16": "Add16",
            "Sub16": "Sub16",
            "BUF_X4": "Not16",
            "BUF_X2": "Not16",
            "BUF_X1": "Not16",
            "BUF_X8": "Not16",
            "BUF_X16": "Not16",
            "BUF_X32": "Not16",
            "MUX2_X1": "Not16",
            "Mux16": "Mux16",
            "Exp16": "Exp16",
            "LShift16": "LShift16",
            "RShift16": "RShift16",
            "FloorDiv16": "FloorDiv16",
            "BitAnd16": "BitAnd16",
            "BitOr16": "BitOr16",
            "BitXor16": "BitXor16",
            "Eq16": "Eq16",
            "Not16": "Not16",
            "NotEq16": "NotEq16",
            "Register16": "Register16",
        }


    def run(
        self,
        graph: nx.DiGraph,
        test_file: str, 
        arg_parasitics: str,
        area_constraint: int,
        L_eff: float
    ):
        """
        Runs the OpenROAD flow.
        params:
            arg_parasitics: estimation, or none. Determines which parasitic calculation is executed.

        """
        self.L_eff = L_eff
        self.alpha = scale_lef.L_EFF_FREEPDK45 / self.L_eff
        self.original_graph = copy.deepcopy(graph)
        logger.info(f"Starting place and route with parasitics: {arg_parasitics}")
        d = {edge: {} for edge in graph.edges()}
        if "none" not in arg_parasitics:
            logger.info("Running setup for place and route.")

            all_call_functions = True
            for node in graph.nodes():
                if graph.nodes[node].get("function", "") != "Call":
                    all_call_functions = False
                    break

            graph, net_out_dict, node_output, lef_data, node_to_num, final_area, dbu_area_estimate = self.setup(graph, test_file, area_constraint, L_eff)

            # If all nodes in the graph have the function type "Call", skip place and route.        
            if all_call_functions:
                logger.info("All nodes in the graph have function type 'Call'. Skipping place and route.")
                return d, graph, final_area

            # if the total DBU^2 area of all macros is greater than a limit, skip place and route.
            if dbu_area_estimate > MAX_TRACTABLE_AREA_DBU:
                logger.info(f"Total DBU area {dbu_area_estimate} exceeds area constraint {area_constraint}. Skipping place and route.")
                return d, graph, final_area

            logger.info("Setup complete. Running extraction.")
            d, graph = self.extraction(graph, arg_parasitics, net_out_dict, node_output, lef_data, node_to_num)
            logger.info("Extraction complete.")
        else: 
            logger.info("No parasitics selected. Running none_place_n_route.")
            graph = self.none_place_n_route(graph)
            final_area = 0
            d = {}
        logger.info("Place and route finished.")
        return d, graph, final_area

    def setup(
        self,
        graph: nx.DiGraph,
        test_file: str,
        area_constraint: int,
        L_eff: float
    ):
        """
        Sets up the OpenROAD environment. This method creates the working directory, copies tcl files, and generates the def file
        param:
            graph: hardware netlist graph
            test_file: tcl file
            
            area_constraint: area constraint for the placement. We will ensure that the final area constraint set to OpenROAD
                achieves at least 60% utilization based on the estimated area from the def generator.
            L_eff: effective channel length used to scale the LEF files.
            
            NOTE about outputs from setup_set_area_constraint:
                This function is very much legacy and not pretty to look at. Apologies in advance.
                There are some quantities which correspond to a "higher level" graph, where edges are from one functional unit to another.
                On the other hand, things like graph have edges for each of the 16 ports on a functional unit, as quantities are 16 bit right now.

                higher level graph: node_output
                lower level graph: graph, net_out_dict, node_to_num

        """

        old_graph = copy.deepcopy(graph)
        graph, net_out_dict, node_output, lef_data, node_to_num, area_estimate, max_dim_macro, macro_dict, dbu_area_estimate = self.setup_set_area_constraint(graph, test_file, area_constraint, L_eff)

        area_constraint_old = area_constraint
        logger.info(f"Max dimension macro: {max_dim_macro}, corresponding area constraint value: {max_dim_macro**2}")
        logger.info(f"Estimated area: {area_estimate}")
        area_constraint = int(max(area_estimate, max_dim_macro**2)/TARGET_UTILIZATION)
        logger.info(f"Info: Final estimated area {area_estimate} compared to area constraint {area_constraint_old}. Area constraint will be scaled from {area_constraint_old} to {area_constraint}.")
        graph, net_out_dict, node_output, lef_data, node_to_num, area_estimate, max_dim_macro, macro_dict, dbu_area_estimate = self.setup_set_area_constraint(old_graph, test_file, area_constraint, L_eff)

        lib_cell_generator = LibCellGenerator()
        lib_cell_generator.generate_and_write_cells(macro_dict, self.circuit_model, self.directory + "/tcl/codesign_files/codesign_typ.lib")

        self.update_clock_period(self.directory + "/tcl/codesign_files/codesign.sdc")

        # --- Circuit Training Integration ---
        use_circuit_training = False
        args_dict = self.cfg.get("args") if isinstance(self.cfg, dict) else None
        if isinstance(args_dict, dict):
            use_circuit_training = args_dict.get("use_circuit_training", False)
            
        if use_circuit_training:
            logger.info("Circuit Training is enabled. Preparing to run macro placement inference.")
            import openroad_interface.circuit_training_bridge as ct_bridge
            design_name = "codesign"
            out_pb_path = os.path.join(self.directory, "results", f"{design_name}.pb.txt")
            def_file = os.path.join(self.directory, "results", "first_generated.def")
            
            # Gather LEF files
            lef_files = [
                os.path.join(self.directory, "tcl", "codesign_files", "codesign_tech.lef"),
                os.path.join(self.directory, "tcl", "codesign_files", "codesign_stdcell.lef")
            ]
            if self.custom_lef_files_to_include:
                lef_files.extend(self.custom_lef_files_to_include)
                
            try:
                # Convert LEF/DEF to PB
                logger.info(f"Converting LEF/DEF to PB format for {design_name}")
                preinstalled = args_dict.get("preinstalled_openroad_path")
                openroad_bin = preinstalled if preinstalled else os.path.join(self.codesign_root_dir, "openroad_src", "build", "bin", "openroad")
                lib_file = os.path.join(self.directory, "tcl", "codesign_files", "codesign_typ.lib")
                ct_bridge.convert_lef_def_to_pb(lef_files, def_file, design_name, out_pb_path, openroad_bin=openroad_bin, lib_file=lib_file)
                
                # Run inference
                out_plc_path = os.path.join(self.directory, "results", f"{design_name}_placed.plc")
                run_dir = args_dict.get("ct_run_dir", "run_00")
                ckpt_id = args_dict.get("ct_ckpt_id", "policy_checkpoint_0000103984")
                
                init_plc_path = os.path.join(self.directory, "results", "initial.plc")
                import sys, re
                pb_helper_path = os.path.join(self.codesign_root_dir, "openroad_interface", "circuit_training", "tools", "TILOS_MacroPlacement", "Flows", "util")
                if pb_helper_path not in sys.path:
                    sys.path.append(pb_helper_path)
                from pb_helper import pb_design
                
                logger.info(f"Generating valid initial.plc for {design_name}")
                design = pb_design(design_name, out_pb_path)
                design.read_netlist()
                
                # ----------------------------------------------------------------
                # Extract core bounds from DEF: parse ROWS for site-snapped origin
                # and DIEAREA as a fallback.
                # ----------------------------------------------------------------
                core_w, core_h = 1000.0, 1000.0   # fallback defaults
                core_x1, core_y1 = DIE_CORE_BUFFER_SIZE, DIE_CORE_BUFFER_SIZE
                try:
                    with open(def_file, 'r') as f:
                        def_content = f.read()

                    units_match = re.search(r'UNITS\s+DISTANCE\s+MICRONS\s+([\d.]+)', def_content)
                    dbu = float(units_match.group(1)) if units_match else 2000.0

                    # Try to get exact core bounds from ROW statements.
                    # ROW <name> <site> <origX> <origY> <orient> DO <numX> BY <numY> STEP <stepX> <stepY> ;
                    row_matches = re.findall(
                        r'ROW\s+\S+\s+\S+\s+([\d.]+)\s+([\d.]+)\s+\S+\s+DO\s+(\d+)\s+BY\s+(\d+)\s+STEP\s+([\d.]+)\s+([\d.]+)',
                        def_content
                    )
                    if row_matches:
                        # All rows share the same X origin (site-snapped core LL)
                        first = row_matches[0]
                        last  = row_matches[-1]
                        core_x1_dbu = float(first[0])
                        core_y1_dbu = float(first[1])
                        # Core upper bound: last row origin + step_y (one site height)
                        last_row_y  = float(last[1])
                        step_y_dbu  = float(last[5])
                        num_x       = int(first[2])
                        step_x_dbu  = float(first[4])

                        core_x1 = core_x1_dbu / dbu
                        core_y1 = core_y1_dbu / dbu
                        # width from first row: numX sites × step_x
                        core_w  = (num_x * step_x_dbu) / dbu
                        # height from first to last row + one site height
                        core_h  = (last_row_y + step_y_dbu - float(first[1])) / dbu
                        logger.info(
                            f"Parsed DEF ROWS: core origin ({core_x1:.4f}, {core_y1:.4f}), "
                            f"size {core_w:.4f}x{core_h:.4f} μm"
                        )
                    else:
                        # Fallback: DIEAREA minus buffer
                        die_match = re.search(
                            r'DIEAREA\s*\(\s*([\d.]+)\s+([\d.]+)\s*\)\s*\(\s*([\d.]+)\s+([\d.]+)\s*\)',
                            def_content
                        )
                        if die_match:
                            dx1, dy1, dx2, dy2 = map(float, die_match.groups())
                            die_w = (dx2 - dx1) / dbu
                            die_h = (dy2 - dy1) / dbu
                            core_w  = die_w - 2 * DIE_CORE_BUFFER_SIZE
                            core_h  = die_h - 2 * DIE_CORE_BUFFER_SIZE
                            core_x1 = DIE_CORE_BUFFER_SIZE
                            core_y1 = DIE_CORE_BUFFER_SIZE
                            logger.info(
                                f"Parsed DEF DIEAREA (fallback): die {die_w:.2f}x{die_h:.2f} → "
                                f"core {core_w:.2f}x{core_h:.2f} μm"
                            )
                except Exception as e:
                    logger.warning(f"Failed to parse DEF for initial.plc geometry: {e}")

                # ----------------------------------------------------------------
                # Extract routes per micron from tech LEF (sum over all routing
                # layers grouped by preferred direction).
                # ----------------------------------------------------------------
                route_hor, route_ver = 0.0, 0.0
                tech_lef = lef_files[0]   # first LEF is always the tech LEF
                try:
                    with open(tech_lef, 'r') as f:
                        lef_content = f.read()

                    for layer_block in re.finditer(
                        r'LAYER\s+\w+\s+(.*?)END\s+\w+', lef_content, re.DOTALL
                    ):
                        blk = layer_block.group(1)
                        if 'TYPE ROUTING' not in blk:
                            continue
                        dir_m   = re.search(r'DIRECTION\s+(HORIZONTAL|VERTICAL)', blk)
                        pitch_m = re.search(r'PITCH\s+([\d.]+)', blk)
                        if dir_m and pitch_m:
                            pitch = float(pitch_m.group(1))
                            if pitch > 0:
                                if dir_m.group(1) == 'HORIZONTAL':
                                    route_hor += 1.0 / pitch
                                else:
                                    route_ver += 1.0 / pitch

                    logger.info(
                        f"Parsed tech LEF routes per μm: hor={route_hor:.4f}, ver={route_ver:.4f}"
                    )
                except Exception as e:
                    logger.warning(f"Failed to parse tech LEF for routing capacity: {e}. "
                                   f"Using Nangate45 defaults (19.73 / 14.28).")
                    route_hor = 19.73
                    route_ver = 14.28

                design.plc_info.columns = 128
                design.plc_info.rows    = 128
                design.plc_info.width   = core_w
                design.plc_info.height  = core_h
                design.plc_info.area    = core_w * core_h
                design.plc_info.route_hor     = route_hor
                design.plc_info.route_ver     = route_ver
                design.plc_info.sm_factor     = 5
                design.plc_info.ovrlp_thrshld = 0.004

                design.write_plc(init_plc_path)
                logger.info(f"initial.plc: canvas {core_w:.1f}×{core_h:.1f} μm, "
                            f"grid 128×128, route_hor={route_hor:.2f}, route_ver={route_ver:.2f}")

                # Store the exact core origin so TCL conversion uses it correctly
                ct_core_origin = (core_x1, core_y1)
                
                ct_fully_placed = ct_bridge.run_circuit_training_inference(out_pb_path, init_plc_path, out_plc_path, run_dir, ckpt_id)
                
                if not os.path.exists(out_plc_path):
                    logger.warning("Circuit Training did not produce a .plc file. "
                                   "Skipping macro pre-placement — OpenROAD will use its default analytical placer.")
                elif not ct_fully_placed:
                    logger.warning(
                        f"Circuit Training placed only a partial set of macros (episode return < 0, "
                        f"wirelength metric = -1). Skipping CT TCL injection — OpenROAD will use "
                        f"its default rtl_macro_placer for all macros."
                    )
                else:
                    # CT placed ALL macros → inject placement constraints
                    partial_plc_path = out_plc_path + ".ct_placed_only.plc"
                    n_placed = _extract_ct_placed_macros(
                        out_plc_path, init_plc_path, partial_plc_path,
                        CT_MIN_PLACEMENT_DISPLACEMENT
                    )
                    if n_placed == 0:
                        logger.warning("CT produced no useful macro placements. Skipping TCL injection.")
                    else:
                        logger.info(f"CT placed {n_placed} macro(s). Generating placement TCL.")
                        out_tcl_file = os.path.join(self.directory, "tcl", "circuit_training_macro_place.tcl")
                        ct_bridge.convert_pb_placement_to_tcl(
                            partial_plc_path, out_pb_path, out_tcl_file,
                            origin_x=ct_core_origin[0], origin_y=ct_core_origin[1]
                        )

                        with open(os.path.join(self.directory, "tcl", "codesign_flow.tcl"), "r") as f:
                            flow_tcl = f.read()

                        # Suppress rtl_macro_placer (CT placed all 148 macros)
                        flow_tcl = re.sub(
                            r'(\n\s*rtl_macro_placer\b.*?)(?=\n\n|\n\s*[a-z_]|$)',
                            lambda m: m.group(1).replace('\n', '\n# '),
                            flow_tcl, flags=re.DOTALL
                        )
                        logger.info("Suppressed rtl_macro_placer (CT placed all macros).")

                        # Always write the latest placeInstance proc with error handling
                        new_place_proc = (
                            "\nproc placeInstance {inst x y orient status} {\n"
                            "    catch {\n"
                            "        set block [[[ord::get_db] getChip] getBlock]\n"
                            "        set iobj [$block findInst $inst]\n"
                            "        if {$iobj ne \"NULL\"} { $iobj setPlacementStatus PLACED }\n"
                            "    }\n"
                            "    if {[catch {place_macro -macro_name $inst"
                            " -location [list $x $y] -orientation $orient} err]} {\n"
                            "        puts \"WARNING: CT could not place $inst: $err\"\n"
                            "    }\n"
                            "}\n"
                        )
                        if "proc placeInstance" in flow_tcl:
                            import re as _re2
                            flow_tcl = _re2.sub(
                                r'proc placeInstance \{[^}]+\}\s*\{[^{}]*(?:\{[^{}]*\}[^{}]*)*\}',
                                new_place_proc.strip(), flow_tcl, flags=_re2.DOTALL
                            )
                        else:
                            flow_tcl = new_place_proc + flow_tcl

                        flow_tcl = flow_tcl.replace("#source macro_place.tcl",
                                                    "source circuit_training_macro_place.tcl")

                        with open(os.path.join(self.directory, "tcl", "codesign_flow.tcl"), "w") as f:
                            f.write(flow_tcl)
                    
            except Exception as e:
                logger.error(f"Circuit training macro placement failed: {e}. Falling back to standard OpenROAD analytical placer.")
        # --- End Circuit Training Integration ---

        self.update_top_level_flag()

        final_area = area_estimate

        return graph, net_out_dict, node_output, lef_data, node_to_num, final_area, dbu_area_estimate

    def update_top_level_flag(self):
        """
        Updates the top level flag in the codesign.vars file.
        """

        ## set at_top_level_of_hierarchy flag in codesign.vars
        with open(self.directory + "/tcl/codesign_files/codesign.vars", "r") as file:
            vars_data = file.readlines()
        for i, line in enumerate(vars_data):
            if line.startswith("set at_top_level_of_hierarchy"):
                vars_data[i] = f"set at_top_level_of_hierarchy {'1' if self.top_level else '0'}\n"
                logger.info(f"Updated at_top_level_of_hierarchy to {'1' if self.top_level else '0'}")
        with open(self.directory + "/tcl/codesign_files/codesign.vars", "w") as file:
            file.writelines(vars_data)
    
    def update_clock_period(self, sdc_file: str):
        """
        Updates the clock period in the SDC file.
        param:
            sdc_file: path to the SDC file
        """
        with open(sdc_file, "r") as file:
            sdc_data = file.readlines()
        assert sdc_data[0].startswith("create_clock")
        new_clock_period = self.circuit_model.tech_model.base_params.tech_values[self.circuit_model.tech_model.base_params.clk_period]
        sdc_data[0] = f"create_clock [get_ports clk] -name core_clock -period {new_clock_period}\n"
        with open(sdc_file, "w") as file:
            file.writelines(sdc_data)
        logger.info(f"Updated clock period in SDC file to {new_clock_period}")

    def setup_set_area_constraint(
        self,
        graph: nx.DiGraph,
        test_file: str,
        area_constraint: int,
        L_eff: float
    ):
        """
        This is a helper method that runs the setup for a single area constraint and provides an area estimate. 
        param:
            graph: hardware netlist graph
            test_file: tcl file
            
            area_constraint: area constraint for the placement. We will ensure that the final area constraint set to OpenROAD
                achieves at least 60% utilization based on the estimated area from the def generator.
            L_eff: effective channel length used to scale the LEF files.
        """

        logger.info("Setting up environment for place and route.")
        if self.run_openroad:
            if os.path.exists(self.directory):
                logger.info(f"Removing existing directory: {self.directory}")
                shutil.rmtree(self.directory)
            os.makedirs(self.directory)
            logger.info(f"Created directory: {self.directory}")
            shutil.copytree(os.path.dirname(os.path.abspath(__file__)) + "/tcl", self.directory + "/tcl")
            logger.info(f"Copied tcl files to {self.directory}/tcl")
            os.makedirs(self.directory + "/results")
            logger.info(f"Created results directory: {self.directory}/results")
        else:
            logger.info("Skipping setup, using previous openroad results.")

        # Build extra macro definitions for memory/fifo nodes in the graph
        extra_area_list = {}
        extra_pin_list = {}
        for node, data in graph.nodes(data=True):
            fn = data.get("function", "")
            if fn not in ("memory", "fifo"):
                continue
            node_name = data.get("name", "")
            macro_name = f"MEM_{node_name}"
            if macro_name in extra_area_list:
                continue
            mem_model = self.memory_models.get(node_name)
            if mem_model and hasattr(mem_model, "cacheArea_mm2"):
                # Destiny areas are already at the simulated tech node (65nm).
                # Divide by AREA_SCALE_FACTOR so MacroMaker's blanket scaling cancels out,
                # leaving the raw Destiny area in the LEF.
                area_um2 = mem_model.cacheArea_mm2 * 1e6 / make_macros.AREA_SCALE_FACTOR
            else:
                raise Exception(f"Memory model or area not found for {node_name}")
            extra_area_list[macro_name] = area_um2
            extra_pin_list[macro_name] = {"input": 32, "output": 16}
            self.component_to_function[macro_name] = fn
            log_info(f"Memory macro {macro_name}: area={area_um2:.2f} um²")

        macro_maker = make_macros.MacroMaker(self.cfg, self.codesign_root_dir, self.tmp_dir, self.run_openroad, self.subdirectory, output_lef_file=self.directory + "/tcl/codesign_files/codesign_stdcell.lef", custom_lef_files_to_include=self.custom_lef_files_to_include)

        macro_maker.create_all_macros(extra_area_list=extra_area_list, extra_pin_list=extra_pin_list)

        self.update_area_constraint(area_constraint)

        self.do_scale_lef = scale_lef.ScaleLefFiles(self.cfg, self.codesign_root_dir, self.tmp_dir, self.subdirectory)
        self.do_scale_lef.scale_lef_files(L_eff)

        logger.info(f"Generating DEF file for {self.codesign_root_dir}/{self.tmp_dir}/{self.subdirectory}")
        df = def_generator.DefGenerator(self.cfg, self.codesign_root_dir, self.tmp_dir, self.do_scale_lef.NEW_database_units_per_micron, self.subdirectory)

        graph, net_out_dict, node_output, lef_data, node_to_num, area_estimate, macro_dict, self.node_to_component_num = df.run_def_generator(
            test_file, graph
        )

        dbu_area_estimate = area_estimate * (self.do_scale_lef.NEW_database_units_per_micron ** 2)

        logger.info(f"DEF generation complete. Area estimate: {area_estimate}")
        logger.info(f"Max dimension macro: {df.max_dim_macro}")
        logger.info(f"DBU area (area estimate in um2 * (DBU per micron)^2): {dbu_area_estimate}")

        self.scale_rc_values()

        return graph, net_out_dict, node_output, lef_data, node_to_num, area_estimate, df.max_dim_macro, macro_dict, dbu_area_estimate

    def scale_rc_values(self):
        """
        Scales the RC values in the tcl file based on the input L_eff.
        param:
            L_eff: effective channel length used to scale the LEF files.
        """
        with open(self.directory + "/tcl/codesign_files/codesign.rc", "r") as file:
            rc_data = file.readlines()
        for i, line in enumerate(rc_data):
            if line.startswith("set_layer_rc"):
                metal_layer = line.split()[2]
                rsq = sim_util.xreplace_safe(self.circuit_model.tech_model.wire_parasitics["R"][metal_layer], self.circuit_model.tech_model.base_params.tech_values) * 1e-9 # convert to kohm/um
                csq = sim_util.xreplace_safe(self.circuit_model.tech_model.wire_parasitics["C"][metal_layer], self.circuit_model.tech_model.base_params.tech_values) * 1e+15 * 1e-6 # convert to fF/um
                # need to scale up RC for mature nodes, because unscaled OpenROAD wirelengths will be too short
                # for advanced nodes the unscaled wirelengths will be too long
                resistance = rsq / self.alpha
                capacitance = csq / self.alpha
                rc_data[i] = f"set_layer_rc -layer {metal_layer} -resistance {resistance} -capacitance {capacitance}\n"
        with open(self.directory + "/tcl/codesign_files/codesign.rc", "w") as file:
            file.writelines(rc_data)

    def update_area_constraint(self, area_constraint: int):
        """
        Updates the area constraint in the tcl file based on the input area constraint.
        param:
            area_constraint: area constraint for the placement
        """
        ## edit the tcl file to have the correct area constraint
        with open(self.directory + "/tcl/codesign_top.tcl", "r") as file:
            tcl_data = file.readlines()

        ## compute the new area constraint
        new_core_sidelength = int(sqrt(area_constraint))

        #new_core_sidelength_x = new_core_sidelength * 2
        #new_core_sidelength_y = int(area_constraint / new_core_sidelength_x)

        ## find a line that contains "set die_area" and replace it with the new area constraint
        for i, line in enumerate(tcl_data):
            if "set die_area" in line:
                tcl_data[i] = f"set die_area {{0 0 {new_core_sidelength + DIE_CORE_BUFFER_SIZE*2} {new_core_sidelength + DIE_CORE_BUFFER_SIZE*2}}}\n"
                #tcl_data[i] = f"set die_area {{0 0 {new_core_sidelength_x + DIE_CORE_BUFFER_SIZE*2} {new_core_sidelength_y + DIE_CORE_BUFFER_SIZE*2}}}\n"
                logger.info(f"Updated die_area to {new_core_sidelength + DIE_CORE_BUFFER_SIZE*2}x{new_core_sidelength + DIE_CORE_BUFFER_SIZE*2}")
                #logger.info(f"Updated die_area to {new_core_sidelength_x + DIE_CORE_BUFFER_SIZE*2}x{new_core_sidelength_y + DIE_CORE_BUFFER_SIZE*2}")
            if "set core_area" in line:
                tcl_data[i] = f"set core_area {{{DIE_CORE_BUFFER_SIZE} {DIE_CORE_BUFFER_SIZE} {new_core_sidelength + DIE_CORE_BUFFER_SIZE} {new_core_sidelength + DIE_CORE_BUFFER_SIZE}}}\n"
                #tcl_data[i] = f"set core_area {{{DIE_CORE_BUFFER_SIZE} {DIE_CORE_BUFFER_SIZE} {new_core_sidelength_x + DIE_CORE_BUFFER_SIZE} {new_core_sidelength_y + DIE_CORE_BUFFER_SIZE}}}\n"
                logger.info(f"Updated core_area to {new_core_sidelength}x{new_core_sidelength}")
                #logger.info(f"Updated core_area to {new_core_sidelength_x + DIE_CORE_BUFFER_SIZE*2}x{new_core_sidelength_y + DIE_CORE_BUFFER_SIZE*2}")

        ## write the new tcl file
        with open(self.directory + "/tcl/codesign_top.tcl", "w") as file:
            file.writelines(tcl_data)
        
        logger.info(f"Wrote updated tcl file with the area constraints: {new_core_sidelength}x{new_core_sidelength}")


    def run_openroad_executable(self):
        """
        Runs the OpenROAD executable. Run this after setup.
        """
        import subprocess
        import shutil
        import os
        
        logger.info("Starting OpenROAD run.")
        old_dir = os.getcwd()
        os.chdir(self.directory + "/tcl")
        logger.info(f"Changed directory to {self.directory + '/tcl'}")
        print("Running OpenROAD and monitoring log for 'ERROR' (if enabled)...")
        logger.info("Running OpenROAD command.")
        
        # Safely handle missing/malformed cfg entries for preinstalled_openroad_path.
        args_dict = self.cfg.get("args") if isinstance(self.cfg, dict) else None
        preinstalled = None
        if isinstance(args_dict, dict):
            preinstalled = args_dict.get("preinstalled_openroad_path")

        # Check if xvfb-run is available
        xvfb_available = shutil.which("xvfb-run") is not None
        
        if preinstalled:
            openroad_cmd = preinstalled
        else:
            openroad_bin = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "openroad_src", "build", "bin", "openroad")
            openroad_cmd = openroad_bin
        
        # Set up environment for Qt/OpenGL software rendering
        env = os.environ.copy()
        
        # Qt platform configuration - use offscreen platform for headless rendering
        # This avoids X11/XCB issues entirely and works natively without Xvfb
        env['QT_QPA_PLATFORM'] = 'offscreen'
        
        # Ensure OpenGL software rendering is available for offscreen platform
        env['LIBGL_ALWAYS_SOFTWARE'] = '1'
        env['GALLIUM_DRIVER'] = 'llvmpipe'
        env['MESA_LOADER_DRIVER_OVERRIDE'] = 'llvmpipe'
        env['MESA_GL_VERSION_OVERRIDE'] = '3.3'
        
        # Add .local/lib64 and .local/lib to LD_LIBRARY_PATH for dependencies
        local_lib64 = os.path.join(self.codesign_root_dir, ".local", "lib64")
        local_lib = os.path.join(self.codesign_root_dir, ".local", "lib")
        
        current_ld_path = env.get("LD_LIBRARY_PATH", "")
        paths = [local_lib64, local_lib]
        if current_ld_path:
            paths.append(current_ld_path)
        env["LD_LIBRARY_PATH"] = ":".join(paths)
        logger.info(f"Set LD_LIBRARY_PATH to {env['LD_LIBRARY_PATH']}")
        
        # Ensure Qt can find image format plugins (PNG support)
        # Try common system locations for Qt5 plugins
        possible_plugin_paths = [
            "/usr/lib64/qt5/plugins",
            "/usr/lib/qt5/plugins", 
            "/usr/lib/x86_64-linux-gnu/qt5/plugins",
            os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                        "OpenROAD", "build", "src", "plugins")
        ]
        for qt_plugin_path in possible_plugin_paths:
            if os.path.exists(qt_plugin_path):
                env['QT_PLUGIN_PATH'] = qt_plugin_path
                logger.info(f"Set QT_PLUGIN_PATH to {qt_plugin_path}")
                break
        
        # Ensure imageformats directory is accessible for PNG support
        # Qt5 should have built-in PNG support, but plugins help
        imageformat_path = "/usr/lib64/qt5/plugins/imageformats"
        if os.path.exists(imageformat_path):
            # Set QT_PLUGIN_PATH to include the parent plugins directory
            # Qt will automatically look in plugins/imageformats subdirectory
            if 'QT_PLUGIN_PATH' not in env:
                env['QT_PLUGIN_PATH'] = "/usr/lib64/qt5/plugins"
                logger.info("Set QT_PLUGIN_PATH to include imageformats")
        
        # Also try setting QT_QPA_PLATFORM_PLUGIN_PATH if needed
        # This helps Qt find platform-specific plugins
        if 'QT_QPA_PLATFORM_PLUGIN_PATH' not in env:
            platform_plugin_path = "/usr/lib64/qt5/plugins/platforms"
            if os.path.exists(platform_plugin_path):
                env['QT_QPA_PLATFORM_PLUGIN_PATH'] = platform_plugin_path
        
        # Build the command - with offscreen platform, we don't need Xvfb
        # Offscreen platform works natively without X server
        cmd = f"{openroad_cmd} codesign_top.tcl"
        logger.info("Using Qt offscreen platform for headless image rendering (no Xvfb needed)")

        # Redirect output to log file
        log_file = f"{self.directory}/codesign_pd.log"
        
        logger.info("Executing OpenROAD command: %s", cmd)
        logger.info("Environment: LIBGL_ALWAYS_SOFTWARE=%s, QT_QPA_PLATFORM=%s", 
                    env.get('LIBGL_ALWAYS_SOFTWARE'), env.get('QT_QPA_PLATFORM'))
        
        # Use subprocess to properly handle environment and output redirection
        with open(log_file, 'w') as log:
            result = subprocess.run(
                cmd,
                shell=True,
                env=env,
                cwd=os.getcwd(),
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT
            )
        
        print("done")
        logger.info("OpenROAD run completed.")
        os.chdir(old_dir)
        logger.info(f"Returned to original directory {old_dir}")


    def mux_listing(self, graph, node_output, wire_length_by_edge):
        """
        goes through the graph and finds nodes that are not Muxs. If it encounters one, it will go through
        the graph to find the path of Muxs until the another non-Mux node is found. All rcl are put into a
        list and added as an edge attribute for the non-mux node to non-mux node connection

        param:
            graph: graph with the net attributes already attached
            node_output: dict of nodes and their respective outputs
        """
        #print(f"wire_length_by_edge before modification: {wire_length_by_edge}")
        logger.info("Starting mux listing.")
        edges_to_remove = set()
        for node in node_output:
            #print(f"considering node {node}")
            if "Mux" not in node:
                #print(f"outputs of {node}: {node_output[node]}")
                for output in node_output[node]:
                    path = []
                    if "Mux" in output:
                        while "Mux" in output:
                            # wire delay doesn't need to take all 16 paths into account, so just use 0. For energy, multiply by 16.
                            path.append(output + "_0")
                            output = node_output[output][0]
                        graph.add_edge(node, output)
                        node_name = graph.nodes[node]["name"]
                        output_name = graph.nodes[output]["name"]
                        #logger.info(f"Src: {node_name}, Dst: {output_name}")
                        #print(f"path from {node} to {output}: {path}")
                        if len(path) != 0 and (node_name, output_name) not in wire_length_by_edge:
                            #print(f"adding wire length by edge")
                            path_dsts = [graph.nodes[p]["name"] for p in path]
                            path_dst = path_dsts[0]
                            wire_length_by_edge[(node_name, output_name)] = wire_length_by_edge[(node_name, path_dst)]
                            edges_to_remove.add((node_name, path_dsts[0]))
                            for i in range(1, len(path)):
                                wire_length_by_edge[(node_name, output_name)]["total_wl"] += wire_length_by_edge[(path_dsts[i-1], path_dsts[i])]["total_wl"]
                                wire_length_by_edge[(node_name, output_name)]["metal1"] += wire_length_by_edge[(path_dsts[i-1], path_dsts[i])]["metal1"]
                                wire_length_by_edge[(node_name, output_name)]["metal2"] += wire_length_by_edge[(path_dsts[i-1], path_dsts[i])]["metal2"]
                                wire_length_by_edge[(node_name, output_name)]["metal3"] += wire_length_by_edge[(path_dsts[i-1], path_dsts[i])]["metal3"]
                                edges_to_remove.add((path_dsts[i-1], path_dsts[i]))
                            wire_length_by_edge[(node_name, output_name)]["total_wl"] += wire_length_by_edge[(path_dsts[-1], output_name)]["total_wl"]
                            wire_length_by_edge[(node_name, output_name)]["metal1"] += wire_length_by_edge[(path_dsts[-1], output_name)]["metal1"]
                            wire_length_by_edge[(node_name, output_name)]["metal2"] += wire_length_by_edge[(path_dsts[-1], output_name)]["metal2"]
                            wire_length_by_edge[(node_name, output_name)]["metal3"] += wire_length_by_edge[(path_dsts[-1], output_name)]["metal3"]
                            edges_to_remove.add((path_dsts[-1], output_name))
                            #print(f"wire length by edge after modification: {wire_length_by_edge[(node, output)]}")
        for edge in edges_to_remove:
            #print(f"removing edge {edge}")
            wire_length_by_edge.pop(edge)
        return wire_length_by_edge


    def mux_removal(self, graph: nx.DiGraph):
        """
        Removes the mux nodes from the graph. Does not do the connecting
        param:
            graph: graph with the new edge connections, after mux listing
        """
        log_info("Removing mux nodes from graph.")
        reference = copy.deepcopy(graph.nodes())
        for node in reference:
            if "Mux" in node:
                graph.remove_node(node)
                log_info(f"Removed mux node: {node}")


    def coord_scraping(
        self,
        graph: nx.DiGraph,
        node_to_num: dict,
        final_def_directory: str = None,
    ):
        """
        going through the .def file and getting macro placements and nets
        param:
            graph: digraph to add coordinate attribute to nodes
            node_to_num: dict that gives component id equivalent for node
            final_def_directory: final def directory, defaults to def directory in openroad
        return:
            graph: digraph with the new coordinate attributes
            component_nets: dict that list components for the respective net id
        """
        log_info("Scraping coordinates and nets from DEF file.")
        pattern = r"_\w+_\s+\w+\s+\+\s+PLACED\s+\(\s*\d+\s+\d+\s*\)\s+\w+\s*;"
        net_pattern = r"-\s(_\d+_)\s((?:\(\s_\d+_\s\w+\s\)\s*)+).*"
        component_pattern = r"(_\w+_)"
        if final_def_directory is None:
            final_def_directory = self.directory + "/results/final_generated-tcl.def"
        final_def_data = open(final_def_directory)
        final_def_lines = final_def_data.readlines()
        macro_coords = {}
        component_nets = {}
        for line in final_def_lines:
            if re.search(pattern, line) is not None:
                coord = re.findall(r"\((.*?)\)", line)[0].split()
                match = re.search(component_pattern, line)
                macro_coords[match.group(0)] = {"x": float(coord[0]), "y": float(coord[1])}
                log_info(f"Found macro {match.group(0)} at ({coord[0]}, {coord[1]})")
            if re.search(net_pattern, line) is not None:
                pins = re.findall(r"\(\s(.*?)\s\w+\s\)", line)
                match = re.search(component_pattern, line)
                component_nets[match.group(0)] = pins
                log_info(f"Found net {match.group(0)} with pins {pins}")

        for node in node_to_num:
            coord = macro_coords[node_to_num[node]]
            graph.nodes[node]["x"] = coord["x"]
            graph.nodes[node]["y"] = coord["y"]
            log_info(f"Assigned coordinates to node {node}: {coord}")
        log_info("Coordinate scraping complete.")
        return graph, component_nets


        
    

    def extraction(self, graph, arg_parasitics, net_out_dict, node_output, lef_data, node_to_num): 
        # 3. extract parasitics
        logger.info(f"Starting extraction with parasitics option: {arg_parasitics}")
        d = {}
        if arg_parasitics == "estimation":
            logger.info("Running estimated place and route.")
            d, graph = self.estimated_place_n_route(
                graph, net_out_dict, node_output, lef_data, node_to_num
            )
            logger.info("Estimated extraction complete.")
        else:
            raise ValueError(f"Invalid parasitics option: {arg_parasitics}")

        return d, graph

    # buffers may have been inserted onto edges of the netlist so add them to a new graph
    def parse_new_netlist_graph(self):
        """
        Parses the new netlist graph and adds attributes to the graph
        """
        net_id_to_src_dsts = {}
        new_graph = nx.DiGraph()
        logger.info("Parsing new netlist graph.")
        with open(self.directory + "/results/final_generated-tcl.def", "r") as file:
            def_data = file.readlines()
        for i in range(len(def_data)):
            if def_data[i].startswith("COMPONENTS"):
                break
        for j in range(i+1, len(def_data)): # PARSE COMPONENTS
            if def_data[j].startswith("END COMPONENTS"):
                break
            component_id = def_data[j].split()[1]
            component_name = def_data[j].split()[2]
            if component_name not in self.component_to_function and "HIERMODULE" not in component_name:
                log_info(f"Component {component_name} not found in component_to_function. Skipping.")
                continue
            if "HIERMODULE" in component_name:
                component_function = "Call"
            else:
                component_function = self.component_to_function[component_name]
            new_graph.add_node(component_id, function=component_function)
            log_info(f"Added node {component_id} with function {component_function}")
        for k in range(j+1, len(def_data)):
            if def_data[k].startswith("NETS"):
                break
        
        # PARSE NETS - handle multi-line nets
        l = k + 1
        while l < len(def_data):
            if def_data[l].startswith("END NETS"):
                break
            
            # Check if this line starts a new net
            if def_data[l].strip().startswith("-"):
                # Collect all lines until the net ends with ";"
                net_lines = [def_data[l]]
                while not net_lines[-1].rstrip().endswith(";"):
                    l += 1
                    assert not (l >= len(def_data) or def_data[l].startswith("END NETS")), f"End of netlist reached before net {net_name} was fully parsed, def_data: {def_data[l]}"
                    net_lines.append(def_data[l])
                
                # Concatenate all lines for this net
                full_net_line = " ".join(net_lines)

                log_info(f"Full net line: {full_net_line}")
                # Parse the net: LINE FORMAT: - <net_name> ( <src_node> <src_pin> ) ( <dst_node_0> <dst_pin_0> ) ( <dst_node_1> <dst_pin_1> ) ...
                line_items = full_net_line.split()
                net_name = line_items[1]
                src_node = line_items[3]
                
                if src_node not in new_graph.nodes():
                    log_info(f"Source node {src_node} not found in graph. Skipping net {net_name}.")
                    l += 1
                    continue
                
                # Extract destination nodes (every 4th item after the 3rd, skipping src_pin)
                dst_nodes = []
                for idx in range(7, len(line_items), 4):
                    if idx >= len(line_items):
                        break
                    dst_node = line_items[idx]
                    if dst_node not in new_graph.nodes():
                        log_info(f"Destination node {dst_node} not found in graph. Skipping.")
                        continue
                    dst_nodes.append(dst_node)
                    new_graph.add_edge(src_node, dst_node, net=net_name)
                    log_info(f"Added edge from {src_node} to {dst_node} for net {net_name}")
                net_id_to_src_dsts[net_name] = (src_node, dst_nodes)
            l += 1
        self.export_graph(new_graph, "new_netlist_graph", self.directory)
        return new_graph, net_id_to_src_dsts

    def estimated_place_n_route(
        self,
        graph: nx.DiGraph,
        net_out_dict: dict,
        node_output: dict,
        lef_data: dict,
        node_to_num: dict,
    ) -> dict:
        """
        runs openroad, calculates rcl, and then adds attributes to the graph

        params:
            graph: networkx graph
            net_out_dict: dict that lists nodes and thier respective edges (all nodes have one output)
            node_output: dict that lists nodes and their respective output nodes
            lef_data: dict with layer information (units, res, cap, width)
            node_to_num: dict that gives component id equivalent for node
        returns:
            dict: contains list of resistance, capacitance, length, and net data
            graph: newly modified digraph with rcl attributes
        """

        # run openroad
        logger.info("Starting estimated place and route.")
        if self.run_openroad:
            self.run_openroad_executable()
        else:
            logger.info("Skipping openroad run.")

        ## if the graph has no edges, then return empty dict
        if len(graph.edges()) == 0:
            logger.info("Graph has no edges. Skipping estimated place and route.")
            return {}, graph
        
        self.updated_graph, net_id_to_src_dsts = self.parse_new_netlist_graph()

        nets = est.parse_route_guide_with_layer_breakdown(
            self.directory + "/results/codesign_codesign-tcl.route_guide",
            updated_graph=self.updated_graph,
            net_id_to_src_dsts=net_id_to_src_dsts,
        )
        for net in nets.values():
            for segment in net.segments:
                #logger.info(f"segment length for net {net.net_id} in layer {segment.layer} was {segment.length}")
                segment.length /= self.alpha * 1e6 # convert to meters
                #logger.info(f"segment length for net {net.net_id} in layer {segment.layer} is {segment.length}")

        # Build mapping from original-graph edge (src, dst) -> list of net ids along the path in updated_graph
        self.edge_to_nets: dict[tuple[str, str], list[str]] = {}

        for src, dst in self.original_graph.edges():
            src_component_name = self.original_graph.nodes[src]["name"]
            dst_component_name = self.original_graph.nodes[dst]["name"]
            src_component_num = self.node_to_component_num[src]
            dst_component_num = self.node_to_component_num[dst]
            # Only process if both endpoints exist in the updated graph
            assert src_component_num in self.updated_graph and dst_component_num  in self.updated_graph, f"Source or destination node not found in updated graph: {src}:{src_component_num}, {dst}:{dst_component_num}"

            log_info(f"Finding path from {src}:{src_component_num} to {dst}:{dst_component_num}")
            # Find a path through repeaters from src to dst in updated_graph
            # There should be a unique simple path; use shortest_simple_paths or single_source shortest path
            try:
                path_nodes = nx.shortest_path(self.updated_graph, source=src_component_num, target=dst_component_num)
            except Exception as e:
                log_warning(f"Error finding path from {src}:{src_component_num} to {dst}:{dst_component_num}: {e}")
                path_nodes = []


            # Collect net ids on each hop of the path
            nets_on_path = []
            for u, v in zip(path_nodes[:-1], path_nodes[1:]):
                if (u, v) in self.updated_graph.edges():
                    nets_on_path.append(copy.deepcopy(nets[self.updated_graph.edges[u, v]["net"]]))
                else:
                    log_warning(f"Edge not found in updated graph: {u}:{v}")
            
            # add self edge if it exists, won't be captured by nx shortest path
            if src == dst:
                if (src_component_num, src_component_num) in self.updated_graph.edges():
                    nets_on_path.append(copy.deepcopy(nets[self.updated_graph.edges[src_component_num, src_component_num]["net"]]))
                else:
                    log_warning(f"Self edge not found in updated graph: {src}:{src_component_num}, {dst}:{dst_component_num}")

            self.edge_to_nets[(src_component_name, dst_component_name)] = nets_on_path
        
        log_info(f"edge_to_nets: {self.edge_to_nets}")

        # Expose for downstream consumers
        self.export_graph(graph, "estimated", self.directory)

        return self.edge_to_nets, graph


    def none_place_n_route(
        self,
        graph: nx.DiGraph,
    ) -> dict:
        """
        runs openroad, calculates rcl, and then adds attributes to the graph
        params:
            graph: networkx graph
        returns:
            graph: newly modified digraph with rcl attributes
        """

        # edge attribution
        logger.info("Running none_place_n_route: setting default edge attributes.")
        for u, v in graph.edges():
            graph[u][v]["net"] = 0
            graph[u][v]["net_length"] = 0
            graph[u][v]["net_res"] = 0
            graph[u][v]["net_cap"] = 0
            #logger.info(f"Set default attributes for edge ({u}, {v})")

        logger.info("none_place_n_route finished.")
        return graph
    

    @staticmethod
    def export_graph(graph, est_or_det: str, directory: str):
        logger.info(f"Exporting graph to GML for {est_or_det}.")
        if not os.path.exists(f"{directory}/results/"):
            os.makedirs(f"{directory}/results/")
            logger.info("Created results directory.")
        nx.write_gml(
            graph, f"{directory}/results/{est_or_det}.gml"
        )
        logger.info(f"Graph exported to {directory}/results/{est_or_det}.gml")
