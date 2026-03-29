import sys
import os
# TILOS FormatTranslators directory 
CT_TOOLS_DIR = os.path.join(
    os.path.dirname(__file__),
    "circuit_training", 
    "tools", 
    "TILOS_MacroPlacement", 
    "CodeElements", 
    "FormatTranslators", 
    "src"
)
sys.path.append(CT_TOOLS_DIR)

from FormatTranslators import LefDef2ProBufFormat

def convert_lef_def_to_pb(lef_files: list, def_file: str, design_name: str, out_pb_path: str, openroad_bin="openroad", lib_file=None):
    """
    Parses LEF and DEF files and converts them into the Circuit Training
    Protocol Buffer (Tensorflow GraphDef) format.
    """
    
    openroad_exe = os.environ.get("OPENROAD_EXE", openroad_bin)
    net_size_threshold = 300 # Default threshold from TILOS test scripts
    
    # The TILOS script automatically writes to "[design_name].pb.txt" in the current directory.
    # We will let it run, and then move/rename the output file to out_pb_path.
    
    print(f"Running TILOS LefDef2ProBufFormat for {design_name}...")
    LefDef2ProBufFormat(lef_files, def_file, design_name, openroad_exe, net_size_threshold, lib_file=lib_file)
    
    # The TILOS script writes to `[design_name].pb.txt`
    tilos_out_file = f"{design_name}.pb.txt"
    if os.path.exists(tilos_out_file):
        os.rename(tilos_out_file, out_pb_path)
        print(f"Successfully created PB netlist at: {out_pb_path}")
    else:
        print(f"Error: Format conversion failed. Expected output {tilos_out_file} not found.")

def run_circuit_training_inference(netlist_pb_path: str, init_plc_path: str, out_plc_path: str, run_dir: str, ckpt_id: str):
    """
    Executes the Circuit Training RL agent to place macros.
    Expects a pre-trained policy to exist at: ./saved_policy/<run_dir>/<seed>/policy_saved_model/checkpoints/<ckpt_id>
    """
    import subprocess
    
    eval_script = os.path.join(
        os.path.dirname(__file__),
        "circuit_training",
        "tools",
        "TILOS_MacroPlacement",
        "CodeElements",
        "EvalCT",
        "eval_ct.py"
    )
    
    # Use the current python executable (the one from your activated conda env)
    # instead of a hardcoded path to the base environment.
    PYTHON_EXE = sys.executable

    cmd = [
        PYTHON_EXE, "-m", "eval_ct",
        "--netlist", netlist_pb_path,
        "--plc", init_plc_path,
        "--rundir", run_dir,
        "--ckptID", ckpt_id
    ]
    
    # run from the EvalCT directory
    cwd = os.path.dirname(eval_script)
    print(f"Running Circuit Training inference: {' '.join(cmd)}")
    
    # this may be scuffed
    openroad_interface_dir = os.path.dirname(os.path.abspath(__file__))
    ct_root_dir = os.path.join(openroad_interface_dir, "circuit_training")
    env = os.environ.copy()
    env["PYTHONPATH"] = ct_root_dir
    result = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, env=env)

    if result.stdout:
        print(f"Inference Output:\n{result.stdout}")
    if result.stderr:
        print(f"Inference Errors:\n{result.stderr}")

    if result.returncode != 0:
        raise RuntimeError("Circuit Training inference failed.")
        
    print("Circuit Training inference completed.")
    
    # Detect whether CT placed all macros successfully.
    # successful placements have wirelength >= 0 (real placement cost).
    # TF/absl logging goes to stderr, so search both streams.
    ct_fully_placed = False
    combined_output = (result.stdout or "") + (result.stderr or "")
    if combined_output:
        import re as _re
        # Primary check: wirelength metric >= 0 means real placement (feasible)
        m_wl = _re.search(r'InfoMetric_wirelength\s*=\s*([\-\d.eE+]+)', combined_output)
        if m_wl:
            wl_metric = float(m_wl.group(1))
            ct_fully_placed = wl_metric >= 0.0
            m_ep = _re.search(r'eval_episode_return\s*=\s*([\-\d.eE+]+)', combined_output)
            ep_return = float(m_ep.group(1)) if m_ep else float('nan')
            m_steps = _re.search(r'EnvironmentSteps\s*=\s*(\d+)', combined_output)
            steps = int(m_steps.group(1)) if m_steps else -1
            status = 'FEASIBLE (all macros placed)' if ct_fully_placed else 'INFEASIBLE (partial placement)'
            print(f"CT result: {status} | steps={steps} | wirelength={wl_metric:.4f} | return={ep_return:.4f}")
    
    import glob, shutil
    # eval_ct.py writes the output as: ./eval_<rundir>_to_<EVAL_TESTCASE>.plc
    # EVAL_TESTCASE is derived from the netlist path: the directory two levels above results/
    # e.g. .../tmp_gemm_edp_1/pd/results/codesign.pb.txt  ->  tmp_gemm_edp_1
    parts = netlist_pb_path.replace("\\", "/").split("/")
    eval_testcase = "codesign_eval"
    if "results" in parts:
        idx = parts.index("results")
        if idx > 0:
            eval_testcase = parts[idx - 1]
            if eval_testcase == "pd" and idx > 1:
                eval_testcase = parts[idx - 2]

    expected_plc_name = f"eval_{run_dir}_to_{eval_testcase}.plc"
    expected_plc_path = os.path.join(cwd, expected_plc_name)
    print(f"Looking for output PLC at: {expected_plc_path}")

    if os.path.exists(expected_plc_path):
        shutil.copy2(expected_plc_path, out_plc_path)
        print(f"Copied output PLC to {out_plc_path}")
    else:
        # Fallback: find the most recently created eval_*.plc in the EvalCT dir
        plc_files = glob.glob(os.path.join(cwd, "eval_*.plc"))
        if plc_files:
            latest_plc = max(plc_files, key=os.path.getctime)
            shutil.copy2(latest_plc, out_plc_path)
            print(f"Copied output PLC (fallback) to {out_plc_path}")
        else:
            msg = f"Could not find generated .plc file in {cwd}. Expected: {expected_plc_name}"
            print(f"Warning: {msg}")
            raise RuntimeError(msg)
    
    return ct_fully_placed

def convert_pb_placement_to_tcl(plc_file: str, pb_file: str, out_tcl_file: str, origin_x: float = 0.0, origin_y: float = 0.0):
    """
    Converts a Circuit Training .plc output and the corresponding .pb.txt 
    into an OpenROAD .tcl script containing placement constraints.
    """
    import subprocess
    
    converter_script = os.path.join(
        os.path.dirname(__file__),
        "circuit_training",
        "tools",
        "TILOS_MacroPlacement",
        "Flows",
        "util",
        "plc_pb_to_placement_tcl.py"
    )
    
    cmd = [
        "python3", converter_script,
        plc_file,
        pb_file,
        out_tcl_file,
        str(origin_x),
        str(origin_y)
    ]
    
    print(f"Converting PLC to TCL: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Error converting PLC to TCL:\n{result.stderr}")
        raise RuntimeError("PLC to TCL conversion failed.")
        
    print(f"Successfully generated OpenROAD placement TCL at {out_tcl_file}")

if __name__ == "__main__":
    if len(sys.argv) < 5:
        print("Usage: python circuit_training_bridge.py <design_name> <out_pb_path> <def_file> <lef_file_1> [<lef_file_2> ...]")
        sys.exit(1)
        
    design_name = sys.argv[1]
    out_pb = sys.argv[2]
    def_file = sys.argv[3]
    lef_files = sys.argv[4:]
    
    convert_lef_def_to_pb(lef_files, def_file, design_name, out_pb)
