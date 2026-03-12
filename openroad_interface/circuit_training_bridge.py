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
    
    MINICONDA_PYTHON = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "miniconda3", "bin", "python3.13")

    cmd = [
        MINICONDA_PYTHON, "-m", "eval_ct",
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

    if result.returncode != 0:
        print(f"Error running Circuit Training:\n{result.stderr}")
        raise RuntimeError("Circuit Training inference failed.")
        
    print("Circuit Training inference completed.")
    
    # search for the expected file or copy the newest .plc file matching pattern
    import glob
    # In eval_ct.py, EVAL_TESTCASE might be the dir name (e.g. "pd")
    plc_files = glob.glob(os.path.join(cwd, "eval_*.plc"))
    if plc_files:
        latest_plc = max(plc_files, key=os.path.getctime)
        os.rename(latest_plc, out_plc_path)
        print(f"Moved output PLC to {out_plc_path}")
    else:
        print(f"Warning: Could not find generated .plc file in {cwd}")

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
