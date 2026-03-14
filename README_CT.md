# Circuit Training (CT) Integration

## 1. Edits to the Original Codebase

### `openroad_run.py`
*   **Canvas Scaling**: Parses `DIEAREA`/`UNITS` to map AI 1000x1000 grid to physical microns.
*   **Tcl Injection**: Replaces `rtl_macro_placer` with AI results in `codesign_flow.tcl`.
*   **Command Mapping**: Maps AI `placeInstance` to OpenROAD `place_macro` via Tcl proc.

### `circuit_training_bridge.py`
*   Bridge between Python flow and TILOS translators.

## 2. Modified Circuit Training Components

Integrated TILOS format converter as recommended on Circuit Training GitHub. 

### Original TILOS Structure (Legacy):
1. Input `.lef`/`.def`
2. Cluster standard cells with `partition_design` (deprecated OpenROAD function, unneeded)
3. OpenROAD translation: outputs `.hgr.io`, `.hgr.instance` (these do not exist in any OpenROAD commit, including the pinned version; the OpenROAD version inside the repo also errors)
4. Converting with Python `ODB2ProBufFormat` (`.hgr.*` -> `.netlist.pb.txt`)

### Resolved Structure:
1.  **Macro Netlist Extraction**: OpenROAD loads the HLS-generated LEF/DEF files. Custom script (`extract_hgr.tcl`) uses the `OpenDB` API to scrape the connectivity graph and physical dimensions.
2.  **Graph Encoding**: Following original TILOS script, the hypergraph data is translated into a TensorFlow `.pb.txt` (GraphDef) format.
3.  **RL Inference**: The Circuit Training agent evaluates the graph against a pre-trained policy and generates optimal (x, y) coordinates for all macros, saved in a `.plc` file.
4.  **OpenROAD Implementation**: The `circuit_training_bridge` converts `.plc` to Tcl and inject into OpenROAD.

#### `extract_hgr.tcl`
*   **Location**: `openroad_interface/circuit_training/.../FormatTranslators/src/`
*   Replaces `partition_design`; extracts IO, macros, and nets for macro-only designs.

| File | Change | Rationale |
| :--- | :--- | :--- |
| `eval_ct.py` | Set `save_placement=True` | Forces the RL agent to export the `.plc` coordinate file upon completion. |
| `plc_client.py` | Added stubs for `get_fake_nets()` | Prevents crashes when certain metrics are null. |
| `observation_config.py` | Normalized `max_num_edges` | Aligned dimensions for policy checkpoint. |
| `plc_pb_to_placement.py` | Fixed Coordinate Offsets & Escaping (`'[]'` in macro names) | Handle macro origin offsets and bracket escaping. |
| `FormatTranslators.py` | Integrated `extract_hgr.tcl` | Use extract_hgr.tcl as entry point. |

---

## 3. Usage
*   **Flag**: `use_circuit_training: True` in config YAML. 
*   **Policy**: `policy_checkpoint_0000103984`.