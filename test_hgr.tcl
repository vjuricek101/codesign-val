set top_design gemm
set report_dir /pool0/vjuricek/codesign-val/src/tmp/tmp_gemm_edp_81/pd/results

read_lef /pool0/vjuricek/codesign-val/src/tmp/tmp_gemm_edp_81/pd/tcl/codesign_files/codesign_tech.lef
read_lef /pool0/vjuricek/codesign-val/src/tmp/tmp_gemm_edp_81/pd/tcl/codesign_files/codesign_stdcell.lef
read_def /pool0/vjuricek/codesign-val/src/tmp/tmp_gemm_edp_81/pd/results/first_generated.def

source /pool0/vjuricek/codesign-val/openroad_interface/circuit_training/tools/TILOS_MacroPlacement/CodeElements/FormatTranslators/src/extract_hgr.tcl
