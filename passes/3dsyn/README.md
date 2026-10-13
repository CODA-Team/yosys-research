# 3D-Syn Yosys Passes

This directory implements the Yosys-side synthesis capabilities used by
3D-Syn. The passes identify and preserve coarse units, normalize instance names
for physical-design handoffs, and generate 2D and tier-specific Liberty views
after coarse-unit characterization.

The sources are compiled directly into the TaiWei Yosys executable through
`Makefile.inc`; they are not a standalone tool or a dynamically loaded plugin.
The complete flow driver is located in `TaiWei-Pin-3D/scripts_3dsyn`.

## Relation to the 3D-Syn flow

| 3D-Syn function | Yosys command | Flow stage |
| --- | --- | --- |
| Identify arithmetic and multibit sequential coarse units and create preserved wrappers | `syn3d_mark_coarse_units` | Coarse-Grained Netlist Construction |
| Produce Verilog- and DEF-safe instance names | `syn3d_normalize_instance_names` | Coarse-Grained Netlist Construction and subsequent handoffs |
| Generate footprint-aware 2D, bottom-tier, and upper-tier Liberty views | `syn3d_write_coarse_unit_libs` | Routability-Aware Area Estimation within `coarse_synth` |

## Source layout

| File | Purpose |
| --- | --- |
| `Makefile.inc` | Adds the 3D-Syn sources to the Yosys build. |
| `syn3d_mark_coarse_units.cc` | Implements coarse-unit recognition, wrapper generation, canonical RTLIL export, and output-connection normalization. |
| `syn3d_normalize_instance_names.cc` | Implements deterministic, unique, handoff-safe instance renaming. |
| `syn3d_write_coarse_unit_libs.cc` | Builds 2D and tier-specific Liberty views from characterized coarse-unit netlists and footprint metrics. |
| `yosys_json_toolkit.h/.cc` | Internal JSON-netlist parsing and graph utilities used by Liberty generation. |

## Build

Build from the `yosys-research` repository root using its standard dependencies
(including Bison 3.6 or newer):

```bash
git submodule update --init --recursive
make -j <jobs>
./yosys -p "help syn3d_mark_coarse_units"
./yosys -p "help syn3d_normalize_instance_names"
./yosys -p "help syn3d_write_coarse_unit_libs"
```

The default full build discovers `passes/3dsyn/Makefile.inc` automatically.
These commands are not supplied by an unmodified upstream Yosys executable.

When used through the TaiWei ORFS integration branch, `build_openroad.sh`
selects `tools/yosys-research` and installs it under `tools/install/yosys`:

```bash
cd ORFS-Research
./build_openroad.sh --local --threads <jobs>
```

The stage drivers in TaiWei-Pin-3D provide the netlists, characterization
metrics and environment variables required by the commands below.

## Commands

### `syn3d_mark_coarse_units`

Identifies eligible internal Yosys cells, groups instances with the same type
and interface into coarse-unit types, and replaces them with preserved wrapper
modules. For each type, the command writes a canonical RTLIL source that can be
synthesized independently.

```text
syn3d_mark_coarse_units [options] [selection]
```

| Option | Description |
| --- | --- |
| `-keep all\|arithmetic\|multibit_ff\|none` | Select the coarse-unit categories to identify. The default is `all`. |
| `-output-dir <directory>` | Existing directory for canonical coarse-unit RTLIL files. Required in wrapping mode. |
| `-manifest <file>` | File receiving the sorted coarse-unit type names, one per line. The default is `<output-dir>/coarse_units.list`. |
| `-normalize` | Normalize non-wire output connections of existing coarse-unit instances. This is a separate mode and cannot be combined with the wrapping options. |
| `[selection]` | Optional standard Yosys selection limiting the modules and cells examined. |

Wrapper modules are marked with the `syn3d_coarse_unit` attribute. The sorted
type list is also stored in the `syn3d.coarse_units` design scratchpad entry.

Example:

```tcl
syn3d_mark_coarse_units \
  -keep all \
  -output-dir coarse_units \
  -manifest coarse_units.list
```

### `syn3d_normalize_instance_names`

Renames selected instances to unique names containing only letters, digits,
and underscores, making them safe for Verilog and DEF-based handoffs.

```text
syn3d_normalize_instance_names [-coarse-units-only] [selection]
```

| Option | Description |
| --- | --- |
| `-coarse-units-only` | Rename only instances whose module is marked with `syn3d_coarse_unit`. Without this option, all selected instances are normalized. |
| `[selection]` | Optional standard Yosys selection limiting the modules and cells renamed. |

### `syn3d_write_coarse_unit_libs`

Reads the characterized bottom- and upper-tier implementations for every module
marked with `syn3d_coarse_unit`. It derives bit-level timing arcs using
per-input path-depth estimation and writes three Liberty views:

```text
syn3d_write_coarse_unit_libs
```

The command has no command-line options. Its flow context is supplied through
environment variables:

| Variable | Required | Description |
| --- | --- | --- |
| `SYN3D_HANDOFF_DIR` | Yes | Root containing tier netlists and footprint metrics; generated Liberty files are written below this directory. |
| `RESULTS_DIR` | Yes | Result directory used for Liberty-generation logs. |
| `FLOW_HOME` | Yes | TaiWei-Pin-3D flow root used to resolve platform data. |
| `LIBRARY` | Yes | 3D platform: `asap7_3D`, `nangate45_3D`, or `asap7_nangate45_3D`. |
| `YOSYS_EXE` | No | Yosys executable used to read back characterized netlists. The TaiWei-installed Yosys is used by default. |

For each `<coarse-unit-type>`, the command consumes:

```text
SYN3D_HANDOFF_DIR/
├── coarse_units/{bottom,upper}/<coarse-unit-type>.v
└── metrics/{bottom,upper}/<coarse-unit-type>/metrics.json
```

It produces:

```text
SYN3D_HANDOFF_DIR/lib/
├── 2d/
│   ├── coarse_units.lib
│   └── <coarse-unit-type>.lib
├── bottom/<coarse-unit-type>.bottom.lib
└── upper/<coarse-unit-type>.upper.lib
```

The 2D view uses the bottom-tier implementation and retains the unsuffixed
coarse-unit type name used by the coarse netlist. Tier-specific cell names use
the `_bottom` and `_upper` suffixes. `coarse_units.lib` combines all unsuffixed
2D cells under one library header and is the coarse-partition handoff; the
per-unit 2D files remain available for inspection. The command also writes
read-back topology files under `SYN3D_HANDOFF_DIR/lib_topology`; these are
diagnostic work products, not downstream flow inputs.

## Terminology

A **coarse-unit instance** is one occurrence in the top-level netlist. A
**coarse-unit type** is the unique primitive, parameter, and interface
combination shared by one or more instances. A **wrapper module** is the Yosys
representation used to preserve that type through synthesis. Physical area is
reported as the **coarse-unit footprint**.
