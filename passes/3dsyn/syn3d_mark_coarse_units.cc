#include "kernel/yosys.h"
#include "kernel/celltypes.h"
#include "backends/rtlil/rtlil_backend.h"

USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

struct Syn3dMarkCoarseUnitsPass : Pass
{
	enum class KeepMode {
		All,
		Arithmetic,
		MultibitFf,
		None,
	};

	Syn3dMarkCoarseUnitsPass() :
		Pass("syn3d_mark_coarse_units", "identify and wrap coarse units for 3D-Syn")
	{
	}

	void help() override
	{
		log("\n");
		log("    syn3d_mark_coarse_units [options] [selection]\n");
		log("\n");
		log("Identify selected arithmetic and multibit flip-flop cells as coarse units.\n");
		log("Each coarse-unit type is represented by a wrapper module and emitted as a\n");
		log("standalone canonical RTLIL source.\n");
		log("\n");
		log("    -keep all|arithmetic|multibit_ff|none\n");
		log("        Select the coarse-unit categories to identify. The default is all.\n");
		log("\n");
		log("    -output-dir <directory>\n");
		log("        Write standalone coarse-unit RTLIL files to this existing directory.\n");
		log("        This option is required in wrapping mode.\n");
		log("\n");
		log("    -manifest <file>\n");
		log("        Write the sorted coarse-unit type names, one per line. The default is\n");
		log("        <output-dir>/coarse_units.list.\n");
		log("\n");
		log("    -normalize\n");
		log("        Normalize output connections of existing coarse-unit instances.\n");
		log("        This mode does not create wrappers or write output files.\n");
		log("\n");
	}

	const char *keep_mode_name(KeepMode mode) const
	{
		switch (mode) {
		case KeepMode::All:
			return "all";
		case KeepMode::Arithmetic:
			return "arithmetic";
		case KeepMode::MultibitFf:
			return "multibit_ff";
		case KeepMode::None:
			return "none";
		}
		log_abort();
	}

	KeepMode parse_keep_mode(const std::string &value) const
	{
		if (value == "all")
			return KeepMode::All;
		if (value == "arithmetic")
			return KeepMode::Arithmetic;
		if (value == "multibit_ff")
			return KeepMode::MultibitFf;
		if (value == "none")
			return KeepMode::None;
		log_cmd_error("Unsupported -keep mode '%s'; expected all, arithmetic, multibit_ff, or none.\n", value.c_str());
	}

	bool is_wrappable_for_mode(RTLIL::IdString type, KeepMode mode) const
	{
		static const pool<RTLIL::IdString> arithmetic_types = {
			ID($add), ID($sub), ID($mul), ID($div), ID($mod),
			ID($divfloor), ID($modfloor), ID($alu), ID($fa), ID($macc),
		};
		static const pool<RTLIL::IdString> multibit_ff_types = {
			ID($dff), ID($dffe), ID($dffsr), ID($dffsre),
			ID($adff), ID($adffe),
			ID($sdff), ID($sdffe), ID($sdffce),
		};

		switch (mode) {
		case KeepMode::All:
			return arithmetic_types.count(type) || multibit_ff_types.count(type);
		case KeepMode::Arithmetic:
			return arithmetic_types.count(type);
		case KeepMode::MultibitFf:
			return multibit_ff_types.count(type);
		case KeepMode::None:
			return false;
		}
		log_abort();
	}

	bool is_ff(RTLIL::IdString type) const
	{
		static const pool<RTLIL::IdString> ff_types = {
			ID($dff), ID($dffe), ID($dffsr), ID($dffsre),
			ID($adff), ID($adffe), ID($aldff), ID($aldffe),
			ID($sdff), ID($sdffe), ID($sdffce),
		};
		return ff_types.count(type);
	}

	bool keep_wrapper_port(const std::pair<RTLIL::IdString, RTLIL::SigSpec> &connection) const
	{
		return connection.second.size() > 0;
	}

	RTLIL::IdString make_wrapper_name(RTLIL::Cell *cell) const
	{
		std::string name = RTLIL::unescape_id(cell->type);
		for (char &character : name)
			if (character == '$')
				character = '_';

		std::vector<std::pair<std::string, int>> ports;
		for (const auto &connection : cell->connections()) {
			if (keep_wrapper_port(connection))
				ports.push_back({RTLIL::unescape_id(connection.first), connection.second.size()});
		}
		std::sort(ports.begin(), ports.end());

		for (const auto &port : ports)
			name += "_" + port.first + std::to_string(port.second);
		return RTLIL::escape_id(name);
	}

	void add_specify_arc(RTLIL::Module *module, RTLIL::Wire *input, RTLIL::Wire *output, int &index) const
	{
		RTLIL::Cell *arc = module->addCell(stringf("\\$specify2_%d", index++), ID($specify2));
		arc->setPort(ID::EN, RTLIL::State::S1);
		arc->setPort(ID::SRC, input);
		arc->setPort(ID::DST, output);
		arc->setParam(ID::T_RISE_MAX, RTLIL::Const(1000, 32));
		arc->setParam(ID::T_FALL_MAX, RTLIL::Const(1000, 32));
		arc->setParam(ID::T_RISE_MIN, RTLIL::Const(1000, 32));
		arc->setParam(ID::T_FALL_MIN, RTLIL::Const(1000, 32));
		arc->setParam(ID::T_RISE_TYP, RTLIL::Const(1000, 32));
		arc->setParam(ID::T_FALL_TYP, RTLIL::Const(1000, 32));
		arc->setParam(ID::FULL, RTLIL::Const(1, 32));
		arc->setParam(ID::SRC_DST_PEN, RTLIL::Const(0, 32));
		arc->setParam(ID::SRC_DST_POL, RTLIL::Const(0, 32));
		arc->setParam(ID::SRC_WIDTH, RTLIL::Const(GetSize(input), 32));
		arc->setParam(ID::DST_WIDTH, RTLIL::Const(GetSize(output), 32));
	}

	RTLIL::Module *create_wrapper(RTLIL::Design *design, RTLIL::Cell *cell,
		RTLIL::IdString wrapper_name, CellTypes &cell_types) const
	{
		RTLIL::Module *wrapper = design->addModule(wrapper_name);
		for (const auto &connection : cell->connections()) {
			if (!keep_wrapper_port(connection))
				continue;
			RTLIL::Wire *wire = wrapper->addWire(connection.first, connection.second.size());
			if (cell_types.cell_output(cell->type, connection.first))
				wire->port_output = true;
			else
				wire->port_input = true;
		}
		wrapper->fixup_ports();

		if (!is_ff(cell->type)) {
			std::vector<RTLIL::Wire *> inputs;
			std::vector<RTLIL::Wire *> outputs;
			for (RTLIL::IdString port_name : wrapper->ports) {
				RTLIL::Wire *wire = wrapper->wire(port_name);
				if (wire->port_input)
					inputs.push_back(wire);
				if (wire->port_output)
					outputs.push_back(wire);
			}
			int arc_index = 0;
			for (RTLIL::Wire *input : inputs)
				for (RTLIL::Wire *output : outputs)
					add_specify_arc(wrapper, input, output, arc_index);
			wrapper->set_bool_attribute(ID(abc9_box));
		}

		wrapper->set_bool_attribute(ID(blackbox));
		wrapper->set_bool_attribute(ID(syn3d_coarse_unit));
		return wrapper;
	}

	void write_primitive_rtlil(RTLIL::Cell *cell, RTLIL::IdString wrapper_name,
		CellTypes &cell_types, const std::string &output_directory) const
	{
		RTLIL::Design primitive_design;
		RTLIL::Module *module = primitive_design.addModule(wrapper_name);
		RTLIL::Cell *primitive = module->addCell(ID($1), cell->type);
		primitive->parameters = cell->parameters;

		for (const auto &connection : cell->connections()) {
			if (!keep_wrapper_port(connection)) {
				primitive->setPort(connection.first, connection.second);
				continue;
			}
			RTLIL::Wire *wire = module->addWire(connection.first, connection.second.size());
			if (cell_types.cell_output(cell->type, connection.first))
				wire->port_output = true;
			else
				wire->port_input = true;
			primitive->setPort(connection.first, wire);
		}
		module->fixup_ports();

		std::string path = output_directory + "/" + RTLIL::unescape_id(wrapper_name) + ".rtlil";
		std::ofstream output(path);
		if (!output.is_open())
			log_cmd_error("Cannot write coarse-unit RTLIL file '%s'.\n", path.c_str());
		RTLIL_BACKEND::dump_design(output, &primitive_design, false, false);
		log("Created coarse-unit source %s.\n", path.c_str());
	}

	void normalize_outputs(RTLIL::Design *design) const
	{
		log_header(design, "Executing SYN3D_MARK_COARSE_UNITS -normalize.\n");
		int normalized_count = 0;
		for (RTLIL::Module *module : design->selected_modules()) {
			for (RTLIL::Cell *cell : module->selected_cells()) {
				RTLIL::Module *wrapper = design->module(cell->type);
				if (wrapper == nullptr || !wrapper->get_bool_attribute(ID(syn3d_coarse_unit)))
					continue;

				std::vector<std::pair<RTLIL::IdString, RTLIL::SigSpec>> connections;
				for (const auto &connection : cell->connections()) {
					RTLIL::Wire *port = wrapper->wire(connection.first);
					if (port != nullptr && port->port_output && !connection.second.is_wire())
						connections.push_back(connection);
				}

				for (const auto &connection : connections) {
					RTLIL::Wire *wire = module->addWire(NEW_ID, connection.second.size());
					module->connect(connection.second, wire);
					cell->setPort(connection.first, wire);
					normalized_count++;
				}
			}
		}
		log("Normalized %d coarse-unit output connection%s.\n", normalized_count,
			normalized_count == 1 ? "" : "s");
	}

	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		KeepMode keep_mode = KeepMode::All;
		bool keep_mode_seen = false;
		bool normalize_mode = false;
		std::string output_directory;
		std::string manifest_path;

		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++) {
			if (args[argidx] == "-keep" && argidx + 1 < args.size()) {
				if (keep_mode_seen)
					log_cmd_error("Option -keep specified more than once.\n");
				keep_mode = parse_keep_mode(args[++argidx]);
				keep_mode_seen = true;
				continue;
			}
			if (args[argidx] == "-output-dir" && argidx + 1 < args.size()) {
				if (!output_directory.empty())
					log_cmd_error("Option -output-dir specified more than once.\n");
				output_directory = args[++argidx];
				continue;
			}
			if (args[argidx] == "-manifest" && argidx + 1 < args.size()) {
				if (!manifest_path.empty())
					log_cmd_error("Option -manifest specified more than once.\n");
				manifest_path = args[++argidx];
				continue;
			}
			if (args[argidx] == "-normalize") {
				if (normalize_mode)
					log_cmd_error("Option -normalize specified more than once.\n");
				normalize_mode = true;
				continue;
			}
			break;
		}
		extra_args(args, argidx, design);

		if (normalize_mode) {
			if (keep_mode_seen || !output_directory.empty() || !manifest_path.empty())
				log_cmd_error("Option -normalize cannot be combined with wrapping options.\n");
			normalize_outputs(design);
			return;
		}
		if (output_directory.empty())
			log_cmd_error("Option -output-dir is required.\n");
		if (manifest_path.empty())
			manifest_path = output_directory + "/coarse_units.list";

		log_header(design, "Executing SYN3D_MARK_COARSE_UNITS (keep=%s).\n", keep_mode_name(keep_mode));
		const int minimum_io_width = 5;
		CellTypes cell_types;
		cell_types.setup();

		int wrapped_instance_count = 0;
		int created_unit_count = 0;
		std::vector<RTLIL::Module *> selected_modules = design->selected_modules();
		for (RTLIL::Module *module : selected_modules) {
			std::vector<RTLIL::Cell *> cells_to_wrap;
			for (RTLIL::Cell *cell : module->selected_cells()) {
				if (!is_wrappable_for_mode(cell->type, keep_mode))
					continue;

				int total_io_width = 0;
				for (const auto &connection : cell->connections())
					total_io_width += connection.second.size();
				if (total_io_width < minimum_io_width)
					continue;
				if (cell->hasPort(ID::D) && cell->getPort(ID::D).size() <= 1)
					continue;
				cells_to_wrap.push_back(cell);
			}

			for (RTLIL::Cell *cell : cells_to_wrap) {
				RTLIL::IdString wrapper_name = make_wrapper_name(cell);
				if (design->module(wrapper_name) == nullptr) {
					create_wrapper(design, cell, wrapper_name, cell_types);
					write_primitive_rtlil(cell, wrapper_name, cell_types, output_directory);
					created_unit_count++;
				}

				dict<RTLIL::IdString, RTLIL::SigSpec> connections;
				for (const auto &connection : cell->connections())
					if (keep_wrapper_port(connection))
						connections[connection.first] = connection.second;
				cell->parameters.clear();
				cell->type = wrapper_name;
				cell->connections_ = connections;
				wrapped_instance_count++;
			}
		}

		std::vector<std::string> coarse_unit_names;
		for (RTLIL::Module *module : design->modules())
			if (module->get_bool_attribute(ID(syn3d_coarse_unit)))
				coarse_unit_names.push_back(RTLIL::unescape_id(module->name));
		std::sort(coarse_unit_names.begin(), coarse_unit_names.end());

		std::ofstream manifest(manifest_path);
		if (!manifest.is_open())
			log_cmd_error("Cannot write coarse-unit manifest '%s'.\n", manifest_path.c_str());
		std::string scratchpad_value;
		for (const std::string &name : coarse_unit_names) {
			manifest << name << "\n";
			if (!scratchpad_value.empty())
				scratchpad_value += " ";
			scratchpad_value += name;
		}
		design->scratchpad_set_string("syn3d.coarse_units", scratchpad_value);

		log("Identified %d coarse-unit instance%s across %d coarse-unit type%s.\n",
			wrapped_instance_count, wrapped_instance_count == 1 ? "" : "s",
			created_unit_count, created_unit_count == 1 ? "" : "s");
		log("Wrote %zu coarse-unit type name%s to %s.\n", coarse_unit_names.size(),
			coarse_unit_names.size() == 1 ? "" : "s", manifest_path.c_str());
	}
} Syn3dMarkCoarseUnitsPass;

PRIVATE_NAMESPACE_END
