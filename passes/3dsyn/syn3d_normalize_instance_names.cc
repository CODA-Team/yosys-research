#include "kernel/yosys.h"

USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

static RTLIL::IdString normalized_instance_name(RTLIL::Module *module, RTLIL::Cell *cell, bool coarse_units_only)
{
	std::string original = RTLIL::unescape_id(cell->name);
	std::string normalized;
	normalized.reserve(original.size() + 8);
	for (char character : original) {
		if ((character >= 'a' && character <= 'z') ||
			(character >= 'A' && character <= 'Z') ||
			(character >= '0' && character <= '9') || character == '_')
			normalized += character;
		else
			normalized += '_';
	}

	if (normalized.empty())
		normalized = coarse_units_only ? "coarse_unit_inst" : "inst";
	if (normalized.front() >= '0' && normalized.front() <= '9')
		normalized = (coarse_units_only ? "coarse_unit_" : "inst_") + normalized;

	std::string compact;
	bool previous_was_underscore = false;
	for (char character : normalized) {
		if (character == '_') {
			if (!previous_was_underscore)
				compact += character;
			previous_was_underscore = true;
		} else {
			compact += character;
			previous_was_underscore = false;
		}
	}
	if (compact.empty())
		compact = coarse_units_only ? "coarse_unit_inst" : "inst";
	return module->uniquify(RTLIL::escape_id(compact));
}

static bool should_normalize(RTLIL::Design *design, RTLIL::Cell *cell, bool coarse_units_only)
{
	if (!coarse_units_only)
		return true;
	RTLIL::Module *module = design->module(cell->type);
	return module != nullptr && module->get_bool_attribute(ID(syn3d_coarse_unit));
}

struct Syn3dNormalizeInstanceNamesPass : Pass
{
	Syn3dNormalizeInstanceNamesPass() :
		Pass("syn3d_normalize_instance_names", "normalize instance names for 3D-Syn handoffs")
	{
	}

	void help() override
	{
		log("\n");
		log("    syn3d_normalize_instance_names [-coarse-units-only] [selection]\n");
		log("\n");
		log("Rename selected instances to unique Verilog- and DEF-friendly names.\n");
		log("\n");
		log("    -coarse-units-only\n");
		log("        Only rename instances of modules marked with syn3d_coarse_unit.\n");
		log("\n");
	}

	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		bool coarse_units_only = false;
		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++) {
			if (args[argidx] == "-coarse-units-only") {
				coarse_units_only = true;
				continue;
			}
			break;
		}
		extra_args(args, argidx, design);

		log_header(design, "Executing SYN3D_NORMALIZE_INSTANCE_NAMES.\n");
		int normalized_count = 0;
		for (RTLIL::Module *module : design->selected_modules()) {
			std::vector<std::pair<RTLIL::Cell *, RTLIL::IdString>> renames;
			for (RTLIL::Cell *cell : module->selected_cells()) {
				if (!should_normalize(design, cell, coarse_units_only))
					continue;
				RTLIL::IdString new_name = normalized_instance_name(module, cell, coarse_units_only);
				if (new_name != cell->name)
					renames.push_back({cell, new_name});
			}
			for (const auto &rename : renames) {
				module->rename(rename.first, rename.second);
				normalized_count++;
			}
		}
		log("Normalized %d instance name%s.\n", normalized_count, normalized_count == 1 ? "" : "s");
	}
} Syn3dNormalizeInstanceNamesPass;

PRIVATE_NAMESPACE_END
