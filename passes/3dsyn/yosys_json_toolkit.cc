#include "kernel/json.h"
#include "kernel/yosys.h"
#include "passes/3dsyn/yosys_json_toolkit.h"

#include <fstream>
#include <set>
#include <sstream>

YOSYS_NAMESPACE_BEGIN

namespace {

using json11::Json;

static std::string describe_context(const std::string &context, const std::string &field)
{
	if (context.empty())
		return field;
	if (field.empty())
		return context;
	return context + "." + field;
}

static Json require_object_field(const Json &obj, const std::string &field, const std::string &context)
{
	if (!obj.is_object())
		log_error("yosys_json_toolkit: expected JSON object at %s\n", context.c_str());

	Json value = obj[field];
	if (value.is_null())
		log_error("yosys_json_toolkit: missing field %s\n", describe_context(context, field).c_str());
	return value;
}

static std::string require_string(const Json &value, const std::string &context)
{
	if (!value.is_string())
		log_error("yosys_json_toolkit: expected string at %s\n", context.c_str());
	return value.string_value();
}

static int require_int(const Json &value, const std::string &context)
{
	if (!value.is_number())
		log_error("yosys_json_toolkit: expected integer at %s\n", context.c_str());

	int int_value = value.int_value();
	if (value.number_value() != int_value)
		log_error("yosys_json_toolkit: expected integer-valued number at %s\n", context.c_str());
	return int_value;
}

static YosysJsonToolkit::OptionalInt parse_optional_int_field(const Json &obj, const std::string &field, const std::string &context)
{
	YosysJsonToolkit::OptionalInt result;
	if (!obj.is_object())
		log_error("yosys_json_toolkit: expected JSON object at %s\n", context.c_str());

	Json value = obj[field];
	if (value.is_null())
		return result;

	result.present = true;
	result.value = require_int(value, describe_context(context, field));
	return result;
}

static YosysJsonToolkit::Bit parse_bit(const Json &value, const std::string &context)
{
	if (value.is_number())
		return YosysJsonToolkit::Bit::from_signal(require_int(value, context));

	if (value.is_string()) {
		std::string s = value.string_value();
		if (GetSize(s) != 1 || std::string("01xz").find(s[0]) == std::string::npos)
			log_error("yosys_json_toolkit: invalid constant bit '%s' at %s\n", s.c_str(), context.c_str());
		return YosysJsonToolkit::Bit::from_const(s[0]);
	}

	log_error("yosys_json_toolkit: expected bit token at %s\n", context.c_str());
	return YosysJsonToolkit::Bit();
}

static std::vector<YosysJsonToolkit::Bit> parse_bits(const Json &value, const std::string &context)
{
	if (!value.is_array())
		log_error("yosys_json_toolkit: expected bit array at %s\n", context.c_str());

	std::vector<YosysJsonToolkit::Bit> bits;
	const Json::array &items = value.array_items();
	bits.reserve(items.size());
	for (size_t i = 0; i < items.size(); i++)
		bits.push_back(parse_bit(items[i], stringf("%s[%zu]", context.c_str(), i)));
	return bits;
}

static std::map<std::string, std::string> parse_string_map(const Json &value, const std::string &context)
{
	if (value.is_null())
		return std::map<std::string, std::string>();
	if (!value.is_object())
		log_error("yosys_json_toolkit: expected object at %s\n", context.c_str());

	std::map<std::string, std::string> result;
	const Json::object &items = value.object_items();
	for (Json::object::const_iterator it = items.begin(); it != items.end(); ++it)
		result[it->first] = require_string(it->second, describe_context(context, it->first));
	return result;
}

static YosysJsonToolkit::Port parse_port(const std::string &name, const Json &value, const std::string &context)
{
	if (!value.is_object())
		log_error("yosys_json_toolkit: expected object at %s\n", context.c_str());

	YosysJsonToolkit::Port port;
	port.name = name;
	port.direction = require_string(require_object_field(value, "direction", context),
	                                describe_context(context, "direction"));
	port.bits = parse_bits(require_object_field(value, "bits", context),
	                       describe_context(context, "bits"));
	port.offset = parse_optional_int_field(value, "offset", context);
	port.upto = parse_optional_int_field(value, "upto", context);
	port.is_signed = parse_optional_int_field(value, "signed", context);
	return port;
}

static YosysJsonToolkit::Cell parse_cell(const std::string &name, const Json &value, const std::string &context)
{
	if (!value.is_object())
		log_error("yosys_json_toolkit: expected object at %s\n", context.c_str());

	YosysJsonToolkit::Cell cell;
	cell.name = name;

	Json hide_name = value["hide_name"];
	if (!hide_name.is_null())
		cell.hide_name = require_int(hide_name, describe_context(context, "hide_name"));

	cell.type = require_string(require_object_field(value, "type", context),
	                           describe_context(context, "type"));
	cell.port_directions = parse_string_map(value["port_directions"],
	                                        describe_context(context, "port_directions"));

	Json connections = require_object_field(value, "connections", context);
	if (!connections.is_object())
		log_error("yosys_json_toolkit: expected object at %s\n",
		          describe_context(context, "connections").c_str());
	const Json::object &conn_items = connections.object_items();
	for (Json::object::const_iterator it = conn_items.begin(); it != conn_items.end(); ++it) {
		cell.connections[it->first] = parse_bits(it->second,
		                                         describe_context(describe_context(context, "connections"),
		                                                          it->first));
	}

	return cell;
}

static YosysJsonToolkit::Netname parse_netname(const std::string &name, const Json &value, const std::string &context)
{
	if (!value.is_object())
		log_error("yosys_json_toolkit: expected object at %s\n", context.c_str());

	YosysJsonToolkit::Netname netname;
	netname.name = name;

	Json hide_name = value["hide_name"];
	if (!hide_name.is_null())
		netname.hide_name = require_int(hide_name, describe_context(context, "hide_name"));

	netname.bits = parse_bits(require_object_field(value, "bits", context),
	                          describe_context(context, "bits"));
	netname.offset = parse_optional_int_field(value, "offset", context);
	netname.upto = parse_optional_int_field(value, "upto", context);
	netname.is_signed = parse_optional_int_field(value, "signed", context);
	return netname;
}

static YosysJsonToolkit::Module parse_module(const std::string &name, const Json &value, const std::string &context)
{
	if (!value.is_object())
		log_error("yosys_json_toolkit: expected object at %s\n", context.c_str());

	YosysJsonToolkit::Module module;
	module.name = name;

	Json ports = require_object_field(value, "ports", context);
	if (!ports.is_object())
		log_error("yosys_json_toolkit: expected object at %s\n",
		          describe_context(context, "ports").c_str());
	const Json::object &port_items = ports.object_items();
	for (Json::object::const_iterator it = port_items.begin(); it != port_items.end(); ++it)
		module.ports[it->first] = parse_port(it->first, it->second,
		                                     describe_context(describe_context(context, "ports"), it->first));

	Json cells = require_object_field(value, "cells", context);
	if (!cells.is_object())
		log_error("yosys_json_toolkit: expected object at %s\n",
		          describe_context(context, "cells").c_str());
	const Json::object &cell_items = cells.object_items();
	for (Json::object::const_iterator it = cell_items.begin(); it != cell_items.end(); ++it)
		module.cells[it->first] = parse_cell(it->first, it->second,
		                                     describe_context(describe_context(context, "cells"), it->first));

	Json netnames = require_object_field(value, "netnames", context);
	if (!netnames.is_object())
		log_error("yosys_json_toolkit: expected object at %s\n",
		          describe_context(context, "netnames").c_str());
	const Json::object &netname_items = netnames.object_items();
	for (Json::object::const_iterator it = netname_items.begin(); it != netname_items.end(); ++it)
		module.netnames[it->first] = parse_netname(it->first, it->second,
		                                           describe_context(describe_context(context, "netnames"), it->first));

	module.bit_references = collect_bit_references(module);
	return module;
}

} // namespace

namespace YosysJsonToolkit {

Bit Bit::from_signal(int signal_id)
{
	Bit bit;
	bit.is_const = false;
	bit.signal_id = signal_id;
	bit.const_value = 'x';
	return bit;
}

Bit Bit::from_const(char const_value)
{
	Bit bit;
	bit.is_const = true;
	bit.signal_id = 0;
	bit.const_value = const_value;
	return bit;
}

bool Bit::operator<(const Bit &other) const
{
	if (is_const != other.is_const)
		return is_const < other.is_const;
	if (is_const)
		return const_value < other.const_value;
	return signal_id < other.signal_id;
}

bool Bit::operator==(const Bit &other) const
{
	if (is_const != other.is_const)
		return false;
	if (is_const)
		return const_value == other.const_value;
	return signal_id == other.signal_id;
}

Design load_design(const std::string &path)
{
	std::ifstream f(path.c_str());
	if (!f.is_open())
		log_error("yosys_json_toolkit: cannot open JSON file %s\n", path.c_str());

	std::ostringstream buffer;
	buffer << f.rdbuf();

	std::string parse_error;
	Json root = Json::parse(buffer.str(), parse_error);
	if (!parse_error.empty())
		log_error("yosys_json_toolkit: failed to parse %s: %s\n", path.c_str(), parse_error.c_str());
	if (!root.is_object())
		log_error("yosys_json_toolkit: root of %s must be an object\n", path.c_str());

	Design design;
	Json creator = root["creator"];
	if (!creator.is_null())
		design.creator = require_string(creator, "creator");

	Json modules = require_object_field(root, "modules", path);
	if (!modules.is_object())
		log_error("yosys_json_toolkit: expected object at %s.modules\n", path.c_str());

	const Json::object &module_items = modules.object_items();
	for (Json::object::const_iterator it = module_items.begin(); it != module_items.end(); ++it)
		design.modules[it->first] = parse_module(it->first, it->second,
		                                         describe_context(describe_context(path, "modules"), it->first));

	return design;
}

const Module *find_module(const Design &design, const std::string &module_name)
{
	std::map<std::string, Module>::const_iterator it = design.modules.find(module_name);
	if (it == design.modules.end())
		return NULL;
	return &it->second;
}

const Port *find_port(const Module &module, const std::string &port_name)
{
	std::map<std::string, Port>::const_iterator it = module.ports.find(port_name);
	if (it == module.ports.end())
		return NULL;
	return &it->second;
}

const Cell *find_cell(const Module &module, const std::string &cell_name)
{
	std::map<std::string, Cell>::const_iterator it = module.cells.find(cell_name);
	if (it == module.cells.end())
		return NULL;
	return &it->second;
}

const Netname *find_netname(const Module &module, const std::string &netname_name)
{
	std::map<std::string, Netname>::const_iterator it = module.netnames.find(netname_name);
	if (it == module.netnames.end())
		return NULL;
	return &it->second;
}

std::map<Bit, std::vector<BitReference> > collect_bit_references(const Module &module)
{
	std::map<Bit, std::vector<BitReference> > refs;

	for (std::map<std::string, Port>::const_iterator it = module.ports.begin(); it != module.ports.end(); ++it) {
		const Port &port = it->second;
		for (int i = 0; i < GetSize(port.bits); i++)
			refs[port.bits[i]].push_back(BitReference{ReferenceKind::Port, port.name, "", i, port.direction});
	}

	for (std::map<std::string, Netname>::const_iterator it = module.netnames.begin(); it != module.netnames.end(); ++it) {
		const Netname &netname = it->second;
		for (int i = 0; i < GetSize(netname.bits); i++)
			refs[netname.bits[i]].push_back(BitReference{ReferenceKind::Netname, netname.name, "", i, ""});
	}

	for (std::map<std::string, Cell>::const_iterator it = module.cells.begin(); it != module.cells.end(); ++it) {
		const Cell &cell = it->second;
		for (std::map<std::string, std::vector<Bit> >::const_iterator conn_it = cell.connections.begin();
		     conn_it != cell.connections.end(); ++conn_it) {
			std::string direction;
			std::map<std::string, std::string>::const_iterator dir_it = cell.port_directions.find(conn_it->first);
			if (dir_it != cell.port_directions.end())
				direction = dir_it->second;

			for (int i = 0; i < GetSize(conn_it->second); i++) {
				refs[conn_it->second[i]].push_back(BitReference{
					ReferenceKind::Cell, cell.name, conn_it->first, i, direction
				});
			}
		}
	}

	return refs;
}

std::vector<Bit> collect_all_bits(const Module &module, bool include_constants)
{
	std::vector<Bit> bits;
	std::set<Bit> seen;
	for (std::map<Bit, std::vector<BitReference> >::const_iterator it = module.bit_references.begin();
	     it != module.bit_references.end(); ++it) {
		if (!include_constants && it->first.is_const)
			continue;
		if (seen.insert(it->first).second)
			bits.push_back(it->first);
	}
	return bits;
}

std::string bit_to_string(const Bit &bit)
{
	if (bit.is_const)
		return std::string(1, bit.const_value);
	return stringf("%d", bit.signal_id);
}

std::string bits_to_string(const std::vector<Bit> &bits)
{
	std::string text = "[";
	for (int i = 0; i < GetSize(bits); i++) {
		if (i != 0)
			text += ", ";
		text += bit_to_string(bits[i]);
	}
	text += "]";
	return text;
}

std::string reference_kind_to_string(ReferenceKind kind)
{
	switch (kind) {
	case ReferenceKind::Port:
		return "port";
	case ReferenceKind::Netname:
		return "netname";
	case ReferenceKind::Cell:
		return "cell";
	}
	return "unknown";
}

} // namespace YosysJsonToolkit

YOSYS_NAMESPACE_END
