#ifndef YOSYS_JSON_TOOLKIT_H
#define YOSYS_JSON_TOOLKIT_H

#include "kernel/yosys.h"
#include <map>
#include <string>
#include <vector>

YOSYS_NAMESPACE_BEGIN

namespace YosysJsonToolkit {

struct OptionalInt
{
	bool present = false;
	int value = 0;
};

struct Bit
{
	bool is_const = false;
	int signal_id = 0;
	char const_value = 'x';

	static Bit from_signal(int signal_id);
	static Bit from_const(char const_value);

	bool operator<(const Bit &other) const;
	bool operator==(const Bit &other) const;
};

enum class ReferenceKind
{
	Port,
	Netname,
	Cell,
};

struct BitReference
{
	ReferenceKind kind;
	std::string owner_name;
	std::string field_name;
	int bit_index = 0;
	std::string direction;
};

struct Port
{
	std::string name;
	std::string direction;
	std::vector<Bit> bits;
	OptionalInt offset;
	OptionalInt upto;
	OptionalInt is_signed;
};

struct Cell
{
	std::string name;
	int hide_name = 0;
	std::string type;
	std::map<std::string, std::string> port_directions;
	std::map<std::string, std::vector<Bit> > connections;
};

struct Netname
{
	std::string name;
	int hide_name = 0;
	std::vector<Bit> bits;
	OptionalInt offset;
	OptionalInt upto;
	OptionalInt is_signed;
};

struct Module
{
	std::string name;
	std::map<std::string, Port> ports;
	std::map<std::string, Cell> cells;
	std::map<std::string, Netname> netnames;
	std::map<Bit, std::vector<BitReference> > bit_references;
};

struct Design
{
	std::string creator;
	std::map<std::string, Module> modules;
};

Design load_design(const std::string &path);

const Module *find_module(const Design &design, const std::string &module_name);
const Port *find_port(const Module &module, const std::string &port_name);
const Cell *find_cell(const Module &module, const std::string &cell_name);
const Netname *find_netname(const Module &module, const std::string &netname_name);

std::map<Bit, std::vector<BitReference> > collect_bit_references(const Module &module);
std::vector<Bit> collect_all_bits(const Module &module, bool include_constants = true);

std::string bit_to_string(const Bit &bit);
std::string bits_to_string(const std::vector<Bit> &bits);
std::string reference_kind_to_string(ReferenceKind kind);

} // namespace YosysJsonToolkit

YOSYS_NAMESPACE_END

#endif
