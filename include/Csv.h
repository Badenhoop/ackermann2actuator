//
// Created by philipp on 10.04.18.
//

#ifndef NS3MNS_CSV_H
#define NS3MNS_CSV_H

#include <fstream>
#include <unordered_map>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <sstream>

namespace csv
{

enum class Type
{
	STRING, NUMBER, BOOL, UNSPECIFIED
};

class Item
{
public:
	Item() : data(""), type(Type::UNSPECIFIED)
	{}

	Item(std::string data, Type type)
		: data(std::move(data)), type(type)
	{}

	Item(std::string value)
		: data(std::move(value)), type(Type::STRING)
	{}

	Item(double value)
		: data(std::to_string(value)), type(Type::NUMBER)
	{}

	Item(bool value)
		: data(std::to_string(value)), type(Type::BOOL)
	{}

	std::string getString() const
	{ return data; }

	double getDouble() const
	{
		try
		{
			return std::stod(data);
		}
		catch (const std::invalid_argument & e)
		{
			throw std::runtime_error{"Error: Data cannot be converted to double"};
		}
	}

	bool getBool() const
	{
		if (data == "true" || data == "1")
			return true;
		else if (data == "false" || data == "0")
			return false;
		throw std::runtime_error{"Error: Data cannot be converted to bool."};
	}

	Type getType() const
	{ return type; }

private:
	std::string data;
	Type type;
};

class IStream
{
public:
	explicit IStream(const std::string & filename, const std::vector<std::string> & columnNames)
	{
		file.open(filename);

		// read column names in first row
		std::vector<Item> firstRow;
		if (!(*this >> firstRow))
			throw std::runtime_error{"Error: Empty .csv file."};

		// find order mapping from read column names to given column names
		for (std::size_t i = 0; i < columnNames.size(); ++i)
		{
			const auto & columnName = columnNames[i];
			auto it = std::find_if(firstRow.begin(), firstRow.end(),
			                       [&](const auto & item) { return item.getString() == columnName; });
			if (it == firstRow.end())
				throw std::runtime_error{"Error: Column not found: '" + columnName + "'."};
			auto index = std::distance(firstRow.begin(), it);
			order[index] = i;
		}
	}

	friend bool operator>>(IStream & stream, std::vector<Item> & items)
	{
		// TODO: implement quotation marks as escape character for item delimiter

		items.resize(stream.order.size());
		std::string line;

		// ignore empty lines
		while (std::getline(stream.file, line))
		{
			boost::trim(line);
			if (!line.empty())
				break;
		}

		std::string item;
		std::istringstream iss{line};
		std::size_t i = 0;
		while (std::getline(iss, item, ','))
		{
			boost::trim(item);

			// remove quotation marks
			if (*(item.begin()) == '"')
				if (*(item.rbegin()) == '"')
					item = item.substr(1, item.length() - 2);
				else
					item = item.substr(1, item.length() - 1);
			else if (*(item.rbegin()) == '"')
				item = item.substr(0, item.length() - 1);

			// place the item at the correct position
			if (stream.order.find(i) != stream.order.end())
				items[stream.order[i]] = Item{item, Type::UNSPECIFIED};

			++i;
		}
		return i != 0;
	}

private:
	std::ifstream file;
	std::unordered_map<std::size_t, std::size_t> order;
};

class OStream
{
public:
	explicit OStream(const std::string & filename, const std::vector<std::string> & columnNames)
	{
		file.open(filename);

		if (!columnNames.empty())
			file << columnNames[0];

		for (std::size_t i = 1; i < columnNames.size(); ++i)
			file << "," << columnNames[i];

		file << "\n";
	}

	friend void operator<<(OStream & stream, const std::vector<Item> & items)
	{
		if (!items.empty())
			stream.file << items[0].getString();

		for (std::size_t i = 1; i < items.size(); ++i)
		{
			stream.file << ",";
			const auto & item = items[i];
			if (item.getType() == Type::STRING)
				stream.file << "\"" << item.getString() << "\"";
			else
				stream.file << item.getString();
		}

		stream.file << "\n";
	}

private:
	std::ofstream file;
};

}

#endif //NS3MNS_CSV_H
