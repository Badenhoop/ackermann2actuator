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
		std::vector<std::string> firstRow;
		readRow(firstRow);
		if (firstRow.empty())
			throw std::runtime_error{"Error: Empty .csv file."};

		// find order mapping from read column names to given column names
		for (std::size_t i = 0; i < columnNames.size(); ++i)
		{
			const auto & columnName = columnNames[i];
			auto it = std::find(firstRow.begin(), firstRow.end(), columnName);
			if (it == firstRow.end())
				throw std::runtime_error{"Error: Column not found: '" + columnName + "'."};
			auto columnIndex = std::distance(firstRow.begin(), it);
			order[columnIndex] = i;
		}
	}

	friend bool operator>>(IStream & stream, std::vector<Item> & items)
	{
		std::vector<std::string> values;
		stream.readRow(values);
		if (values.empty())
			return false;
		items.resize(stream.order.size());
		for (std::size_t i = 0; i < items.size(); ++i)
		{
			if (stream.order.find(i) != stream.order.end())
				items[stream.order[i]] = values[i];
		}
		return true;
	}

private:
	std::ifstream file;
	std::unordered_map<std::size_t, std::size_t> order;

	void readRow(std::vector<std::string> & values)
	{
		// TODO: implement quotation marks as escape character for value delimiter
		std::string line;

		// ignore empty lines
		while (std::getline(file, line))
		{
			boost::trim(line);
			if (!line.empty())
				break;
		}

		std::string value;
		std::istringstream iss{line};
		std::size_t i = 0;
		while (std::getline(iss, value, ','))
		{
			boost::trim(value);

			// remove quotation marks
			if (*(value.begin()) == '"')
				if (*(value.rbegin()) == '"')
					value = value.substr(1, value.length() - 2);
				else
					value = value.substr(1, value.length() - 1);
			else if (*(value.rbegin()) == '"')
				value = value.substr(0, value.length() - 1);

			values.push_back(value);
		}
	}
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
