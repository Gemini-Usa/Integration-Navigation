#pragma once
#include <string>
#include <vector>
#include <fstream>
#include "insdata.h"
class InsFile
{
public:
	InsFile()
		: m_filename()
	{}
	InsFile(const std::string& filename)
		: m_filename(filename)
	{}
	void readInsData(std::vector<InsData>& id) const;
private:
	std::string m_filename;
};


