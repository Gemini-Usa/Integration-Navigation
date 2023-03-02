#pragma once
#include <string>
#include <vector>
#include <fstream>
#include "ododata.h"
class odofile
{
public:
	odofile()
		: m_filename()
	{}
	odofile(const std::string& filename)
		: m_filename(filename)
	{}
	void readOdometerData(std::vector<ododata>& od) const;
private:
	std::string m_filename;
};

