#pragma once
#include <string>
#include <vector>
#include <fstream>
#include "insdata.h"
class insfile
{
public:
	insfile()
		: m_filename()
	{}
	insfile(const std::string& filename)
		: m_filename(filename)
	{}
	void readInsData(std::vector<insdata>& id) const;
private:
	std::string m_filename;
};


