#pragma once
#include <string>
#include <vector>
#include <fstream>
#include "ododata.h"
class OdoFile
{
public:
	OdoFile()
		: m_filename()
	{}
	OdoFile(const std::string& filename)
		: m_filename(filename)
	{}
	void readOdometerData(std::vector<OdoData>& od) const;
private:
	std::string m_filename;
};

