#pragma once
#include <string>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include "gnssdata.h"
class gnssfile
{
public:
	gnssfile()
		: m_filename()
	{}
	gnssfile(const std::string& filename)
		: m_filename(filename)
	{}
	static void parseDataString(const std::string& str, gnssdata& gd);
	void readGnssData(std::vector<gnssdata>& gd) const;
private:
	std::string m_filename;
};

