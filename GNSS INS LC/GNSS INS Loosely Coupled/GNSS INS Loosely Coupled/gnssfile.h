#pragma once
#include <string>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include "gnssdata.h"
class GnssFile
{
public:
	GnssFile()
		: m_filename()
	{}
	GnssFile(const std::string& filename)
		: m_filename(filename)
	{}
	static void parseDataString(const std::string& str, GnssData& gd);
	void readGnssData(std::vector<GnssData>& gd) const;
private:
	std::string m_filename;
};

