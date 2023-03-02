#include "gnssfile.h"
using v3 = Eigen::Vector3d;
void gnssfile::parseDataString(const std::string& str, gnssdata& gd)
{
	gd.setTime(std::stod(str.substr(0, 10)));
	gd.setBLH(blh(angle(std::stod(str.substr(10, 17)) * angle::D2R),
		angle(std::stod(str.substr(27, 17)) * angle::D2R),
		std::stod(str.substr(44, 11))));
	gd.setBLHStd(v3(std::stod(str.substr(55, 9)), 
		std::stod(str.substr(64, 9)), 
		std::stod(str.substr(73, 9))));
	gd.setVel(v3(std::stod(str.substr(82, 11)),
		std::stod(str.substr(93, 11)), 
		std::stod(str.substr(104, 11))));
	gd.setVelStd(v3(std::stod(str.substr(115, 9)),
		std::stod(str.substr(124, 9)),
		std::stod(str.substr(133, 9))));
}

void gnssfile::readGnssData(std::vector<gnssdata>& gd) const
{
	if (m_filename.empty()) return;
	std::ifstream fp{ m_filename, std::ios::in };
	if (!fp.is_open()) return;
	std::string line;
	blh pos;
	gnssdata dataline;
	//skip first two rows
	int i = 0;
	while (std::getline(fp, line)) {
		i++;
		if (i <= 2) continue;
		gnssfile::parseDataString(line, dataline);
		gd.push_back(dataline);
		line.clear();
	}
	fp.close();
}
