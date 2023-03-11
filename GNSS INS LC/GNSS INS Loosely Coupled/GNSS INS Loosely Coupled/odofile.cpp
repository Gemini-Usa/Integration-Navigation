#include "odofile.h"
void OdoFile::readOdometerData(std::vector<OdoData>& od) const
{
	if (m_filename.empty()) return;
	std::ifstream fp{ m_filename, std::ios::in | std::ios::binary };
	if (!fp.is_open()) return;
	double temp[8 * 2];
	while (!fp.eof()) {
		fp.read(reinterpret_cast<char*>(temp), 8);
		fp.read(reinterpret_cast<char*>(temp + 1), 8);
		od.emplace_back(temp[0], temp[1]);
	}
	fp.close();
}
