#include "insfile.h"

void InsFile::readInsData(std::vector<InsData>& id) const
{
	if (m_filename.empty()) return;
	std::ifstream fp{ m_filename, std::ios::in | std::ios::binary };
	if (!fp.is_open()) return;
	double temp[8 * 7];
	while (!fp.eof()) {
		for (size_t i = 0; i < 7; i++) {
			fp.read(reinterpret_cast<char*>(temp + i), 8);
		}
		id.emplace_back(temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], temp[6]);
	}
	fp.close();
}
