/*
 * Xmlio.h
 *
 *  Created on: Oct 27, 2016
 *      Author: josh
 */

#ifndef SRC_XMLIO_H_
#define SRC_XMLIO_H_

namespace std {

class Xmlio {
public:
	static std::vector<int> readXml();
	static void writeXml(int x[]);
	void writeText(int x[]);
};

} /* namespace std */

#endif /* SRC_XMLIO_H_ */
