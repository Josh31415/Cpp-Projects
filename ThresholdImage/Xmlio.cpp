#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <string>
#include "Xmlio.h"

using namespace std;
using boost::property_tree::ptree;
using boost::property_tree::write_xml;
using boost::property_tree::xml_writer_settings;


std::vector<int> Xmlio::readXml(){
	ptree pt;
	read_xml("Thresholds.xml", pt);
	string xthresh[6];
	std::vector<int> ans;
	//std::vector<int> ans;

	xthresh[0] = pt.get<std::string>("Threshold.RGB.Red");
	xthresh[1] = pt.get<std::string>("Threshold.RGB.Green");
	xthresh[2] = pt.get<std::string>("Threshold.RGB.Blue");
	xthresh[3] = pt.get<std::string>("Threshold.HSV.Hue");
	xthresh[4] = pt.get<std::string>("Threshold.HSV.Sateration");
	xthresh[5] = pt.get<std::string>("Threshold.HSV.Value");

	for(int i; i < 6; i++){
		istringstream buffer(xthresh[i]);
		buffer >> ans[i];
	}

	return ans;
}

void Xmlio::writeXml(int x[]){
	ptree tree;
	tree.add("Threshold.<xmlattr>.version", "1.0");
	ptree& rgbml = tree.add("Threshold.RGB", "");
	ptree& hsvml = tree.add("Threshold.HSV", "");
	rgbml.add("Red", x[0]);
	rgbml.add("Green", x[1]);
	rgbml.add("Blue", x[2]);
	hsvml.add("Hue", x[3]);
	hsvml.add("Sateration", x[4]);
	hsvml.add("Value", x[5]);

	write_xml("Thresholds.xml", tree);
}

void Xmlio::writeText(int x[]){
	std::ofstream outfile ("SavedThresholds.txt");

	outfile << "RGB " << endl << x[0] << " ";
	outfile << x[1] << " ";
	outfile << x[2] << std::endl;
	outfile << "HSV " << endl << x[3] << " ";
	outfile << x[4] << " ";
	outfile << x[5] << std::endl;
}
