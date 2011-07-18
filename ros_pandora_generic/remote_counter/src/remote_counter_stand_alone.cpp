#include "remote_counter/remote_counter.h"

int main(int argc, char** argv) {
	std::string name = "RemoteCounter";
	name = argv[1] + name;
	ros::init(argc,argv,name);	
	RemoteCounter g(argv[1]);
	ros::spin();
}
