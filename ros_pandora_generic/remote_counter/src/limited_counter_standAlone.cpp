#include "remote_counter/limited_counter.h"

int main(int argc, char** argv) {
	std::string name = "Counter";
	name = argv[1] + name;
	ros::init(argc,argv,name);	
	if (argc >= 4) {
		LimitedCounter g(argv[1], atoi(argv[2]),atoi(argv[3]));
		ros::spin();
	} else {
		LimitedCounter g(argv[1], atoi(argv[2]),10);
		ros::spin();
	}
	
}
