#include <string>
#include "ros/ros.h"
#include "ecocar_encoder/Encoder.h"
#include "serial/serial.h"
#include "serial/utils/serial_listener.h"
#include "serial/utils/concurrent_queue.h"

using namespace std;
using namespace serial;
using namespace serial::utils;

ConcurrentQueue<string> buffer;
const char *format = "A:%d B:%d C:%d";

void callback(string token) {
	// add token to queue
	buffer.push( token );
}

int main( int argc, char ** argv ) {
	// Node initialization
	ros::init( argc, argv, "encoder_parser" );
	ros::NodeHandle nh;

	// create a publisher
	ros::Publisher encoder_pub = nh.advertise<ecocar_encoder::Encoder>("encoder", 1000);

	// setup serial
	Serial serial("/dev/eco_car/motor", 9600, serial::Timeout::simpleTimeout(1000) );
	if(!serial.isOpen()) {
		ROS_ERROR( "Couldn't open serial port" );
		return 0;
	}

	// Serial listener
	SerialListener listener;
	listener.startListening( serial );
	listener.setTokenizer( SerialListener::delimeter_tokenizer( "\r\n" ) );
	FilterPtr filter = listener.createFilter( SerialListener::startsWith( "A" ), callback );

	// main loop
	ros::Rate loop_rate(50);
	while( ros::ok() ) {
		// string 
		string token;
		// data
		int a, b, c;
		ecocar_encoder::Encoder msg;
		if( buffer.try_pop( token ) ) {
			sscanf( token.c_str(), format, &msg.a, &msg.b, &msg.c );
			ROS_DEBUG( "A:%d B:%d C:%d", msg.a, msg.b, msg.c );
			encoder_pub.publish( msg );
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	listener.removeFilter( filter );
	return 0;
}
