 #include <ros/ros.h>
 #include <std_msgs/Int32.h>
 #include <std_msgs/MultiArrayLayout.h>
 #include <std_msgs/MultiArrayDimension.h>
 #include <std_msgs/Int32MultiArray.h> 

 #include <mrp2_sonars/mrp2_serial.h>

MRP2_Serial *sonar_serial;

ros::Publisher sonars_pub;

std::vector<int> recieved_array;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mrp2_sonars_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);


	sonar_serial = new MRP2_Serial(0, 9600);

	sonars_pub = n.advertise<std_msgs::Int32MultiArray>("sonars", 100);

	recieved_array.reserve(20);
	recieved_array.clear();

	for (int i = 0; i <= 20; i++) // Initial values for sonars.
	{
		recieved_array.push_back(-1);
	}

	while (ros::ok())
    { 
    	std_msgs::Int32MultiArray bumpers_array;

    	recieved_array.reserve(20);
		recieved_array.clear();
		recieved_array = sonar_serial->get_sonars();


		bumpers_array.data.clear();
		bumpers_array.data.push_back(recieved_array[0]);
		bumpers_array.data.push_back(recieved_array[1]);
		bumpers_array.data.push_back(recieved_array[2]);
		bumpers_array.data.push_back(recieved_array[3]);
		bumpers_array.data.push_back(recieved_array[4]);
		bumpers_array.data.push_back(recieved_array[5]);
		bumpers_array.data.push_back(recieved_array[6]);

		sonars_pub.publish(bumpers_array);

    	ros::spinOnce();
    	loop_rate.sleep();
    }
}

