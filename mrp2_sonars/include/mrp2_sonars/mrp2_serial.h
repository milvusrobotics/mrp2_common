#ifndef MRP2_SERIAL_H
#define MRP2_SERIAL_H

#include "rs232.h"
#include "time.h"
#include <sys/time.h>
#include <string>
#include <stdint.h>
#include <unistd.h>
#include <vector>
#include "mrp2_sonars/serial_comm.h"

using std::string;

class MRP2_Serial
{
	public:
		MRP2_Serial(int port_nr=0, uint32_t baudrate = 9600);
		virtual ~MRP2_Serial ();
		std::vector<int> get_sonars(void);
				
		void update();	

		typedef enum {
		ACK = 52
	}serial_t;
	private:
		void array_chopper(uint8_t *buf, int start, int end);
		unsigned char checksum(int size);
		bool checksum_match(uint8_t *buf, int size);
		unsigned char checksum_check_array(uint8_t *arr, int size);
		int first_validator(uint8_t *buf);
		int second_validator(uint8_t *buf, int data_len);
		int find_message_start(uint8_t *buf,  int lastIndex);
		int execute_command(uint8_t *buf);
		void print_array(uint8_t *buf, int length);
		int send_and_get_reply(uint8_t _command, uint8_t *send_array, int send_size, bool is_ack);
		int read_serial(uint8_t _command_to_read);
		int process(uint8_t *inData, int recievedData, uint8_t _command_to_read);
		bool _get_ack(uint8_t command);

		std::vector<int> _sonars;

		int _port_nr;
		int _baudrate;
		char _mode[3];
		char sendArray[20];

		uint8_t tempData[1000];
		uint8_t tempDataIndex;

		bool seekForChar;
		char startChar;
		uint8_t _ack_data;
};

#endif
