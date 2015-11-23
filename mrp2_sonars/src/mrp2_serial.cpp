#include "mrp2_sonars/mrp2_serial.h"

MRP2_Serial::MRP2_Serial (int port_nr, uint32_t baudrate)
 : _baudrate(baudrate), _port_nr(port_nr)
{
  
  tempDataIndex = 0;
  seekForChar = true;
  startChar = '$';
  _mode[0] = '8'; _mode[1] = 'N'; _mode[2] = '1';
  
  if(RS232_OpenComport(_port_nr,_baudrate,_mode))
  {
    printf("Can't open comport.\n");
  }
  /*else
    printf("Port %d opened with baud %d.\n", _port_nr, _baudrate);*/
  _sonars.reserve(20);
  _sonars[0] = -1;
  _sonars[1] = -1;
  _sonars[2] = -1;
  _sonars[3] = -1;
  _sonars[4] = -1;
  _sonars[5] = -1;
  _sonars[6] = -1;

}

MRP2_Serial::~MRP2_Serial ()
{
  
}

void
MRP2_Serial::update ()
{
    //read_serial();  
    //printf("Finished\n");
}

std::vector<int> 
MRP2_Serial::get_sonars(void)
{
  uint8_t send_array[2];
  //send_array[0] = '$';
  //send_array[1] = 'S';
  int ret = send_and_get_reply('S', send_array, 2, false);

  return _sonars;
}

int
MRP2_Serial::send_and_get_reply(uint8_t _command, uint8_t *send_array, int send_size, bool is_ack)
{
  struct timeval  tv1, tv2;

  time_t time_1 = time(0);
  time_t time_2 = time(0);
  double _time_diff = 0;
  int _ret_val = 0;
  double _time_out = 0.1;

  gettimeofday(&tv1, NULL);
  gettimeofday(&tv2, NULL);

  send_array[0] = 'S';

  //printf("\nSent and get reply called at %f...\n", (double)(tv1.tv_sec + (double)(tv1.tv_usec/1000000.0)) );
  int ret =RS232_SendBuf(_port_nr,send_array,1);
  
  if (is_ack)
  {
    _ack_data = 0;
    while (_ack_data != _command && _time_diff < _time_out)
    {
      _ret_val = read_serial(ACK);
      gettimeofday(&tv2, NULL);
      _time_diff = (double) (tv2.tv_usec - tv1.tv_usec) / 1000000 + (double) (tv2.tv_sec - tv1.tv_sec);
    }
    //printf("ACK is %d for command %d\n", _ack_data, _command);
  }
  else
  {
    while (_ret_val == 0 && _time_diff < _time_out)
    {
        _ret_val = read_serial(_command);
        gettimeofday(&tv2, NULL);
        _time_diff = (double) (tv2.tv_usec - tv1.tv_usec) / 1000000 + (double) (tv2.tv_sec - tv1.tv_sec);
    }
    //printf("Reply is %d for command %d\n", _ret_val, _command);
  }

  if (_time_diff >= _time_out)
    return -1;
  
  //gettimeofday(&tv2, NULL);
  //printf("\nSent and get reply returned at %f...\n", (double)(tv2.tv_sec + (double)(tv2.tv_usec/1000000.0)) );
  return _ret_val;
}

int
MRP2_Serial::read_serial (uint8_t _command_to_read)
{
  uint8_t inData[50] = {0};
  int recievedData = 0;
  //sleep(1);
  
  

  //for (int i = 0; i < 10; ++i)
  //{
    
    recievedData = RS232_PollComport(_port_nr, inData, 50);
    usleep(1000);
    //printf("\nCalling process For Command %d\n", _command_to_read);
    return process(inData, recievedData, _command_to_read);
    //printf("while count: %d\n", process(inData, recievedData));
  
  //}
}

int 
MRP2_Serial::process (uint8_t *inData, int recievedData, uint8_t _command_to_read)
{
  //uint8_t inData[255] = {0};
  //int recievedData = sizeof(inData)/sizeof(inData[0]);
  int startIndex = 0;
  int proc_length = 0;
  int while_cnt = 0;
  int _ret_val = 0;
  //recievedData = mrp_serial.read(inData,20);
  //usleep(50);
  //recievedData = RS232_PollComport(_port_nr, inData, 255); // Immediately returns
//ROS_INFO("recieved: %d\n",recievedData);
  //printf("Port %d polled with baud %d, received %d bytes.\n", _port_nr, _baudrate, recievedData);
  

  if (recievedData > 0) // Eğer data alınmışsa
  {

      //printf("-------------------------------------------------\n");
     // ROS_INFO("recieved: %d\n",recievedData);
      //print_array(inData, recievedData);
      if(tempDataIndex == 0) // Daha önceden hiç data yoksa
      {
          memcpy(tempData, inData, recievedData); // datayı temp'e al
          tempDataIndex = recievedData;
      }else{ // Daha önceden data var fakat yeterli değil
          //ROS_INFO("recievedData: %d\n", recievedData);
          memcpy(&tempData[tempDataIndex], &inData[0], recievedData); // datayı temp'e ekler
          tempDataIndex += recievedData;
      }

      //ROS_INFO("Total with recieved ");
      //printArray(tempData,tempDataIndex);

      tempDataIndex = find_message_start(tempData,tempDataIndex);
  }
  //ROS_INFO("tempDataIndex1:  %d", tempDataIndex);
  if (tempDataIndex > 3 && tempData[0] == startChar) // ilk checksum için yeterli data varsa ve ilk karakter doğruysa
  {
      //print_array(tempData, 50);
      int data_len = first_validator(tempData);
      //printf("TempdataIndex :%d, data_len: %d\n",tempDataIndex, data_len);

      if (tempDataIndex < data_len+5)
      {
        return _ret_val;
      }

      if (data_len != -1)
      {
        if (tempData[1] != _command_to_read)
        {
          //printf("\nData (i.e, %d) is valid but not the one we are waiting (i.e: %d). Trashing and Continuing...\n",tempData[1], _command_to_read);
          tempData[0] = '0';
          return _ret_val;
          /*if (tempData[4] != _command_to_read)
          {
            printf("\nThis is not the ACK we wait for. It is for %d. Trashing and Continuing...\n", tempData[4]);
            tempData[0] = '0';
            continue;
          }*/
          
        }
      }

      //printArray(tempData,tempDataIndex);
      if(data_len == 0 && tempDataIndex >= 3)
      {
          //ROS_INFO("Data is VALID! 1 checksum\n");
          _ret_val = execute_command(tempData);
          tempData[0] = '0'; // ilk mesajı boz
          tempDataIndex = find_message_start(tempData,tempDataIndex);
      }else if(data_len > 0 && tempDataIndex >= data_len+5)
      {
          if(second_validator(tempData, data_len) != -1)
          {
              //ROS_INFO("Data is VALID! 2 checksum\n");
              _ret_val = execute_command(tempData);
              //proc_length = sizeof(tempData)/sizeof(tempData[0]);
          }
          tempData[0] = '0'; // ilk mesajı boz
          tempDataIndex = find_message_start(tempData,tempDataIndex);
          //ROS_INFO("tempDataIndex2:  %d", tempDataIndex);
      }else if(data_len == -1){
          // Eğer checksum doğru değilse
          tempData[0] = '0'; // ilk mesajı boz
          tempDataIndex = find_message_start(tempData,tempDataIndex);
      }
      //while_cnt++;
  }
  return _ret_val;

}

void 
MRP2_Serial::array_chopper(uint8_t *buf, int start, int end) {
  //memcpy(&tempData[0], &buf[start], end);
  int k = 0;
  for (int i = start; i < start+end; i++)
  {
    tempData[k] = buf[i];
    k++;
  }
};

unsigned char 
MRP2_Serial::checksum(int size)
{
  int ret = 0;
  for(int i=0; i<size; i++)
  {
    ret = ret + sendArray[i];
  }
  if(size > 3)
  {
    return (ret & 255) - 1; 
  }
  return ret & 255 ;
}

unsigned char 
MRP2_Serial::checksum_check_array(uint8_t *arr, int size)
{
  int ret = 0;
  for(int i=0; i<size; i++)
  {
    ret = ret + arr[i];
  }
  if(size > 3)
  {
    ret = (ret & 0xFF) - 1; 
  }
  else{
    ret = ret & 0xFF; 
  }
  return ret;
}

bool 
MRP2_Serial::checksum_match(uint8_t *buf, int size) {
  int checksum = 0;
  int i = 0;
  for(i = 0; i<size; i++) // Sum-up all bytes
  {
      checksum = checksum + buf[i];
  }
  if(size > 3)
  {
    checksum = (checksum & 0xFF) - 1; // Standard MRP Serial checksum procedure
    //ROS_INFO("chksm: %02x",checksum);
  }else{
    checksum = checksum & 0xFF; // Standard MRP Serial checksum procedure
  }
  

  if(checksum == buf[size])    // Return result of the calculation
  {
      return true;
  }    
  return false;
};

int 
MRP2_Serial::first_validator(uint8_t *buf) {
  if(checksum_match(buf, 3))
    {
        return buf[2];
    }
    return -1;
};

int 
MRP2_Serial::second_validator(uint8_t *buf, int data_len) {
  int total_len = data_len + 5;
  if(checksum_match(buf, 4 + data_len))
  {
      return 1;
  }
  return -1;
};

int 
MRP2_Serial::find_message_start(uint8_t *buf,  int lastIndex) {
  int start = 0;
  for (start = 0; start < lastIndex ; start++)
  {
      if(buf[start] == startChar)
      {
          break;
      }
  }
  
  lastIndex = lastIndex - start;
          //ROS_INFO("char found at index: %d\n", start);
          //ROS_INFO("kopyalanacak sayı: %d",lastIndex+1);
          //printf("oldDataIndex: %d\n", old_tempDataIndex);
  array_chopper(buf, start, lastIndex+1);// '$' öncesini çöpe at
  
  return lastIndex;
};

int 
MRP2_Serial::execute_command(uint8_t *buf) {
  //std_msgs::Int32MultiArray array;
  //std_msgs::Int32 int_msg;
  //array.data.clear();
  //printf("\nReturn Command is %d\n", buf[1]);
  //print_array(buf,20);

  if(buf[1] == 'S')
  {
    _sonars.clear();
    _sonars.push_back(buf[4] + (buf[5] << 8));
    _sonars.push_back(buf[6] + (buf[7] << 8));
    _sonars.push_back(buf[8] + (buf[9] << 8));
    _sonars.push_back(buf[10] + (buf[11] << 8));
    _sonars.push_back(buf[12] + (buf[13] << 8));
    _sonars.push_back(buf[14] + (buf[15] << 8));
    _sonars.push_back(buf[16] + (buf[17] << 8));
  }

  return buf[1];

};

void 
MRP2_Serial::print_array(uint8_t *buf, int length) {
  printf("Array: ");
  int i = 0;
  for (i = 0; i < length; i++)
  {
      printf("%02x ",buf[i] );
  }
  printf("\n");
};
