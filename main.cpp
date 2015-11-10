/* 
 * File:   main.cpp
 * Author: Wen (reference.osch)
 * Single state xbee trasmission request and responding
 * Created on 10. NOV 2015, 11:15
 */
#include <iostream>
//#include <wiringSerial.h>   // wiring library https://projects.drogon.net/raspberry-pi/wiringpi/serial-library/
#include <cstdlib>
#include <stdio.h>
#include <time.h>
#include <bcm2835.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
using namespace std;

#define idle 1
#define node1on 2
#define node1off 3

volatile char state= 0;


int Xbeetransmit(int ref, unsigned char *packet,  int length);  //Xbee transmit function call declaration
unsigned char checkSum (unsigned char* packet_tx, int length);  //checkSum for the API frame 

//int packetLength=0;                                              //?
// frame buffers
//unsigned char resp_tx_on[21];                                   
//unsigned char resp_tx_off[21];



unsigned char packet_tx_on[] ={0x7E, 0x00, 0x0F, 0x10, 0x01, 0x00, 0x13, 0xA2, 0x00, 0x40, 0xAD, 0xD0, 0xE9, 0xFF, 0xFE, 0x00, 0x00, 0x7E, 0x18};   //uart transmit request21 bytes
void setup(int uart0_filestream ){                              //setup for serial transmission
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);  
}

int Xbeetransmit(int ref, unsigned char *packet, int packetLength) 
{
    std::cout << "transmit" << std::endl; 
    // Write to the port
    int n = write(ref, packet, packetLength);
    std::cout << n<< std::endl;
    if (n < 0) {
    //perror("Write failed - ");
        std::cout << "error" << std::endl; 
    return -1;
    }
  return n;
}
unsigned char checkSum (unsigned char* packet_tx, int length) {
	unsigned char res=0;
	long sum=0;
	for (int i =3 ; i<length-1; i++) {   
		sum = sum+packet_tx[i];
		}
	res=(unsigned char) 0xFF -(sum & 0xFF);	

 return res;	
 }

int main(int argc, char** argv) {                               //main loop
const  char *P="node1on";
int loopNo=4;                                                   //number of state
     int i=0;                                                   //index
 //int analogHigh=0;
 int fd;
 int n=0;
  // Open the Port. We want read/write, no "controlling tty" status, and open it no matter what state DCD is in
  fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);  //serialOpen ("/dev/ttyAMA0", 9600) ;used wiringpi
  if (fd == -1) {
      std::cout << "no uart" << std::endl; 
    //perror("open_port: Unable to open /dev/ttyAMA0 - ");
    return(-1);
  } else {
   std::cout << " uart" << std::endl;
   setup(fd);}
   fcntl(fd, F_SETFL, 0);
   state=idle;
  // Turn off blocking for reads, use (fd, F_SETFL, FNDELAY) if you want that
    char buf[256];
  
    delay(5000);
    std::cout << "start" << std::endl; 
    for (int i=0; i<loopNo; i++) {
    switch (state) {
   //std::cout << "state idle" << std::endl;
	 case idle : buf[0]='\0';printf("idle"); 
         if((Xbeetransmit(fd, packet_tx_on, packet_tx_on[2]+4))>-1) {
             state= node1on; delay(1000); 
             n = read(fd, (void*)buf, 19);
             delay(1000); break;
         } 
         else state=idle;
         break;  //delay(10000);
	 
	 case node1on:  std::cout << "node 1on" << std::endl;     //18 bytes returned in remote response
                                                       
         if (n < 0) {
            perror("Read failed - ");
            return -1;
         } 
         else if (n == 0){
             printf("No data on port\n");
         }
         else {
            buf[n] = '\0';
            for (int i=0; i<=n; i++){
                printf("%i bytes read : %X\n", i, buf[i]);
                delay(10);
            }
            buf[0]='\0';
            }          
            i=0; 
            if((Xbeetransmit(fd, packet_tx_on, packet_tx_on[2]+4))>-1) {
                state=node1off; 
                delay(5000);
            }
            break;

       
            printf("checksum : %X\n", checkSum((unsigned char*)buf, (n-1)));
            delay(5000);
             // Wait until we have a mouthful of data
            state=idle; 
            break;
        default: std::cout << "node 1off" << std::endl;
            state=idle;
        break;
    }
}
    close(fd);
    std::cout << "slut" << std::endl; 
    return 0;
}


