
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>
#include <iostream>
static int ret;
static int fd;

#define BAUD 115200 //115200 for JY61 ,9600 for others
class IMUData{
    public:
        IMUData(){
            ros::NodeHandle private_nh_("~");
            std::string _port;
            private_nh_.param("port", _port, std::string(default_port));
            PORT = _port.c_str();
        }
        ~IMUData(){};
        int uart_open(int fd, const char* pathname)
        {
            fd = open(pathname, O_RDWR|O_NOCTTY); 
            if (-1 == fd)
            { 
                perror("Can't Open Serial Port"); 
                return(-1); 
            } 
            else
                // printf("open %s success!\n",pathname);
            if(isatty(STDIN_FILENO)==0) 
                printf("standard input is not a terminal device\n"); 
            else 
                printf("isatty success!\n"); 
            return fd; 
        }

        int uart_set(int fd,int nSpeed, int nBits, char nEvent, int nStop)
        {
            struct termios newtio,oldtio; 
            if  ( tcgetattr( fd,&oldtio)  !=  0) {  
            perror("SetupSerial 1");
            printf("tcgetattr( fd,&oldtio) -> %d\n",tcgetattr( fd,&oldtio)); 
            return -1; 
            } 
            bzero( &newtio, sizeof( newtio ) ); 
            newtio.c_cflag  |=  CLOCAL | CREAD;  
            newtio.c_cflag &= ~CSIZE;  
            switch( nBits ) 
            { 
            case 7: 
            newtio.c_cflag |= CS7; 
            break; 
            case 8: 
            newtio.c_cflag |= CS8; 
            break; 
            } 
            switch( nEvent ) 
            { 
            case 'o':
            case 'O': 
            newtio.c_cflag |= PARENB; 
            newtio.c_cflag |= PARODD; 
            newtio.c_iflag |= (INPCK | ISTRIP); 
            break; 
            case 'e':
            case 'E': 
            newtio.c_iflag |= (INPCK | ISTRIP); 
            newtio.c_cflag |= PARENB; 
            newtio.c_cflag &= ~PARODD; 
            break;
            case 'n':
            case 'N': 
            newtio.c_cflag &= ~PARENB; 
            break;
            default:
            break;
            } 

            /*设置波特率*/ 

        switch( nSpeed ) 
            { 
            case 2400: 
            cfsetispeed(&newtio, B2400); 
            cfsetospeed(&newtio, B2400); 
            break; 
            case 4800: 
            cfsetispeed(&newtio, B4800); 
            cfsetospeed(&newtio, B4800); 
            break; 
            case 9600: 
            cfsetispeed(&newtio, B9600); 
            cfsetospeed(&newtio, B9600); 
            break; 
            case 115200: 
            cfsetispeed(&newtio, B115200); 
            cfsetospeed(&newtio, B115200); 
            break; 
            case 460800: 
            cfsetispeed(&newtio, B460800); 
            cfsetospeed(&newtio, B460800); 
            break; 
            default: 
            cfsetispeed(&newtio, B9600); 
            cfsetospeed(&newtio, B9600); 
            break; 
            } 
            if( nStop == 1 ) 
            newtio.c_cflag &=  ~CSTOPB; 
            else if ( nStop == 2 ) 
            newtio.c_cflag |=  CSTOPB; 
            newtio.c_cc[VTIME]  = 0; 
            newtio.c_cc[VMIN] = 0; 
            tcflush(fd,TCIFLUSH); 

        if((tcsetattr(fd,TCSANOW,&newtio))!=0) 
            { 
            perror("com set error"); 
            return -1; 
            } 
            printf("set done!\n"); 
            return 0; 
        }

        int uart_close(int fd)
        {
            assert(fd);
            close(fd);

            return 0;
        }
        int send_data(int  fd, char *send_buffer,int length)
        {
            length=write(fd,send_buffer,length*sizeof(unsigned char));
            return length;
        }
        int recv_data(int fd, char* recv_buffer,int length)
        {
            length=read(fd,recv_buffer,length);
            return length;
        }
        

        void ParseData(char chr)
        {
                static char chrBuf[100];
                static unsigned char chrCnt=0;
                signed short sData[4];
                unsigned char i;
                char cTemp=0;
                time_t now;
                chrBuf[chrCnt++]=chr;
                if (chrCnt<11) return;
                for (i=0;i<10;i++) cTemp+=chrBuf[i];
                if ((chrBuf[0]!=0x55)||((chrBuf[1]&0x50)!=0x50)||(cTemp!=chrBuf[10])) {printf("Error:%x %x\r\n",chrBuf[0],chrBuf[1]);memcpy(&chrBuf[0],&chrBuf[1],10);chrCnt--;return;}
                
                memcpy(&sData[0],&chrBuf[2],8);
                switch(chrBuf[1])
                {
                        case 0x51:
                            for (i=0;i<3;i++) a[i] = (float)sData[i]/32768.0*16.0;
                            time(&now);
                            // printf("\r\nT:%s a:%6.3f %6.3f %6.3f ",asctime(localtime(&now)),a[0],a[1],a[2]);
                            
                            break;
                        case 0x52:
                            for (i=0;i<3;i++) w[i] = (float)sData[i]/32768.0*2000.0;
                            // printf("w:%7.3f %7.3f %7.3f ",w[0],w[1],w[2]);					
                            break;
                        case 0x53:
                            for (i=0;i<3;i++) Angle[i] = (float)sData[i]/32768.0*180.0;
                            // printf("A:%7.3f %7.3f %7.3f ",Angle[0],Angle[1],Angle[2]);
                            break;
                        case 0x54:
                            for (i=0;i<3;i++) h[i] = (float)sData[i];
                            // printf("h:%4.0f %4.0f %4.0f ",h[0],h[1],h[2]);
                            
                            break;
                }		
                chrCnt=0;		
                //store values as sensor_msgs 
                double stamp = ros::Time::now().toSec();
                publishIMUData(stamp, Angle, a, w);
        }
        void publishIMUData(double stamp, float angle[3], float linear_acceleration[3], float angular_velocity[3] ){
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time(stamp);
            imu_msg.header.frame_id = "imu_link";

            imu_msg.linear_acceleration.x = a[0];
            imu_msg.linear_acceleration.y = a[1];
            imu_msg.linear_acceleration.z = a[2];

            imu_msg.angular_velocity.x = angular_velocity[0];
            imu_msg.angular_velocity.y = angular_velocity[1];
            imu_msg.angular_velocity.z = angular_velocity[2];
            tf::Quaternion q;
            q.setRPY(0.0, 0.0, degToRad(angle[2]));

            imu_msg.orientation.x = q.x();
            imu_msg.orientation.y = q.y();
            imu_msg.orientation.z = q.z();
            imu_msg.orientation.w = q.w();

            imu_msg.linear_acceleration_covariance[0] = 1.0f;
            imu_msg.linear_acceleration_covariance[4] = 1.0f;
            imu_msg.linear_acceleration_covariance[8] = 1.0f;

            imu_msg.angular_velocity_covariance[0] = 1e-6;
            imu_msg.angular_velocity_covariance[4] = 1e-6;
            imu_msg.angular_velocity_covariance[8] = 1e-6;

            imu_msg.orientation_covariance[0] = 1e-6;
            imu_msg.orientation_covariance[4] = 1e-6;
            imu_msg.orientation_covariance[8] = 1e-6;


            imu_pub_.publish(imu_msg);
        }

        double degToRad(double deg){
            return deg/180.0*M_PI;
        }
        //Publisher/Subscriber
        ros::NodeHandle nh;
        
        ros::Publisher imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu_data",10);
        //Publish

        //Variables
        const char* default_port = "/dev/ttyUSB0";
        const char* PORT;
        float a[3],w[3],Angle[3],h[3];
        // std::string IMU_TOPIC = "/imu_data";
};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "IMU_DATA");
    IMUData imu;
    
    char r_buf[1024];
    bzero(r_buf,1024);

    fd = imu.uart_open(fd,imu.PORT); 
    if(fd == -1)
    {
        fprintf(stderr,"uart_open error\n");
        exit(EXIT_FAILURE);
    }

    if(imu.uart_set(fd,BAUD,8,'N',1) == -1)
    {
        fprintf(stderr,"uart set failed!\n");
        exit(EXIT_FAILURE);
    }

	FILE *fp;
	fp = fopen("Record.txt","w");
    ros::Rate rate(100);
    while(ros::ok())
    {
        ROS_INFO("parse_data");
        ret = imu.recv_data(fd,r_buf,44);
        if(ret == -1)
        {
            fprintf(stderr,"uart read failed!\n");
            exit(EXIT_FAILURE);
        }
		for (int i=0;i<ret;i++) {fprintf(fp,"%2X ",r_buf[i]);imu.ParseData(r_buf[i]);}
        ros::spinOnce();
        rate.sleep();
    }

    ret = imu.uart_close(fd);
    if(ret == -1)
    {
        fprintf(stderr,"uart_close error\n");
        exit(EXIT_FAILURE);
    }

    exit(EXIT_SUCCESS);
}
