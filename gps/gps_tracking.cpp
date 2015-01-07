#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <netinet/in.h>
#include <pthread.h>
#include <string.h>
#include <arpa/inet.h>
#include <iostream>
#include <queue>

using namespace std;

const char APM_ADDRESS =    0x04;

const char REQUEST_GPS_LONG =   0x01;
const char REQUEST_GPS_LAT  =   0x00;

//I2C globals
char main_loop = 1; 
char i2c_receive_buffer[16];
char i2c_send_buffer[16];
double i2c_get_latitude(int device_handle);
double i2c_get_longitude(int device_handle);

//Network globals
char receive_loop = 1;
pthread_t receive_thread;
pthread_t send_thread;

int network_socket;
int port = 2550;
queue<char*> send_queue;
struct sockaddr_in client_addr, server_addr;
socklen_t server_socket_len = sizeof(server_addr); //documentation specifies init of this variable

char network_receive_buffer[16];
char network_send_buffer[16];
void* receive_function(void*);
void* send_function(void*);

pthread_t input_thread;
void* input_function(void*);

void cleanup()
{
    printf("Exiting...");
    receive_loop=main_loop=0;
    //fprintf(stdin, "%c", 'e');
    printf("Done\n");
}


int main()
{

    int result = 0;

    atexit(&cleanup);

    //set up i2c communication
    printf("Opening i2c device /dev/i2c-1.");
    int deviceHandle = open("/dev/i2c-1", O_RDWR);
    if(deviceHandle == -1)
    {
        printf("open Failed %0x\n", deviceHandle);
        exit(-1); 
    }
    printf(".Done\n");
    
    printf("Initiating communication with APM at address: %d", APM_ADDRESS);
    result = ioctl(deviceHandle, I2C_SLAVE, APM_ADDRESS);
    if(result != 0)
    {
        printf("ioctl Failed %0x\n", result);
        exit(-1); 
    }
    printf(".Done\n");


    //Set up UDP communciation
    network_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    
    //receive here
    bzero(&client_addr, sizeof(client_addr));
    client_addr.sin_family = AF_INET;;
    client_addr.sin_port = htons(port);
    printf("Binded to port %d.\n", port);
    bind(network_socket, (struct sockaddr *)&client_addr, sizeof(client_addr));
    
    //send here
    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    inet_aton("192.168.43.1", (struct in_addr*)&server_addr.sin_addr);
    server_addr.sin_port = htons(port);
    server_socket_len = sizeof(server_addr); 


    //start input and network threads
    pthread_create(&receive_thread, NULL, receive_function, NULL);
    pthread_create(&send_thread, NULL, send_function, NULL);
    pthread_create(&input_thread, NULL, input_function, NULL);
    
    
    //start i2c loop
    int counter = 0;
    char* drone_longitude = (char*)malloc(16);
    char* drone_latitude = (char*)malloc(16);
    double dLong = 5.0;
    double dLat = 6.0;
    
    while(main_loop == 1)
    {
        dLat = i2c_get_latitude(deviceHandle);
        usleep(50000);
        dLong = i2c_get_longitude(deviceHandle);
        
        sprintf(drone_longitude, "$LON%f", dLong);
        sprintf(drone_latitude, "$LAT%f", dLat);

        send_queue.push(drone_longitude);
        send_queue.push(drone_latitude);
        //printf("Added to the queue, Size %d\n", send_queue.size());
        
        //printf("%d\n", counter++);
        
        usleep(400000);
    }
    
        close(deviceHandle);

        pthread_join(receive_thread, NULL);
        pthread_join(input_thread, NULL);

        return 0;
}

double i2c_get_latitude(int device_handle)
{
    char temp[5];
    int result = write(device_handle, &REQUEST_GPS_LAT, 1);
    if(result == -1)
    {
        printf("write Failed %0x\n", result);
        close(device_handle);
        exit(-1); 
    }
    usleep(10000); //sleep for 10 ms so the APM can process the request
    
    //Now read the what we requested
    result = read(device_handle, i2c_receive_buffer, 4);
    if(result == -1)
    {
        printf("read Failed %0x\n", result);
        close(device_handle);
        exit(-1); 
    }
    

    temp[0] = 0x00 | i2c_receive_buffer[3];
    temp[1] = 0x00 | i2c_receive_buffer[2];
    temp[2] = 0x00 | i2c_receive_buffer[1];
    temp[3] = 0x00 | i2c_receive_buffer[0];
    
    int tempp;
    memcpy((void*)&tempp, i2c_receive_buffer, 4);
    double dtemp = (double)(tempp/10000000.0);
    //printf("\nDrone Latitude: %f\n", dtemp);
    

    return dtemp;
}

double i2c_get_longitude(int device_handle)
{
    char temp[5];
    int result = write(device_handle, &REQUEST_GPS_LONG, 1);
    if(result == -1)
    {
        printf("write Failed %0x\n", result);
        close(device_handle);
        exit(-1); 
    }
    usleep(50000);
    
    result = read(device_handle, i2c_receive_buffer, 4);
    if(result == -1)
    {
        printf("read Failed %0x\n", result);
        close(device_handle);
        exit(-1); 
    }
    temp[0] = 0x00 | i2c_receive_buffer[3];
    temp[1] = 0x00 | i2c_receive_buffer[2];
    temp[2] = 0x00 | i2c_receive_buffer[1];
    temp[3] = 0x00 | i2c_receive_buffer[0];
    
    int tempp;
    memcpy((void*)&tempp, i2c_receive_buffer, 4);
    double dtemp = (double)(tempp/10000000.0);
    //printf("\nDrone Longitude: %d\n", tempp);

    return dtemp;
}

 
void* receive_function(void*)
{
    printf("Receive thread started.\n");

    int n = 0;
    printf("Receive loop started.\n");
    while(receive_loop == 1)
    {
        n = recvfrom(network_socket, network_receive_buffer, 16, 0, (struct sockaddr *)&server_addr, &server_socket_len);
        printf("Received: %s\t\IP:%s\tBytes:%d\n", network_receive_buffer, inet_ntoa(server_addr.sin_addr), n);
        memset(network_receive_buffer, 0, strlen(network_receive_buffer));
    }

    printf("Exiting receive thread.\n");
    
}
 
void* send_function(void*)
{
    printf("Send thread started.\n");
    printf("Send loop started.\n");
    while(receive_loop ==1)
    {
        if(!send_queue.empty())
        {
            sendto(network_socket, send_queue.front(), strlen(send_queue.front()), 0,(struct sockaddr *)&server_addr, server_socket_len);
            printf("Send: %s\t\tIP: %s\n", send_queue.front(), inet_ntoa(server_addr.sin_addr));
            send_queue.pop();
        }
        usleep(20000);
    }
    printf("Send thread exiting.\n");
}

void* input_function(void*)
{
    
    printf("Input thread started.\n");
    char loop = 1;
    char input = 0;
    while(loop ==1)
    {
        input = getchar();
        if(input == 'e')
            loop=receive_loop=main_loop=0;
            
    }
    
    printf("Input thread exiting.\n");
    shutdown(network_socket, SHUT_RDWR);
    close(network_socket);
    
    
    
    
}
