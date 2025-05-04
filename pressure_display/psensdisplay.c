#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>

#define SERIAL_PORT "/dev/ttyACM0"
#define MAX_PSI 50.0
#define GRAPH_WIDTH 70
#define FRAME_FREQ 10

int fd = -1;

void signal_handler(int signum){
    if((signum == SIGINT | signum == SIGTERM | signum == SIGKILL)
            && fd != -1){
        close(fd);
        exit(EXIT_SUCCESS);
    }
}

int init_serial(const char* port){
    fd = open(port, O_RDONLY | O_NOCTTY);
    if (fd == -1){
        perror("Error opening serial port");
        exit(1);
    }

    struct termios options;
    tcgetattr(fd, &options);

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag = CS8 | CLOCAL | CREAD; // 8 data bits, local, read enabled
    options.c_iflag = IGNPAR;   // ignore framing and parity errors

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

void draw_graph(float data, int max_val, char* units){
    int bar_length = (int)((data/max_val)*GRAPH_WIDTH);

    printf("[");
    for(int i = 0; i < bar_length; i++){
        printf("#");
    }
    for(int i = bar_length; i < GRAPH_WIDTH; i++){
        printf(" ");
    }
    printf("] %.2f %s \n", data, units);
    fflush(stdout);
}

int main(int argc, char** argv){
    int serial_fd = init_serial(SERIAL_PORT);

    char buffer[256];
    memset(buffer, 0, sizeof(buffer));

    printf("Reading voltage data from %s\n", SERIAL_PORT);
    // set console width to 80 columns and height to 40 rows
    //system("echo -ne \'e[8;80;40t\'");
    while(1){
        int n = read(serial_fd, buffer, sizeof(buffer)-1);
        if(strchr(buffer, '\n') != NULL){
            //printf("Data: %s", buffer);
            buffer[n] = '\0';
            int v_raw = 0;
            int tpressmicro = 0;
            int dpressmicro = 0;
            float tpress = 0.0;
            float dpress = 0.0;
            int result = sscanf(buffer, "DP:%d\nTP:%d\n", &dpressmicro, &tpressmicro);
            if(dpressmicro != 0 && tpressmicro != 0){
                
                system("clear");
                printf("   →→ PRESSURE DATA FROM %s: ←←   \n",
                     SERIAL_PORT);
                tpress = tpressmicro/1000000.0;
                dpress = dpressmicro/1000000.0;
                printf("Device pressure:\t\t\t %.2f\n", dpress);
                printf("Intersection pressure:\t\t\t %.2f\n", tpress);
                //draw_graph(tpress, MAX_PSI, "Intersection");
                //draw_graph(dpress, MAX_PSI, "Device");
            }
        }
        usleep((int)(1000000/FRAME_FREQ)); // 10 Hz ; 100 ms
    }

    close(serial_fd);
    exit(EXIT_SUCCESS);
}
