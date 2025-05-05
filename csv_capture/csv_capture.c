#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <time.h>
#include <ncurses.h>
#include <curses.h>

#define BUFFER_SIZE 256
#define PREFIX_COUNT 8
#define SERIAL_PORT "/dev/ttyACM0"
#define SAMPLE_RATE 25
#define DISPLAY_RATE 10

const char *prefixes[PREFIX_COUNT] = {"PD:", "PT:", "AX:", "AY:", "AZ:", "GX:", "GY:", "GZ:"};

int32_t buffer[PREFIX_COUNT] = {0};
float buffer_conv[PREFIX_COUNT] = {0};

int index_from_prefix(const char *line) {
    for (int i = 0; i < PREFIX_COUNT; ++i) {
        if (strncmp(line, prefixes[i], 3) == 0) {
            return i;
        }
    }
    return -1;
}

int setup_serial(const char *device) {
    int fd = open(device, O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        perror("open serial");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        return -1;
    }

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);    // Enable receiver, set local mode
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                 // 8-bit characters
    tty.c_cflag &= ~PARENB;             // No parity bit
    tty.c_cflag &= ~CSTOPB;             // 1 stop bit
    #ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;            // No hardware flow control
    #endif
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // No software flow control
    tty.c_oflag &= ~OPOST;                         // Raw output

    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIFLUSH); // flush any remaining input

    return fd;
}

void setup_display(){
    initscr();
    noecho();
    curs_set(FALSE);
}

uint64_t current_timestamp_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

void convert_buffer(){
    for(int i = 0; i < PREFIX_COUNT; i++){
        if(i < 2){ /*Pressure*/
            (buffer_conv[i]) = (float)((buffer[i])/1000000.0); /*psi*/
        }else if(i < 5){ /*Acceleration*/
            (buffer_conv[i]) = (float)((buffer[i])/100.0); /*m/s/s*/
        }else if(i < 8){ /*Angular Velocity*/
            (buffer_conv[i]) = (float)((buffer[i])/16.0); /*deg/s*/
        }
    }
}

void update_display(uint64_t stamp){
    
    mvprintw(0, 0, "TIME: \t\t\t\t %ld", stamp);
    mvprintw(2, 0, "DEVICE PRESSURE: \t\t %.2f", (double) buffer_conv[0]);
    mvprintw(4, 0, "INTERSECTION PRESSURE: \t\t %.2f", (double) buffer_conv[1]);
    mvprintw(6, 0, "ACCELERATION: \t\t\t (%.2f, %.2f, %.2f)", 
            (double) buffer_conv[2], (double) buffer_conv[3], (double)buffer_conv[4]);
    mvprintw(8, 0, "ANGULAR VELOCITY: \t\t (%.2f, %.2f, %.2f)", 
            (double) buffer_conv[5], (double) buffer_conv[6], (double) buffer_conv[7]);
    refresh();
    clear();
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s output.csv\n", argv[0]);
        return 1;
    }

    FILE *csv = fopen(argv[1], "w");
    if (!csv) {
        perror("fopen");
        return 1;
    }


    int serial_fd = setup_serial(SERIAL_PORT);
    if (serial_fd < 0) return 1;

    setup_display();

    char line[BUFFER_SIZE];
    size_t line_len = 0;
    char c;

    uint64_t last_write = 0;
    uint64_t last_display = 0;

    int read_number = 0;

    while (1) {
        ssize_t n = read(serial_fd, &c, 1);
        if (n <= 0) continue;

        if(c == ':'){
            read_number = 1;
        }else if(c == '\n'){
            read_number = 0;

            line[line_len] = '\0';

            int idx = index_from_prefix(line);
            mvprintw(10, 0, "added %s to index %d", line, idx);
            if (idx != -1) {
                int val = strtol((line), NULL, 10);
                buffer[idx] = val;
                
            }

            line_len = 0;
        } else if (line_len < BUFFER_SIZE - 1 && read_number) {
            line[line_len++] = c;
        }
        //mvprintw(11, 0, "line: %s", line);
        uint64_t now = current_timestamp_ms();
        if (now - last_write >= (1000/SAMPLE_RATE)) {
            fprintf(csv, "%ld", now); // time(NULL) is unix seconds
            for (int i = 0; i < PREFIX_COUNT; ++i) {
                fprintf(csv, ",%d", buffer[i]);
            }
            fprintf(csv, "\n");
            fflush(csv);
            last_write = now;
        }
        if(now - last_display >= (1000/DISPLAY_RATE)){
            convert_buffer();
            update_display(now);
        }
    }

    close(serial_fd);
    fclose(csv);
    return 0;
}
