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

#define BUFFER_SIZE 256
#define PREFIX_COUNT 8
#define SERIAL_PORT "/dev/ttyUSB0"
#define SAMPLE_RATE 25
#define DISPLAY_RATE 10

const char *prefixes[PREFIX_COUNT] = {"PD:", "PT:", "AX:", "AY:", "AZ:", "GX:", "GY:", "GZ:"};

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

void convert_buffer(int32_t* buf, float* cbuf){
    for(int i = 0; i < PREFIX_COUNT; i++){
        if(i < 2){ /*Pressure*/
            *(cbuf+i) = (float)(*(buf+i)/1000000.0); /*psi*/
        }else if(i < 5){ /*Acceleration*/
            *(cbuf+i) = (float)(*(buf+i)/100.0); /*m/s/s*/
        }else if(i < 8){ /*Angular Velocity*/
            *(cbuf+i) = (float)(*(buf+i)/16.0); /*deg/s*/
        }
    }
}

void update_display(float* buf, uint64_t stamp){
    clear();
    mvprintw(0, 0, "TIME: \t\t\t %ld", stamp);
    mvprintw(0, 1, "DEVICE PRESSURE: \t\t\t %.2f", *buf);
    mvprintw(0, 2, "INTERSECTION PRESSURE: \t\t\t %.2f", *(buf+1));
    mvprintw(0, 3, "ACCELERATION: (%.2f, %.2f, %.2f)", 
            *(buf+2), *(buf+3), *(buf+4));
    mvprintw(0, 4, "ANGULAR VELOCITY: (%.2f, %.2f, %.2f)", 
            *(buf+5), *(buf+6), *(buf+7));
    refresh();
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

    int32_t buffer[PREFIX_COUNT] = {0};
    float buffer_conv[PREFIX_COUNT] = {0};

    int serial_fd = setup_serial(SERIAL_PORT);
    if (serial_fd < 0) return 1;

    setup_display();

    char line[BUFFER_SIZE];
    size_t line_len = 0;
    char c;

    uint64_t last_write = 0;
    uint64_t last_display = 0;

    while (1) {
        ssize_t n = read(serial_fd, &c, 1);
        if (n <= 0) continue;

        if (c == '\n') {
            line[line_len] = '\0';

            int idx = index_from_prefix(line);
            if (idx != -1) {
                int val = atoi(line + 3);
                buffer[idx] = val;
            }

            line_len = 0;
        } else if (line_len < BUFFER_SIZE - 1) {
            line[line_len++] = c;
        }

        uint64_t now = current_timestamp_ms();
        if (now - last_write >= (1000/SAMPLE_RATE)) {
            fprintf(csv, "%ld", time(NULL)); // unix seconds
            for (int i = 0; i < PREFIX_COUNT; ++i) {
                fprintf(csv, ",%d", buffer[i]);
            }
            fprintf(csv, "\n");
            fflush(csv);
            last_write = now;
        }

        if(now - last_display >= (1000/DISPLAY_RATE)){
            convert_buffer(&buffer, &buffer_conv);
            update_display(&buffer_conv, now);
        }
    }

    close(serial_fd);
    fclose(csv);
    return 0;
}
