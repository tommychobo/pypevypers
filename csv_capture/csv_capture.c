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
#include <locale.h>

#define SERIAL_BUF_SIZE 256
#define PREFIX_COUNT 8
#define SERIAL_PORT "/dev/ttyACM0"
#define SAMPLE_RATE 25
#define DISPLAY_RATE 10

const char *prefixes[PREFIX_COUNT] = {"DP:", "TP:", "AX:", "AY:", "AZ:", "GX:", "GY:", "GZ:"};

const char *pypevypers[6] ={
    "╔════╗      ╔════╗     ╗     ╔       ╔════╗       ╔════╗ ╝╗╝╗╝╗╝",
    "║░   ╚╣    ║║░   ╚╦════╩╗   ╔╝╚╣    ║║░   ╚╦════  ║░   ╚║╗",
    "╠═══╝ ╠╗╚╝╔╣╠═══╝ ╣░    ║╳ ╳║  ╠╗╚╝╔╣╠═══╝ ╣░     ╠═══╣  ╚══╬═╗",
    "║░     ╚══╝║║░    ╠═══  ╚╣╳╠╝   ╚══╝║║░    ╠═══   ║░  ╚╗   ╳  ║",
    "║░    ▐╠╗  ║║░    ║░     ╚╦╝   ▐╠╗  ║║░    ║░     ║░   ╚╗    ╔╝",
    "║░  ●   ╚══╝║░  ● ╚═══════╬ ● ●  ╚══╝║░  ● ╚══════╣░  ● ╬════╝"
};

/*
PYPEVYPERS 2025 Lead design engineers
~~~ THOMAS CHOBOTER ~~~
~~~ RITVIK    DUTTA ~~~
~~~ JASON       MAO ~~~
~~~ JOHNNY   ROURKE ~~~
*/

WINDOW *console_win;
WINDOW *static_win;

int32_t buffer[PREFIX_COUNT] = {0};
float buffer_conv[PREFIX_COUNT] = {0};
char serial_buf[SERIAL_BUF_SIZE] = {0};
char user_buf[SERIAL_BUF_SIZE] = {0};
bool has_serial_line = false;
int user_line_len = 0;


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

void setup_display(){
    setlocale(LC_ALL, "");
    initscr();
    cbreak();
    noecho();
    curs_set(1);
    keypad(stdscr, TRUE);

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    // Top static display (no scrolling)
    static_win = newwin(rows - 4, cols, 0, 0);

    // Bottom serial console (4 lines, scrolling)
    console_win = newwin(4, cols, rows - 4, 0);
    scrollok(console_win, TRUE);
    nodelay(console_win, TRUE);

    box(console_win, 0, 0);
    mvwprintw(console_win, 0, 2, "[ Serial Console ]");

    wrefresh(static_win);
    wrefresh(console_win);
    for(int i = 0; i < 6; i++){
        mvprintw(10+i, 0, "%s", pypevypers[i]);
    }
}



void update_display(int serial_fd, uint64_t stamp){
    
    mvwprintw(static_win, 0, 0, "TIME: \t\t\t %ld", stamp);
    mvwprintw(static_win, 2, 0, "DEVICE PRESSURE: \t %.2f", (double) buffer_conv[0]);
    mvwprintw(static_win, 4, 0, "INTERSECTION PRESSURE: \t %.2f", (double) buffer_conv[1]);
    mvwprintw(static_win, 6, 0, "ACCELERATION: \t\t (%.2f,\t%.2f,\t%.2f)", 
            (double) buffer_conv[2], (double) buffer_conv[3], (double)buffer_conv[4]);
    mvwprintw(static_win, 8, 0, "ANGULAR VELOCITY: \t (%.2f,\t%.2f,\t%.2f)", 
            (double) buffer_conv[5], (double) buffer_conv[6], (double) buffer_conv[7]);

    mvwprintw(console_win, 2, 2, "> ");
    wclrtoeol(console_win);
    wrefresh(console_win);
    wgetnstr(console_win, serial_buf, SERIAL_BUF_SIZE);

    // Show serial data if not a command
    if (serial_buf[0] != L'~') {
        int max_y = getmaxy(console_win);
        // Leave 1 line for input at the bottom
        wmove(console_win, max_y - 3, 2);
        wprintw(console_win, "%ls\n", (wchar_t*)serial_buf);
        wrefresh(console_win);
    }

    if(has_serial_line) {
        has_serial_line = false;
        int max_y = getmaxy(console_win);

        // Save input position
        wmove(console_win, max_y - 1, 0);
        wclrtoeol(console_win);

        // Insert serial message just before prompt
        wprintw(console_win, "%ls\n", (wchar_t*)serial_buf);
        wrefresh(console_win);

        // Re-print user input after scrolling
        mvwprintw(console_win, max_y - 1, 0, "> %ls", (wchar_t*)user_buf);
        wrefresh(console_win);
    }
    refresh();
    //clear();
}

void handle_user_input(int serial_fd) {
    int ch = wgetch(console_win);
    if (ch != ERR) {
        if (ch == '\n') {
            // Enter pressed: finish input
            user_buf[user_line_len] = L'\0';

            // Echo the full typed line as a "sent command"
            wprintw(console_win, "> %ls\n", (wchar_t*)user_buf);
            wrefresh(console_win);

            if(user_buf[0]== L'~'){
                // Send command
                user_buf[user_line_len] = L'\n';
                user_buf[user_line_len + 1] = L'\0';
                write(serial_fd, user_buf, user_line_len + 1);
            }

            user_line_len = 0; // reset buffer

        } else if (ch == KEY_BACKSPACE || ch == 127 || ch == '\b') {
            if (user_line_len > 0) {
                user_line_len--;
                user_buf[user_line_len] = L'\0';
            }
        } else if (user_line_len < SERIAL_BUF_SIZE - 1 && ch >= 32 && ch <= 126) {
            user_buf[user_line_len++] = ch;
            user_buf[user_line_len] = L'\0';
        }

        // Re-draw the input line
        int max_y = getmaxy(console_win);
        mvwprintw(console_win, max_y - 1, 0, "> %ls", (wchar_t*)user_buf);
        wclrtoeol(console_win);
        wrefresh(console_win);
    }
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
    if (serial_fd < 0) return 1; //commented out for testing

    setup_display();

    size_t serial_buf_len = 0;
    char c;

    uint64_t last_write = 0;
    uint64_t last_display = 0;


    while (1) {
        handle_user_input(serial_fd);
        ssize_t n = read(serial_fd, &c, 1); //altered for testing
        //c = 'l'; // for testing
        if (n <= 0) continue;

        if(c == '\n'){

            serial_buf[serial_buf_len] = '\0';
            if(serial_buf[0] == '~'){
                int idx = index_from_prefix(serial_buf+1);
            
                if (idx != -1) {
                    int val = strtol((serial_buf+4), NULL, 10);
                    buffer[idx] = val;
                    //mvprintw(10, 0, "index %d", idx);
                }    
            }
            else{
                has_serial_line = true;
            }
            serial_buf_len = 0;
        } else if (serial_buf_len < SERIAL_BUF_SIZE - 1) {
            serial_buf[serial_buf_len++] = c;
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
            update_display(serial_fd, now);
        }
    }

    close(serial_fd);
    fclose(csv);
    return 0;
}
