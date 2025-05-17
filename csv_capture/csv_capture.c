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
#include <wchar.h>
#include <signal.h>

#define SERIAL_BUF_SIZE 255
#define PREFIX_COUNT 8
#define SERIAL_PORT "/dev/ttyACM0"
#define DISPLAY_RATE 10
#define NUM_TERMINAL_LINES 6
#define SENSOR_DISPLAY_OFFSET 7

const char *prefixes[PREFIX_COUNT] = {"DP:", "TP:", "AX:", "AY:", "AZ:", "GX:", "GY:", "GZ:"};

const char *pypevypers[6] ={
    "╔════╗      ╔════╗     ╗     ╔       ╔════╗       ╔════╗ ╝╗╝╗╝╗╝",
    "║░   ╚╣    ║║░   ╚╦════╩╗   ╔╝╚╣    ║║░   ╚╦════  ║░   ╚║╗",
    "╠═══╝ ╠╗╚╝╔╣╠═══╝ ╣░    ║╳ ╳║  ╠╗╚╝╔╣╠═══╝ ╣░     ╠═══╣  ╚══╬═╗",
    "║░     ╚══╝║║░    ╠═══  ╚╣╳╠╝   ╚══╝║║░    ╠═══   ║░  ╚╗   ╳  ║",
    "║░    ▐╠╗  ║║░    ║░     ╚╦╝   ▐╠╗  ║║░    ║░     ║░   ╚╗    ╔╝",
    "║░Chobot╚══╝║░erDu╚═══════╬ttaMao╚══╝║░Rour╚══════╣░ke ●╬════╝"
};



/*
PYPEVYPERS 2025 Lead design engineers
~~~ THOMAS CHOBOTER ~~~
~~~ RITVIK    DUTTA ~~~
~~~ JASON       MAO ~~~
~~~ JOHNNY   ROURKE ~~~
*/


/*
KEY
~R    = Reset
~P### = Set pressure sensor frequency
~E    = Start/Stop test
~F### = Set data capture rate
~N### = Set target PSI
~I### = Set IMU frequency
~S    = Start/Stop solenoid control
~D    = Set solenoid duty cycle (NOT IMPLEMENTED)
*/

WINDOW *console_win;
WINDOW *static_win;

int sample_rate = 25;
int test_running = 0;
int32_t buffer[PREFIX_COUNT] = {0};
float buffer_conv[PREFIX_COUNT] = {0};
char serial_buf[SERIAL_BUF_SIZE] = {0};
wchar_t user_buf[SERIAL_BUF_SIZE] = {0};
wchar_t console_buf[NUM_TERMINAL_LINES][SERIAL_BUF_SIZE] = {0};
int console_buf_top = 0;
int user_line_len = 0;
int serial_fd = 0;
FILE* csv_f = NULL;


/*void sig_handler(int signum) {
    werase(console_win);
    werase(static_win);
    tcflush(serial_fd, TCIOFLUSH);
    clear();
    close(serial_fd);
    fclose(csv_f);
    exit(0);
}*/

int index_from_prefix(const char *line) {
    for (int i = 0; i < PREFIX_COUNT; ++i) {
        if (strncmp(line, prefixes[i], 3) == 0) {
            return i;
        }
    }
    return -1;
}

int setup_serial(const char *device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
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
    tcflush(fd, TCIOFLUSH); // flush any remaining input & output data

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
    curs_set(0);
    keypad(stdscr, TRUE);

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    // Top static display (no scrolling)
    static_win = newwin(rows - NUM_TERMINAL_LINES-2, cols, 0, 0);

    // Bottom serial console (4 lines, scrolling)
    console_win = newwin(NUM_TERMINAL_LINES+2, cols, rows - NUM_TERMINAL_LINES-2, 0);
    scrollok(console_win, TRUE);
    nodelay(console_win, TRUE);

    box(console_win, 0, 0);
    mvwprintw(console_win, 0, 2, "[ Serial Console ]");

    for(int i = 0; i < 6; i++){
        mvwprintw(static_win, i, 10, "%s", pypevypers[i]);
    }
    
    wrefresh(static_win);
    wrefresh(console_win);
    
}

wchar_t* get_wchars(const char *str) {
    size_t len = strlen(str);
    wchar_t *wstr = malloc((len + 1) * sizeof(wchar_t));
    mbstowcs(wstr, str, len);
    wstr[len] = L'\0';
    return wstr;
}


void push_to_console(wchar_t *wserial_buf) {
    // Push the new line to the console buffer
    wcsncpy((wchar_t*)console_buf[console_buf_top], wserial_buf, SERIAL_BUF_SIZE);
    console_buf_top = (console_buf_top + 1) % NUM_TERMINAL_LINES;
    // Clear the console window
    werase(console_win);
    box(console_win, 0, 0);
    mvwprintw(console_win, 0, 2, "[ Serial Console ]");

    // Print the console buffer
    for (int i = 0; i < NUM_TERMINAL_LINES; ++i) {
        int index = (console_buf_top + i) % NUM_TERMINAL_LINES;
        if(wcslen(console_buf[index]) != 0) {
            mvwprintw(console_win, i + 1, 2, "> %ls", console_buf[index]);
        }
        
    }
}

void update_display(int serial_fd, uint64_t stamp){
    mvwprintw(static_win, SENSOR_DISPLAY_OFFSET+0, 4, "TIME: \t\t\t %ld", stamp);
    mvwprintw(static_win, SENSOR_DISPLAY_OFFSET+2, 4, "DEVICE PRESSURE: \t\t %7.2f", (double) buffer_conv[0]);
    mvwprintw(static_win, SENSOR_DISPLAY_OFFSET+4, 4, "INTERSECTION PRESSURE: \t %7.2f", (double) buffer_conv[1]);
    mvwprintw(static_win, SENSOR_DISPLAY_OFFSET+6, 4, "ACCELERATION: \t\t (%7.2f, %7.2f, %7.2f)", 
            (double) buffer_conv[2], (double) buffer_conv[3], (double)buffer_conv[4]);
    mvwprintw(static_win, SENSOR_DISPLAY_OFFSET+8, 4, "ANGULAR VELOCITY: \t\t (%7.2f, %7.2f, %7.2f)", 
            (double) buffer_conv[5], (double) buffer_conv[6], (double) buffer_conv[7]);
    wrefresh(static_win);
}

void update_display_hex(int serial_fd, uint64_t stamp){
    mvwprintw(static_win, SENSOR_DISPLAY_OFFSET+0, 4, "TIME: \t\t\t %lx", stamp);
    mvwprintw(static_win, SENSOR_DISPLAY_OFFSET+2, 4, "DEVICE PRESSURE: \t\t %7x", buffer[0]);
    mvwprintw(static_win, SENSOR_DISPLAY_OFFSET+4, 4, "INTERSECTION PRESSURE: \t %7x", buffer[1]);
    mvwprintw(static_win, SENSOR_DISPLAY_OFFSET+6, 4, "ACCELERATION: \t\t (%7x, %7x, %7x)", 
            buffer[2], buffer[3],buffer[4]);
    mvwprintw(static_win, SENSOR_DISPLAY_OFFSET+8, 4, "ANGULAR VELOCITY: \t\t (%7x, %7x, %7x)", 
            buffer[5], buffer[6], buffer[7]);
    wrefresh(static_win);
}

void handle_user_input() {
    wchar_t ch = wgetch(console_win);
    if (ch == ERR) {
        return; // No input
    }
    if (ch == '\n') {
        // Enter pressed: finish input
        user_buf[user_line_len+1] = L'\0';
        char output_buf[user_line_len + 1];
        wcstombs(output_buf, user_buf, user_line_len + 1);
        
        push_to_console(user_buf);
        if(user_buf[0]== L'~'){
            // Send command
            write(serial_fd, output_buf, user_line_len + 1);
            //push_to_console(get_wchars(output_buf+1));
            if(output_buf[1] == 'F'){
                char* user_string = malloc(SERIAL_BUF_SIZE);
                wcstombs(user_string, user_buf+2, SERIAL_BUF_SIZE);
                int freq = strtol(user_string, NULL, 10);
                free(user_string);
                if(freq > 0 && freq < 1000){
                    sample_rate = freq;
                    push_to_console(L"TTY:Data capture rate set");
                }else{
                    push_to_console(L"TTY:Invalid csv file data capture rate");
                }
            }else if(user_buf[1] == L'R'){
                sample_rate = 25;
                test_running = 0;
                push_to_console(L"TTY:Test stopped, reset all peripherals");
            }else if(user_buf[1] == L'E'){
                if(test_running){
                    test_running = 0;
                    push_to_console(L"TTY:Test stopped");
                }else{
                    test_running = 1;
                    push_to_console(L"TTY:Test started");
                }
            }
        }
        user_line_len = 0; // reset buffer
        user_buf[0] = L'\0'; // reset buffer
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
    mvwprintw(console_win, max_y - 1, 2, "> %ls", user_buf);
    wclrtoeol(console_win);
    wrefresh(console_win);
}


int main(int argc, char *argv[]) {

    /*if(signal(SIGINT, sig_handler) == SIG_ERR || signal(SIGTERM, sig_handler) == SIG_ERR){
        perror("signal");
        return 1;
    }*/

    if (argc != 2) {
        fprintf(stderr, "Usage: %s output.csv\n", argv[0]);
        return 1;
    }

    csv_f = fopen(argv[1], "w");
    if (!csv_f) {
        perror("fopen");
        return 1;
    }


    serial_fd = setup_serial(SERIAL_PORT);
    if (serial_fd < 0) return 1; //commented out for testing

    setup_display();

    size_t serial_buf_len = 0;
    char c;

    uint64_t last_write = 0;
    uint64_t last_display = 0;


    while (1) {
        handle_user_input();
        ssize_t n = read(serial_fd, &c, 1); //altered for testing
        //c = 'l'; // for testing

        if(n > 0 && c == '\n'){

            serial_buf[serial_buf_len] = '\0';
            if(serial_buf[0] == '~'){
                int idx = index_from_prefix(serial_buf+1);
            
                if (idx != -1) {
                    int val = strtol((serial_buf+4), NULL, 10);
                    buffer[idx] = val;
                    //mvprintw(10, 0, "index %d", idx);
                }    
            }
            else{ // print the serial line to console
                wclrtoeol(console_win);
                //wgetnstr(console_win, serial_buf, SERIAL_BUF_SIZE);
                wchar_t *wserial_buf = get_wchars(serial_buf);
                push_to_console(wserial_buf);
                wrefresh(console_win);
                free(wserial_buf);
            }
            serial_buf_len = 0;
        } else if (n > 0 && serial_buf_len < SERIAL_BUF_SIZE - 1) {
            serial_buf[serial_buf_len++] = c;
        }
        //mvprintw(11, 0, "line: %s", line);
        uint64_t now = current_timestamp_ms();
        if (now - last_write >= (1000/sample_rate) && test_running) {
            fprintf(csv_f, "%ld", now); // time(NULL) is unix seconds
            for (int i = 0; i < PREFIX_COUNT; ++i) {
                fprintf(csv_f, ",%d", buffer[i]);
            }
            fprintf(csv_f, "\n");
            fflush(csv_f);
            last_write = now;
        }
        if(now - last_display >= (1000/DISPLAY_RATE)){
            convert_buffer();
            update_display(serial_fd, now);
        }
    }

    close(serial_fd);
    fclose(csv_f);
    return 0;
}
