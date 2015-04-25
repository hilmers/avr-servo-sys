#include <serialport.h>
#include <stdio.h>

void print_init(){
    printf("Welcome to the inside of ATMEL AVR, architecture by hilms.\n");
}

int main() {
    int fd = serial_init("/dev/ttyS0");
    FILE *serial_comm = fdopen(fd, "r+");

    char buff[20];
    char* result = fgets(buff, sizeof(buff), stdin);
    while (result == buff) {
        fputs(buff, serial);
        result = fgets(buff, sizeof(buff), serial);
        if (result == buff) {
            fputs(buff, stdout);
        }
        fputs("> ", stdout);
        result = fgets(buff, sizeof(buff), stdin);
    }
    putchar("\n");
    fclose(serial);
    serial_cleanup(fd);
    return 0;
}