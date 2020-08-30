#!/usr/bin/env python3

#http://toilers.mines.edu/~jrosenth/101python/code/curses_plot/
#http://www.ironalbatross.net/wiki/index.php?title=Python_Curses
#https://ascii.co.uk/art/spider
#https://stackoverflow.com/questions/2953462/pinging-servers-in-python
#https://stackoverflow.com/questions/37340049/how-do-i-print-colored-output-to-the-terminal-in-python/37340245
#https://stackoverflow.com/questions/5977395/ncurses-and-esc-alt-keys
#https://ozzmaker.com/add-colour-to-text-in-python/
#https://stackoverflow.com/questions/11067800/ncurses-key-enter-is-fail
#https://lerneprogrammieren.de/python-http-requests-tutorial/

import curses
import subprocess as sp
import time
import requests

def check_spider_is_up():

    #constant ip - hard coded into c-code:
    spider_ip="192.168.4.1"

    status,result = sp.getstatusoutput("ping -c1 -w2 " + spider_ip)
    if status == 0:
        print("")
        print("Spider is coming online ...")
        print("Everything is ok.")
        print("")
        time.sleep(3)
    else:
        print("");
        print("Cannot connect to Spider !?")
        print("")
        print("Please connect to:") 
        print("Wifi Name: Robot - D33D")
        print("Wifi Pwd : 12345678")
        print("")
        exit()

# Curses main ...
def main():

    check_spider_is_up()

    stdscr = curses.initscr()
    curses.cbreak()
    stdscr.keypad(1)
    curses.curs_set(0)
    curses.noecho()

    #Chinesen Name: Corey-Little Spider
    stdscr.addstr(1,0,"---------------------------")
    stdscr.addstr(2,0,"Corey Little Spider Control")
    stdscr.addstr(3,0,"---------------------------")

    stdscr.addstr(5,0,"List of commands:   ")
    #up, down, left, right ... sort and add

    stdscr.addstr(7,0, "'up' - walk forward  ")
    stdscr.addstr(8,0, "'down' - walk backward ")
    stdscr.addstr(9,0, "'left' - turn left     ")
    stdscr.addstr(10,0,"'right' - turn right    ")
    # TODO ...
    stdscr.addstr(11,0,"'enter' - wave hello ")
    stdscr.addstr(12,0,"'q' - quit          ")
    stdscr.addstr(13,0,"")
    stdscr.refresh()

    key = ''
    while key != ord('q'):
        key = stdscr.getch()
        #stdscr.addch(13,0,key)
        stdscr.refresh()
        if key == curses.KEY_UP: 
            stdscr.addstr(13, 0, "up      ")
            r = requests.get("http://192.168.4.1/controller?pm=2")
        elif key == curses.KEY_DOWN: 
            stdscr.addstr(13, 0, "down    ")
            r = requests.get("http://192.168.4.1/controller?pm=3")
        elif key == curses.KEY_LEFT: 
            stdscr.addstr(13, 0, "left turn   ")
            r = requests.get("http://192.168.4.1/controller?pm=6")
        elif key == curses.KEY_RIGHT: 
            stdscr.addstr(13, 0, "right turn  ")
            r = requests.get("http://192.168.4.1/controller?pm=7")
        elif key == 10: #hack! enter
            stdscr.addstr(13, 0, "enter   ")
            r = requests.get("http://192.168.4.1/controller?pm=9")
        #elif key == ord('w'):
        #    stdscr.addstr(13, 0, "forward ")
        #elif key == ord('s'):
        #    stdscr.addstr(13, 0, "backward")
        elif key == ord('a'):
            stdscr.addstr(13, 0, "left shift   ")
            r = requests.get("http://192.168.4.1/controller?pm=4")
        elif key == ord('d'):
            stdscr.addstr(13, 0, "right shift  ")
            r = requests.get("http://192.168.4.1/controller?pm=5")
        elif key == ord('#'):
            stdscr.addstr(13, 0, "standby  ")
            r = requests.get("http://192.168.4.1/controller?pm=1")
        elif key == ord('+'):
            stdscr.addstr(13, 0, "push up  ")
            r = requests.get("http://192.168.4.1/controller?pm=11")
        elif key == ord('-'):
            stdscr.addstr(13, 0, "push up  ")
            r = requests.get("http://192.168.4.1/controller?pm=12")
#        elif key == 27: # escape with 1 sec delay !?!
#            stdscr.addstr(13, 0, "escape   ")
#            exit()
        stdscr.addstr(14, 0, "")
    curses.endwin()
    print("Goodbye!")
    print("")

# Main magic goes here
if __name__ == "__main__":
    main()
