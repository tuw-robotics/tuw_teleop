/***************************************************************************
 *   Copyright (C) 2013 by Markus Bader                                    *
 *   markus.bader@tuwien.ac.at                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <ncurses.h>
#include <tuw_keyboard/tuw_keyboard.h>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <termios.h>

using namespace tuw;

Keyboard::Keyboard ( )
    : quit_ ( false )
	, velocity_forward_steps_ ( 0 )
	, velocity_angular_steps_ ( 0 )
	, velocity_forward_ ( 0 )
	, velocity_angular_ ( 0 )
	, velocity_forward_max_ ( 0 )
	, velocity_angular_max_ ( 0 ){
      
}

Keyboard::~Keyboard () {
    t1.join();
    endwin(); // End curses mode
}

int getch_noblock()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

void Keyboard::keyboardListener() {
    int key;
    int row,col;
    char msg[0xFF];
    bool increase = false;
    sprintf ( msg, "Esc -> exit, 's' -> stop, 'i' -> increase." );
    getmaxyx ( stdscr,row,col );
    mvprintw ( row/4, ( col-strlen ( msg ) ) /2, "%s", msg );
    do {
        key = getch(); /* If raw() hadn't been called we have to press enter before it gets to the program */
        boost::interprocess::scoped_lock<boost::mutex> scoped_lock ( mutex_ );
        switch ( key ) {
        case KEY_UP:
            if ( increase ) {
                cmd_.v ( cmd_.v() + velocity_forward_steps_ );
            } else {
                cmd_.v (+velocity_forward_ );
                cmd_.w ( 0 );
            }
            break;
        case KEY_DOWN:
            if ( increase ) {
                cmd_.v ( cmd_.v() - velocity_forward_steps_ );
            } else {
                cmd_.v ( -velocity_forward_ );
                cmd_.w ( 0 );
            }
            break;
        case KEY_LEFT:
            if ( increase ) {
                cmd_.w ( cmd_.w() - velocity_angular_steps_ );
            } else {
                cmd_.v ( 0 );
                cmd_.w (velocity_angular_ );
            }
            break;
        case KEY_RIGHT:
            if ( increase ) {
                cmd_.w ( cmd_.w() + velocity_angular_steps_ );
            } else {
                cmd_.v ( 0 );
                cmd_.w ( -velocity_angular_ );
            }
            break;
        case KEY_ESC: //Esc
            cmd_.stop();
            quit_ = true;
            break;
        case 's':
            cmd_.stop();
            break;
        case 'i':
            increase = !increase;
            break;
        default:
            cmd_.stop();
            quit_ = true;
        }
        if ( cmd_.w() > +velocity_angular_max_ ) cmd_.w ( +velocity_angular_max_ );
        if ( cmd_.w() < -velocity_angular_max_ ) cmd_.w ( -velocity_angular_max_ );
        if ( fabs ( cmd_.w() ) < 0.001 ) cmd_.w ( 0.0f );
        if ( cmd_.v() > +velocity_forward_max_ )  cmd_.v ( +velocity_forward_max_ );
        if ( cmd_.v() < -velocity_forward_max_ )  cmd_.v ( -velocity_forward_max_ );
        if ( fabs ( cmd_.v() ) < 0.001 ) cmd_.v ( 0.0f );
        getmaxyx ( stdscr,row,col );
        sprintf ( msg, "v = %+3.2f, w = %+3.2f, increase = %4s", cmd_.v(), cmd_.w(), (increase?"on":"off") );
        mvprintw ( row/2+0, ( col-strlen ( msg ) ) /2, "%s", msg );
    } while ( !quit_ );
}


void Keyboard::initTeleop() {

    initscr();                        // Start curses mode
    raw();                            // Line buffering disabled
    keypad ( stdscr, TRUE );          // We get F1, F2 etc..
    noecho();                         // Don't echo() while we do getch
    int row,col;
    char mesg[0xFF];
    t1 = boost::thread ( &Keyboard::keyboardListener, this );
}