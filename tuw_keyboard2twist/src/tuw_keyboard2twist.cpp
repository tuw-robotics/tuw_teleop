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
#include <tuw_keyboard2twist/tuw_keyboard2twist.h>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <termios.h>

using namespace tuw;

Keyboard2Twist::Keyboard2Twist ( )
    : quit_ ( false )
	, control_mode_ ( MODE_NA )
	, velocity_forward_steps_ ( VELOCITY_FORWARD_STEPS )
	, velocity_angular_steps_ ( VELOCITY_ANGULAR_STEPS )
	, velocity_forward_ ( VELOCITY_FORWARD )
	, velocity_angular_ ( VELOCITY_ANGULAR )
	, velocity_forward_max_ ( VELOCITY_FORWARD_MAX )
	, velocity_angular_max_ ( VELOCITY_ANGULAR_MAX ){
      
}

Keyboard2Twist::~Keyboard2Twist () {
    switch ( control_mode_ ) {
    case TELEOP:
        t1.join();
        endwin(); // End curses mode
        break;
    }
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




void Keyboard2Twist::keyboardListener() {
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


void Keyboard2Twist::laserListener() {
   boost::interprocess::scoped_lock<boost::mutex> scoped_lock ( mutex_ );
    cmd_.stop();
    int right_idx = laser_.size() / 4;
    int left_idx =  (laser_.size() / 4)*3;
    double v = velocity_forward_;
    double w = velocity_angular_;
    for(int i = right_idx; i < left_idx; i++){
        if (laser_[i] < 1.0){
            v = velocity_forward_/2.0;
            w = velocity_angular_/2.0;
        }
    }
    if(laser_[right_idx] < 1.0){
      cmd_.w(-w);
    } else if(laser_[left_idx] < 1.0){
        cmd_.w(w);
    } else {
      cmd_.v(v);
    }
}

void Keyboard2Twist::initTeleop() {

    initscr();                        // Start curses mode
    raw();                            // Line buffering disabled
    keypad ( stdscr, TRUE );          // We get F1, F2 etc..
    noecho();                         // Don't echo() while we do getch
    int row,col;
    char mesg[0xFF];
    t1 = boost::thread ( &Keyboard2Twist::keyboardListener, this );
}
void Keyboard2Twist::initWanderer(){
    
}
