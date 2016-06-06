/***************************************************************************
 *   Copyright (C) 2010 by Markus Bader                                    *
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

#ifndef KEYBOARD_2_TWIST_H
#define KEYBOARD_2_TWIST_H


#include <tuw_keyboard2twist/tuw_keyboard2twist_defaults.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

namespace tuw {
  
enum Control_Mode{
  MODE_NA = 0,
  TELEOP = 1,
  WANDERER = 2
};

class DriveCommand {
public:
    DriveCommand() :v_ ( 0.0f ), w_ ( 0.0f ) {};
    DriveCommand ( float v, float w ) :v_ ( v ), w_ ( w ) {};
    const float &v() const {
        return v_;
    }
    const float &w() const {
        return w_;
    }    
    void w ( float w ) {
        w_ = w;
    }
    void v ( float v ) {
        v_ = v;
    }
    void stop(){
        v_ = 0.0f, w_ = 0.0f;
	}
private:
    float v_, w_;
};

class Keyboard2Twist {
public:

    Keyboard2Twist ();
    ~Keyboard2Twist ();
	void keyboardListener();
	int control_mode() const {return control_mode_;}
	bool quit() const {return quit_;}
	void initTeleop();
	void initWanderer();
	void laserListener();
protected:
	bool quit_;
	int control_mode_;
	double velocity_forward_steps_;
	double velocity_angular_steps_;
	double velocity_forward_;
	double velocity_angular_;
	double velocity_forward_max_;
	double velocity_angular_max_;
	std::vector<float> laser_, laser_angle_;
	
    DriveCommand cmd_;
    boost::mutex mutex_;           /// mutex
    boost::thread t1;
};

}
#endif // KEYBOARD_2_TWIST_H
