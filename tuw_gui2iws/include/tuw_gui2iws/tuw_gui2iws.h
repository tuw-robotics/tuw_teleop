/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2015 by Horatiu George Todoran <todorangrg@gmail.com>   *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/


#ifndef GUI_2_IWS_H
#define GUI_2_IWS_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

#include <tuw_geometry/tuw_geometry.h>
#include <tuw_i2ws_flat_ctrl/iws_param_funcs_vrp_to_state_ch.h>

#include <tuw_gui2iws/Gui2IwsConfig.h>

namespace tuw {

double rad2deg(double angle_rad){
    return (angle_rad*180.0)/(double)M_PI;
}
double normalizeAngle(double angle) {
    while(angle > M_PI){
      angle = angle - (2*M_PI);
    }
    while(angle <= -M_PI){
      angle = angle + (2*M_PI);
    }
    return angle;
}
double sgn(double x){
    if(x>=0){ return  1; }
    else    { return -1; }
}

class CurrentState{
public:
    tuw::Polar2D ICC;
    double       v;
};

class Gui2Iws {
public:
    Gui2Iws(const std::string &ns); /// Constructor
    virtual ~Gui2Iws(){  }
    void init();                         /// initialization
    void initFigure(std::size_t pix_size, int radius_size, double grid_size);
    void plot();                         /// plots sensor input

    std::vector<cv::Point2d> legInfo;
    
    static void onMouseMap( int event, int x, int y, int flags, void* param );
protected:

    //Command cmd_;  /// output variables  v, w
    unsigned long loop_count_; /// counts the filter cycles
    
    double wheelRadius_;
    double wheelWidth_;

    //MeasurementLaser measurement_laser_;    /// laser measurements
    tuw::Figure  figure_local_;  /// Figure for data visualization
    tuw::Point2D buttonTrigMaxPWorld;
    tuw::Point2D buttonTrigMinPWorld;
    tuw::Point2D buttonTrigMaxP;
    tuw::Point2D buttonTrigMinP;
    tuw::Point2D buttonVSlideMaxPWorld;
    tuw::Point2D buttonVSlideMinPWorld;
    tuw::Point2D buttonVSlideMaxP;
    tuw::Point2D buttonVSlideMinP;
    
    double computeBodyStateTargetDeltaT();
    
    enum class JointsTypes { REVOL, STEER, ENUM_SIZE };
    std::vector<std::array<double, asInt(JointsTypes::ENUM_SIZE)> > jointStates_;
    
    enum class BodyVRP { V, RHO, PHI, ENUM_SIZE };
    std::array<double, asInt(BodyVRP::ENUM_SIZE)> bodyStateTarget_;
    std::array<double, asInt(BodyVRP::ENUM_SIZE)> bodyStateBuffer_;
    std::array<double, asInt(BodyVRP::ENUM_SIZE)> bodyStateNow_;
    
    void plotLocal();      /// plots sensor input in robot coordinates
    
    bool new_trajectory;
    
    tuw_teleop::Gui2IwsConfig config_;
    
private:
    
    void initFigureRobotBase();
    void fillLocalPlot();
};

}

#endif // GUI_2_IWS_H

