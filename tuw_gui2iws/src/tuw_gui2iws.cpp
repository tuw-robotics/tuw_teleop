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


#include <tuw_gui2iws/tuw_gui2iws.h>
#include <opencv/cv.h>
#include <boost/concept_check.hpp>
#include <cmath>


using namespace cv;
using namespace tuw;
using namespace std;


Gui2Iws::Gui2Iws ( const std::string &ns )
    : loop_count_ ( 0 )
    , figure_local_ ( ns + ", Control" )
    , new_trajectory( false ) {

    ROS_INFO("OpenCV: %s", CV_VERSION);
    
    
}

void Gui2Iws::init() {
    
}

double Gui2Iws::computeBodyStateTargetDeltaT() {
    double diffV   = config_.vel_base;
    double diffRho = config_.vel_rinv;
    double diffPhi = config_.vel_phi;
    
    double deltaV    = bodyStateTarget_[asInt(BodyVRP::V  )] - bodyStateNow_[asInt(BodyVRP::V  )];
    double deltaRho  = bodyStateTarget_[asInt(BodyVRP::RHO)] - bodyStateNow_[asInt(BodyVRP::RHO)];
    double deltaPhi  = normalizeAngle( bodyStateTarget_[asInt(BodyVRP::PHI)] - bodyStateNow_[asInt(BodyVRP::PHI)] );
    
    diffV   = diffV   * sgn( deltaV   );
    diffRho = diffRho * sgn( deltaRho ); 
    diffPhi = diffPhi * sgn( deltaPhi );
    
    double deltaTTrajV   = 0; if(fabs(diffV  ) > FLT_MIN){ deltaTTrajV   = deltaV   /   diffV;}
    double deltaTTrajRho = 0; if(fabs(diffRho) > FLT_MIN){ deltaTTrajRho = deltaRho / diffRho;}
    double deltaTTrajPhi = 0; if(fabs(diffPhi) > FLT_MIN){ deltaTTrajPhi = deltaPhi / diffPhi ;}
    
    
    double deltaTTraj = deltaTTrajPhi;
    if     (deltaTTrajPhi <= deltaTTrajRho) { deltaTTraj = deltaTTrajRho; }
    else if(deltaTTrajPhi >= deltaTTrajRho) { deltaTTraj = deltaTTrajPhi; }
    if     (deltaTTraj    <= deltaTTrajV  ) { deltaTTraj = deltaTTrajV;   }
    return deltaTTraj;
}

void Gui2Iws::onMouseMap ( int event, int x, int y, int flags, void* param ) {
    Gui2Iws *teleopVRA = ( Gui2Iws * ) param;
    Point2D pMap(x,y);
    
    double& bodyBufferV   = teleopVRA->bodyStateBuffer_[asInt(BodyVRP::V  )];
    double& bodyBufferRho = teleopVRA->bodyStateBuffer_[asInt(BodyVRP::RHO)];
    double& bodyBufferPhi = teleopVRA->bodyStateBuffer_[asInt(BodyVRP::PHI)];
    
    if       ( pMap.inside(teleopVRA->buttonTrigMinP.x()  , teleopVRA->buttonTrigMinP.y()  , teleopVRA->buttonTrigMaxP.x()  , teleopVRA->buttonTrigMaxP.y()  ) ) {
	if ( event == CV_EVENT_LBUTTONDOWN ) { teleopVRA->bodyStateTarget_ = teleopVRA->bodyStateBuffer_; teleopVRA->new_trajectory = true; ROS_DEBUG("Triggered targetState from Buffer"); }
	if ( event == CV_EVENT_LBUTTONUP )   {}
	if ( event == CV_EVENT_MBUTTONUP )   {}
	return;
    } else if( pMap.inside(teleopVRA->buttonVSlideMinP.x(), teleopVRA->buttonVSlideMinP.y(), teleopVRA->buttonVSlideMaxP.x(), teleopVRA->buttonVSlideMaxP.y()) ) {
	if ( event == CV_EVENT_LBUTTONDOWN ) { 
	    bodyBufferV =  + (teleopVRA->figure_local_.m2w ( pMap ).y() - teleopVRA->figure_local_.m2w( teleopVRA->buttonVSlideMaxP ).y() ) 
	                   / (teleopVRA->buttonVSlideMinPWorld.y() - teleopVRA->buttonTrigMinPWorld.y() ); 
	    ROS_DEBUG("Loaded target v= %f into buffer", bodyBufferV); 
	}
	if ( event == CV_EVENT_LBUTTONUP )   {}
	if ( event == CV_EVENT_MBUTTONUP )   {}
	return;
    } else if( ( event == CV_EVENT_LBUTTONDOWN ) || ( event == CV_EVENT_RBUTTONDOWN ) || ( event == CV_EVENT_MBUTTONDOWN ) ) { 
	Polar2D icc( teleopVRA->figure_local_.m2w ( pMap ) );
	bodyBufferPhi = icc.alpha();
	bodyBufferRho = 1. / icc.rho();
	
	if      ( event == CV_EVENT_RBUTTONDOWN ) { bodyBufferRho *= -1.; bodyBufferPhi = normalizeAngle(bodyBufferPhi + M_PI); } 
	else if ( event == CV_EVENT_MBUTTONDOWN ) { bodyBufferRho  =  0.;  }
	ROS_DEBUG("Loaded target rho=%f, phi=%f into buffer", bodyBufferRho, bodyBufferPhi);
    }
}

void Gui2Iws::initFigure(std::size_t pix_size, int radius_size, double grid_size){
    figure_local_.init ( pix_size, pix_size,
                         - radius_size, radius_size,
                         - radius_size, radius_size,
                         M_PI,
                         grid_size, grid_size );
    cv::namedWindow ( figure_local_.title(), CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL);
    cv::setMouseCallback ( figure_local_.title(), Gui2Iws::onMouseMap, this );
    
    initFigureRobotBase();
    
    buttonTrigMaxPWorld = Point2D(2,-2);///@todo
    buttonTrigMinPWorld = Point2D(1.8,-1.8);
    
    buttonTrigMaxP = figure_local_.w2m( buttonTrigMaxPWorld );
    buttonTrigMinP = figure_local_.w2m( buttonTrigMinPWorld );
    
    buttonVSlideMaxPWorld = Point2D(  buttonTrigMaxPWorld.x(),buttonTrigMinPWorld.y() );
    buttonVSlideMinPWorld = Point2D(  buttonTrigMinPWorld.x(),buttonTrigMinPWorld.y()+0.5);
    buttonVSlideMaxP = figure_local_.w2m( buttonVSlideMaxPWorld );
    buttonVSlideMinP = figure_local_.w2m( buttonVSlideMinPWorld );
    
    rectangle(figure_local_.background(), Point2d(buttonTrigMinP.x(), buttonTrigMinP.y()), Point2d(buttonTrigMaxP.x(),buttonTrigMaxP.y()), Figure::red, CV_FILLED,  CV_AA);
    figure_local_.line( figure_local_.background(), Point2D(buttonTrigMinPWorld.x() + (buttonVSlideMaxPWorld.x() - buttonVSlideMinPWorld.x()) / 2.,  buttonVSlideMaxPWorld.y()), 
			                            Point2D(buttonTrigMinPWorld.x() + (buttonVSlideMaxPWorld.x() - buttonVSlideMinPWorld.x()) / 2.,  buttonVSlideMinPWorld.y()), 
			Figure::black, 3,  CV_AA );
}
void Gui2Iws::initFigureRobotBase(){
    for(size_t i = 0; i < jointStates_.size(); i++){ 
	figure_local_.line( figure_local_.background(), Point2D(legInfo[                   i].x, legInfo[                   i].y), 
			                                Point2D(legInfo[(i+1)%legInfo.size()].x, legInfo[(i+1)%legInfo.size()].y), Figure::gray, 3,  CV_AA );
    }
}

void Gui2Iws::plot() {
    initFigureRobotBase();
    plotLocal();
    cv::waitKey ( 10 );
}

void Gui2Iws::plotLocal() {
    figure_local_.clear();
    
    double y_v;//plotting the v bars
    y_v = bodyStateTarget_[asInt(BodyVRP::V  )] *  (buttonVSlideMinPWorld.y() - buttonTrigMinPWorld.y() ) + buttonVSlideMaxPWorld.y() ;
    figure_local_.line( Point2D(buttonTrigMaxPWorld.x(), y_v), Point2D(buttonTrigMinPWorld.x(), y_v), Figure::black, 3,  CV_AA );
    y_v = bodyStateBuffer_[asInt(BodyVRP::V  )] *  (buttonVSlideMinPWorld.y() - buttonTrigMinPWorld.y() ) + buttonVSlideMaxPWorld.y() ;
    figure_local_.line( Point2D(buttonTrigMaxPWorld.x(), y_v), Point2D(buttonTrigMinPWorld.x(), y_v), Figure::green, 3,  CV_AA );
    
    fillLocalPlot();
    cv::imshow ( figure_local_.title(),figure_local_.view() );
}

void Gui2Iws::fillLocalPlot(){
    
    double rhoTarget = bodyStateTarget_[asInt(BodyVRP::RHO)];
    double angTarget = bodyStateTarget_[asInt(BodyVRP::PHI)];
    figure_local_.circle( Point2D( 1./rhoTarget * cos(angTarget), 1./rhoTarget * sin(angTarget) ), 2, Figure::red, 1,  CV_AA );
    
    for(size_t i = 0; i < jointStates_.size(); ++i){ 
	
	const double& lx = legInfo[i].x; const double& ly = legInfo[i].y;
	double csteer   = cos(jointStates_[i][asInt(JointsTypes::STEER)]), csteer90 = cos(jointStates_[i][asInt(JointsTypes::STEER)] + M_PI/2.);
	double ssteer   = sin(jointStates_[i][asInt(JointsTypes::STEER)]), ssteer90 = sin(jointStates_[i][asInt(JointsTypes::STEER)] + M_PI/2.);
	double des_wheel_speed = jointStates_[i][asInt(JointsTypes::REVOL)] * wheelRadius_;
	figure_local_.line( Point2D(lx + .01 * csteer,   
				    ly + .01 * ssteer ), 
			    Point2D(lx + .01 * csteer + des_wheel_speed * csteer90,   
				    ly + .01 * ssteer + des_wheel_speed * ssteer90 ), Figure::blue_dark, 1,  CV_AA );
				
	figure_local_.line( Point2D(lx + .01 * csteer + des_wheel_speed * csteer90,   
				    ly + .01 * ssteer + des_wheel_speed * ssteer90 ), 
			    Point2D(lx + des_wheel_speed * csteer90,   
				    ly + des_wheel_speed * ssteer90 ), Figure::blue_dark, 1,  CV_AA );
	
	
	
	cv::Point2f vertices[4];//plotting the wheels
	cv::RotatedRect(cv::Point2d(lx, ly), cv::Size2f (wheelWidth_, 2*wheelRadius_), rad2deg(jointStates_[i][asInt(JointsTypes::STEER)]) ).points(vertices);
	for (size_t j = 0; j < 4; ++j){ figure_local_.line( vertices[j], vertices[(j+1)%4], Figure::green_dark, 1, CV_AA); }
	
	bool plot_right = true, plot_left = true;
	if( bodyStateNow_[asInt(BodyVRP::RHO)] < 0.1){ plot_right = false; plot_left =  true; }
	else                                               { plot_right =  true; plot_left = false; }
	if( plot_right ) { figure_local_.line( Point2D(lx, ly),  Point2D(lx + 2. * csteer, ly + 2. * ssteer), Figure::blue_dark, 1,  CV_AA ); }
	if( plot_left  ) { figure_local_.line( Point2D(lx, ly),  Point2D(lx - 2. * csteer, ly - 2. * ssteer), Figure::blue_dark, 1,  CV_AA ); }
    }
}

