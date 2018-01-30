/*
	PULUROBOT RN1-HOST Computer-on-RobotBoard main software

	(c) 2017-2018 Pulu Robotics and other contributors
	Maintainer: Antti Alhonen <antti.alhonen@iki.fi>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as 
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.


	Names for status bits coming from the RobotBoard microcontroller (rn1-brain)


*/

const char* const MCU_FEEDBACK_COLLISION_NAMES[] =
{
/* 0 */ "Undefined",
/* 1 */ "Acceleration sensor (jerk)",
/* 2 */ "Wheel differential integral compared to gyro integral (wheel slip) : obstacle RIGHT",
/* 3 */ "Wheel differential integral compared to gyro integral (wheel slip) : obstacle LEFT",
/* 4 */ "Host watchdog ran out",
/* 5 */ "Wheel differential integral compared to gyro integral (wheel slip) : obstacle BACK LEFT",
/* 6 */ "Wheel differential integral compared to gyro integral (wheel slip) : obstacle BACK RIGHT"
};

const char* const MCU_NAVI_STOP_NAMES[] =
{
/* 0*/ "Object suddenly on the fwd or back collision course, quick stop",
/* 1*/ "Object on the fwd or back collision course, careful stop",
/* 2*/ "Object preventing turning, arse would hit",
/* 3*/ "res",
/* 4*/ "res",
/* 5*/ "res",
/* 6*/ "res",
/* 7*/ "res",
/* 8*/ "res",
/* 9*/ "res",
/*10*/ "res",
/*11*/ "res",
/*12*/ "res",
/*13*/ "res",
/*14*/ "res",
/*15*/ "res",
/*16*/ "res",
/*17*/ "res",
/*18*/ "res",
/*19*/ "res",
/*20*/ "res",
/*21*/ "res",
/*22*/ "res",
/*23*/ "res",
/*24*/ "res",
/*25*/ "res",
/*26*/ "res",
/*27*/ "res",
/*28*/ "res",
/*29*/ "res",
/*30*/ "res",
/*31*/ "res"
};

const char* const MCU_NAVI_ACTION_NAMES[] =
{
/* 0*/ "res",
/* 1*/ "very close object front RIGHT, turn LEFT",
/* 2*/ "close object front RIGHT, turn LEFT",
/* 3*/ "object front RIGHT, turn LEFT",
/* 4*/ "far-away object front RIGHT, turn LEFT early",
/* 5*/ "very far-away object front RIGHT, turn LEFT extra early",
/* 6*/ "very far-away, and probably non-colliding object front RIGHT, turn LEFT very slightly",
/* 7*/ "passing/going to pass object on RIGHT, do NOT turn RIGHT",
/* 8*/ "going to pass object farther away on RIGHT, do NOT turn RIGHT",
/* 9*/ "arse very close to hit if turned, do NOT turn RIGHT mode 2",
/*10*/ "arse close to hit if turned more than slightly, do NOT turn RIGHT mode 2",
/*11*/ "LEFT sonar sees an object, turn RIGHT",
/*12*/ "RIGHT sonar sees an object, turn LEFT",
/*13*/ "Both turn left and turn right active simultaneously, but slightly enough; compromise was calculated",
/*14*/ "res",
/*15*/ "res",
/*16*/ "res",
/*17*/ "very close object front LEFT, turn RIGHT",
/*18*/ "close object front LEFT, turn RIGHT",
/*19*/ "object front LEFT, turn RIGHT",
/*20*/ "far-away object front LEFT, turn RIGHT early",
/*21*/ "very far-away object front LEFT, turn RIGHT extra early",
/*22*/ "very far-away, and probably non-colliding object front LEFT, turn RIGHT very slightly",
/*23*/ "passing/going to pass object on LEFT, do NOT turn LEFT",
/*24*/ "going to pass object farther away on LEFT, do NOT turn LEFT",
/*25*/ "arse very close to hit if turned, do NOT turn LEFT",
/*26*/ "arse close to hit if turned more than slightly, do NOT turn LEFT mode 2",
/*27*/ "res",
/*28*/ "while REVERSING, trying to avoid an obstacle by turning RIGHT (relative to direction of motion)",
/*29*/ "while REVERSING, trying to avoid an obstacle by turning LEFT (relative to direction of motion)",
/*30*/ "res",
/*31*/ "res"
};

