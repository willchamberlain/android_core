/*
 *  Licensed under variant of 3-Clause BSD License / BSD-3-Clause
 *
 *  Copyright (c) 2017; William Chamberlain; ARC Centre of Excellence for Robotic Vision, Queensland University of Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, the above attributions of authorship and contribution, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice, the above attributions of authorship and contribution, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its authors nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

package william.chamberlain.androidvosopencvros.time;

/**
 * Date & time - wraps time considerations for underlying technologies including Android and ROS.
 *
 * Android can be very lax about device/OS time accuracy, only updates from 3G/4G mobile network
 * sources, does not allow applications to update datetime without rooting the device.
 *
 * ROS relies on accurate time consensus - see the ROS documentation and technical specifications,
 * and the org.ros.timeNtpTimeProvider.java class - but WallTimeProvider defaults to
 * ClockTopicTimeProvider will use the /clock topic for RosBag replays or simulations, via a switch in DefaultNode.
 *
 *
 -------------------------------------------------------------
 On Android,
 - determine and maintain the offset-over-time compared to the NTP Server
 - do not use the System date, or the new Date(), or new Calendar()
 - be careful with the date use of all libraries - pass dates in
 - wrap a date utility around a single library/impl of this offset-over-time - e.g. ROSJava as this is the most time-sensitive externally-facing interface
 - application only uses dates from a date utility
 - use the offset-over-time whenever interpreting dates

 -------------------------------------------------------------
 -- Set up NTP, to get full control of datetime. --
 On each robot, and each VOS Server:

 sudo apt-get update
 sudo apt-get install ntp ntpdate

 -------------------------------------------------------------

 sudo timedatectl set-ntp off

 ntpdate -q time.qut.edu.au

 sudo ntpdate -b time.qut.edu.au

 ntpdate -q time.qut.edu.au

 sudo service ntp stop
 sudo ntpdate -b time.qut.edu.au
 sudo ntpdate -b time.qut.edu.au
 sudo ntpdate -b time.qut.edu.au
 sudo ntpdate -b time.qut.edu.au
 sudo ntpdate -b time.qut.edu.au
 sudo ntpdate -b time.qut.edu.au
 sudo service ntp start
 ntpdate -q time.qut.edu.au
 watch ntpq -cpe -cas
 *
 *
 *
 *
 * Created by will on 1/12/17.
 */

public class Datetime {
}
