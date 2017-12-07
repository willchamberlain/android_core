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

package william.chamberlain.androidvosopencvros;

import static william.chamberlain.androidvosopencvros.Hardcoding.CALIBRATED_FOCAL_LENGTH_X;
import static william.chamberlain.androidvosopencvros.Hardcoding.CALIBRATED_FOCAL_LENGTH_Y;
import static william.chamberlain.androidvosopencvros.Hardcoding.CALIBRATED_IMAGE_HEIGHT;
import static william.chamberlain.androidvosopencvros.Hardcoding.CALIBRATED_IMAGE_WIDTH;

/**
 * Created by will on 7/12/17.
 */
class FocalLengthCalculator {

    public static float getFocal_length_in_pixels_x(float imageWidth_) {
        // TODO - 640 is now a magic number : it is the image width in pixels at the time of calibration of focal length
        return CALIBRATED_FOCAL_LENGTH_X * ((float) imageWidth_ / CALIBRATED_IMAGE_WIDTH);  // TODO - for Samsung Galaxy S3s from /mnt/nixbig/ownCloud/project_AA1__1_1/results/2016_12_04_callibrate_in_ROS/calibrationdata_grey/ost.txt
    }

    public static float getFocal_length_in_pixels_y(float imageHeight_) {
        // TODO - 640 is now a magic number : it is the image width in pixels at the time of calibration of focal length
        return CALIBRATED_FOCAL_LENGTH_Y * ((float) imageHeight_ / CALIBRATED_IMAGE_HEIGHT);  // TODO - for Samsung Galaxy S3s from /mnt/nixbig/ownCloud/project_AA1__1_1/results/2016_12_04_callibrate_in_ROS/calibrationdata_grey/ost.txt
    }
}
