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

import org.bytedeco.javacpp.opencv_core;

/**
 * Created by will on 7/12/17.
 */

public class Geometry_OpenCV {
    public static String matToString(opencv_core.Mat opencv_matrix) {
        String string = "[";
        for (int rowNum_ = 0; rowNum_<opencv_matrix.size().height(); rowNum_++) {   // rows
            string = string+"\n";
            for(int colNum_ = 0; colNum_<opencv_matrix.size().width(); colNum_++) { // columns
                string = string + opencv_matrix.getDoubleBuffer().get( rowNum_*opencv_matrix.size().width() + colNum_) + ",";
            }
        }
        string = string + "]";
        return string;
    }
}
