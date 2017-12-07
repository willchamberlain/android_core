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



import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.rosjava_geometry.FrameTransform;
import org.ros.rosjava_geometry.Transform;
import org.ros.time.NtpTimeProvider;
import org.ros.time.TimeProvider;


/**
 * Created by will on 7/12/17.
 */

public class Test_RosJavaCompileAndRunExternal {

    public void doStuff() {
        Transform transform = Transform.translation(1.0, 2.0, 3.0);
        GraphName source = GraphName.of("map");
        GraphName target = GraphName.of("robot");
        Time time = new Time();
        FrameTransform frameTransform = new FrameTransform(transform,source,target,time);
        frameTransform.getTransform();
    }

    public static void main(String[] args) {
        Test_RosJavaCompileAndRunExternal test_rosJavaCompileAndRunExternal = new Test_RosJavaCompileAndRunExternal();
        test_rosJavaCompileAndRunExternal.doStuff();
    }

}
