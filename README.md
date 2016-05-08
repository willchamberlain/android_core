# Instructions
## Install
Somewhere on your computer:
```bash
git clone https://github.com/c-h-/android_core
cd android_core
git checkout indigo
git submodule update --init --recursive
```

#### Important Install Note
The Android SDK is required for this project to compile. To get set up for this project:
- Install and configure the Android SDK (`brew install android-sdk` on Mac OS X with Homebrew installed)
- Run `android`
- Install API 15 SDK Platform (Android 4.0.3)
- Install Android Build Tools 21.1.2

## Compile
Gradle is the build manager for these projects. Some commands you can run:
- `./gradlew assembleDebug`
- `./gradlew assembleRelease`
- `./gradlew clean`

`assembleDebug` and `assembleRelease` create Android-installable apps (apk files) for the child projects of this project.
To get the apk file of a build, navigate to the child project you want, and then to `build/outputs/apk`. The build you created will be there with some info in the file name. *Important:* Apps that are not signed for release and zipaligned are not acceptable for release via the Play Store. If you build a releasable app you'll need to sign it with a signing key and zip align it before it's ready to be distributed. You may want to optimize a releasable app; use [Proguard](https://developer.android.com/tools/help/proguard.html) and/or [Redex](http://fbredex.com/) to do so.

The `clean` command simply clears out cached/past build files.

## Install, Debug
For commands to install an apk to a device and see logs, see the commands in `android_sensors_driver/build_and_run.sh`.


## Original Readme:
rosjava is the first pure Java implementation of ROS.

From [ROS.org](http://www.ros.org/wiki/): ROS is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management.

Developed at Google in cooperation with Willow Garage, rosjava enables integration of Android and ROS compatible robots. This project is under active development and currently alpha quality software. Please report bugs and feature requests on the [issues list](https://github.com/rosjava/rosjava/issues?state=open).

To get started, visit the [rosjava_core](http://rosjava.github.com/rosjava_core/) and [android_core](http://rosjava.github.com/android_core/) documentation.

Still have questions? Check out the ros-users [discussion list](https://code.ros.org/mailman/listinfo/ros-users), post questions to [ROS Answers](http://answers.ros.org/questions/) with the tag "rosjava," or join #ROS on irc.oftc.net.

rosjava was announced publicly during the [Cloud Robotics tech talk at Google I/O 2011](http://www.youtube.com/watch?feature=player_embedded&v=FxXBUp-4800).

Looking for a robot platform to experiment with ROS, Android, and cloud robotics? The [Willow Garage](http://www.willowgarage.com/) [TurtleBot](http://www.willowgarage.com/turtlebot) is a great mobile perception platform for [getting started with robotics development](http://www.youtube.com/watch?feature=player_embedded&v=MOEjL8JDvd0).

Visit the rosjava_core wiki for instructions.

http://ros.org/wiki/android_core
