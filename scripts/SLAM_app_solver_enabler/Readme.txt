This is a simple script which allows changing enable flags
of different problem solvers in the SLAM++ console app.

It is supposed to be run from the build directory, so that
the source files are in "../src/slam_app/*.cpp". You would
have to modify the script if you are using different build
environment.

To back-up the source codes first, run:

> ./slam_app_solver_enabler.sh --save


At any point later, one can:

> ./slam_app_solver_enabler.sh --restore


To see the state of the solvers:

> ./slam_app_solver_enabler.sh


To enable all (or some) solvers:

> ./slam_app_solver_enabler.sh --enable-all


To disable all (or some) solvers:

> ./slam_app_solver_enabler.sh --disable-all


The script prompts for user confirmation unless --force is
specified, and prints diff for each file to be modified so
that mistakes can be prevented and at the same time any of
the solvers can be selectively modified or not.
