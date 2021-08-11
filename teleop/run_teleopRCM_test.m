addpath(genpath(fullfile('..','..','soft-rcm-control')))

controller = teleopRCM();
controller.start_teleop();
pause(2);
controller.stop_teleop();


