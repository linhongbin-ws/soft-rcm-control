addpath(genpath(fullfile('..','..','soft-rcm-control')))

controller = teleopRCM();
controller.start_teleop();
pause(60);
controller.stop_teleop();
delete(controller)
