addpath(genpath(fullfile('..','..','soft-rcm-control')))

controller = teleopRCM();
controller.start_teleop();
pause(3);
controller.stop_teleop();
delete(controller)
