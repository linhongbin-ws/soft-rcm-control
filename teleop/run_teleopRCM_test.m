addpath(genpath(fullfile('..','..','soft-rcm-control')))

controller = teleopRCM();
fprintf('before teleop\n')
controller.start_teleop();
fprintf('start teleop\n')
pause(120);
controller.stop_teleop();
fprintf('stop teleop\n')
controller.close()
fprintf('close teleop\n')
delete(controller)
fprintf('delete teleop\n')
