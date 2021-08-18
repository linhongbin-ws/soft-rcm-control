addpath(genpath(fullfile('..','..','soft-rcm-control')))
controller = teleopRCM('MTML');
fprintf('before teleop\n')
controller.start();
fprintf('start teleop\n')
pause(5);
controller.stop();
fprintf('stop teleop\n')
controller.close()
fprintf('close teleop\n')
delete(controller)
fprintf('delete teleop\n')
