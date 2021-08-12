addpath(genpath(fullfile('..','..','soft-rcm-control')))
visualizer = Visualizer();
pause(0.5)
duration = 30;
for i = 1:round(duration/0.5)
    visualizer.render()
end
delete(visualizer)