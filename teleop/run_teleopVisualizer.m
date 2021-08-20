addpath(genpath(fullfile('..','..','soft-rcm-control')))
visualizer = Visualizer();
pause(1)
duration = 120;
fprintf('start render\n')
for i = 1:round(duration/0.2)
    tic
    visualizer.render()
    toc
end
fprintf('close render\n')
visualizer.close()
delete(visualizer)