visualizer = Visualizer()

duration = 10
for i = 1:round(duration/0.2)
    visualizer.render()
    pause(0.2)
end