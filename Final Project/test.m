ros2 topic list

while true
    circle = detectCircle();
    if circle.found
        disp("circle identified")
    end
    pause(0.1)
end