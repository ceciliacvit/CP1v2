ros2 topic list
setenv('ROS_DOMAIN_ID', '30')

while true
    circle = detectCircle();
    if circle.found
        disp("circle identified")
        circle.angle_error
        circle.distance
    end
    %pause(0.1)
end