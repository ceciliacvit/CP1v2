function odom = read_odom(odomSub)
% [x y yaw] from latest /odom
    msg = odomSub.LatestMessage;
    odom = [msg.pose.pose.position.x, msg.pose.pose.position.y, ...
            quat_to_yaw(msg.pose.pose.orientation)];
end
