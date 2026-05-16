function odom = read_odom(odomSub)
% [x, y, yaw] from the latest /odom message. Errors if no message yet;
% callers that may run before odom is available must guard for that.
    msg = odomSub.LatestMessage;
    odom = [msg.pose.pose.position.x, msg.pose.pose.position.y, ...
            quat_to_yaw(msg.pose.pose.orientation)];
end
