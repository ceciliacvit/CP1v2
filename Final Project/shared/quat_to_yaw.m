function yaw = quat_to_yaw(q)
% Yaw (rad) from a ROS quaternion message (fields w, x, y, z).
    eul = quat2eul([q.w, q.x, q.y, q.z]);
    yaw = eul(1);
end
