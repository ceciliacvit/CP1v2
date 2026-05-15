function is_valid = validate_path(path, map)
    sv = validatorOccupancyMap;
    sv.Map = map;
    sv.ValidationDistance = 0.05;

    is_valid = true;
    for i = 1:size(path,1)-1
        if ~isMotionValid(sv, [path(i,:), 0], [path(i+1,:), 0])
            is_valid = false;
            return;
        end
    end
end
