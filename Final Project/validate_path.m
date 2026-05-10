function is_valid = validate_path(path,map)
    mapCopy = copy(map);
    path = world2grid(mapCopy,path);
    updateOccupancy(mapCopy,max(checkOccupancy(mapCopy),0));
    % inflate(map,5);
    show(mapCopy);

    min_value = min(improfile(mapCopy.occupancyMatrix,path(:,1),path(:,2),"nearest"));
    is_valid = min_value < 0.5;
end