function is_valid = validate_path(path,map)
    path = world2grid(map,path);
    updateOccupancy(map,max(checkOccupancy(map),0));


    min_value = min(improfile(map.occupancyMatrix,path(:,1),path(:,2),"nearest"))
    is_valid = min_value < 0.5;
end