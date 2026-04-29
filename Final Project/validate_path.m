function is_valid = validate_path(path,map)
    path = world2grid(map,path);
    is_valid = max(improfile(map.occupancyMatrix,path(:,1),path(:,2),"nearest")) < 0.65;
end