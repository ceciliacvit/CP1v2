function path = createPath(start, goal, map)
    mapCopy = copy(map);
    updateOccupancy(mapCopy, max(checkOccupancy(mapCopy), 0));
    inflate(mapCopy, 0.15);

    goal  = nearest_free(goal,  mapCopy);
    start = nearest_free(start, mapCopy);

    if isempty(goal) || isempty(start)
        path = [];
        return;
    end

    prm = mobileRobotPRM(map, 1000);
    prm.ConnectionDistance = 5;
    try
        path = prm.findpath(start, goal);
    catch
        path = [];
    end
end


function pt = nearest_free(pt, map)
    if checkOccupancy(map, pt) < 0.5
        return;
    end
    rc  = world2grid(map, pt);
    occ = map.occupancyMatrix;
    [nR, nC] = size(occ);
    for radius = 1:50
        for dr = -radius:radius
            for dc = -radius:radius
                if abs(dr) ~= radius && abs(dc) ~= radius, continue; end
                r = rc(1) + dr;
                c = rc(2) + dc;
                if r >= 1 && r <= nR && c >= 1 && c <= nC && occ(r,c) < 0.5
                    pt = grid2world(map, [r, c]);
                    return;
                end
            end
        end
    end
    pt = [];
end
