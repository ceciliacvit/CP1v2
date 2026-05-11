map1 = occupancyMap(10, 10, 1);
setOccupancy(map1, [1 2], 0.8);
occ = map1.occupancyMatrix;
loc = map1.GridLocationInWorld;
res = map1.Resolution;

map2 = occupancyMap(occ, res);
map2.GridLocationInWorld = loc;
disp('Map 1 loc:'); disp(getOccupancy(map1, [1 2]));
disp('Map 2 loc:'); disp(getOccupancy(map2, [1 2]));
disp('Map 2 other loc:'); disp(getOccupancy(map2, [2 1]));
