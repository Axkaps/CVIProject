pedestrianDb(10) = struct("ID", 0, "Centroid", [0, 0]);

for i = 1:10
    pedestrianDb(i).ID = i;
    pedestrianDb(i).Centroid = [i, i];
end

disp(pedestrianDb);