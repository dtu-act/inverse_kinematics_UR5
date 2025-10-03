function grid_plane = gen_gridplane(dims, res)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

Lx = dims(1);
Ly = dims(2);
Lz = dims(3);

x = -Lx/2:res:Lx/2;
y = -Ly/2:res:Ly/2;
z = -Lz/2:res:Lz/2;

[XX, YY, ZZ] = meshgrid(x,y,z);

grid_plane = [XX(:) YY(:) ZZ(:)];

disp(['X points = ' num2str(numel(x)) ' | Y points = ' num2str(numel(y)) ' | Z points = ' num2str(numel(z))])
disp(['Total positions = ' num2str(size(grid_plane,1))])

end