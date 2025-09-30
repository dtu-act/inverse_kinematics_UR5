function scenario = loadScenario(filename)
% loadScenario - Load a scenario from a JSON file
%
% Usage:
%   scenario = loadScenario('scenario1.json')

fid = fopen(filename,'r');
if fid == -1
    error('Could not open file %s', filename);
end
raw = fread(fid,inf);
fclose(fid);

scenario = jsondecode(char(raw'));
end