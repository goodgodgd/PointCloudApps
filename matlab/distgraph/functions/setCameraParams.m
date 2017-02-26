function setCameraParams(datasetPath)

global fu fv cu cv deadDepth depthFactor

if ~isempty(strfind(datasetPath, 'CoRBS'))
    fu = 468.60/2;  % focal length x
    fv = 468.61/2;  % focal length y
    cu = 318.27/2 + 1;  % optical center x + one base coordinate
    cv = 243.99/2 + 1;  % optical center y + one base coordinate
    deadDepth = 0.1;
    depthFactor = 5000;
elseif  ~isempty(strfind(datasetPath, 'rgbd-scenes'))
    fu = 570.3/2;  % focal length x
    fv = 570.3/2;  % focal length y
    cu = 320/2 + 1;  % optical center x + one base coordinate
    cv = 240/2 + 1;  % optical center y + one base coordinate
    deadDepth = 0.1;
    depthFactor = 1000;
else
    error('inappropriate path name')
end
end

