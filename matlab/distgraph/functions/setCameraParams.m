function setCameraParams(datasetPath)

global imscale imwidth imheight
global fu fv cu cv deadDepth depthFactor

imscale = 2;
imwidth = 640/imscale;
imheight = 480/imscale;

if ~isempty(strfind(datasetPath, 'CoRBS'))
    fu = 468.60/imscale;  % focal length x
    fv = 468.61/imscale;  % focal length y
    cu = 318.27/imscale + 1;  % optical center x + one base coordinate
    cv = 243.99/imscale + 1;  % optical center y + one base coordinate
    deadDepth = 0.1;
    depthFactor = 5000;
elseif  ~isempty(strfind(datasetPath, 'rgbd-scenes'))
    fu = 570.3/imscale;  % focal length x
    fv = 570.3/imscale;  % focal length y
    cu = 320/imscale + 1;  % optical center x + one base coordinate
    cv = 240/imscale + 1;  % optical center y + one base coordinate
    deadDepth = 0.1;
    depthFactor = 1000;
else
    error('inappropriate path name')
end
end

