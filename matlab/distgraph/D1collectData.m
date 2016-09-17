function D1collectData(dsetIndex, radius)

frameCount = 1;
data = zeros(0,0);
datasetPath = workingDir(dsetIndex, radius);

while(1)
    frameCount=frameCount+1;
    filename = sprintf('%s/DDS_%05d.txt', datasetPath, frameCount);
    if(exist(filename, 'file')==0)
        break;
    end
    
    curdata = load(filename);
    % set frame index
    curdata(:,1) = frameCount*ones(size(curdata,1),1);
    if isempty(data)
        data = curdata;
    else
        data = [data; curdata];
    end
    sprintf('frame %d is added, %d', frameCount, size(data,1))
end

datasetPath = workingDir(dsetIndex, radius);
filename = sprintf('%s/data.mat', datasetPath);
save(filename, 'data')
