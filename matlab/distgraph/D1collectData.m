function D1collectData(datasetPath)
'D1collectData'
global dataIndices
data = zeros(0,0);

curPath = pwd;
cd(datasetPath)
fileList = dir('DDS*.txt');
cd(curPath)
sprintf('Descriptor files: %d', length(fileList))

for i=1:length(fileList)
    filename = [fileList(i).folder, '/', fileList(i).name];
    if(exist(filename, 'file')==0)
        ['file not exists: ', filename]
        break;
    end
    
    framepos = dataIndices.frame;
    curdata = load(filename);
    curdata(:,framepos) = i;
    curdata = normalizeAllDescs(curdata);
    if isempty(data)
        data = curdata;
    else
        data = [data; curdata];
    end
    
    if mod(i, 100)==1
        sprintf('frame %d is added, %d, descMag %.2f %.2f %.2f', i, size(data,1), ...
                sum(data(end,dataIndices.descrs(3))), sum(data(end,dataIndices.descrs(5))), ...
                sum(data(end,dataIndices.descrs(6))) )
    end
end

filename = sprintf('%s/data.mat', datasetPath);
save(filename, 'data')
end

function outdata = normalizeAllDescs(inpdata)
global dataIndices
outdata = inpdata;

% FPFH
outdata(:,dataIndices.descrs(3)) = normalizeDescs(outdata(:,dataIndices.descrs(3)));
% SHOT
outdata(:,dataIndices.descrs(4)) = normalizeDescs(outdata(:,dataIndices.descrs(4)));
% SPIN
outdata(:,dataIndices.descrs(5)) = normalizeDescs(outdata(:,dataIndices.descrs(5)));
% TRIS
indices = dataIndices.descrs(5) + length(dataIndices.descrs(5));
outdata(:,indices) = normalizeDescs(outdata(:,indices));
indices = dataIndices.descrs(5) + length(dataIndices.descrs(5))*2;
outdata(:,indices) = normalizeDescs(outdata(:,indices));
end

function data = normalizeDescs(data)
sumDescs = sum(data,2);
for i=1:size(data,1)
    data(i,:) = data(i,:)/sumDescs(i);
end    
end