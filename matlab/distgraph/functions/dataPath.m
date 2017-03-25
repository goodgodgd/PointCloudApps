function [output] = dataPath(datasetIndex, radius)

persistent datasets rootpath
if isempty(datasets)
    datasets = {'CoRBS/cabinet', 'CoRBS/desk', 'CoRBS/human', 'CoRBS/racingcar', ...
                'rgbd-scenes/desk', 'rgbd-scenes/kitchen_small', 'rgbd-scenes/meeting_small', ...
                'rgbd-scenes/table', 'rgbd-scenes/table_small'};
    rootpath = '/home/cideep/Work/descriptor/datatset';
end

if nargin==0
    output = length(datasets);
    return
end

if length(datasets) < datasetIndex
    error('dataset index out of bound')
end

if nargin==1
    output = sprintf('%s/%s', rootpath, datasets{datasetIndex});
elseif nargin==2
    output = sprintf('%s/%s/DescriptorR%d', rootpath, datasets{datasetIndex}, radius);
end

if exist(output, 'dir')==0
    if mkdir(output)==0
        error('path does not exist: %s', output)
    end
end
end
