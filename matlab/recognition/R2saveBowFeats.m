function R2saveBowFeats(radius)
% clc
% clear
% radius = 3;

global dataPath bowFeatDim
[descIndices, descWords] = getDescIndicesWords();

videoSt = struct('frames', []);
emptyVideos = struct('frames', {});
InstanceSt = struct('videos', emptyVideos);
emptyInstances = struct('videos', {});
categorySt = struct('instances', emptyInstances);
emptyCategories = struct('instances', {});

categoryLimit=51;
instanceLimit=10;
videoLimit=5;
frameLimit=5;

minFrames=5;
minVideos=3;

categories = emptyCategories;
for ci=1:categoryLimit
    instances = emptyInstances;
    for ni=1:instanceLimit
        % read video frames
        videos = emptyVideos;
        for vi=1:videoLimit
            tmpVideo = videoSt;
            tmpVideo.frames = readFramesBoW([ci ni vi], frameLimit, descIndices, descWords);
            if size(tmpVideo.frames,1) < minFrames
%                 sprintf('continue video: ci=%d, ni=%d, vi=%d, frames=%d', ci, ni, vi ...
%                                                                 , size(tmpVideo.frames,1))
                continue;
            else
                sprintf('added video: ci=%d, ni=%d, vi=%d, frames=%d', ci, ni, vi ...
                                                                , size(tmpVideo.frames,1))
            end
            
            videos(end+1) = tmpVideo;
        end
        
        tmpInstance = InstanceSt;
        tmpInstance.videos = videos;        
        if length(videos) < minVideos
            sprintf('continue instance: ci=%d, ni=%d, instances=%d, videos=%d', ci, ni ...
                , length(instances), length(videos))
            continue;
        end
        
        instances(end+1) = tmpInstance;        
        sprintf('ci=%d, instances=%d, videos=%d', ci, length(instances), length(videos))
    end
    
    tmpCategory = categorySt;
    tmpCategory.instances = instances;
    if isempty(instances)
        sprintf('continue category: ci=%d, categories=%d, instances=%d', ci ...
                                , length(categories), length(instances))
        continue;
    end

    categories(end+1) = tmpCategory;
    sprintf('ci=%d, categories=%d, instances=%d', ci, length(categories), length(instances))
end

filename = sprintf('%s/bowfeat_%d.mat', dataPath, bowFeatDim);
save(filename, 'categories');
