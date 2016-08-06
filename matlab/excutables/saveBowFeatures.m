clc
clear

% bruteforce로 모든것을 불러내서 일단 BoW 벡터로 바꾸고
% category{ instance{ video{ frames } } } hierarchical structure array 만들고
% video당 frame 5개 안 되는거 다 지우고
% instance당 video 3개 안 되는거 다 지우고
% (exclusive indexing: (1:3)'~=2 (2빼고 나머지))
% ->1)
% instance들 다 하나로 concatenation 하고 instance.mat으로 저장
%   recognition에서는 여기서 matrix refBoW(# instance x desc dim) from video1 과
%   matrix queryBow(# frames x desc dim) + gtInstanceVector from video2 를
%   만들고 둘을 비교해 결과 내고 video1과 2를 바꿔서 또 결과를 낸다.
% ->2)
% category당 instance 5개 안 되는거 다 지우고 category.mat 으로 저장
%   recognition에서는 여기서 matrix refBoW(# category x desc dim) from video1 과
%   matrix queryBow(# videos x desc dim) + gtInstanceVector from video2 를
%   만들고 둘을 비교해 결과 내고 video1과 2를 바꿔서 또 결과를 낸다.

% example
% a = magic(5);
% a((a(:,1)==4)&(a(:,2)==6),:)

global totalDimension; global eachDescIndices; global descWords;

addpath('../utils')
initGlobals;
dsetPath = descPath()
descWords = loadWords();

% filename = sprintf('%s/OBJ%d_%d_%d_%d.txt', dsetPath,1,1,1,1);
% frameData = load(filename);
% bowFeat = BoWFeature(frameData);
% return

emptyDesc = zeros(0, totalDimension);
videoSt = struct('frames', emptyDesc);
instanceSt = struct('videos', videoSt);
categorySt = struct('instances', instanceSt);

categoryLimit=5;
instanceLimit=10;
videoLimit=5;
frameLimit=5;

minFrames=5;
minVideos=3;

categoriesInit = 0;
categories = categorySt;

for ci=1:categoryLimit
    categoryInit = 0;
    tmpCategory = categorySt;
    for ni=1:instanceLimit
        instanceInit = 0;
        tmpInstance = instanceSt;
        % read video frames
        for vi=1:videoLimit
            tmpVideo = videoSt;
            tmpVideo.frames = readFramesBoW(dsetPath, [ci ni vi], frameLimit);
            if isempty(tmpVideo.frames)
                continue;
            end
            % add video to instance
            if size(tmpVideo.frames,1) < minFrames
                continue;
            elseif instanceInit==0
                tmpInstance.videos = tmpVideo;
            else
                tmpInstance.videos(end+1) = tmpVideo;
            end
            instanceInit = 1;
        end
        
        % add instance to category
        if instanceInit==0 || length(tmpInstance.videos) < minVideos
            continue;
        elseif categoryInit==0
            tmpCategory.instances = tmpInstance;
        else
            tmpCategory.instances(end+1) = tmpInstance;
        end
        categoryInit = 1;
        
        sprintf('ci=%d, instance(%d) has %d videos', length(categories)+categoriesInit ...
                , length(tmpCategory.instances) , length(tmpInstance.videos))
    end
    
    if categoryInit==0
        ci
        continue;
    elseif categoriesInit==0
        categories = tmpCategory;
    else
        categories(end+1) = tmpCategory;
    end
    categoriesInit = 1;
    sprintf('category(%d) has %d instances', length(categories), length(tmpCategory.instances))
end

filename = sprintf('%s/bowfeat.mat', dsetPath);
save(filename, 'categories');
