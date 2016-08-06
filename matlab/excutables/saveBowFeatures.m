clc
clear

% bruteforce�� ������ �ҷ����� �ϴ� BoW ���ͷ� �ٲٰ�
% category{ instance{ video{ frames } } } hierarchical structure array �����
% video�� frame 5�� �� �Ǵ°� �� �����
% instance�� video 3�� �� �Ǵ°� �� �����
% (exclusive indexing: (1:3)'~=2 (2���� ������))
% ->1)
% instance�� �� �ϳ��� concatenation �ϰ� instance.mat���� ����
%   recognition������ ���⼭ matrix refBoW(# instance x desc dim) from video1 ��
%   matrix queryBow(# frames x desc dim) + gtInstanceVector from video2 ��
%   ����� ���� ���� ��� ���� video1�� 2�� �ٲ㼭 �� ����� ����.
% ->2)
% category�� instance 5�� �� �Ǵ°� �� ����� category.mat ���� ����
%   recognition������ ���⼭ matrix refBoW(# category x desc dim) from video1 ��
%   matrix queryBow(# videos x desc dim) + gtInstanceVector from video2 ��
%   ����� ���� ���� ��� ���� video1�� 2�� �ٲ㼭 �� ����� ����.

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
