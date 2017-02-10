function dstpath = workingDir(dsetIndex, radius)

rootfolder = 'E:\PaperData\rgbd-scene-dataset';
session = 'session1';

if nargin==1
    if dsetIndex == 1
        dstpath = sprintf('%s/corbs-cabinet2', rootfolder);
    elseif dsetIndex == 2
        dstpath = sprintf('%s/corbs-desk2', rootfolder);
    elseif dsetIndex == 3
        dstpath = sprintf('%s/corbs-human2', rootfolder);
    else
        error('dataset index out of bound')
    end
else
    if dsetIndex == 1
        dstpath = sprintf('%s/corbs-cabinet2/DescriptorR%d/%s', rootfolder, radius, session);
    elseif dsetIndex == 2
        dstpath = sprintf('%s/corbs-desk2/DescriptorR%d/%s', rootfolder, radius, session);
    elseif dsetIndex == 3
        dstpath = sprintf('%s/corbs-human2/DescriptorR%d/%s', rootfolder, radius, session);
    else
        error('dataset index out of bound')
    end
end

if exist(dstpath, 'dir')==0
    error('path does not exist: %s', dstpath)
end

end