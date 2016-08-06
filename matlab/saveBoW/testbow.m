clc
clear

global totalDimension; global eachDescIndices; global descWords;

addpath('../utils')
initGlobals;
dsetPath = descPath()
descWords = loadWords();

frames2 = readFramesBoW(dsetPath, [1 1 2], 10);
frames3 = readFramesBoW(dsetPath, [1 1 3], 10);
diff = zeros(5,0);
vocSize = 100;

for k=1:4
    descIndices = vocSize*(k-1)+1:vocSize*k;
    tdiff = frames2(:,descIndices) - frames3(:,descIndices);
    diff = [diff sum(abs(tdiff),2)];
end
diff