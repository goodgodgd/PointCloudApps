function descWords = loadWords()

global eachDescIndices;

dsetPath = descPath();
numDescTypes = length(eachDescIndices);
value = zeros(2,2);
descWords = containers.Map({1, 2, 3, 4}, {value, value, value, value});

for i=1:numDescTypes
    filename = sprintf('%s/word%d.mat', dsetPath, i);
    words = load(filename);
    descWords(i) = words.words;
end
