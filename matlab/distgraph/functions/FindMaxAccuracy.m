function [accuracy realThresh estiThresh] = FindMaxAccuracy(realDist, estiDist)

realThresh = mean(realDist);
estiThresh = mean(estiDist);
TP = find(realDist<realThresh & estiDist<estiThresh);
TN = find(realDist>realThresh & estiDist>estiThresh);
FP = find(realDist>realThresh & estiDist<estiThresh);
FN = find(realDist<realThresh & estiDist>estiThresh);
accuracy = (length(TP) + length(TN)) / (length(TP) + length(TN) + length(FP) + length(FN));

return

accuracy = 0;
realRange = [max(realDist)*0.3 max(realDist)*0.7]
realUnit = (realRange(2) - realRange(1))/100;
estiRange = [max(estiDist)*0.3 max(estiDist)*0.7]
estiUnit = (estiRange(2) - estiRange(1))/100;

for realVal=realRange(1):realUnit:realRange(2)
    for estiVal=estiRange(1):estiUnit:estiRange(2)
        TP = find(realDist<realVal & estiDist<estiVal);
        TN = find(realDist>realVal & estiDist>estiVal);
        FP = find(realDist>realVal & estiDist<estiVal);
        FN = find(realDist<realVal & estiDist>estiVal);
        tmpacc = (length(TP) + length(TN)) / (length(TP) + length(TN) + length(FP) + length(FN));
        if tmpacc > accuracy
            accuracy = tmpacc;
            realThresh = realVal;
            estiThresh = estiVal;
        end
    end
end

end
