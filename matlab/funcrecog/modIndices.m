function dstIndices = modIndices(srcIndices, divider)

dstIndices = mod(srcIndices, divider)
zeroIndices = (dstIndices==0)
dstIndices(zeroIndices) = divider;
